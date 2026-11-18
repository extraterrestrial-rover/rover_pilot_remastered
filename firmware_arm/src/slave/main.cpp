// Arm slave firmware: one ESP32 per goBILDA gearmotor, an I2C slave of the arm master.
// Build one environment per joint (pio run -e slave_shoulder | slave_elbow | slave_wrist_a | slave_wrist_b): the joint,
// and with it the I2C address, is compiled in.
//
// Safety behaviour (implemented and unit-tested in lib/arm_core, JointController):
//  - motor off after power-up until a neutral command from the master;
//  - no valid frame from the master for SLAVE_COMMAND_TIMEOUT_MS -> motor off, neutral needed again;
//  - e-stop flag -> motor off, latched until a neutral release;
//  - stall or wrong direction (from the encoder) -> motor off, FAULT latched until an e-stop; the master then stops
//    the other three motors too;
//  - corrupted I2C frames are discarded (CRC-16).
// The USB serial port prints a human-readable line twice a second for bench checks; it is not used for control.
#include <Arduino.h>
#include <Wire.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <soc/gpio_reg.h>
#include <string.h>

#include "arm_core.h"
#include "config.h"

#ifndef SLAVE_JOINT
#error "Build a slave with its environment (pio run -e slave_shoulder ...): SLAVE_JOINT selects the joint"
#endif
static_assert(SLAVE_JOINT >= 0 && SLAVE_JOINT < arm::kMotors, "SLAVE_JOINT must be 0..3");
static_assert(ENCODER_A_PIN < 40 && ENCODER_B_PIN < 40, "encoder pins must be ESP32 GPIOs");

namespace {

const JointHardware& kJoint = kJoints[SLAVE_JOINT];
const arm::JointConfig kConfig = {
    kJoint.max_rpm,
    kJoint.counts_per_rev,
    SLAVE_COMMAND_TIMEOUT_MS,
    kJoint.max_rpm * 1000.0f / SLEW_FULL_SCALE_MS,
    kJoint.encoder_fitted,
    kJoint.closed_loop,
    SPEED_KP,
    SPEED_KI,
    SPEED_INTEGRAL_LIMIT,
    STALL_PWM_FRACTION,
    STALL_RPM_FRACTION,
    STALL_MS,
    REVERSE_PWM_FRACTION,
    REVERSE_RPM_FRACTION,
    REVERSE_MS,
};
const char* const kStateNames[] = {"BOOT", "NO_COMMAND", "ESTOP", "WAIT_NEUTRAL", "ENABLED", "FAULT", "WAIT_SLAVES"};

arm::JointController controller(kConfig);
uint8_t reset_reason = 0;
uint8_t counter = 0;
uint16_t rx_bad = 0;
uint32_t last_control_ms = 0;
uint32_t last_print_ms = 0;

// Quadrature decoding in the pin interrupts. The count is 32-bit, so reading it from the loop is atomic.
volatile int32_t encoder_count = 0;
volatile uint8_t encoder_state = 0;

// Plain register reads: safe and fast inside an interrupt.
inline uint32_t IRAM_ATTR read_pin(uint8_t pin) {
  if (pin < 32) return (*reinterpret_cast<volatile uint32_t*>(GPIO_IN_REG) >> pin) & 1u;
  return (*reinterpret_cast<volatile uint32_t*>(GPIO_IN1_REG) >> (pin - 32)) & 1u;
}

void IRAM_ATTR on_encoder_edge() {
  // Index: previous AB state * 4 + new AB state. 00 -> 01 -> 11 -> 10 counts up; impossible jumps count 0.
  static const int8_t kStep[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
  const uint8_t state = static_cast<uint8_t>((read_pin(ENCODER_A_PIN) << 1) | read_pin(ENCODER_B_PIN));
  encoder_count = encoder_count + kStep[(encoder_state << 2) | state];
  encoder_state = state;
}

int32_t read_encoder() {
  const int32_t count = encoder_count;
  return kJoint.encoder_reversed ? -count : count;
}

// The I2C callbacks run in the Wire library's task, not in loop(): frames cross over under this lock.
portMUX_TYPE frame_lock = portMUX_INITIALIZER_UNLOCKED;
uint8_t pending_frame[arm::kI2cCommandSize];
bool frame_pending = false;
uint16_t wrong_length = 0;
uint8_t status_frame[arm::kI2cStatusSize];

void on_receive(int) {
  uint8_t frame[arm::kI2cCommandSize];
  size_t length = 0;
  while (Wire.available() > 0) {
    const int byte = Wire.read();
    if (length < sizeof(frame)) frame[length] = static_cast<uint8_t>(byte);
    length++;
  }
  if (length == 0) return;  // address probe
  portENTER_CRITICAL(&frame_lock);
  if (length == sizeof(frame)) {
    memcpy(pending_frame, frame, sizeof(frame));
    frame_pending = true;
  } else if (wrong_length < 0xFFFF) {
    wrong_length++;
  }
  portEXIT_CRITICAL(&frame_lock);
}

// On the ESP32 (no clock stretching) the bytes written here are what the master gets on its *next* read. The master
// reads every bus cycle, so the status it sees is at most one cycle old; the frame counter shows it is still fresh.
void on_request() {
  uint8_t frame[arm::kI2cStatusSize];
  portENTER_CRITICAL(&frame_lock);
  memcpy(frame, status_frame, sizeof(frame));
  portEXIT_CRITICAL(&frame_lock);
  Wire.write(frame, sizeof(frame));
}

void publish_status(uint32_t now_ms) {
  arm::SlaveStatus status;
  status.counter = counter++;
  status.state = controller.state();
  status.faults = controller.faults();
  status.joint = SLAVE_JOINT;
  status.pwm = controller.pwm();
  status.velocity_decirpm = arm::rpm_to_decirpm(controller.velocity_rpm());
  status.position = controller.position();
  const uint32_t age = controller.command_age_ms(now_ms);
  status.command_age_ms = age > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(age);
  status.max_rpm_x10 = static_cast<uint16_t>(kJoint.max_rpm * 10.0f + 0.5f);
  status.counts_per_rev_x100 = static_cast<uint32_t>(kJoint.counts_per_rev * 100.0f + 0.5f);
  status.version[0] = FIRMWARE_VERSION_MAJOR;
  status.version[1] = FIRMWARE_VERSION_MINOR;
  status.version[2] = FIRMWARE_VERSION_PATCH;
  status.reset_reason = reset_reason;
  uint8_t frame[arm::kI2cStatusSize];
  portENTER_CRITICAL(&frame_lock);
  const uint32_t bad = static_cast<uint32_t>(rx_bad) + wrong_length;
  portEXIT_CRITICAL(&frame_lock);
  status.rx_bad = bad > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(bad);
  arm::encode_slave_status(status, frame);
  portENTER_CRITICAL(&frame_lock);
  memcpy(status_frame, frame, sizeof(frame));
  portEXIT_CRITICAL(&frame_lock);
}

void drive_motor(int16_t pwm) {
  const bool forward = pwm >= 0;
  digitalWrite(MOTOR_DIR_PIN, forward ? kJoint.forward_level : !kJoint.forward_level);
  analogWrite(MOTOR_PWM_PIN, forward ? pwm : -pwm);
}

void update_led(uint32_t now_ms) {
  bool on;
  switch (controller.state()) {
    case arm::State::Enabled: on = true; break;
    case arm::State::Estop: on = (now_ms / 100) % 2 == 0; break;  // 5 Hz
    case arm::State::Fault: on = (now_ms / 50) % 2 == 0; break;   // 10 Hz
    default: on = (now_ms / 500) % 2 == 0; break;                 // 1 Hz
  }
  digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
}

void print_status() {
  if (Serial.availableForWrite() < 100) return;  // never block the control loop
  const uint8_t faults = controller.faults();
  Serial.printf("%s 0x%02X fw %d.%d.%d  %-12s pwm %4d  %7.1f rpm  pos %8ld  bad %u%s%s\n", kJoint.name,
                kJoint.i2c_address, FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_PATCH,
                kStateNames[static_cast<uint8_t>(controller.state())], controller.pwm(), controller.velocity_rpm(),
                static_cast<long>(controller.position()), rx_bad, (faults & arm::kSlaveFaultStall) ? "  STALL" : "",
                (faults & arm::kSlaveFaultDirection) ? "  WRONG DIRECTION" : "");
}

}  // namespace

void setup() {
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  drive_motor(0);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  encoder_state = static_cast<uint8_t>((read_pin(ENCODER_A_PIN) << 1) | read_pin(ENCODER_B_PIN));
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), on_encoder_edge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), on_encoder_edge, CHANGE);
  reset_reason = static_cast<uint8_t>(esp_reset_reason());
  Serial.begin(115200);

  publish_status(millis());
  Wire.onReceive(on_receive);
  Wire.onRequest(on_request);
  Wire.begin(kJoint.i2c_address, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY_HZ);
#if CONFIG_IDF_TARGET_ESP32
  Wire.slaveWrite(status_frame, sizeof(status_frame));  // the answer to the master's first read
#endif

  enableLoopWDT();
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  const esp_task_wdt_config_t watchdog = {LOOP_WATCHDOG_MS, 1u << 0, true};
  esp_task_wdt_reconfigure(&watchdog);
#endif
}

void loop() {
  const uint32_t now_ms = millis();  // one timestamp per pass

  uint8_t frame[arm::kI2cCommandSize];
  portENTER_CRITICAL(&frame_lock);
  const bool pending = frame_pending;
  if (pending) memcpy(frame, pending_frame, sizeof(frame));
  frame_pending = false;
  portEXIT_CRITICAL(&frame_lock);
  if (pending) {
    arm::SlaveCommand command;
    if (arm::parse_slave_command(frame, sizeof(frame), command)) {
      controller.on_command(command.estop, command.target_decirpm, now_ms);
      if (!controller.motor_enabled()) drive_motor(0);  // stops take effect immediately
    } else if (rx_bad < 0xFFFF) {
      rx_bad++;
    }
  }

  if (now_ms - last_control_ms >= CONTROL_PERIOD_MS) {
    last_control_ms = now_ms;
    controller.update(now_ms, read_encoder());
    drive_motor(controller.pwm());
    update_led(now_ms);
    publish_status(now_ms);
  }
  if (now_ms - last_print_ms >= 500) {
    last_print_ms = now_ms;
    print_status();
  }
  delay(1);  // yield to the idle task (keeps the task watchdog quiet)
}
