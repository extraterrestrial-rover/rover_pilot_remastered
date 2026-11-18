// Arm master firmware: USB serial to the PC, I2C bus master for the four motor slaves. It drives no motor itself.
//
// Each bus cycle (BUS_PERIOD_MS) it writes one command frame to every slave and reads back one status frame.
// Safety behaviour (implemented and unit-tested in lib/arm_core):
//  - the same rules as the rover's drive ESP32: motors off after power-up until a neutral command; no valid command
//    for COMMAND_TIMEOUT_MS -> stop; e-stop flag latched until a neutral release; corrupted input is discarded;
//  - slaves get the e-stop flag whenever the master is not ENABLED, and zero targets until all four follow it;
//  - a slave that stops answering, reports a fault or answers for the wrong joint while enabled stops all four motors
//    and latches FAULT, cleared by an e-stop and a neutral release;
//  - each slave also stops its own motor when the master goes quiet for SLAVE_COMMAND_TIMEOUT_MS.
// Nothing but protocol packets is written to the serial line.
#include <Arduino.h>
#include <Wire.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

#include "arm_core.h"
#include "config.h"

namespace {

const arm::MasterConfig kMasterConfig = {COMMAND_TIMEOUT_MS, SLAVE_TIMEOUT_MS, ENABLE_GRACE_MS};

arm::ArmMaster master(kMasterConfig);
arm::PacketDecoder decoder;
uint8_t tx_seq = 0;
uint8_t bus_seq = 0;
bool bus_now = false;
uint32_t last_bus_ms = 0;
uint32_t last_led_ms = 0;
uint32_t last_status_ms = 0;
uint32_t last_info_ms = 0;
uint8_t reset_reason = 0;

void read_serial(uint32_t now_ms) {
  arm::Packet packet;
  arm::ArmCommand command;
  while (Serial.available() > 0) {
    if (decoder.feed(static_cast<uint8_t>(Serial.read()), packet) && arm::parse_arm_command(packet, command)) {
      const bool was_enabled = master.motors_enabled();
      master.on_command(command, now_ms);
      if (was_enabled && !master.motors_enabled()) bus_now = true;  // pass a stop on to the slaves at once
    }
  }
}

void exchange(int motor, uint32_t now_ms) {
  const uint8_t address = kJoints[motor].i2c_address;
  arm::SlaveCommand command = master.slave_command(motor);
  command.seq = bus_seq;
  uint8_t frame[arm::kI2cCommandSize];
  arm::encode_slave_command(command, frame);
  Wire.beginTransmission(address);
  Wire.write(frame, sizeof(frame));
  if (Wire.endTransmission(true) != 0) {
    master.on_slave_error(motor);
    return;
  }
  uint8_t reply[arm::kI2cStatusSize];
  const size_t received = Wire.requestFrom(address, static_cast<uint8_t>(sizeof(reply)));
  size_t length = 0;
  while (Wire.available() > 0) {
    const int byte = Wire.read();
    if (length < sizeof(reply)) reply[length] = static_cast<uint8_t>(byte);
    length++;
  }
  arm::SlaveStatus status;
  if (received == sizeof(reply) && length == sizeof(reply) && arm::parse_slave_status(reply, length, status)) {
    master.on_slave_status(motor, status, now_ms);
  } else {
    master.on_slave_error(motor);
  }
}

void bus_cycle(uint32_t now_ms) {
  for (int motor = 0; motor < arm::kMotors; motor++) {
    read_serial(now_ms);  // an e-stop arriving mid-cycle reaches the remaining slaves in this same cycle
    exchange(motor, now_ms);
  }
  bus_seq++;
}

void update_led(uint32_t now_ms) {
  bool on;
  switch (master.state()) {
    case arm::State::Enabled: on = true; break;
    case arm::State::Estop: on = (now_ms / 100) % 2 == 0; break;  // 5 Hz
    case arm::State::Fault: on = (now_ms / 50) % 2 == 0; break;   // 10 Hz
    default: on = (now_ms / 500) % 2 == 0; break;                 // 1 Hz: waiting for commands / neutral
  }
  digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
}

void send(uint8_t type, const uint8_t* payload, size_t length) {
  uint8_t wire[arm::kMaxWire];
  const size_t wire_length = arm::encode_packet(type, tx_seq++, payload, length, wire);
  if (Serial.availableForWrite() >= static_cast<int>(wire_length)) Serial.write(wire, wire_length);  // never block
}

void send_status(uint32_t now_ms) {
  arm::StatusInfo status;
  status.state = master.state();
  status.flags = master.slaves_ready() ? arm::kStatusFlagSlavesReady : 0;
  const uint32_t age = master.command_age_ms(now_ms);
  status.command_age_ms = age > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(age);
  status.rx_good = decoder.good();
  status.rx_bad = decoder.bad();
  status.uptime_ms = now_ms;
  status.reset_reason = reset_reason;
  status.version[0] = FIRMWARE_VERSION_MAJOR;
  status.version[1] = FIRMWARE_VERSION_MINOR;
  status.version[2] = FIRMWARE_VERSION_PATCH;
  for (int motor = 0; motor < arm::kMotors; motor++) {
    arm::MotorReport& report = status.motors[motor];
    const bool known = master.has_status(motor);
    const arm::SlaveStatus& slave = master.status(motor);
    report.slave_state = known ? slave.state : arm::State::Boot;
    report.slave_faults = master.slave_faults(motor);
    report.link_faults = master.link_faults(motor);
    report.pwm = known ? slave.pwm : 0;
    report.velocity_decirpm = known ? slave.velocity_decirpm : 0;
    report.position = known ? slave.position : 0;
    report.i2c_errors = master.errors(motor);
  }
  uint8_t payload[arm::kStatusPayload];
  arm::encode_status(status, payload);
  send(arm::kTypeArmStatus, payload, sizeof(payload));
}

void send_info() {
  arm::MotorInfo motors[arm::kMotors] = {};
  for (int motor = 0; motor < arm::kMotors; motor++) {
    if (!master.has_status(motor)) continue;  // all zero: not heard from yet
    const arm::SlaveStatus& slave = master.status(motor);
    motors[motor] = {slave.max_rpm_x10, slave.counts_per_rev_x100, {slave.version[0], slave.version[1], slave.version[2]},
                     slave.reset_reason};
  }
  uint8_t payload[arm::kInfoPayload];
  arm::encode_info(motors, payload);
  send(arm::kTypeArmInfo, payload, sizeof(payload));
}

}  // namespace

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  reset_reason = static_cast<uint8_t>(esp_reset_reason());
  Serial.begin(SERIAL_BAUD);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  enableLoopWDT();  // a hung loop resets the ESP32; the slaves stop on their own timeout meanwhile
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  // Arduino's task watchdog fires after 5 s; 1 s is far above one pass (a bus cycle is ~15 ms). Core 2.x keeps 5 s.
  const esp_task_wdt_config_t watchdog = {LOOP_WATCHDOG_MS, 1u << 0, true};
  esp_task_wdt_reconfigure(&watchdog);
#endif
}

void loop() {
  // One timestamp per pass (see the rover firmware 2.0.2: a command stamped after the update looked ~49 days old).
  const uint32_t now_ms = millis();
  read_serial(now_ms);
  const bool was_enabled = master.motors_enabled();
  master.update(now_ms);
  if (was_enabled && !master.motors_enabled()) bus_now = true;  // timeout or fault: stop the slaves now

  if (bus_now || now_ms - last_bus_ms >= BUS_PERIOD_MS) {
    bus_now = false;
    last_bus_ms = now_ms;
    bus_cycle(now_ms);
  }
  if (now_ms - last_led_ms >= CONTROL_PERIOD_MS) {
    last_led_ms = now_ms;
    update_led(now_ms);
  }
  if (now_ms - last_status_ms >= STATUS_PERIOD_MS) {
    last_status_ms = now_ms;
    send_status(now_ms);
  }
  if (now_ms - last_info_ms >= INFO_PERIOD_MS) {
    last_info_ms = now_ms;
    send_info();
  }
  delay(1);  // yield to the idle task (keeps the task watchdog quiet)
}
