// Rover drive firmware: one ESP32 driving four wheel motors from DRIVE_COMMAND packets sent by the onboard PC.
// Ethernet edition (branch ethernet-udp): the packets arrive in UDP datagrams through a W5500 (src/net_link, LINK.md).
//
// Safety behaviour (implemented and unit-tested in lib/drive_core):
//  - motors are off after power-up until a neutral (all-zero) command arrives;
//  - no valid command for COMMAND_TIMEOUT_MS -> motors off, and a neutral command is needed again;
//  - e-stop flag -> motors off and latched until commands without the flag arrive, starting with a neutral one;
//  - garbled, partial or corrupted input is discarded (COBS + CRC framing), never interpreted as a speed, and only the
//    onboard PC's address may send commands (lib/udp_frames);
//  - the network never blocks the loop: with the cable out, the command timeout stops the motors as for any lost link;
//  - once commands have been heard, LINK_RESTART_SILENCE_MS without one restarts the ESP32, so a W5500 that dropped
//    out is initialised again (lib/udp_frames/src/link_watchdog.h). It boots with the motors off, as always.
#include <Arduino.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

#include "config.h"
#include "drive_core.h"
#include "link_watchdog.h"
#include "net_link.h"
#include "udp_frames.h"
#if DRIVE_PROFILE
#include "profiler.h"  // profiling build only (pio run -e profile)
#endif

namespace {

const drive::ControllerConfig kControllerConfig = {MAX_WHEEL_RPM, COMMAND_TIMEOUT_MS, SLEW_PWM_PER_S};

const uint8_t kPcIp[4] = {PC_IP_OCTETS};

drive::DriveController controller(kControllerConfig);
udp::DatagramReceiver receiver(kPcIp);
udp::LinkWatchdog link_watchdog(LINK_RESTART_SILENCE_MS);
uint8_t tx_seq = 0;
uint32_t last_control_ms = 0;
uint32_t last_status_ms = 0;
uint8_t reset_reason = 0;

void setup_motor(const MotorPins& motor) {
  pinMode(motor.dir_pin, OUTPUT);
  pinMode(motor.pwm_pin, OUTPUT);
  digitalWrite(motor.dir_pin, motor.forward_level);
  analogWrite(motor.pwm_pin, 0);
}

void drive_motor(const MotorPins& motor, int16_t pwm) {
  const bool forward = pwm >= 0;
  digitalWrite(motor.dir_pin, forward ? motor.forward_level : !motor.forward_level);
  analogWrite(motor.pwm_pin, forward ? pwm : -pwm);
}

void apply_outputs() {
  for (int wheel = 0; wheel < drive::kWheels; wheel++) drive_motor(kDriveMotors[wheel], controller.pwm(wheel));
#if DRIVE_MIDDLE_WHEELS
  drive_motor(kMiddleMotors[0], (controller.pwm(0) + controller.pwm(2)) / 2);
  drive_motor(kMiddleMotors[1], (controller.pwm(1) + controller.pwm(3)) / 2);
#endif
}

void update_led(uint32_t now_ms) {
  bool on;
  switch (controller.state()) {
    case drive::State::Driving: on = true; break;
    case drive::State::Estop: on = (now_ms / 100) % 2 == 0; break;  // 5 Hz
    default: on = (now_ms / 500) % 2 == 0; break;                   // 1 Hz: waiting for commands / neutral
  }
  digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
}

void send_status(uint32_t now_ms) {
  drive::StatusInfo status;
  status.state = controller.state();
  const uint32_t age = controller.command_age_ms(now_ms);
  status.command_age_ms = age > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(age);
  for (int wheel = 0; wheel < drive::kWheels; wheel++) status.pwm[wheel] = controller.pwm(wheel);
  status.rx_good = receiver.good();
  status.rx_bad = receiver.bad();
  status.uptime_ms = now_ms;
  status.reset_reason = reset_reason;
  status.max_wheel_rpm_x10 = static_cast<uint16_t>(MAX_WHEEL_RPM * 10.0f + 0.5f);
  status.version[0] = FIRMWARE_VERSION_MAJOR;
  status.version[1] = FIRMWARE_VERSION_MINOR;
  status.version[2] = FIRMWARE_VERSION_PATCH;

  uint8_t payload[drive::kStatusPayload];
  drive::encode_status(status, payload);
  uint8_t wire[drive::kMaxWire];
  const size_t length = drive::encode_packet(drive::kTypeDriveStatus, tx_seq++, payload, sizeof(payload), wire);
  net::send(wire, length);  // one packet per datagram; never blocks the control loop
}

#if DRIVE_PROFILE
void send_profile() {
  uint8_t payload[profiler::kReportPayload];
  const size_t payload_length = profiler::build_report(payload);
  uint8_t wire[drive::kMaxWire];
  const size_t length = drive::encode_packet(profiler::kTypeProfile, tx_seq++, payload, payload_length, wire);
  net::send(wire, length);
}
#endif

}  // namespace

void setup() {
  for (const MotorPins& motor : kDriveMotors) setup_motor(motor);
  for (const MotorPins& motor : kMiddleMotors) setup_motor(motor);
  pinMode(STATUS_LED_PIN, OUTPUT);
  reset_reason = static_cast<uint8_t>(esp_reset_reason());
  Serial.begin(SERIAL_BAUD);  // bring-up text only
  net::begin();  // the Ethernet link comes up 1-3 s later; the motors stay off until a neutral command anyway
  enableLoopWDT();  // a hung loop resets the ESP32, which boots with motors off
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  // Arduino's task watchdog fires after 5 s: a hung loop would keep the last PWM that long, because the 300 ms command
  // timeout runs in that same loop. One pass takes about 1 ms, so 1 s never fires in normal running. Only the timeout
  // changes; the idle task watched by default (CPU 0) stays watched. (Core 2.x keeps its 5 s default.)
  const esp_task_wdt_config_t watchdog = {LOOP_WATCHDOG_MS, 1u << 0, true};
  esp_task_wdt_reconfigure(&watchdog);
#endif
#if DRIVE_PROFILE
  profiler::begin();
#endif
}

void loop() {
#if DRIVE_PROFILE
  profiler::loop_begin();
#endif
  drive::DriveCommand command;
  // One timestamp per pass. Stamping commands with a later millis() than the control update below made a command
  // look 1 ms in the future, which the timeout check read as ~49 days old: a spurious stop while driving.
  const uint32_t now_ms = millis();

  net::service(now_ms);
  uint8_t datagram[udp::kMaxDatagram];
  uint8_t source_ip[4];
  for (int i = 0; i < 8; i++) {  // a few per pass: the PC sends one every 20 ms
    const size_t length = net::receive(datagram, sizeof(datagram), source_ip);
    if (length == 0) break;
    receiver.feed(source_ip, datagram, length, [&](const drive::Packet& packet) {
      if (!drive::parse_drive_command(packet, command)) return;
      link_watchdog.on_command(now_ms);
      controller.on_command(command, now_ms);
      if (!controller.motors_enabled()) apply_outputs();  // stops take effect immediately
    });
  }

  if (now_ms - last_control_ms >= CONTROL_PERIOD_MS) {
    last_control_ms = now_ms;
    controller.update(now_ms);
    apply_outputs();
    update_led(now_ms);
  }
  if (now_ms - last_status_ms >= STATUS_PERIOD_MS) {
    last_status_ms = now_ms;
    send_status(now_ms);
  }
  if (link_watchdog.restart_due(now_ms, !controller.motors_enabled())) {
    // Commands were arriving and stopped: if the W5500 lost its setup, only initialising it again brings the link
    // back, and a restart does that from scratch (as the EN button does). The motors are already off.
    Serial.println("commands stopped: restarting to initialise the W5500 again");
    Serial.flush();
    ESP.restart();
  }
#if DRIVE_PROFILE
  if (profiler::report_due()) send_profile();  // counted in this pass's busy time, so the measurement errs high
  profiler::loop_end();
#endif
  delay(1);  // yield to the idle task (keeps the task watchdog quiet)
}
