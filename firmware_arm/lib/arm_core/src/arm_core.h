// arm_core: protocols and safety state machines of the arm firmware, free of Arduino dependencies so they can be
// unit-tested on a PC (pio test -e native).
//
// PC <-> master (USB serial): the rover's framing, version | type | seq | payload | crc16-le, COBS-encoded and
//   terminated by 0x00, with arm packet types. A rover drive ESP32 and the arm master never accept each other's commands.
// master <-> slaves (I2C): one fixed-size frame each way per bus cycle, protected by the same CRC-16.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace arm {

constexpr uint8_t kProtocolVersion = 1;
constexpr uint8_t kTypeArmStatus = 0x05;   // master -> PC, 20 Hz
constexpr uint8_t kTypeArmInfo = 0x06;     // master -> PC, 1 Hz
constexpr uint8_t kTypeArmCommand = 0x83;  // PC -> master, 50 Hz
constexpr uint8_t kFlagEstop = 0x01;
constexpr uint8_t kStatusFlagSlavesReady = 0x01;  // ARM_STATUS flags: every slave linked, healthy and enabled
constexpr size_t kMaxPacket = 96;  // decoded bytes: header + payload + CRC
constexpr size_t kMaxWire = kMaxPacket + 2;  // COBS overhead + delimiter
constexpr int kMotors = 4;  // order everywhere: shoulder, elbow, wrist_a, wrist_b
constexpr size_t kCommandPayload = 9;
constexpr size_t kMotorStatusSize = 13;
constexpr size_t kStatusPayload = 16 + kMotors * kMotorStatusSize;
constexpr size_t kMotorInfoSize = 10;
constexpr size_t kInfoPayload = kMotors * kMotorInfoSize;

constexpr uint8_t kI2cTypeCommand = 0x10;  // master -> slave
constexpr uint8_t kI2cTypeStatus = 0x11;   // slave -> master
constexpr size_t kI2cCommandSize = 8;
constexpr size_t kI2cStatusSize = 30;  // must fit the ESP32's 32-byte I2C slave TX FIFO

// Faults a slave detects itself (SlaveStatus::faults).
constexpr uint8_t kSlaveFaultStall = 0x01;      // high PWM but the motor does not turn
constexpr uint8_t kSlaveFaultDirection = 0x02;  // the motor turns against its PWM: encoder or motor wired reversed
// Faults the master sees on a slave's link (ARM_STATUS link_faults).
constexpr uint8_t kLinkLost = 0x01;        // no fresh, valid status for longer than the slave timeout
constexpr uint8_t kLinkWrongJoint = 0x02;  // the slave at this address was built for another joint
constexpr uint8_t kLinkNotEnabled = 0x04;  // the master is enabled but this slave did not follow
constexpr uint8_t kLinkSlaveFault = 0x08;  // the slave reports a fault of its own

// Milliseconds from `then` to `now`, wraparound-safe; 0 if `then` is (slightly) after `now`.
uint32_t elapsed_ms(uint32_t now, uint32_t then);
// rpm -> 0.1 rpm, clamped to +-32767; NaN gives 0.
int16_t rpm_to_decirpm(float rpm);

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF; crc16("123456789") == 0x29B1.
uint16_t crc16(const uint8_t* data, size_t len);
// Returns the encoded length; `out` needs len + len / 254 + 1 bytes. The result never contains 0x00.
size_t cobs_encode(const uint8_t* in, size_t len, uint8_t* out);
// Returns the decoded length, or 0 if `in` is not valid COBS.
size_t cobs_decode(const uint8_t* in, size_t len, uint8_t* out);
// Builds the complete wire bytes (including the 0x00 delimiter) into `out` (kMaxWire bytes). Returns 0 if too long.
size_t encode_packet(uint8_t type, uint8_t seq, const uint8_t* payload, size_t payload_len, uint8_t* out);

struct Packet {
  uint8_t type;
  uint8_t seq;
  uint8_t payload[kMaxPacket];
  size_t length;
};

// Byte-at-a-time decoder. Drops anything that fails COBS, length, CRC or version checks and resynchronises on 0x00.
class PacketDecoder {
 public:
  // Returns true when `packet` now holds a complete, valid packet.
  bool feed(uint8_t byte, Packet& packet);
  uint16_t good() const { return good_; }
  uint16_t bad() const { return bad_; }

 private:
  uint8_t buffer_[kMaxWire];
  size_t length_ = 0;
  bool overflow_ = false;
  bool first_ = true;  // the first block after power-up may be the tail of a packet joined mid-stream
  uint16_t good_ = 0;
  uint16_t bad_ = 0;
};

struct ArmCommand {
  bool estop;
  int16_t target_decirpm[kMotors];  // gearmotor output targets in 0.1 rpm
};
bool parse_arm_command(const Packet& packet, ArmCommand& command);

enum class State : uint8_t {
  Boot = 0,         // nothing received since power-up; motors off
  NoCommand = 1,    // no valid command for longer than the timeout; motors off
  Estop = 2,        // e-stop flag received; motors off
  WaitNeutral = 3,  // commands arriving but not all zero yet after a stop; motors off
  Enabled = 4,      // following targets
  Fault = 5,        // master: a slave was lost or faulted while enabled; slave: stall / wrong direction.
                    // Motors off, latched until an e-stop
  WaitSlaves = 6,   // master only: released with neutral commands, but not every slave answers healthy yet
};
constexpr uint8_t kLastState = 6;

// ---------------------------------------------------------------------------------------------------- I2C frames
struct SlaveCommand {
  uint8_t seq;
  bool estop;
  int16_t target_decirpm;
};
// Writes kI2cCommandSize bytes.
size_t encode_slave_command(const SlaveCommand& command, uint8_t* out);
bool parse_slave_command(const uint8_t* in, size_t len, SlaveCommand& command);

struct SlaveStatus {
  uint8_t counter;  // advances every slave control period: a frozen counter means a hung slave
  State state;
  uint8_t faults;  // kSlaveFault* bits
  uint8_t joint;   // the joint this slave was built for (0..3)
  int16_t pwm;     // applied duty -255..255
  int16_t velocity_decirpm;  // measured gearmotor output speed
  int32_t position;  // encoder counts since the slave powered up
  uint16_t rx_bad;   // I2C command frames rejected
  uint16_t command_age_ms;
  uint16_t max_rpm_x10;
  uint32_t counts_per_rev_x100;
  uint8_t version[3];
  uint8_t reset_reason;
};
// Writes kI2cStatusSize bytes.
size_t encode_slave_status(const SlaveStatus& status, uint8_t* out);
bool parse_slave_status(const uint8_t* in, size_t len, SlaveStatus& status);

// ---------------------------------------------------------------------------------------------------- master
struct MasterConfig {
  uint32_t command_timeout_ms;  // the PC silent this long -> NoCommand
  uint32_t slave_timeout_ms;    // no fresh status from a slave this long -> that slave is lost
  uint32_t enable_grace_ms;     // time the slaves get to follow the master into Enabled
};

// The master's safety state machine. Same rules as the rover's drive firmware (neutral interlock after boot, timeout
// and e-stop; e-stop latched until a neutral release), plus the slaves: the master enables only when every slave
// answers healthy (until then it waits in WaitSlaves), and any lost, misaddressed or faulted slave while enabled stops
// all four motors and latches Fault. With a bevel-gear wrist, one wrist motor stopping while the other runs would move
// the wrist the wrong way, so nothing moves unless everything is healthy. Fault clears only with an e-stop followed by
// a neutral release.
class ArmMaster {
 public:
  explicit ArmMaster(const MasterConfig& config) : config_(config) {}

  void on_command(const ArmCommand& command, uint32_t now_ms);
  // A valid status frame from the slave of `motor`.
  void on_slave_status(int motor, const SlaveStatus& status, uint32_t now_ms);
  // No answer, or a frame that failed its checks.
  void on_slave_error(int motor);
  // Call every pass: timeouts and link checks.
  void update(uint32_t now_ms);

  // What to send to the slave of `motor` this bus cycle (seq is left 0 for the caller to number).
  SlaveCommand slave_command(int motor) const;
  State state() const { return state_; }
  bool motors_enabled() const { return state_ == State::Enabled; }
  bool slaves_ready() const;
  uint8_t link_faults(int motor) const { return static_cast<uint8_t>(link_faults_[motor] | latched_link_[motor]); }
  uint8_t slave_faults(int motor) const;
  bool has_status(int motor) const { return have_status_[motor]; }
  const SlaveStatus& status(int motor) const { return status_[motor]; }
  uint16_t errors(int motor) const { return errors_[motor]; }
  uint32_t command_age_ms(uint32_t now_ms) const;

 private:
  void evaluate_links(uint32_t now_ms);
  bool any_link_fault() const;
  void stop(State reason);
  void latch();

  MasterConfig config_;
  State state_ = State::Boot;
  int16_t target_decirpm_[kMotors] = {0, 0, 0, 0};
  bool have_command_ = false;
  uint32_t last_command_ms_ = 0;
  uint32_t enabled_since_ms_ = 0;
  bool have_status_[kMotors] = {false, false, false, false};
  SlaveStatus status_[kMotors] = {};
  uint32_t last_fresh_ms_[kMotors] = {0, 0, 0, 0};
  uint8_t link_faults_[kMotors] = {0, 0, 0, 0};
  uint8_t latched_link_[kMotors] = {0, 0, 0, 0};
  uint8_t latched_slave_[kMotors] = {0, 0, 0, 0};
  uint16_t errors_[kMotors] = {0, 0, 0, 0};
};

// ---------------------------------------------------------------------------------------------------- slave
struct JointConfig {
  float max_rpm;                // gearmotor output rpm at PWM 255 (feed-forward and scaling)
  float counts_per_rev;         // encoder counts per gearmotor output revolution (quadrature, x4)
  uint32_t command_timeout_ms;  // stop if no valid frame from the master for this long
  float slew_rpm_per_s;         // max setpoint change per second while enabled (0 = unlimited); stops are immediate
  bool encoder_fitted;          // false: open loop, no encoder checks, speed reported as 0
  bool closed_loop;             // PI speed correction on top of the feed-forward
  float kp;                     // PWM fraction per unit of (speed error / max_rpm)
  float ki;                     // PWM fraction per second per unit of (speed error / max_rpm)
  float integral_limit;         // cap on the integral term, as a PWM fraction
  float stall_pwm_fraction;     // stall: |PWM| at least this fraction of full ...
  float stall_rpm_fraction;     // ... while the speed stays under this fraction of max_rpm ...
  uint32_t stall_ms;            // ... for longer than this (0 disables the check)
  float reverse_pwm_fraction;   // wrong direction: |PWM| at least this fraction of full ...
  float reverse_rpm_fraction;   // ... while the motor turns the other way faster than this fraction of max_rpm ...
  uint32_t reverse_ms;          // ... for longer than this (0 disables the check)
};

// One joint motor: the same neutral interlock, timeout and e-stop latch as the master, a setpoint ramp, speed from
// the encoder, feed-forward + PI speed control, and stall / wrong-direction detection (latched Fault until an e-stop).
class JointController {
 public:
  explicit JointController(const JointConfig& config) : config_(config) {}

  void on_command(bool estop, int16_t target_decirpm, uint32_t now_ms);
  // Call every control period with the current encoder count.
  void update(uint32_t now_ms, int32_t encoder_count);

  State state() const { return state_; }
  uint8_t faults() const { return faults_; }
  int16_t pwm() const { return pwm_; }
  float velocity_rpm() const { return velocity_rpm_; }
  int32_t position() const { return position_; }
  uint32_t command_age_ms(uint32_t now_ms) const;
  bool motor_enabled() const { return state_ == State::Enabled; }

 private:
  static constexpr int kWindow = 5;  // encoder samples in the speed estimate (40 ms at a 10 ms control period)

  void stop(State reason);
  void measure(uint32_t now_ms, int32_t count);
  void check_faults(uint32_t now_ms);

  JointConfig config_;
  State state_ = State::Boot;
  uint8_t faults_ = 0;
  int16_t target_decirpm_ = 0;
  float setpoint_rpm_ = 0;
  float integral_ = 0;
  int16_t pwm_ = 0;
  float velocity_rpm_ = 0;
  int32_t position_ = 0;
  uint32_t sample_ms_[kWindow] = {};
  int32_t sample_count_[kWindow] = {};
  int samples_ = 0;
  int next_sample_ = 0;
  bool stalling_ = false;
  uint32_t stall_since_ms_ = 0;
  bool reversing_ = false;
  uint32_t reverse_since_ms_ = 0;
  bool have_command_ = false;
  uint32_t last_command_ms_ = 0;
  bool have_update_ = false;
  uint32_t last_update_ms_ = 0;
};

// ---------------------------------------------------------------------------------------------------- PC packets
struct MotorReport {
  State slave_state;
  uint8_t slave_faults;
  uint8_t link_faults;
  int16_t pwm;
  int16_t velocity_decirpm;
  int32_t position;
  uint16_t i2c_errors;
};

struct StatusInfo {
  State state;
  uint8_t flags;
  uint16_t command_age_ms;
  uint16_t rx_good;
  uint16_t rx_bad;
  uint32_t uptime_ms;
  uint8_t reset_reason;
  uint8_t version[3];
  MotorReport motors[kMotors];
};
// Writes the ARM_STATUS payload (kStatusPayload bytes).
size_t encode_status(const StatusInfo& status, uint8_t* out);

struct MotorInfo {
  uint16_t max_rpm_x10;
  uint32_t counts_per_rev_x100;
  uint8_t version[3];
  uint8_t reset_reason;
};
// Writes the ARM_INFO payload (kInfoPayload bytes).
size_t encode_info(const MotorInfo* motors, uint8_t* out);

}  // namespace arm
