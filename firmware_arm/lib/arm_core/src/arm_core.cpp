#include "arm_core.h"

#include <math.h>
#include <string.h>

namespace arm {

namespace {

void put_u16(uint8_t* out, uint16_t value) {
  out[0] = static_cast<uint8_t>(value & 0xFF);
  out[1] = static_cast<uint8_t>(value >> 8);
}

void put_u32(uint8_t* out, uint32_t value) {
  put_u16(out, static_cast<uint16_t>(value & 0xFFFF));
  put_u16(out + 2, static_cast<uint16_t>(value >> 16));
}

uint16_t get_u16(const uint8_t* in) {
  return static_cast<uint16_t>(static_cast<uint16_t>(in[0]) | (static_cast<uint16_t>(in[1]) << 8));
}

uint32_t get_u32(const uint8_t* in) {
  return static_cast<uint32_t>(get_u16(in)) | (static_cast<uint32_t>(get_u16(in + 2)) << 16);
}

int16_t get_i16(const uint8_t* in) { return static_cast<int16_t>(get_u16(in)); }

int32_t get_i32(const uint8_t* in) { return static_cast<int32_t>(get_u32(in)); }

float clampf(float value, float limit) {
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

}  // namespace

int16_t rpm_to_decirpm(float rpm) {
  const float raw = rpm * 10.0f;
  if (raw != raw) return 0;  // NaN
  if (raw >= 32767.0f) return 32767;
  if (raw <= -32767.0f) return -32767;
  return static_cast<int16_t>(lroundf(raw));
}

uint32_t elapsed_ms(uint32_t now, uint32_t then) {
  const int32_t delta = static_cast<int32_t>(now - then);
  return delta > 0 ? static_cast<uint32_t>(delta) : 0;
}

uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

size_t cobs_encode(const uint8_t* in, size_t len, uint8_t* out) {
  size_t read = 0, write = 1, code_index = 0;
  uint8_t code = 1;
  while (read < len) {
    if (in[read] == 0) {
      out[code_index] = code;
      code = 1;
      code_index = write++;
      read++;
    } else {
      out[write++] = in[read++];
      if (++code == 0xFF) {
        out[code_index] = code;
        code = 1;
        code_index = write++;
      }
    }
  }
  out[code_index] = code;
  return write;
}

size_t cobs_decode(const uint8_t* in, size_t len, uint8_t* out) {
  size_t read = 0, write = 0;
  while (read < len) {
    uint8_t code = in[read++];
    if (code == 0 || read + code - 1 > len) return 0;
    for (uint8_t i = 1; i < code; i++) {
      if (in[read] == 0) return 0;
      out[write++] = in[read++];
    }
    if (code != 0xFF && read < len) out[write++] = 0;
  }
  return write;
}

size_t encode_packet(uint8_t type, uint8_t seq, const uint8_t* payload, size_t payload_len, uint8_t* out) {
  if (payload_len + 5 > kMaxPacket) return 0;
  uint8_t body[kMaxPacket];
  body[0] = kProtocolVersion;
  body[1] = type;
  body[2] = seq;
  memcpy(body + 3, payload, payload_len);
  const size_t body_len = 3 + payload_len;
  put_u16(body + body_len, crc16(body, body_len));
  const size_t encoded = cobs_encode(body, body_len + 2, out);
  out[encoded] = 0x00;
  return encoded + 1;
}

bool PacketDecoder::feed(uint8_t byte, Packet& packet) {
  if (byte != 0) {
    if (length_ < sizeof(buffer_)) {
      buffer_[length_++] = byte;
    } else {
      overflow_ = true;
    }
    return false;
  }
  const size_t length = length_;
  const bool overflow = overflow_;
  const bool first = first_;
  length_ = 0;
  overflow_ = false;
  first_ = false;
  if (length == 0) return false;

  uint8_t decoded[kMaxWire];
  const size_t decoded_len = overflow ? 0 : cobs_decode(buffer_, length, decoded);
  bool valid = decoded_len >= 5 && decoded_len <= kMaxPacket;
  if (valid) {
    valid = crc16(decoded, decoded_len - 2) == get_u16(decoded + decoded_len - 2) && decoded[0] == kProtocolVersion;
  }
  if (!valid) {
    if (!first && bad_ < 0xFFFF) bad_++;
    return false;
  }
  if (good_ < 0xFFFF) good_++;
  packet.type = decoded[1];
  packet.seq = decoded[2];
  packet.length = decoded_len - 5;
  memcpy(packet.payload, decoded + 3, packet.length);
  return true;
}

bool parse_arm_command(const Packet& packet, ArmCommand& command) {
  if (packet.type != kTypeArmCommand || packet.length != kCommandPayload) return false;
  if (packet.payload[0] & ~kFlagEstop) return false;  // reserved flag bits must be zero
  command.estop = (packet.payload[0] & kFlagEstop) != 0;
  for (int motor = 0; motor < kMotors; motor++) command.target_decirpm[motor] = get_i16(packet.payload + 1 + 2 * motor);
  return true;
}

// ---------------------------------------------------------------------------------------------------- I2C frames

size_t encode_slave_command(const SlaveCommand& command, uint8_t* out) {
  out[0] = kProtocolVersion;
  out[1] = kI2cTypeCommand;
  out[2] = command.seq;
  out[3] = command.estop ? kFlagEstop : 0;
  put_u16(out + 4, static_cast<uint16_t>(command.target_decirpm));
  put_u16(out + 6, crc16(out, 6));
  return kI2cCommandSize;
}

bool parse_slave_command(const uint8_t* in, size_t len, SlaveCommand& command) {
  if (len != kI2cCommandSize || crc16(in, 6) != get_u16(in + 6)) return false;
  if (in[0] != kProtocolVersion || in[1] != kI2cTypeCommand || (in[3] & ~kFlagEstop)) return false;
  command.seq = in[2];
  command.estop = (in[3] & kFlagEstop) != 0;
  command.target_decirpm = get_i16(in + 4);
  return true;
}

size_t encode_slave_status(const SlaveStatus& status, uint8_t* out) {
  out[0] = kProtocolVersion;
  out[1] = kI2cTypeStatus;
  out[2] = status.counter;
  out[3] = static_cast<uint8_t>(status.state);
  out[4] = status.faults;
  out[5] = status.joint;
  put_u16(out + 6, static_cast<uint16_t>(status.pwm));
  put_u16(out + 8, static_cast<uint16_t>(status.velocity_decirpm));
  put_u32(out + 10, static_cast<uint32_t>(status.position));
  put_u16(out + 14, status.rx_bad);
  put_u16(out + 16, status.command_age_ms);
  put_u16(out + 18, status.max_rpm_x10);
  put_u32(out + 20, status.counts_per_rev_x100);
  out[24] = status.version[0];
  out[25] = status.version[1];
  out[26] = status.version[2];
  out[27] = status.reset_reason;
  put_u16(out + 28, crc16(out, 28));
  return kI2cStatusSize;
}

bool parse_slave_status(const uint8_t* in, size_t len, SlaveStatus& status) {
  if (len != kI2cStatusSize || crc16(in, 28) != get_u16(in + 28)) return false;
  if (in[0] != kProtocolVersion || in[1] != kI2cTypeStatus || in[3] > kLastState || in[5] >= kMotors) return false;
  status.counter = in[2];
  status.state = static_cast<State>(in[3]);
  status.faults = in[4];
  status.joint = in[5];
  status.pwm = get_i16(in + 6);
  status.velocity_decirpm = get_i16(in + 8);
  status.position = get_i32(in + 10);
  status.rx_bad = get_u16(in + 14);
  status.command_age_ms = get_u16(in + 16);
  status.max_rpm_x10 = get_u16(in + 18);
  status.counts_per_rev_x100 = get_u32(in + 20);
  status.version[0] = in[24];
  status.version[1] = in[25];
  status.version[2] = in[26];
  status.reset_reason = in[27];
  return true;
}

// ---------------------------------------------------------------------------------------------------- master

void ArmMaster::stop(State reason) {
  state_ = reason;
  for (int motor = 0; motor < kMotors; motor++) target_decirpm_[motor] = 0;
}

void ArmMaster::latch() {
  for (int motor = 0; motor < kMotors; motor++) {
    latched_link_[motor] = static_cast<uint8_t>(latched_link_[motor] | link_faults_[motor]);
    if (have_status_[motor]) latched_slave_[motor] = static_cast<uint8_t>(latched_slave_[motor] | status_[motor].faults);
  }
}

void ArmMaster::evaluate_links(uint32_t now_ms) {
  const bool settled = state_ == State::Enabled && elapsed_ms(now_ms, enabled_since_ms_) > config_.enable_grace_ms;
  for (int motor = 0; motor < kMotors; motor++) {
    uint8_t faults = 0;
    if (!have_status_[motor] || elapsed_ms(now_ms, last_fresh_ms_[motor]) > config_.slave_timeout_ms) faults |= kLinkLost;
    if (have_status_[motor]) {
      const SlaveStatus& status = status_[motor];
      if (status.joint != motor) faults |= kLinkWrongJoint;
      if (status.faults != 0 || status.state == State::Fault) {
        faults |= kLinkSlaveFault;  // reported on its own: the slave's fault is why it does not follow
      } else if (settled && status.state != State::Enabled) {
        faults |= kLinkNotEnabled;
      }
    }
    link_faults_[motor] = faults;
  }
}

bool ArmMaster::any_link_fault() const {
  for (int motor = 0; motor < kMotors; motor++) {
    if (link_faults_[motor] != 0) return true;
  }
  return false;
}

bool ArmMaster::slaves_ready() const {
  for (int motor = 0; motor < kMotors; motor++) {
    if (!have_status_[motor] || link_faults_[motor] != 0 || status_[motor].state != State::Enabled) return false;
  }
  return true;
}

uint8_t ArmMaster::slave_faults(int motor) const {
  const uint8_t live = have_status_[motor] ? status_[motor].faults : 0;
  return static_cast<uint8_t>(live | latched_slave_[motor]);
}

void ArmMaster::on_command(const ArmCommand& command, uint32_t now_ms) {
  have_command_ = true;
  last_command_ms_ = now_ms;
  if (command.estop) {
    stop(State::Estop);
    for (int motor = 0; motor < kMotors; motor++) latched_link_[motor] = latched_slave_[motor] = 0;
    return;
  }
  if (state_ == State::Fault) return;  // latched until an e-stop
  bool neutral = true;
  for (int motor = 0; motor < kMotors; motor++) neutral = neutral && command.target_decirpm[motor] == 0;
  if (state_ != State::Enabled) {
    // After power-up, a timeout or an e-stop, motion resumes only after a neutral command, and only with all four
    // slaves healthy. Until they are, the master waits (the link faults in ARM_STATUS say which slave is missing).
    if (!neutral) {
      stop(State::WaitNeutral);
      return;
    }
    evaluate_links(now_ms);
    if (any_link_fault()) {
      stop(State::WaitSlaves);
      return;
    }
    state_ = State::Enabled;
    enabled_since_ms_ = now_ms;
  }
  for (int motor = 0; motor < kMotors; motor++) target_decirpm_[motor] = command.target_decirpm[motor];
}

void ArmMaster::on_slave_status(int motor, const SlaveStatus& status, uint32_t now_ms) {
  if (!have_status_[motor] || status.counter != status_[motor].counter) last_fresh_ms_[motor] = now_ms;
  status_[motor] = status;
  have_status_[motor] = true;
}

void ArmMaster::on_slave_error(int motor) {
  if (errors_[motor] < 0xFFFF) errors_[motor]++;
}

void ArmMaster::update(uint32_t now_ms) {
  if ((state_ == State::Enabled || state_ == State::WaitNeutral || state_ == State::WaitSlaves) &&
      elapsed_ms(now_ms, last_command_ms_) > config_.command_timeout_ms) {
    stop(State::NoCommand);
  }
  evaluate_links(now_ms);
  if (state_ == State::Enabled && any_link_fault()) stop(State::Fault);
  if (state_ == State::Fault) latch();
}

SlaveCommand ArmMaster::slave_command(int motor) const {
  SlaveCommand command = {0, true, 0};  // slaves are enabled only while the master is
  if (state_ == State::Enabled) {
    command.estop = false;
    // Hold every motor until all four slaves follow, so the bevel-gear pair always starts together.
    command.target_decirpm = slaves_ready() ? target_decirpm_[motor] : 0;
  }
  return command;
}

uint32_t ArmMaster::command_age_ms(uint32_t now_ms) const {
  return have_command_ ? elapsed_ms(now_ms, last_command_ms_) : 0xFFFFFFFFu;
}

// ---------------------------------------------------------------------------------------------------- slave

void JointController::stop(State reason) {
  state_ = reason;
  target_decirpm_ = 0;
  setpoint_rpm_ = 0;
  integral_ = 0;
  pwm_ = 0;
  stalling_ = reversing_ = false;
}

void JointController::on_command(bool estop, int16_t target_decirpm, uint32_t now_ms) {
  have_command_ = true;
  last_command_ms_ = now_ms;
  if (estop) {
    faults_ = 0;
    stop(State::Estop);
    return;
  }
  if (state_ == State::Fault) return;  // latched until an e-stop
  if (state_ != State::Enabled) {
    if (target_decirpm != 0) {
      stop(State::WaitNeutral);
      return;
    }
    state_ = State::Enabled;
  }
  target_decirpm_ = target_decirpm;
}

void JointController::measure(uint32_t now_ms, int32_t count) {
  position_ = count;
  sample_ms_[next_sample_] = now_ms;
  sample_count_[next_sample_] = count;
  next_sample_ = (next_sample_ + 1) % kWindow;
  if (samples_ < kWindow) samples_++;
  velocity_rpm_ = 0;
  if (!config_.encoder_fitted || samples_ < 2 || config_.counts_per_rev <= 0) return;
  const int oldest = samples_ < kWindow ? 0 : next_sample_;
  const uint32_t span_ms = elapsed_ms(now_ms, sample_ms_[oldest]);
  if (span_ms == 0) return;
  const int32_t counts = static_cast<int32_t>(static_cast<uint32_t>(count) - static_cast<uint32_t>(sample_count_[oldest]));
  velocity_rpm_ = static_cast<float>(counts) * 60000.0f / (config_.counts_per_rev * static_cast<float>(span_ms));
}

void JointController::check_faults(uint32_t now_ms) {
  if (!config_.encoder_fitted || config_.max_rpm <= 0) return;
  const float pwm_fraction = fabsf(static_cast<float>(pwm_)) / 255.0f;
  const float speed_fraction = fabsf(velocity_rpm_) / config_.max_rpm;

  const bool stalling = config_.stall_ms > 0 && pwm_fraction >= config_.stall_pwm_fraction &&
                        speed_fraction < config_.stall_rpm_fraction;
  if (!stalling) {
    stalling_ = false;
  } else if (!stalling_) {
    stalling_ = true;
    stall_since_ms_ = now_ms;
  } else if (elapsed_ms(now_ms, stall_since_ms_) > config_.stall_ms) {
    faults_ |= kSlaveFaultStall;
    stop(State::Fault);
    return;
  }

  const bool reversing = config_.reverse_ms > 0 && pwm_fraction >= config_.reverse_pwm_fraction &&
                         speed_fraction >= config_.reverse_rpm_fraction && (pwm_ > 0) != (velocity_rpm_ > 0);
  if (!reversing) {
    reversing_ = false;
  } else if (!reversing_) {
    reversing_ = true;
    reverse_since_ms_ = now_ms;
  } else if (elapsed_ms(now_ms, reverse_since_ms_) > config_.reverse_ms) {
    faults_ |= kSlaveFaultDirection;
    stop(State::Fault);
  }
}

void JointController::update(uint32_t now_ms, int32_t encoder_count) {
  const uint32_t dt_ms = have_update_ ? elapsed_ms(now_ms, last_update_ms_) : 0;
  last_update_ms_ = now_ms;
  have_update_ = true;
  measure(now_ms, encoder_count);

  if ((state_ == State::Enabled || state_ == State::WaitNeutral) &&
      elapsed_ms(now_ms, last_command_ms_) > config_.command_timeout_ms) {
    stop(State::NoCommand);
  }
  if (state_ != State::Enabled || config_.max_rpm <= 0) return;

  const float target = target_decirpm_ / 10.0f;
  const float max_step = config_.slew_rpm_per_s > 0 ? config_.slew_rpm_per_s * dt_ms / 1000.0f : 1e9f;
  if (target > setpoint_rpm_) setpoint_rpm_ = (target - setpoint_rpm_ > max_step) ? setpoint_rpm_ + max_step : target;
  else if (target < setpoint_rpm_) setpoint_rpm_ = (setpoint_rpm_ - target > max_step) ? setpoint_rpm_ - max_step : target;

  if (target == 0 && setpoint_rpm_ == 0) {
    // At rest: no output at all (the worm gearboxes hold the pitch joints), and nothing left to wind up.
    pwm_ = 0;
    integral_ = 0;
    stalling_ = reversing_ = false;
    return;
  }

  float output = setpoint_rpm_ / config_.max_rpm;  // feed-forward: the open-loop calibration
  if (config_.encoder_fitted && config_.closed_loop) {
    const float error = (setpoint_rpm_ - velocity_rpm_) / config_.max_rpm;
    const float unclamped = output + config_.kp * error + integral_;
    // Conditional integration: never wind further into saturation.
    if (!((unclamped >= 1.0f && error > 0) || (unclamped <= -1.0f && error < 0))) {
      integral_ = clampf(integral_ + config_.ki * error * dt_ms / 1000.0f, config_.integral_limit);
    }
    output += config_.kp * error + integral_;
  }
  pwm_ = static_cast<int16_t>(lroundf(clampf(output, 1.0f) * 255.0f));
  check_faults(now_ms);
}

uint32_t JointController::command_age_ms(uint32_t now_ms) const {
  return have_command_ ? elapsed_ms(now_ms, last_command_ms_) : 0xFFFFFFFFu;
}

// ---------------------------------------------------------------------------------------------------- PC packets

size_t encode_status(const StatusInfo& status, uint8_t* out) {
  out[0] = static_cast<uint8_t>(status.state);
  out[1] = status.flags;
  put_u16(out + 2, status.command_age_ms);
  put_u16(out + 4, status.rx_good);
  put_u16(out + 6, status.rx_bad);
  put_u32(out + 8, status.uptime_ms);
  out[12] = status.reset_reason;
  out[13] = status.version[0];
  out[14] = status.version[1];
  out[15] = status.version[2];
  for (int motor = 0; motor < kMotors; motor++) {
    const MotorReport& report = status.motors[motor];
    uint8_t* at = out + 16 + motor * kMotorStatusSize;
    at[0] = static_cast<uint8_t>(report.slave_state);
    at[1] = report.slave_faults;
    at[2] = report.link_faults;
    put_u16(at + 3, static_cast<uint16_t>(report.pwm));
    put_u16(at + 5, static_cast<uint16_t>(report.velocity_decirpm));
    put_u32(at + 7, static_cast<uint32_t>(report.position));
    put_u16(at + 11, report.i2c_errors);
  }
  return kStatusPayload;
}

size_t encode_info(const MotorInfo* motors, uint8_t* out) {
  for (int motor = 0; motor < kMotors; motor++) {
    uint8_t* at = out + motor * kMotorInfoSize;
    put_u16(at, motors[motor].max_rpm_x10);
    put_u32(at + 2, motors[motor].counts_per_rev_x100);
    at[6] = motors[motor].version[0];
    at[7] = motors[motor].version[1];
    at[8] = motors[motor].version[2];
    at[9] = motors[motor].reset_reason;
  }
  return kInfoPayload;
}

}  // namespace arm
