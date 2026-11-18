// Native unit tests for arm_core: pio test -e native
#include <string.h>
#include <unity.h>

#include "arm_core.h"
#include "vectors.h"

using arm::ArmCommand;
using arm::ArmMaster;
using arm::JointController;
using arm::Packet;
using arm::PacketDecoder;
using arm::SlaveCommand;
using arm::SlaveStatus;
using arm::State;

static const arm::MasterConfig kMasterConfig = {300, 100, 200};
// Open loop, no ramp, no encoder checks unless a test turns them on.
static const arm::JointConfig kOpenLoop = {117.0f, 1425.1f, 100, 0.0f, true, false, 0.5f, 2.0f, 0.3f,
                                           0.7f, 0.05f, 0, 0.3f, 0.2f, 0};

static ArmCommand command(bool estop, int16_t a, int16_t b, int16_t c, int16_t d) {
  ArmCommand result = {estop, {a, b, c, d}};
  return result;
}

static SlaveStatus slave(int joint, State state, uint8_t counter, uint8_t faults = 0) {
  SlaveStatus status = {};
  status.counter = counter;
  status.state = state;
  status.faults = faults;
  status.joint = static_cast<uint8_t>(joint);
  return status;
}

// One bus cycle: every slave answers with `state` and a fresh counter.
static void answer_all(ArmMaster& master, State state, uint8_t counter, uint32_t now_ms) {
  for (int motor = 0; motor < arm::kMotors; motor++) master.on_slave_status(motor, slave(motor, state, counter), now_ms);
}

// Brings a master to Enabled with all four slaves following it; returns the time reached.
static uint32_t enable(ArmMaster& master, uint32_t now_ms) {
  answer_all(master, State::Estop, 1, now_ms);
  master.on_command(command(true, 0, 0, 0, 0), now_ms);
  master.on_command(command(false, 0, 0, 0, 0), now_ms + 10);
  master.update(now_ms + 10);
  answer_all(master, State::Enabled, 2, now_ms + 20);
  master.update(now_ms + 20);
  return now_ms + 20;
}

static bool decode_all(PacketDecoder& decoder, const uint8_t* bytes, size_t len, Packet& packet) {
  bool got = false;
  for (size_t i = 0; i < len; i++) got = decoder.feed(bytes[i], packet) || got;
  return got;
}

void setUp() {}
void tearDown() {}

// ------------------------------------------------------------------------------------------------ protocol

void test_crc_check_value() {
  TEST_ASSERT_EQUAL_HEX16(kCrcCheck, arm::crc16(reinterpret_cast<const uint8_t*>("123456789"), 9));
  TEST_ASSERT_EQUAL_HEX16(0x29B1, kCrcCheck);
}

void test_cobs_round_trip_with_zero_runs() {
  uint8_t data[300], encoded[310], decoded[310];
  for (size_t len = 1; len < sizeof(data); len++) {
    for (size_t i = 0; i < len; i++) data[i] = (i % 7 == 0) ? 0 : static_cast<uint8_t>(i * 13);
    const size_t encoded_len = arm::cobs_encode(data, len, encoded);
    for (size_t i = 0; i < encoded_len; i++) TEST_ASSERT_NOT_EQUAL(0, encoded[i]);
    TEST_ASSERT_EQUAL(len, arm::cobs_decode(encoded, encoded_len, decoded));
    TEST_ASSERT_EQUAL_MEMORY(data, decoded, len);
  }
}

void test_encodes_and_decodes_commands_exactly_like_the_python_reference() {
  PacketDecoder decoder;
  Packet packet;
  decoder.feed(0, packet);  // leading delimiter: in sync
  for (const CommandVector& vector : kCommandVectors) {
    uint8_t payload[arm::kCommandPayload];
    payload[0] = vector.estop ? arm::kFlagEstop : 0;
    for (int motor = 0; motor < arm::kMotors; motor++) {
      const int16_t raw = arm::rpm_to_decirpm(vector.targets[motor]);
      payload[1 + 2 * motor] = static_cast<uint8_t>(raw & 0xFF);
      payload[2 + 2 * motor] = static_cast<uint8_t>((static_cast<uint16_t>(raw) >> 8) & 0xFF);
    }
    uint8_t wire[arm::kMaxWire];
    const size_t wire_len = arm::encode_packet(arm::kTypeArmCommand, vector.seq, payload, sizeof(payload), wire);
    TEST_ASSERT_EQUAL_MESSAGE(vector.wire_len, wire_len, vector.name);
    TEST_ASSERT_EQUAL_MEMORY_MESSAGE(vector.wire, wire, wire_len, vector.name);

    TEST_ASSERT_TRUE_MESSAGE(decode_all(decoder, vector.wire, vector.wire_len, packet), vector.name);
    ArmCommand parsed;
    TEST_ASSERT_TRUE(arm::parse_arm_command(packet, parsed));
    TEST_ASSERT_EQUAL(vector.estop, parsed.estop);
    TEST_ASSERT_EQUAL(vector.seq, packet.seq);
    for (int motor = 0; motor < arm::kMotors; motor++) TEST_ASSERT_FLOAT_WITHIN(0.051f, vector.targets[motor], parsed.target_decirpm[motor] / 10.0f);
  }
  TEST_ASSERT_EQUAL(0, decoder.bad());
}

void test_status_and_info_encoding_match_python() {
  arm::StatusInfo status = {State::Fault, 0, 20, 1234, 2, 600000, 1, {1, 0, 0},
                            {{State::Enabled, 0, 0, 128, 585, 100000, 0},
                             {State::Enabled, 0, 0, -255, -1170, -5, 3},
                             {State::Fault, arm::kSlaveFaultStall, arm::kLinkSlaveFault, 0, 0, -2000000, 0},
                             {State::Boot, 0, arm::kLinkLost, 0, 0, 0, 65535}}};
  uint8_t payload[arm::kStatusPayload];
  TEST_ASSERT_EQUAL(arm::kStatusPayload, arm::encode_status(status, payload));
  uint8_t wire[arm::kMaxWire];
  size_t wire_len = arm::encode_packet(arm::kTypeArmStatus, kStatusSeq, payload, sizeof(payload), wire);
  TEST_ASSERT_EQUAL(sizeof(kStatusWire), wire_len);
  TEST_ASSERT_EQUAL_MEMORY(kStatusWire, wire, wire_len);

  const arm::MotorInfo info[arm::kMotors] = {{1170, 142510, {1, 0, 0}, 1}, {1170, 142510, {1, 0, 0}, 1},
                                             {3120, 53770, {1, 0, 0}, 3}, {0, 0, {0, 0, 0}, 0}};
  uint8_t info_payload[arm::kInfoPayload];
  TEST_ASSERT_EQUAL(arm::kInfoPayload, arm::encode_info(info, info_payload));
  wire_len = arm::encode_packet(arm::kTypeArmInfo, kInfoSeq, info_payload, sizeof(info_payload), wire);
  TEST_ASSERT_EQUAL(sizeof(kInfoWire), wire_len);
  TEST_ASSERT_EQUAL_MEMORY(kInfoWire, wire, wire_len);
}

void test_i2c_frames_match_python_and_round_trip() {
  for (const SlaveCommandVector& vector : kSlaveCommandVectors) {
    const SlaveCommand cmd = {vector.seq, vector.estop, arm::rpm_to_decirpm(vector.target)};
    uint8_t frame[arm::kI2cCommandSize];
    TEST_ASSERT_EQUAL(arm::kI2cCommandSize, arm::encode_slave_command(cmd, frame));
    TEST_ASSERT_EQUAL_MEMORY_MESSAGE(vector.frame, frame, sizeof(frame), vector.name);
    SlaveCommand parsed;
    TEST_ASSERT_TRUE(arm::parse_slave_command(vector.frame, arm::kI2cCommandSize, parsed));
    TEST_ASSERT_EQUAL(vector.seq, parsed.seq);
    TEST_ASSERT_EQUAL(vector.estop, parsed.estop);
    TEST_ASSERT_EQUAL(cmd.target_decirpm, parsed.target_decirpm);
  }
  SlaveStatus status = {42, State::Enabled, 0, 2, -128, -585, -123456, 7, 15, 1170, 142510, {1, 0, 0}, 1};
  uint8_t frame[arm::kI2cStatusSize];
  TEST_ASSERT_EQUAL(arm::kI2cStatusSize, arm::encode_slave_status(status, frame));
  TEST_ASSERT_EQUAL_MEMORY(kSlaveStatusFrame, frame, sizeof(frame));
  SlaveStatus parsed;
  TEST_ASSERT_TRUE(arm::parse_slave_status(frame, sizeof(frame), parsed));
  TEST_ASSERT_EQUAL(-123456, parsed.position);
  TEST_ASSERT_EQUAL(2, parsed.joint);
  TEST_ASSERT_EQUAL(142510u, parsed.counts_per_rev_x100);
}

void test_bad_i2c_frames_are_rejected() {
  uint8_t frame[arm::kI2cStatusSize];
  memcpy(frame, kSlaveStatusFrame, sizeof(frame));
  SlaveStatus status;
  frame[12] ^= 0x04;  // one bit of the position
  TEST_ASSERT_FALSE(arm::parse_slave_status(frame, sizeof(frame), status));
  memset(frame, 0xFF, sizeof(frame));  // an empty bus reads as all ones
  TEST_ASSERT_FALSE(arm::parse_slave_status(frame, sizeof(frame), status));
  memset(frame, 0x00, sizeof(frame));
  TEST_ASSERT_FALSE(arm::parse_slave_status(frame, sizeof(frame), status));
  TEST_ASSERT_FALSE(arm::parse_slave_status(kSlaveStatusFrame, sizeof(kSlaveStatusFrame) - 1, status));

  SlaveCommand cmd;
  uint8_t command_frame[arm::kI2cCommandSize];
  memcpy(command_frame, kSlaveFrame_reverse, sizeof(command_frame));
  command_frame[4] ^= 0x01;
  TEST_ASSERT_FALSE(arm::parse_slave_command(command_frame, sizeof(command_frame), cmd));
  // Reserved flag bits set (with a valid CRC): rejected.
  const SlaveCommand reserved = {1, false, 0};
  arm::encode_slave_command(reserved, command_frame);
  command_frame[3] = 0x02;
  const uint16_t crc = arm::crc16(command_frame, 6);
  command_frame[6] = static_cast<uint8_t>(crc & 0xFF);
  command_frame[7] = static_cast<uint8_t>(crc >> 8);
  TEST_ASSERT_FALSE(arm::parse_slave_command(command_frame, sizeof(command_frame), cmd));
}

void test_garbage_and_corruption_never_produce_a_command() {
  PacketDecoder decoder;
  Packet packet;
  const uint8_t garbage[] = {0x13, 0x37, 0x00, 0x01, 0x02, 0x00, 0xFF, 0xFF, 0xFF, 0x00};
  TEST_ASSERT_FALSE(decode_all(decoder, garbage, sizeof(garbage), packet));
  uint8_t corrupted[sizeof(kWire_shoulder_full)];
  memcpy(corrupted, kWire_shoulder_full, sizeof(corrupted));
  corrupted[6] ^= 0x10;
  TEST_ASSERT_FALSE(decode_all(decoder, corrupted, sizeof(corrupted), packet));
  TEST_ASSERT_FALSE(decode_all(decoder, kWire_shoulder_full, 5, packet));  // half a packet, then a full one
  decoder.feed(0, packet);
  TEST_ASSERT_TRUE(decode_all(decoder, kWire_neutral, sizeof(kWire_neutral), packet));
  TEST_ASSERT_TRUE(decoder.bad() >= 3);
  uint8_t long_block[200];
  memset(long_block, 0x41, sizeof(long_block));
  TEST_ASSERT_FALSE(decode_all(decoder, long_block, sizeof(long_block), packet));
  TEST_ASSERT_FALSE(decoder.feed(0, packet));
  // A rover DRIVE_COMMAND (type 0x82) is never an arm command.
  Packet rover = {0x82, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0}, arm::kCommandPayload};
  ArmCommand cmd;
  TEST_ASSERT_FALSE(arm::parse_arm_command(rover, cmd));
  Packet reserved = {arm::kTypeArmCommand, 0, {0x02, 0, 0, 0, 0, 0, 0, 0, 0}, arm::kCommandPayload};
  TEST_ASSERT_FALSE(arm::parse_arm_command(reserved, cmd));
}

// ------------------------------------------------------------------------------------------------ master

void test_master_boots_stopped_and_sends_estop_to_the_slaves() {
  ArmMaster master(kMasterConfig);
  master.update(0);
  TEST_ASSERT_EQUAL(State::Boot, master.state());
  for (int motor = 0; motor < arm::kMotors; motor++) TEST_ASSERT_TRUE(master.slave_command(motor).estop);
  master.on_command(command(false, 500, 0, 0, 0), 10);
  TEST_ASSERT_EQUAL(State::WaitNeutral, master.state());
  TEST_ASSERT_TRUE(master.slave_command(0).estop);
}

void test_release_waits_for_every_slave_without_latching() {
  // Neutral commands can arrive before the slaves have answered (master just booted or reset while the operator is
  // already released). That must not latch a fault: the master waits until all four answer healthy.
  ArmMaster master(kMasterConfig);
  master.on_command(command(false, 0, 0, 0, 0), 10);
  TEST_ASSERT_EQUAL(State::WaitSlaves, master.state());
  for (int motor = 0; motor < arm::kMotors; motor++) {
    TEST_ASSERT_EQUAL_HEX8(arm::kLinkLost, master.link_faults(motor));
    TEST_ASSERT_TRUE(master.slave_command(motor).estop);
  }
  for (int motor = 0; motor < 3; motor++) master.on_slave_status(motor, slave(motor, State::Boot, 1), 20);
  master.update(20);
  master.on_command(command(false, 0, 0, 0, 0), 20);
  TEST_ASSERT_EQUAL(State::WaitSlaves, master.state());  // wrist_b still missing
  master.on_slave_status(3, slave(3, State::Boot, 1), 40);
  master.update(40);
  master.on_command(command(false, 0, 0, 0, 0), 40);
  TEST_ASSERT_EQUAL(State::Enabled, master.state());
  // A stick pushed while waiting goes through the neutral interlock as usual, and silence times out.
  ArmMaster other(kMasterConfig);
  other.on_command(command(false, 0, 0, 0, 0), 0);
  other.on_command(command(false, 100, 0, 0, 0), 10);
  TEST_ASSERT_EQUAL(State::WaitNeutral, other.state());
  other.on_command(command(false, 0, 0, 0, 0), 20);
  other.update(400);
  TEST_ASSERT_EQUAL(State::NoCommand, other.state());
}

void test_targets_reach_the_slaves_only_once_all_four_follow() {
  ArmMaster master(kMasterConfig);
  answer_all(master, State::Estop, 1, 0);
  master.on_command(command(false, 0, 0, 0, 0), 0);
  master.on_command(command(false, 500, -500, 300, 300), 10);
  master.update(10);
  TEST_ASSERT_EQUAL(State::Enabled, master.state());
  for (int motor = 0; motor < arm::kMotors; motor++) {
    TEST_ASSERT_FALSE(master.slave_command(motor).estop);
    TEST_ASSERT_EQUAL(0, master.slave_command(motor).target_decirpm);  // slaves still in Estop: hold
  }
  for (int motor = 0; motor < 3; motor++) master.on_slave_status(motor, slave(motor, State::Enabled, 2), 20);
  master.update(20);
  TEST_ASSERT_EQUAL(0, master.slave_command(0).target_decirpm);  // wrist_b not yet
  master.on_slave_status(3, slave(3, State::Enabled, 2), 30);
  master.update(30);
  TEST_ASSERT_TRUE(master.slaves_ready());
  TEST_ASSERT_EQUAL(500, master.slave_command(0).target_decirpm);
  TEST_ASSERT_EQUAL(-500, master.slave_command(1).target_decirpm);
  TEST_ASSERT_EQUAL(300, master.slave_command(3).target_decirpm);
}

void test_a_lost_slave_stops_everything_and_latches_fault() {
  ArmMaster master(kMasterConfig);
  uint32_t now = enable(master, 0);
  master.on_command(command(false, 500, 500, 500, 500), now);
  // Three slaves keep answering; wrist_b goes silent.
  for (uint8_t cycle = 3; cycle < 10; cycle++) {
    now += 20;
    for (int motor = 0; motor < 3; motor++) master.on_slave_status(motor, slave(motor, State::Enabled, cycle), now);
    master.on_slave_error(3);
    master.on_command(command(false, 500, 500, 500, 500), now);
    master.update(now);
  }
  TEST_ASSERT_EQUAL(State::Fault, master.state());
  TEST_ASSERT_EQUAL_HEX8(arm::kLinkLost, master.link_faults(3));
  TEST_ASSERT_EQUAL_HEX8(0, master.link_faults(0));
  TEST_ASSERT_TRUE(master.errors(3) >= 7);
  for (int motor = 0; motor < arm::kMotors; motor++) TEST_ASSERT_TRUE(master.slave_command(motor).estop);
  // wrist_b answers again: still latched (the latched bit stays visible) until e-stop + neutral release.
  answer_all(master, State::Estop, 20, now + 20);
  master.update(now + 20);
  TEST_ASSERT_EQUAL(State::Fault, master.state());
  TEST_ASSERT_EQUAL_HEX8(arm::kLinkLost, master.link_faults(3));
  master.on_command(command(false, 0, 0, 0, 0), now + 30);
  TEST_ASSERT_EQUAL(State::Fault, master.state());
  master.on_command(command(true, 0, 0, 0, 0), now + 40);
  master.on_command(command(false, 0, 0, 0, 0), now + 50);
  TEST_ASSERT_EQUAL(State::Enabled, master.state());
}

void test_a_frozen_slave_counter_counts_as_lost() {
  ArmMaster master(kMasterConfig);
  uint32_t now = enable(master, 0);
  for (int pass = 0; pass < 8; pass++) {  // valid frames, but the slave's loop has hung: the counter never moves
    now += 20;
    for (int motor = 0; motor < arm::kMotors; motor++) {
      master.on_slave_status(motor, slave(motor, State::Enabled, motor == 1 ? 2 : static_cast<uint8_t>(3 + pass)), now);
    }
    master.on_command(command(false, 0, 0, 0, 0), now);
    master.update(now);
  }
  TEST_ASSERT_EQUAL(State::Fault, master.state());
  TEST_ASSERT_EQUAL_HEX8(arm::kLinkLost, master.link_faults(1));
}

void test_a_slave_fault_stops_everything_and_stays_reported() {
  ArmMaster master(kMasterConfig);
  uint32_t now = enable(master, 0);
  master.on_slave_status(2, slave(2, State::Fault, 3, arm::kSlaveFaultStall), now + 20);
  master.update(now + 20);
  TEST_ASSERT_EQUAL(State::Fault, master.state());
  TEST_ASSERT_EQUAL_HEX8(arm::kLinkSlaveFault, master.link_faults(2));  // not also "did not follow"
  // The master's e-stop to the slaves clears the slave's own latch; the master keeps the reason until its own e-stop.
  master.on_slave_status(2, slave(2, State::Estop, 4), now + 40);
  master.update(now + 40);
  TEST_ASSERT_EQUAL_HEX8(arm::kSlaveFaultStall, master.slave_faults(2));
  master.on_command(command(true, 0, 0, 0, 0), now + 50);
  TEST_ASSERT_EQUAL_HEX8(0, master.slave_faults(2));
}

void test_a_slave_built_for_another_joint_is_refused() {
  ArmMaster master(kMasterConfig);
  for (int motor = 0; motor < arm::kMotors; motor++) master.on_slave_status(motor, slave(motor == 3 ? 2 : motor, State::Estop, 1), 0);
  master.on_command(command(false, 0, 0, 0, 0), 10);
  TEST_ASSERT_EQUAL(State::WaitSlaves, master.state());
  TEST_ASSERT_EQUAL_HEX8(arm::kLinkWrongJoint, master.link_faults(3));
  for (int motor = 0; motor < arm::kMotors; motor++) TEST_ASSERT_TRUE(master.slave_command(motor).estop);
}

void test_a_slave_that_does_not_follow_into_enabled_faults_after_the_grace_time() {
  ArmMaster master(kMasterConfig);
  answer_all(master, State::Estop, 1, 0);
  master.on_command(command(false, 0, 0, 0, 0), 0);
  uint32_t now = 0;
  for (uint8_t cycle = 2; cycle < 14; cycle++) {  // shoulder rebooted into WaitNeutral and never enables
    now += 20;
    for (int motor = 0; motor < arm::kMotors; motor++) {
      master.on_slave_status(motor, slave(motor, motor == 0 ? State::WaitNeutral : State::Enabled, cycle), now);
    }
    master.on_command(command(false, 0, 0, 0, 0), now);
    master.update(now);
    if (now <= kMasterConfig.enable_grace_ms) TEST_ASSERT_EQUAL(State::Enabled, master.state());
  }
  TEST_ASSERT_EQUAL(State::Fault, master.state());
  TEST_ASSERT_EQUAL_HEX8(arm::kLinkNotEnabled, master.link_faults(0));
}

void test_master_command_timeout_requires_neutral_again() {
  ArmMaster master(kMasterConfig);
  uint32_t now = enable(master, 0);
  master.on_command(command(false, 600, 0, 0, 0), now);
  for (uint8_t cycle = 3; cycle < 20; cycle++) {  // slaves fine, the PC goes quiet
    now += 20;
    answer_all(master, State::Enabled, cycle, now);
    master.update(now);
  }
  TEST_ASSERT_EQUAL(State::NoCommand, master.state());
  TEST_ASSERT_TRUE(master.slave_command(0).estop);
  master.on_command(command(false, 600, 0, 0, 0), now);
  TEST_ASSERT_EQUAL(State::WaitNeutral, master.state());
  master.on_command(command(false, 0, 0, 0, 0), now + 10);
  TEST_ASSERT_EQUAL(State::Enabled, master.state());
}

void test_master_estop_does_not_time_out_into_motion() {
  ArmMaster master(kMasterConfig);
  enable(master, 0);
  master.on_command(command(true, 0, 0, 0, 0), 30);
  master.update(5000);
  TEST_ASSERT_EQUAL(State::Estop, master.state());
  master.on_command(command(false, 100, 0, 0, 0), 5010);  // released with a stick pushed
  TEST_ASSERT_EQUAL(State::WaitNeutral, master.state());
}

// ------------------------------------------------------------------------------------------------ joint

void test_joint_requires_neutral_and_latches_estop() {
  JointController joint(kOpenLoop);
  joint.update(0, 0);
  TEST_ASSERT_EQUAL(State::Boot, joint.state());
  joint.on_command(false, 585, 10);
  joint.update(10, 0);
  TEST_ASSERT_EQUAL(State::WaitNeutral, joint.state());
  TEST_ASSERT_EQUAL(0, joint.pwm());
  joint.on_command(false, 0, 20);
  joint.on_command(false, 585, 30);
  joint.update(30, 0);
  TEST_ASSERT_EQUAL(State::Enabled, joint.state());
  TEST_ASSERT_INT_WITHIN(1, 128, joint.pwm());  // open loop: 58.5 / 117 rpm = half PWM
  joint.on_command(true, 585, 40);
  TEST_ASSERT_EQUAL(State::Estop, joint.state());
  TEST_ASSERT_EQUAL(0, joint.pwm());  // immediately, before the next update
  joint.on_command(false, 585, 50);
  TEST_ASSERT_EQUAL(State::WaitNeutral, joint.state());
}

void test_joint_stops_when_the_master_goes_quiet() {
  JointController joint(kOpenLoop);
  joint.on_command(false, 0, 0);
  joint.on_command(false, -1170, 10);
  joint.update(10, 0);
  TEST_ASSERT_EQUAL(-255, joint.pwm());
  joint.update(110, 0);
  TEST_ASSERT_EQUAL(State::Enabled, joint.state());  // exactly at the limit
  joint.update(111, 0);
  TEST_ASSERT_EQUAL(State::NoCommand, joint.state());
  TEST_ASSERT_EQUAL(0, joint.pwm());
}

void test_joint_ramps_the_setpoint_but_stops_at_once() {
  arm::JointConfig config = kOpenLoop;
  config.slew_rpm_per_s = 468.0f;  // 0 -> 117 rpm in 250 ms
  JointController joint(config);
  joint.on_command(false, 0, 0);
  joint.update(0, 0);
  joint.on_command(false, 1170, 0);
  joint.update(50, 0);
  TEST_ASSERT_INT_WITHIN(1, 51, joint.pwm());
  joint.on_command(false, 1170, 100);
  joint.update(100, 0);
  TEST_ASSERT_INT_WITHIN(1, 102, joint.pwm());
  joint.on_command(true, 0, 110);
  TEST_ASSERT_EQUAL(0, joint.pwm());
}

void test_joint_measures_speed_and_position_from_the_encoder() {
  JointController joint(kOpenLoop);
  // 1425.1 counts per rev at 60 rpm = 1425.1 counts/s = 14.251 counts per 10 ms.
  for (int step = 0; step <= 10; step++) joint.update(static_cast<uint32_t>(step * 10), static_cast<int32_t>(step * 14.251f));
  TEST_ASSERT_FLOAT_WITHIN(1.5f, 60.0f, joint.velocity_rpm());
  TEST_ASSERT_EQUAL(142, joint.position());
  for (int step = 11; step <= 20; step++) joint.update(static_cast<uint32_t>(step * 10), static_cast<int32_t>(142 - (step - 10) * 14.251f));
  TEST_ASSERT_FLOAT_WITHIN(1.5f, -60.0f, joint.velocity_rpm());
}

// A gearmotor that reaches only 80 % of the open-loop speed (load, low battery): first-order, 30 ms time constant.
struct Motor {
  float rpm = 0;
  float counts = 0;
  void step(int16_t pwm, float gain, uint32_t dt_ms) {
    const float steady = pwm / 255.0f * 117.0f * gain;
    rpm += (steady - rpm) * (dt_ms / 30.0f);
    counts += rpm / 60.0f * 1425.1f * dt_ms / 1000.0f;
  }
};

void test_speed_control_corrects_a_slow_motor() {
  arm::JointConfig config = kOpenLoop;
  config.closed_loop = true;
  JointController joint(config);
  Motor motor;
  joint.on_command(false, 0, 0);
  uint32_t now = 0;
  for (; now < 2000; now += 10) {
    joint.on_command(false, 585, now);  // 58.5 rpm
    joint.update(now, static_cast<int32_t>(motor.counts));
    motor.step(joint.pwm(), 0.8f, 10);
  }
  TEST_ASSERT_FLOAT_WITHIN(2.0f, 58.5f, motor.rpm);  // open loop would settle at 46.8 rpm
  TEST_ASSERT_TRUE(joint.pwm() > 140);
  joint.on_command(false, 0, now);
  joint.update(now, static_cast<int32_t>(motor.counts));
  TEST_ASSERT_EQUAL(0, joint.pwm());  // at rest: no output, integrator cleared
}

void test_stall_is_detected_and_latched_until_an_estop() {
  arm::JointConfig config = kOpenLoop;
  config.closed_loop = true;
  config.stall_ms = 1000;
  JointController joint(config);
  joint.on_command(false, 0, 0);
  uint32_t now = 0;
  for (; now < 3000 && joint.state() == State::Enabled; now += 10) {
    joint.on_command(false, 1000, now);  // pushed against a hard stop: the encoder never moves
    joint.update(now, 0);
  }
  TEST_ASSERT_EQUAL(State::Fault, joint.state());
  TEST_ASSERT_EQUAL_HEX8(arm::kSlaveFaultStall, joint.faults());
  TEST_ASSERT_TRUE(now > 1000 && now < 1200);
  TEST_ASSERT_EQUAL(0, joint.pwm());
  joint.on_command(false, 0, now);
  TEST_ASSERT_EQUAL(State::Fault, joint.state());  // a neutral command does not clear it
  joint.on_command(true, 0, now + 10);
  TEST_ASSERT_EQUAL(State::Estop, joint.state());
  TEST_ASSERT_EQUAL_HEX8(0, joint.faults());
}

void test_a_reversed_encoder_is_detected_before_it_runs_away() {
  arm::JointConfig config = kOpenLoop;
  config.closed_loop = true;
  config.reverse_ms = 300;
  JointController joint(config);
  Motor motor;
  joint.on_command(false, 0, 0);
  uint32_t now = 0;
  for (; now < 2000 && joint.state() == State::Enabled; now += 10) {
    joint.on_command(false, 585, now);
    joint.update(now, -static_cast<int32_t>(motor.counts));  // encoder wired backwards
    motor.step(joint.pwm(), 1.0f, 10);
  }
  TEST_ASSERT_EQUAL(State::Fault, joint.state());
  TEST_ASSERT_EQUAL_HEX8(arm::kSlaveFaultDirection, joint.faults());
  TEST_ASSERT_TRUE(now < 600);
}

void test_millis_wraparound_is_handled() {
  ArmMaster master(kMasterConfig);
  const uint32_t near_wrap = 0xFFFFFF00u;
  uint32_t now = enable(master, near_wrap);
  for (uint8_t cycle = 3; cycle < 20; cycle++) {  // runs past zero
    now += 20;
    answer_all(master, State::Enabled, cycle, now);
    master.on_command(command(false, 300, 0, 0, 0), now);
    master.update(now);
  }
  TEST_ASSERT_EQUAL(State::Enabled, master.state());
  JointController joint(kOpenLoop);
  joint.on_command(false, 0, near_wrap);
  joint.on_command(false, 300, near_wrap + 200);
  joint.update(near_wrap + 290, 0);
  TEST_ASSERT_EQUAL(State::Enabled, joint.state());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_crc_check_value);
  RUN_TEST(test_cobs_round_trip_with_zero_runs);
  RUN_TEST(test_encodes_and_decodes_commands_exactly_like_the_python_reference);
  RUN_TEST(test_status_and_info_encoding_match_python);
  RUN_TEST(test_i2c_frames_match_python_and_round_trip);
  RUN_TEST(test_bad_i2c_frames_are_rejected);
  RUN_TEST(test_garbage_and_corruption_never_produce_a_command);
  RUN_TEST(test_master_boots_stopped_and_sends_estop_to_the_slaves);
  RUN_TEST(test_release_waits_for_every_slave_without_latching);
  RUN_TEST(test_targets_reach_the_slaves_only_once_all_four_follow);
  RUN_TEST(test_a_lost_slave_stops_everything_and_latches_fault);
  RUN_TEST(test_a_frozen_slave_counter_counts_as_lost);
  RUN_TEST(test_a_slave_fault_stops_everything_and_stays_reported);
  RUN_TEST(test_a_slave_built_for_another_joint_is_refused);
  RUN_TEST(test_a_slave_that_does_not_follow_into_enabled_faults_after_the_grace_time);
  RUN_TEST(test_master_command_timeout_requires_neutral_again);
  RUN_TEST(test_master_estop_does_not_time_out_into_motion);
  RUN_TEST(test_joint_requires_neutral_and_latches_estop);
  RUN_TEST(test_joint_stops_when_the_master_goes_quiet);
  RUN_TEST(test_joint_ramps_the_setpoint_but_stops_at_once);
  RUN_TEST(test_joint_measures_speed_and_position_from_the_encoder);
  RUN_TEST(test_speed_control_corrects_a_slow_motor);
  RUN_TEST(test_stall_is_detected_and_latched_until_an_estop);
  RUN_TEST(test_a_reversed_encoder_is_detected_before_it_runs_away);
  RUN_TEST(test_millis_wraparound_is_handled);
  return UNITY_END();
}
