// Native unit tests for lib/udp_frames (the drive packets in UDP datagrams): pio test -e native
#include <string.h>
#include <unity.h>

#include "../test_native/vectors.h"
#include "link_watchdog.h"
#include "udp_frames.h"

static const uint8_t kPc[4] = {192, 168, 77, 1};
static const uint8_t kStranger[4] = {192, 168, 77, 3};

struct Collected {
  int count = 0;
  drive::Packet last = {};
};

static void feed(udp::DatagramReceiver& receiver, const uint8_t (&source)[4], const uint8_t* data, size_t length,
                 Collected& out) {
  receiver.feed(source, data, length, [&](const drive::Packet& packet) {
    out.count++;
    out.last = packet;
  });
}

void setUp() {}
void tearDown() {}

void test_a_command_from_the_pc_is_accepted() {
  udp::DatagramReceiver receiver(kPc);
  Collected got;
  feed(receiver, kPc, kWire_forward_full, sizeof(kWire_forward_full), got);
  TEST_ASSERT_EQUAL(1, got.count);
  drive::DriveCommand command;
  TEST_ASSERT_TRUE(drive::parse_drive_command(got.last, command));
  TEST_ASSERT_EQUAL_INT16(947, command.target_decirpm[0]);
  TEST_ASSERT_EQUAL_UINT16(1, receiver.good());
  TEST_ASSERT_EQUAL_UINT16(0, receiver.bad());
}

void test_datagrams_from_elsewhere_are_dropped_and_counted() {
  udp::DatagramReceiver receiver(kPc);
  Collected got;
  feed(receiver, kStranger, kWire_estop, sizeof(kWire_estop), got);
  TEST_ASSERT_EQUAL(0, got.count);
  TEST_ASSERT_EQUAL_UINT16(1, receiver.foreign());
  TEST_ASSERT_EQUAL_UINT16(1, receiver.bad());
  feed(receiver, kPc, kWire_estop, sizeof(kWire_estop), got);  // the PC still gets through
  TEST_ASSERT_EQUAL(1, got.count);
}

void test_several_packets_in_one_datagram() {
  udp::DatagramReceiver receiver(kPc);
  uint8_t datagram[64];
  memcpy(datagram, kWire_neutral, sizeof(kWire_neutral));
  memcpy(datagram + sizeof(kWire_neutral), kWire_arc_left, sizeof(kWire_arc_left));
  Collected got;
  feed(receiver, kPc, datagram, sizeof(kWire_neutral) + sizeof(kWire_arc_left), got);
  TEST_ASSERT_EQUAL(2, got.count);
  TEST_ASSERT_EQUAL_UINT8(2, got.last.seq);  // arc_left, in order
}

void test_a_cut_off_tail_never_joins_the_next_datagram() {
  udp::DatagramReceiver receiver(kPc);
  Collected got;
  feed(receiver, kPc, kWire_neutral, sizeof(kWire_neutral), got);  // a good first datagram
  feed(receiver, kPc, kWire_forward_full, sizeof(kWire_forward_full) - 1, got);  // its 0x00 is missing
  TEST_ASSERT_EQUAL(1, got.count);
  TEST_ASSERT_EQUAL_UINT16(1, receiver.bad());
  feed(receiver, kPc, kWire_estop, sizeof(kWire_estop), got);  // the next datagram decodes on its own
  TEST_ASSERT_EQUAL(2, got.count);
  TEST_ASSERT_EQUAL_UINT8(255, got.last.seq);
}

void test_garbage_is_counted_not_decoded() {
  udp::DatagramReceiver receiver(kPc);
  Collected got;
  feed(receiver, kPc, kWire_neutral, sizeof(kWire_neutral), got);
  const uint8_t garbage[] = {0x55, 0x13, 0x77, 0x00, 0x01, 0x00};
  feed(receiver, kPc, garbage, sizeof(garbage), got);
  TEST_ASSERT_EQUAL(1, got.count);
  TEST_ASSERT_EQUAL_UINT16(2, receiver.bad());
  feed(receiver, kPc, garbage, 0, got);  // an empty datagram: nothing
  TEST_ASSERT_EQUAL_UINT16(2, receiver.bad());
}

void test_no_restart_before_the_pc_was_ever_heard() {
  udp::LinkWatchdog watchdog(2000);
  TEST_ASSERT_FALSE(watchdog.restart_due(0, true));
  TEST_ASSERT_FALSE(watchdog.restart_due(3600000, true));  // an hour of waiting for the PC: no restart loop
}

void test_restart_when_commands_stop_after_being_heard() {
  udp::LinkWatchdog watchdog(2000);
  watchdog.on_command(1000);
  TEST_ASSERT_FALSE(watchdog.restart_due(1000, true));
  TEST_ASSERT_FALSE(watchdog.restart_due(2999, true));  // 50 Hz commands: gaps far below this never restart
  TEST_ASSERT_TRUE(watchdog.restart_due(3000, true));
  watchdog.on_command(3100);  // the PC is back before the restart happened: nothing to do
  TEST_ASSERT_FALSE(watchdog.restart_due(3200, true));
}

void test_no_restart_while_the_motors_run() {
  udp::LinkWatchdog watchdog(2000);
  watchdog.on_command(0);
  TEST_ASSERT_FALSE(watchdog.restart_due(5000, false));  // cannot happen for long: the 300 ms timeout stops them
  TEST_ASSERT_TRUE(watchdog.restart_due(5000, true));
}

void test_the_watchdog_can_be_turned_off_and_survives_the_millis_wrap() {
  udp::LinkWatchdog off(0);
  off.on_command(0);
  TEST_ASSERT_FALSE(off.restart_due(100000, true));
  udp::LinkWatchdog watchdog(2000);
  const uint32_t near_wrap = 0xFFFFFF00u;
  watchdog.on_command(near_wrap);
  TEST_ASSERT_FALSE(watchdog.restart_due(near_wrap + 1999, true));  // wraps past zero
  TEST_ASSERT_TRUE(watchdog.restart_due(near_wrap + 2000, true));
  watchdog.on_command(near_wrap + 2100);
  TEST_ASSERT_FALSE(watchdog.restart_due(near_wrap + 2099, true));  // a check stamped just before the command
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_a_command_from_the_pc_is_accepted);
  RUN_TEST(test_datagrams_from_elsewhere_are_dropped_and_counted);
  RUN_TEST(test_several_packets_in_one_datagram);
  RUN_TEST(test_a_cut_off_tail_never_joins_the_next_datagram);
  RUN_TEST(test_garbage_is_counted_not_decoded);
  RUN_TEST(test_no_restart_before_the_pc_was_ever_heard);
  RUN_TEST(test_restart_when_commands_stop_after_being_heard);
  RUN_TEST(test_no_restart_while_the_motors_run);
  RUN_TEST(test_the_watchdog_can_be_turned_off_and_survives_the_millis_wrap);
  return UNITY_END();
}
