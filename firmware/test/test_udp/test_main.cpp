// Native unit tests for lib/udp_frames (the drive packets in UDP datagrams): pio test -e native
#include <string.h>
#include <unity.h>

#include "../test_native/vectors.h"
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

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_a_command_from_the_pc_is_accepted);
  RUN_TEST(test_datagrams_from_elsewhere_are_dropped_and_counted);
  RUN_TEST(test_several_packets_in_one_datagram);
  RUN_TEST(test_a_cut_off_tail_never_joins_the_next_datagram);
  RUN_TEST(test_garbage_is_counted_not_decoded);
  return UNITY_END();
}
