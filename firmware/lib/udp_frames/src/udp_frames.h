// udp_frames: the drive packets in UDP datagrams (the rover's Ethernet link rules, telemetry_dashboard
// protocol/ETHERNET_LINK.md), free of Arduino code so they can be unit-tested on a PC (pio test -e native).
//  - Each datagram carries one or more complete packets, each ending in 0x00. A tail without its 0x00 is discarded
//    and counted; it never joins the next datagram.
//  - Only the onboard PC's address may command the ESP32: datagrams from anywhere else are dropped and counted.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "drive_core.h"

namespace udp {

constexpr size_t kMaxDatagram = 512;  // UDP payload; the PC sends one 16-byte packet per datagram

class DatagramReceiver {
 public:
  explicit DatagramReceiver(const uint8_t (&pc_ip)[4]) { memcpy(pc_ip_, pc_ip, sizeof(pc_ip_)); }

  // Feeds one datagram received from `source_ip`; calls on_packet(const drive::Packet&) for every valid packet in it.
  template <typename OnPacket>
  void feed(const uint8_t (&source_ip)[4], const uint8_t* data, size_t length, OnPacket on_packet) {
    if (memcmp(source_ip, pc_ip_, sizeof(pc_ip_)) != 0) {
      if (foreign_ < 0xFFFF) foreign_++;
      return;
    }
    // Complete packets end at the datagram's last 0x00; anything after it is a cut-off tail. It is rejected without
    // decoding (with a 0x00 added, a packet missing only its delimiter would pass), so the decoder stays empty.
    size_t end = length;
    while (end > 0 && data[end - 1] != 0) end--;
    drive::Packet packet;
    for (size_t i = 0; i < end; i++) {
      if (decoder_.feed(data[i], packet)) on_packet(packet);
    }
    if (end < length && tails_ < 0xFFFF) tails_++;
  }

  uint16_t good() const { return decoder_.good(); }
  // Everything rejected, as DRIVE_STATUS.rx_bad reports it: bad packets, cut-off tails, datagrams from elsewhere.
  uint16_t bad() const {
    const uint32_t total = static_cast<uint32_t>(decoder_.bad()) + foreign_ + tails_;
    return total > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(total);
  }
  uint16_t foreign() const { return foreign_; }

 private:
  drive::PacketDecoder decoder_;
  uint8_t pc_ip_[4];
  uint16_t foreign_ = 0;
  uint16_t tails_ = 0;
};

}  // namespace udp
