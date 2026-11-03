// net_link: the Ethernet link to the onboard PC (ESP32 glue): a W5500 on SPI through Arduino-ESP32's ETH (ESP-IDF's
// W5500 driver and lwIP), the static address from config.h, and one UDP socket that never blocks. The datagram rules
// and the source check are in lib/udp_frames.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace net {

// Starts the W5500 and opens the socket. False if the W5500 did not start; service() keeps retrying every 2 s.
bool begin();
// Call every pass: retries a failed start and prints link changes on USB.
void service(uint32_t now_ms);
// Takes one received datagram, if any: returns its length (0 if none) and the sender's IPv4 address.
size_t receive(uint8_t* buffer, size_t size, uint8_t (&source_ip)[4]);
// Sends one datagram to the PC. False if it could not be handed to lwIP (link down, no buffer); never blocks.
bool send(const uint8_t* data, size_t length);

}  // namespace net
