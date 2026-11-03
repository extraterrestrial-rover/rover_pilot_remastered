#include "net_link.h"

#include <Arduino.h>
#include <ETH.h>
#include <lwip/sockets.h>
#include <string.h>

#include "config.h"

#if ESP_ARDUINO_VERSION_MAJOR < 3
#error "The W5500 needs Arduino-ESP32 core 3.x (ETH with SPI Ethernet); core 2.x is only for the USB serial branch"
#endif

namespace net {

namespace {

constexpr uint32_t kRetryMs = 2000;
const uint8_t kEspIp[4] = {ESP_IP_OCTETS};
const uint8_t kPcIp[4] = {PC_IP_OCTETS};
const uint8_t kNetmask[4] = {NETMASK_OCTETS};

bool started = false;
int sock = -1;
bool link_was_up = false;
uint32_t last_attempt_ms = 0;
sockaddr_in pc_address = {};

IPAddress address(const uint8_t (&octets)[4]) { return IPAddress(octets[0], octets[1], octets[2], octets[3]); }

bool open_socket() {
  sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock < 0) return false;
  lwip_fcntl(sock, F_SETFL, lwip_fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
  sockaddr_in local = {};
  local.sin_family = AF_INET;
  local.sin_port = htons(ESP_UDP_PORT);
  local.sin_addr.s_addr = htonl(INADDR_ANY);  // receive whatever arrives; lib/udp_frames checks the sender
  if (lwip_bind(sock, reinterpret_cast<sockaddr*>(&local), sizeof(local)) == 0) return true;
  lwip_close(sock);
  sock = -1;
  return false;
}

bool start() {
  if (!ETH.begin(ETH_PHY_W5500, 1, W5500_CS_PIN, W5500_INT_PIN, W5500_RST_PIN, SPI3_HOST, W5500_SCLK_PIN,
                 W5500_MISO_PIN, W5500_MOSI_PIN, W5500_SPI_MHZ)) {
    ETH.end();
    return false;
  }
  // Static address, no gateway: the PC is on the same two-device subnet.
  if (ETH.config(address(kEspIp), IPAddress(0, 0, 0, 0), address(kNetmask)) && open_socket()) return true;
  ETH.end();  // so the next attempt starts from scratch
  return false;
}

}  // namespace

bool begin() {
  pc_address.sin_family = AF_INET;
  pc_address.sin_port = htons(PC_UDP_PORT);
  memcpy(&pc_address.sin_addr.s_addr, kPcIp, sizeof(kPcIp));  // the octets in order are network byte order
  started = start();
  Serial.printf("drive ESP32 fw %d.%d.%d: W5500 %s; %d.%d.%d.%d:%d -> %d.%d.%d.%d:%d\n", FIRMWARE_VERSION_MAJOR,
                FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_PATCH, started ? "started" : "NOT answering (check SPI wiring)",
                kEspIp[0], kEspIp[1], kEspIp[2], kEspIp[3], ESP_UDP_PORT, kPcIp[0], kPcIp[1], kPcIp[2], kPcIp[3], PC_UDP_PORT);
  return started;
}

void service(uint32_t now_ms) {
  if (!started) {
    if (now_ms - last_attempt_ms < kRetryMs) return;
    last_attempt_ms = now_ms;
    started = start();
    if (started) Serial.println("W5500 started");
    return;
  }
  const bool up = ETH.linkUp();
  if (up != link_was_up) {
    link_was_up = up;
    Serial.println(up ? "Ethernet link up" : "Ethernet link down (cable, or the PC's port is off)");
  }
}

size_t receive(uint8_t* buffer, size_t size, uint8_t (&source_ip)[4]) {
  if (sock < 0) return 0;
  sockaddr_in from = {};
  socklen_t from_length = sizeof(from);
  const int length = lwip_recvfrom(sock, buffer, size, MSG_DONTWAIT, reinterpret_cast<sockaddr*>(&from), &from_length);
  if (length <= 0) return 0;
  memcpy(source_ip, &from.sin_addr.s_addr, sizeof(source_ip));
  return static_cast<size_t>(length);
}

bool send(const uint8_t* data, size_t length) {
  if (sock < 0 || !ETH.linkUp()) return false;
  return lwip_sendto(sock, data, length, MSG_DONTWAIT, reinterpret_cast<const sockaddr*>(&pc_address),
                     sizeof(pc_address)) == static_cast<int>(length);
}

}  // namespace net
