// link_watchdog: when the ESP32 must restart itself so that its W5500 is initialised again. Free of Arduino code, so
// it is unit-tested on a PC (pio test -e native).
//
// The W5500 can reset, or lose its registers, while the ESP32 keeps running: a dip on its 3.3 V supply or noise on
// the SPI wires when the motors start. Its link LEDs come back on, because the PHY negotiates by itself, but its MAC
// address and socket setup are gone and the driver does not know. Nothing gets through from then on, the onboard PC
// cannot even ping the ESP32, and only a reset (the EN button) repaired it.
//
// From here that looks like this: commands were arriving, and then they stopped. The onboard PC sends 50 a second for
// as long as its drive node runs, so a silence right after hearing it means the link is broken somewhere. The ESP32
// then restarts itself, which is what pressing EN does and takes about 3 s. It does so once per silence: after the
// restart it has heard nothing yet, so it simply waits for the PC. If the PC stopped on purpose, that one restart
// costs nothing.
#pragma once

#include <stdint.h>

namespace udp {

class LinkWatchdog {
 public:
  // silence_ms: restart after this long without a valid command, once one has been heard; 0 turns the watchdog off.
  explicit LinkWatchdog(uint32_t silence_ms) : silence_ms_(silence_ms) {}

  // A valid DRIVE_COMMAND from the onboard PC arrived.
  void on_command(uint32_t now_ms) {
    heard_ = true;
    last_command_ms_ = now_ms;
  }

  // True when the ESP32 should restart now. Never while the motors run: the command timeout stops them first.
  bool restart_due(uint32_t now_ms, bool motors_off) const {
    if (silence_ms_ == 0 || !heard_ || !motors_off) return false;
    const int32_t silent_ms = static_cast<int32_t>(now_ms - last_command_ms_);  // safe across the millis() wrap
    return silent_ms > 0 && static_cast<uint32_t>(silent_ms) >= silence_ms_;
  }

 private:
  uint32_t silence_ms_;
  bool heard_ = false;
  uint32_t last_command_ms_ = 0;
};

}  // namespace udp
