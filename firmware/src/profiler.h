// Profiling build only (pio run -e profile); see profiler.cpp. The production build compiles none of this.
#pragma once

#if DRIVE_PROFILE
#include <stddef.h>
#include <stdint.h>

namespace profiler {

constexpr uint8_t kTypeProfile = 0x05;  // PROFILE packet, ESP32 -> PC once a second; drive_node and wheel_test ignore it
constexpr size_t kReportPayload = 49;

void begin();
void loop_begin();  // first thing in loop()
void loop_end();    // last thing in loop(), before delay(1)
bool report_due();
// Writes the PROFILE payload (kReportPayload bytes, layout in profiler.cpp) and starts a new report period.
size_t build_report(uint8_t* out);

}  // namespace profiler
#endif
