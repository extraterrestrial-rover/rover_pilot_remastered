// Profiling build only (pio run -e profile): measures what the firmware costs the ESP32 and reports it once a second
// in a PROFILE packet that tools/profile.py prints. The production build (esp32doit-devkit-v1) compiles none of this.
//
//  - CPU load of each core: FreeRTOS run-time stats (microsecond counters) say how long each core's idle task ran;
//    load = 1 - idle / elapsed, per 100 ms window. The peak is the busiest window. This counts everything the chip
//    does (this firmware, the serial/PWM drivers, the watchdog, task switches), not just the drive code. Interrupt
//    time is charged to whatever task it interrupted, so a little of it can land on the idle task.
//  - Loop passes: how long each pass of loop() is busy (start of loop() to just before delay(1)), and the longest.
//  - Memory: free heap now and its lowest value since boot; the deepest the loop task's stack has ever been used.
//  - The profiler's own cost, so it can be subtracted (the production firmware does not have it).
//  - Optional marker pin, high while loop() is busy, for a logic analyser or scope.
#if DRIVE_PROFILE
#include "profiler.h"

#include <Arduino.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/gpio_reg.h>

#ifndef PROFILE_MARKER_PIN
#define PROFILE_MARKER_PIN 21  // free on this board (not a motor, LED or strapping pin); -1 disables the marker
#endif
static_assert(PROFILE_MARKER_PIN < 32, "the marker uses the GPIO 0-31 set/clear registers");

namespace profiler {

// Outside the anonymous namespace: REG_WRITE's compile-time check declares an extern function, which inside an
// anonymous namespace would warn (-Wunused-function).
inline void marker(bool high) {
#if PROFILE_MARKER_PIN >= 0
  REG_WRITE(high ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, 1u << PROFILE_MARKER_PIN);  // a single register write
#else
  (void)high;
#endif
}

namespace {

constexpr int64_t kWindowUs = 100000;   // CPU load per 100 ms window
constexpr int64_t kReportUs = 1000000;  // one report per second
constexpr int kCores = 2;

// Current window.
int64_t window_start_us = 0;
uint32_t window_idle_us[kCores] = {0, 0};
// Current report period.
int64_t report_start_us = 0;
uint16_t windows = 0;
uint32_t load_sum[kCores] = {0, 0};   // 0.01 % units, summed over the period's windows
uint16_t load_peak[kCores] = {0, 0};  // busiest window of the period, 0.01 %
uint32_t passes = 0;
uint32_t busy_total_us = 0;
uint32_t busy_max_us = 0;
uint32_t own_us = 0;  // the profiler's own time
// Since boot.
uint16_t load_peak_ever[kCores] = {0, 0};
uint32_t busy_max_ever_us = 0;

int64_t loop_start_us = 0;

void close_window(int64_t now_us) {
  const uint32_t elapsed_us = static_cast<uint32_t>(now_us - window_start_us);
  for (int core = 0; core < kCores; core++) {
    const uint32_t idle_now = ulTaskGetIdleRunTimeCounterForCore(core);
    const uint32_t idle_us = idle_now - window_idle_us[core];  // the 32-bit counter wraps after 71 min; deltas don't care
    window_idle_us[core] = idle_now;
    const uint32_t idle_share = elapsed_us > 0 ? static_cast<uint32_t>((uint64_t)idle_us * 10000u / elapsed_us) : 10000u;
    const uint16_t load = idle_share >= 10000u ? 0 : static_cast<uint16_t>(10000u - idle_share);
    load_sum[core] += load;
    if (load > load_peak[core]) load_peak[core] = load;
    if (load > load_peak_ever[core]) load_peak_ever[core] = load;
  }
  windows++;
  window_start_us = now_us;
}

void put_u16(uint8_t* out, uint32_t value) {
  const uint16_t clamped = value > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(value);
  out[0] = static_cast<uint8_t>(clamped & 0xFF);
  out[1] = static_cast<uint8_t>(clamped >> 8);
}

void put_u32(uint8_t* out, uint32_t value) {
  for (int i = 0; i < 4; i++) out[i] = static_cast<uint8_t>(value >> (8 * i));
}

}  // namespace

void begin() {
#if PROFILE_MARKER_PIN >= 0
  pinMode(PROFILE_MARKER_PIN, OUTPUT);
  marker(false);
#endif
  const int64_t now_us = esp_timer_get_time();
  window_start_us = report_start_us = now_us;
  for (int core = 0; core < kCores; core++) window_idle_us[core] = ulTaskGetIdleRunTimeCounterForCore(core);
}

void loop_begin() {
  loop_start_us = esp_timer_get_time();
  marker(true);
}

void loop_end() {
  const int64_t end_us = esp_timer_get_time();
  marker(false);
  const uint32_t busy_us = static_cast<uint32_t>(end_us - loop_start_us);
  passes++;
  busy_total_us += busy_us;
  if (busy_us > busy_max_us) busy_max_us = busy_us;
  if (busy_us > busy_max_ever_us) busy_max_ever_us = busy_us;
  if (end_us - window_start_us >= kWindowUs) close_window(end_us);
  own_us += static_cast<uint32_t>(esp_timer_get_time() - end_us);
}

bool report_due() {
  return esp_timer_get_time() - report_start_us >= kReportUs;
}

// PROFILE payload, little-endian (tools/profile.py decodes it):
//   0 u8 layout version (1)        1 u16 window ms        3 u16 windows in this report
//   5 u16 core 0 average load      7 u16 core 0 peak window    9 u16 core 1 average   11 u16 core 1 peak window
//  13 u16 core 0 peak since boot  15 u16 core 1 peak since boot              (loads in 0.01 %)
//  17 u32 loop passes  21 u32 loop busy total us  25 u16 longest pass us  27 u16 longest pass us since boot
//  29 u32 profiler's own us  33 u32 free heap B  37 u32 lowest free heap B  41 u16 loop stack never used B
//  43 u16 loop stack size B  45 u32 uptime ms
size_t build_report(uint8_t* out) {
  const int64_t start_us = esp_timer_get_time();
  out[0] = 1;
  put_u16(out + 1, kWindowUs / 1000);
  put_u16(out + 3, windows);
  for (int core = 0; core < kCores; core++) {
    put_u16(out + 5 + 4 * core, windows ? load_sum[core] / windows : 0);
    put_u16(out + 7 + 4 * core, load_peak[core]);
    put_u16(out + 13 + 2 * core, load_peak_ever[core]);
  }
  put_u32(out + 17, passes);
  put_u32(out + 21, busy_total_us);
  put_u16(out + 25, busy_max_us);
  put_u16(out + 27, busy_max_ever_us);
  put_u32(out + 29, own_us);
  put_u32(out + 33, esp_get_free_heap_size());
  put_u32(out + 37, esp_get_minimum_free_heap_size());
  put_u16(out + 41, uxTaskGetStackHighWaterMark(nullptr));  // ESP-IDF counts this in bytes
  put_u16(out + 43, getArduinoLoopTaskStackSize());
  put_u32(out + 45, millis());

  windows = 0;
  passes = busy_total_us = busy_max_us = 0;
  for (int core = 0; core < kCores; core++) load_sum[core] = load_peak[core] = 0;
  report_start_us = esp_timer_get_time();
  own_us = static_cast<uint32_t>(report_start_us - start_us);  // the next period starts with this report's cost
  return kReportPayload;
}

}  // namespace profiler
#endif
