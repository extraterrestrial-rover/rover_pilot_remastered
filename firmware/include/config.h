// Rover drive firmware configuration. Pins and directions match the original rover_pilot firmware.
#pragma once

#include <stdint.h>

#define FIRMWARE_VERSION_MAJOR 3  // 3.x: the Ethernet (W5500) edition, branch ethernet-udp; 2.x is the USB serial one
#define FIRMWARE_VERSION_MINOR 0
#define FIRMWARE_VERSION_PATCH 0

// USB: power and flashing only, plus a few lines of bring-up text at this baud. The packets travel over Ethernet.
#define SERIAL_BAUD 115200

// Ethernet to the onboard PC (LINK.md): a W5500 module on SPI. The ESP32's usual SPI pins (18/19/23, 12/13/14) drive
// motors, so the bus is routed to free GPIOs; at this clock that works through the GPIO matrix.
#define W5500_SCLK_PIN 16
#define W5500_MOSI_PIN 17
#define W5500_MISO_PIN 34  // input-only GPIO: fine for an input
#define W5500_CS_PIN 5     // a strapping pin, but its boot level (high) is a chip select's idle level
#define W5500_INT_PIN 35   // input-only GPIO; -1 if INT is not wired (the driver then polls the W5500 every 10 ms)
#define W5500_RST_PIN 4
#define W5500_SPI_MHZ 16  // try 8 if the SPI wires are long
// The same addresses and ports as the rover's CAN master (telemetry_dashboard protocol/ETHERNET_LINK.md), on a
// private two-device subnet with no gateway. Must match the drive node's master_host / master_port / local_port.
#define ESP_IP_OCTETS 192, 168, 77, 2
#define PC_IP_OCTETS 192, 168, 77, 1
#define NETMASK_OCTETS 255, 255, 255, 0
#define ESP_UDP_PORT 47700  // the ESP32 receives DRIVE_COMMAND here, only from PC_IP
#define PC_UDP_PORT 47701   // and sends DRIVE_STATUS (and PROFILE) to PC_IP here

// Open-loop calibration: PWM 255 drives a wheel at this many rpm. 94.7 rpm on a 0.28 m wheel is 5 km/h.
// Must match the drive node's `max_wheel_rpm` parameter (the node warns if they differ). It is a motor property, so it
// stays the same when the wheels are swapped; only the drive node's wheel diameter changes.
#define MAX_WHEEL_RPM 94.7f

// Safety and smoothness.
#define COMMAND_TIMEOUT_MS 300  // stop if no valid DRIVE_COMMAND for this long
#define SLEW_PWM_PER_S 1020.0f  // max PWM change per second while driving (255 in 250 ms); 0 = unlimited. Stops are immediate.

// Timing.
#define LOOP_WATCHDOG_MS 1000  // a loop that stops running for this long resets the ESP32 (it boots with motors off)
#define CONTROL_PERIOD_MS 10  // state machine + PWM update
#define STATUS_PERIOD_MS 100  // DRIVE_STATUS to the PC

// Status LED (on-board LED of the DOIT DevKit V1): solid = driving, fast blink = e-stop, slow blink = waiting.
#define STATUS_LED_PIN 2

// Motor drivers: one direction pin and one PWM pin each. FORWARD_LEVEL is the direction pin level that drives the
// wheel forward (from the original firmware: front/back wheels HIGH = forward, middle wheels LOW = forward).
struct MotorPins {
  uint8_t dir_pin;
  uint8_t pwm_pin;
  uint8_t forward_level;
};

// Wheel order everywhere: FL, FR, RL, RR.
// Note: GPIO 12 (left-back direction) is an ESP32 strapping pin. If the motor driver pulls it high at power-up the
// ESP32 may fail to boot; move it to a free non-strapping GPIO if that happens.
static const MotorPins kDriveMotors[4] = {
    {25, 26, 1},  // FL  (leftFront  {dir, pwm})
    {22, 23, 1},  // FR  (rightFront)
    {12, 13, 1},  // RL  (leftBack)
    {18, 19, 1},  // RR  (rightBack)
};

// Middle wheels of the 6-wheel chassis. Not driven on this rover; their pins are held at PWM 0.
// Set DRIVE_MIDDLE_WHEELS to 1 to drive them with the average of their side's front and rear wheels.
#define DRIVE_MIDDLE_WHEELS 0
static const MotorPins kMiddleMotors[2] = {
    {14, 27, 0},  // left middle
    {32, 33, 0},  // right middle
};
