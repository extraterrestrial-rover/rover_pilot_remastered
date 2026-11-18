// Arm firmware configuration, shared by the master and the four slaves (one goBILDA gearmotor each).
// Values marked PLACEHOLDER are not decided yet: see ARM_OPEN_QUESTIONS.md at the repository root.
#pragma once

#include <stdint.h>

#define FIRMWARE_VERSION_MAJOR 1
#define FIRMWARE_VERSION_MINOR 0
#define FIRMWARE_VERSION_PATCH 0

// ------------------------------------------------------------------------------------------- master <-> PC (USB)
#define SERIAL_BAUD 115200      // must match the arm driver's `baud` parameter
#define COMMAND_TIMEOUT_MS 300  // stop if no valid ARM_COMMAND from the PC for this long
#define STATUS_PERIOD_MS 50     // ARM_STATUS to the PC (20 Hz, also feeds /arm/joint_states)
#define INFO_PERIOD_MS 1000     // ARM_INFO to the PC

// ------------------------------------------------------------------------------------------- I2C bus
// PLACEHOLDER pins, speed and addresses: the defaults of the DOIT DevKit V1 (SDA 21, SCL 22) at 100 kHz.
// Every board on the bus needs a common ground. Fit one pair of pull-ups (2.2-4.7 kOhm to 3.3 V) for the whole bus.
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQUENCY_HZ 100000
#define I2C_TIMEOUT_MS 5        // master: give up on a stuck transaction after this long
#define BUS_PERIOD_MS 20        // master: one command + status exchange with every slave (50 Hz); takes ~15 ms at 100 kHz
#define SLAVE_TIMEOUT_MS 100    // master: a slave with no fresh status for this long is lost -> Fault
#define ENABLE_GRACE_MS 200     // master: slaves get this long to follow it into ENABLED
#define SLAVE_COMMAND_TIMEOUT_MS 100  // slave: stop if no valid frame from the master for this long

// ------------------------------------------------------------------------------------------- timing, LED
#define LOOP_WATCHDOG_MS 1000  // a loop that stops running for this long resets the ESP32 (it boots with motors off)
#define CONTROL_PERIOD_MS 10   // state machine (+ speed control on the slaves)
// On-board LED: solid = enabled, 5 Hz = e-stop, 10 Hz = fault, 1 Hz = waiting for commands / neutral.
#define STATUS_LED_PIN 2

// ------------------------------------------------------------------------------------------- slave wiring
// The same on every slave board: one motor driver (direction + PWM, as on the rover) and the motor's encoder.
// goBILDA encoders: power them from 3.3 V, or level-shift A/B: ESP32 inputs are not 5 V tolerant.
#define MOTOR_DIR_PIN 25
#define MOTOR_PWM_PIN 26
#define ENCODER_A_PIN 32
#define ENCODER_B_PIN 33

// ------------------------------------------------------------------------------------------- joints
// Motor order everywhere: shoulder, elbow, wrist_a, wrist_b. A slave is built for one joint (pio run -e slave_<joint>);
// the master reads the address table and rejects a slave that reports another joint.
//
// Positive directions (set forward_level so the motor turns this way at positive PWM, then encoder_reversed so the
// encoder counts up): shoulder and elbow positive = pitch up. wrist_a and wrist_b positive together = wrist pitch up;
// wrist_a positive with wrist_b negative = positive wrist roll (the auger turning its drilling direction).
struct JointHardware {
  const char* name;
  uint8_t i2c_address;    // PLACEHOLDER
  uint8_t forward_level;  // DIR pin level for positive rotation
  bool encoder_reversed;  // true if the encoder counts down while the motor turns positive
  float max_rpm;          // gearmotor output rpm at PWM 255 (PLACEHOLDER; must match motor_max_rpm in config/arm.yaml)
  float counts_per_rev;   // encoder counts per gearmotor output revolution (PLACEHOLDER)
  bool encoder_fitted;    // false = open loop, no stall / direction checks
  bool closed_loop;       // PI speed control on top of the open-loop feed-forward
};

// PLACEHOLDER motors: goBILDA 5203 Yellow Jacket 50.9:1 (117 rpm at 12 V, 1425.1 counts per output revolution).
static const JointHardware kJoints[4] = {
    {"shoulder", 0x10, 1, false, 117.0f, 1425.1f, true, true},
    {"elbow", 0x11, 1, false, 117.0f, 1425.1f, true, true},
    {"wrist_a", 0x12, 1, false, 117.0f, 1425.1f, true, true},
    {"wrist_b", 0x13, 1, false, 117.0f, 1425.1f, true, true},
};

// ------------------------------------------------------------------------------------------- slave control tuning
#define SLEW_FULL_SCALE_MS 250   // setpoint ramp: 0 -> max_rpm in this long; stops are immediate
#define SPEED_KP 0.5f            // PWM fraction per unit of speed error / max_rpm
#define SPEED_KI 2.0f            // the same, per second
#define SPEED_INTEGRAL_LIMIT 0.3f  // at most 30 % PWM from the integral term
// Stall: PWM >= 70 % but under 5 % of max_rpm for 1 s -> Fault (hard stop reached, jammed auger, dead encoder).
#define STALL_PWM_FRACTION 0.7f
#define STALL_RPM_FRACTION 0.05f
#define STALL_MS 1000
// Wrong direction: PWM >= 30 % but turning the other way faster than 20 % of max_rpm for 300 ms -> Fault
// (encoder_reversed or forward_level wrong; with speed control that would otherwise run away at full PWM).
#define REVERSE_PWM_FRACTION 0.3f
#define REVERSE_RPM_FRACTION 0.2f
#define REVERSE_MS 300
