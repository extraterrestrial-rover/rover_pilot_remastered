// Arm firmware configuration, shared by the master and the four slaves (one goBILDA gearmotor each).
// I2C is on SDA 21 / SCL 22 and the motor / encoder pins follow the arm PCB schematic (board wiring below).
// Values marked PLACEHOLDER are not decided yet: see ARM_OPEN_QUESTIONS.md at the repository root.
#pragma once

#include <stdint.h>

#define FIRMWARE_VERSION_MAJOR 1
#define FIRMWARE_VERSION_MINOR 2
#define FIRMWARE_VERSION_PATCH 0

// ------------------------------------------------------------------------------------------- control mode
// Current plan: open loop with the encoders read and reported only. The PWM never depends on the encoder.
// SPEED_CONTROL 0 = open loop (as on the rover): PWM = command / max_rpm * 255, no PID. 1 = PI speed control on the
// encoder (needs ENCODERS_WIRED 1). The setpoint ramp and every safety rule apply in both modes.
#define SPEED_CONTROL 0
// ENCODERS_WIRED 1 = encoder A/B connected: speed and count are measured and sent to the PC (/diagnostics,
// /arm/joint_states, the slave's USB printout). 0 = not connected: nothing reported.
#define ENCODERS_WIRED 1
// ENCODER_FAULT_CHECKS 1 = a slave stops its motor and latches FAULT on a stall or on turning against its PWM
// (STALL_* / REVERSE_* below). 0 = off: the encoder value is only reported, never acted on.
#define ENCODER_FAULT_CHECKS 0
#if SPEED_CONTROL && !ENCODERS_WIRED
#error "SPEED_CONTROL needs ENCODERS_WIRED"
#endif
#if ENCODER_FAULT_CHECKS && !ENCODERS_WIRED
#error "ENCODER_FAULT_CHECKS needs ENCODERS_WIRED"
#endif

// ------------------------------------------------------------------------------------------- master <-> PC (USB)
#define SERIAL_BAUD 115200      // must match the arm driver's `baud` parameter
#define COMMAND_TIMEOUT_MS 300  // stop if no valid ARM_COMMAND from the PC for this long
#define STATUS_PERIOD_MS 50     // ARM_STATUS to the PC (20 Hz, also feeds /arm/joint_states)
#define INFO_PERIOD_MS 1000     // ARM_INFO to the PC

// ------------------------------------------------------------------------------------------- I2C bus
// SDA 21 / SCL 22 from the arm PCB schematic. PLACEHOLDER speed and addresses: 100 kHz, 0x10-0x13.
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

// ------------------------------------------------------------------------------------------- PWM limit
// Hard cap on every motor's PWM, in percent of full (255). The slave clamps its output to it whatever it is sent, so
// 35 % = PWM 89. The ROS side maps full stick to the same limit (pwm_limit in config/arm.yaml).
#define PWM_LIMIT_PERCENT 35
#if PWM_LIMIT_PERCENT <= 0 || PWM_LIMIT_PERCENT > 100
#error "PWM_LIMIT_PERCENT must be 1..100"
#endif

// ------------------------------------------------------------------------------------------- board wiring
// The arm PCB (ESP32-WROOM-32 30-pin dev board, schematic ESP32_INFERNO): two motor-driver channels (DIR + PWM, as on
// the rover) and two encoder inputs, encoders powered from 3.3 V. Each joint below says which channel its motor uses.
// GPIO 34 / 35 are input-only and have no internal pull-up: if the encoder outputs are open-collector, fit external
// pull-ups to 3.3 V. GPIO 12 (MD_1 DIR) is a strapping pin: if the driver pulls it high at power-up the ESP32 does not
// boot. On the master, the motor-driver pins of both channels are held low.
struct ChannelPins {
  uint8_t dir_pin;
  uint8_t pwm_pin;
  uint8_t encoder_a_pin;
  uint8_t encoder_b_pin;
};
static constexpr ChannelPins kChannels[2] = {
    {33, 25, 34, 35},  // channel 0: MD_0 (DIR 33, PWM 25), ENCODER_0 (A 34, B 35)
    {12, 13, 18, 5},   // channel 1: MD_1 (DIR 12, PWM 13), ENCODER_1 (A 18, B 5)
};

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
  uint8_t channel;        // motor-driver + encoder channel on the slave board (kChannels); PLACEHOLDER: 0 on every slave
  uint8_t forward_level;  // DIR pin level for positive rotation
  bool encoder_reversed;  // true if the encoder counts down while the motor turns positive
  // Gearmotor output rpm at PWM 255 (PLACEHOLDER; must match motor_max_rpm in config/arm.yaml). In open loop it is only
  // the scale between the rpm in a command and the PWM: full stick is PWM 255 whatever the real motor speed.
  float max_rpm;
  float counts_per_rev;   // encoder counts per gearmotor output revolution (PLACEHOLDER)
  bool encoder_fitted;    // false = no stall / direction checks, speed and position not reported
  bool closed_loop;       // PI speed control on top of the open-loop feed-forward
};

// PLACEHOLDER motors: goBILDA 5203 Yellow Jacket 50.9:1 (117 rpm at 12 V, 1425.1 counts per output revolution).
// The last two fields follow the control-mode switches above; set them per joint to mix modes.
static constexpr JointHardware kJoints[4] = {
    {"shoulder", 0x10, 0, 1, false, 117.0f, 1425.1f, ENCODERS_WIRED != 0, SPEED_CONTROL != 0},
    {"elbow", 0x11, 0, 1, false, 117.0f, 1425.1f, ENCODERS_WIRED != 0, SPEED_CONTROL != 0},
    {"wrist_a", 0x12, 0, 1, false, 117.0f, 1425.1f, ENCODERS_WIRED != 0, SPEED_CONTROL != 0},
    {"wrist_b", 0x13, 0, 1, false, 117.0f, 1425.1f, ENCODERS_WIRED != 0, SPEED_CONTROL != 0},
};

// ------------------------------------------------------------------------------------------- slave control tuning
#define SLEW_FULL_SCALE_MS 250   // setpoint ramp: 0 -> max_rpm in this long; stops are immediate
#define SPEED_KP 0.5f            // PWM fraction per unit of speed error / max_rpm
#define SPEED_KI 2.0f            // the same, per second
#define SPEED_INTEGRAL_LIMIT 0.3f  // at most 30 % PWM from the integral term
// Stall: PWM >= 30 % but under 5 % of max_rpm for 1 s -> Fault (hard stop reached, jammed auger, dead encoder).
// Keep STALL_PWM_FRACTION below PWM_LIMIT_PERCENT / 100, or the check can never trigger.
#define STALL_PWM_FRACTION 0.3f
#define STALL_RPM_FRACTION 0.05f
#define STALL_MS 1000
// Wrong direction: PWM >= 30 % but turning the other way faster than 20 % of max_rpm for 300 ms -> Fault
// (encoder_reversed or forward_level wrong; with speed control that would otherwise run away at full PWM).
#define REVERSE_PWM_FRACTION 0.3f
#define REVERSE_RPM_FRACTION 0.2f
#define REVERSE_MS 300
