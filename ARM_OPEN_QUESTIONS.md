# arm_pilot_redemption: open questions and decisions

Decisions still open for the arm, with what the code does until they are made. Update this file when one is settled, then change the setting named in the "Where" column. (Kiro reads this file through `.kiro/steering/arm-open-questions.md` when you work on the arm.)

## Still to decide

| # | Question | Placeholder in use now | Where |
|---|---|---|---|
| 1 | **Gear ratios**: the worm gearbox ratio of the shoulder and the elbow (and any stage after it), and the bevel gear ratios of the wrist for pitch and for roll | 1.0 everywhere, so `/arm/joint_states` is in gearmotor turns rather than joint angles. Speed limits are not affected: full stick = motors at full PWM | `config/arm.yaml`: `shoulder_ratio`, `elbow_ratio`, `wrist_pitch_ratio`, `wrist_roll_ratio` |
| 2 | **goBILDA motor model per joint**: max rpm and encoder counts per output revolution | 5203 Yellow Jacket 50.9:1 (117 rpm, 1425.1 counts) on all four | `config.h` `kJoints` (`max_rpm`, `counts_per_rev`) **and** `arm.yaml` `motor_max_rpm` (they must match; the driver warns) |
| 3 | **I2C**: SDA/SCL pins, bus speed, slave addresses, pull-up resistors | GPIO 21 / 22, 100 kHz, 0x10–0x13, one 2.2–4.7 kΩ pair for the bus | `config.h`: `I2C_SDA_PIN`, `I2C_SCL_PIN`, `I2C_FREQUENCY_HZ`, `kJoints[].i2c_address` |
| 4 | **Slave pins**: motor driver DIR/PWM and encoder A/B on each slave board | DIR 25, PWM 26, encoder 32 / 33 (same on every slave) | `config.h`: `MOTOR_DIR_PIN`, `MOTOR_PWM_PIN`, `ENCODER_A_PIN`, `ENCODER_B_PIN` |
| 5 | **Encoder supply**: 3.3 V, or 5 V with a level shifter (ESP32 inputs are not 5 V tolerant) | assumed 3.3 V (internal pull-ups on) | wiring |
| 6 | **Directions**: which DIR level turns each motor positive, and whether each encoder counts up | `forward_level = 1`, `encoder_reversed = false` | `config.h` `kJoints`; found with `tools/arm_joint_test.py` |
| 7 | **PS5 indices** on the base station's controller | usual Linux `joy_node` layout: axes 1 / 4 / 7 / 6, Circle 1, Options 9, L1 4. The rover's `drive.yaml` found different indices on its PS5, so these need checking | `config/arm.yaml` `arm_teleop` section |
| 8 | **Joint limits and homing**: limit switches, or a homing routine, or soft limits | none. Positions count from power-up; only the stall detection protects the hard stops | firmware + driver (new work) |
| 9 | **Gripper**: when the auger is replaced, does the gripper open/close with the bevel roll, or does it get its own (5th) motor and slave? | auger on wrist roll; protocol and code have exactly 4 motors | protocol, firmware, teleop (new work if a 5th motor) |
| 10 | **Auger speed control**: fixed speed on the D-pad is fine, or proportional (e.g. on R2)? | D-pad: full speed, 35 % with L1 | `arm_teleop` mapping (a trigger rests at +1.0, so it would need its own handling) |
| 11 | **Which computer runs `arm_driver`**: the base station laptop with the master on its USB, or a computer on the arm connected through the switch? | either works; both launch files can run on one laptop | launch / DDS setup |
| 12 | **Power and kill switch**: are the slaves powered from the motor battery (through a regulator) or from USB? Does the kill switch cut them? | not assumed; the boot interlock and timeouts cover both | wiring |
| 13 | **Encoder fault checks**: should a slave stop its motor on a stall (≥ 70 % PWM, < 5 % speed for 1 s) or when its encoder counts against its PWM? Later: speed-loop tuning if speed control is turned on | off: the encoder is only reported (`ENCODER_FAULT_CHECKS 0`); KP 0.5, KI 2.0 unused | `config.h`: `ENCODER_FAULT_CHECKS`, `STALL_*`, `REVERSE_*`, `SPEED_*` |
| 14 | **Rover integration**: whether the arm later sits on the rover, and whether the rover's e-stop should then stop the arm too | separate: own topics (`/arm/...`), own e-stop, no link to the rover | `config/arm.yaml` topics |

## Decided

| Decision | Answer |
|---|---|
| Joints | Shoulder pitch and elbow pitch (one motor each, worm gearboxes, so they hold with the motors off), plus a bevel-gear differential wrist: both motors the same way = pitch, opposite ways = roll |
| End effector | An auger now, driven by the wrist roll (the bevel gear only); a gripper later (see question 9) |
| Motors | goBILDA gearmotors with built-in encoders; the same motor drivers as the rover (DIR + PWM) |
| Control mode | Open loop, no PID: PWM in proportion to the joystick. Encoders are read and their count and speed reported to the laptop, but not used (`SPEED_CONTROL 0`, `ENCODERS_WIRED 1`) |
| Electronics | 1 master ESP32 (USB to the PC) + 4 slave ESP32s, one per motor, on I2C; ESP32 DevKit V1 |
| Bus length | Short, so 100 kHz with CRC-checked frames |
| How a slave knows its joint and address | One PlatformIO environment per joint (`pio run -e slave_shoulder` and so on), built in at compile time. The master rejects a slave that reports another joint. (Chosen by Kiro, user to confirm) |
| Lost slave | The master stops all four motors and latches FAULT; each slave also stops on its own after 100 ms without the master |
| Controller layout | Left stick = shoulder, right stick = elbow, D-pad up/down = wrist pitch, D-pad left/right = wrist roll, L1 = slow, Circle = e-stop, Options = release |
| Setup now | Not on the rover; the base station runs from a laptop |
| Rover | Kept separate (own package, own firmware, own topics, own e-stop) |
| ROS interface | `sensor_msgs/JointState`: velocities on `/arm/joint_cmd`, state on `/arm/joint_states`; no custom messages |
| Package layout | `ros2_ws/src/arm_pilot_redemption` and `firmware_arm/`, separate from the rover's code; version 1.0.0 |
