# arm_pilot_redemption

Manual control of the arm, built like `rover_pilot_remastered` (see [README.md](README.md)) and kept separate from it. The arm has **four goBILDA gearmotors**: shoulder pitch and elbow pitch (each through a worm gearbox), and two motors driving a **bevel-gear differential wrist**. Right now an auger hangs on the wrist; a gripper comes later. **One master ESP32** talks to the PC over USB, and **four slave ESP32s** (one per motor) sit on its I2C bus.

```text
BASE STATION LAPTOP                                ARM COMPUTER                                    ARM MASTER ESP32      4 SLAVE ESP32s
PS5 ─▶ joy_node ─/joy─▶ arm_teleop ─┬─ /arm/joint_cmd (20 Hz) ─▶ arm_driver ─USB serial─▶ firmware ─I2C 50 Hz─▶ motor driver + encoder
                                    └─ /arm/estop (10 Hz) ─────▶   │   ARM_COMMAND 50 Hz         (shoulder, elbow,
                                                                   │   ◀── ARM_STATUS 20 Hz       wrist_a, wrist_b)
                                         /arm/joint_states ◀───────┘   ◀── ARM_INFO 1 Hz
```

The base station and the arm computer can be the same laptop: run both launch files on it. Open decisions (gear ratios, I2C pins and addresses, and so on) are listed in [ARM_OPEN_QUESTIONS.md](ARM_OPEN_QUESTIONS.md).

## Features carried over from the rover

- A latched e-stop with a 10 Hz heartbeat, joystick-lost detection, a deadzone and slow mode.
- The firmware command timeout (300 ms) with a 50 Hz resend, and the neutral interlock after every stop.
- COBS + CRC-16 framing, a fixed udev name, a 1 s loop watchdog and a status LED.
- `/diagnostics`, a simulator, a bench test tool, a dress-rehearsal script, and shared C++/Python test vectors.

The arm packet types (`0x83`, `0x05`, `0x06`) differ from the rover's, so a rover drive ESP32 and the arm master never act on each other's commands.

New for the arm:
- **Open-loop control with encoder readout (the current default).** Each slave sets PWM in proportion to the command, as on the rover, with no PID. It still reads its goBILDA encoder and reports the count and speed, but never acts on them. You can see the values in `/arm/joint_states`, `/diagnostics` (`position_counts`, `rpm`), `tools/arm_joint_test.py` and each slave's USB printout. PI speed control and the encoder fault checks are built in but switched off (`SPEED_CONTROL`, `ENCODER_FAULT_CHECKS` in `config.h`).
- **Bevel-gear kinematics.** Turning both wrist motors the same way pitches the wrist. Turning them opposite ways rolls it, which drives the auger. If both commands together would exceed a motor's top speed, both wrist motors are scaled down together, so the wrist keeps moving in the commanded direction.
- **Master/slave safety.** Nothing moves unless all four slaves answer and are healthy. A lost, hung, misaddressed or faulted slave stops **all four** motors, because with a differential wrist one motor stopping alone would move the wrist the wrong way.
- **Stall and wrong-direction detection (off by default, `ENCODER_FAULT_CHECKS 1` to enable).** When it is on, a motor driving hard against a hard stop, a jammed auger or a dead encoder is stopped after 1 s. A motor whose encoder counts against its PWM (reversed wiring) is stopped after 0.3 s. Turn this check on before enabling speed control: with a reversed encoder, speed control would otherwise run the motor away.

## Controls (PS5)

| Control | Action |
|---|---|
| Left stick up / down | Shoulder up / down (proportional) |
| Right stick up / down | Elbow up / down (proportional) |
| D-pad up / down | Wrist pitch up / down |
| D-pad right / left | Wrist roll: auger drilling way / back |
| **L1** (hold) | Slow mode: 35 % |
| **Circle** | **E-stop**: latches immediately, every motor stops |
| **Options** | Release the e-stop: only works with **both sticks and the D-pad centred** |

The e-stop starts **latched**. After launching, centre everything and press Options. These are the default indices of the usual Linux `joy_node` layout. **Check them on your PS5** (setup step 3): the rover's `drive.yaml` found different indices on its controller.

## Safety behaviour

| Situation | What happens | Time to stop | To move again |
|---|---|---|---|
| Circle pressed | E-stop latched at the base station, sent to the arm | next command + one bus cycle (measured 0.04–0.06 s in the rehearsal) | Options with inputs centred |
| PS5 unplugged / `joy` dies | `arm_teleop` latches the e-stop | ≤ 0.6 s | reconnect, Options |
| `arm_teleop` stopped with Ctrl-C | It sends a final e-stop | as Circle | restart, Options |
| Base station dies or link lost | Heartbeat goes stale; `arm_driver` sends the e-stop flag | about 0.5 s | link back, Options |
| `arm_driver` killed, arm computer sleeps, USB unplugged | No commands reach the master; master timeout | ≤ 0.3 s | restart; the master waits for centred inputs |
| Master resets or hangs | The slaves hear nothing; each stops its own motor (a hung master is then reset by its watchdog within 1 s) | 0.1 s | the master waits for centred inputs |
| A slave stops answering (wire, power, crash) | Master: FAULT, every motor stops, the log names the slave | ≤ 0.1 s + one bus cycle | fix it, then **Circle, then Options** |
| A slave stalls or turns against its PWM (only with `ENCODER_FAULT_CHECKS 1`; off by default) | Slave: FAULT; master: FAULT, every motor stops | stall 1 s, direction 0.3 s | **Circle, then Options** |
| A slave built for the wrong joint | The master will not enable (`WAIT_SLAVES`); the log names it | does not start | reflash that board |
| Commands resume with an input pushed | Stays stopped (`WAIT_NEUTRAL`) | — | centre the inputs |
| Kill switch | Motor power cut | immediate | switch on, then as above |

The pitch joints have worm gearboxes, so they hold their position with the motors off. A stop simply cuts the PWM.

`FAULT` is latched on purpose. When the cause is gone, the operator must acknowledge it with **Circle then Options**, so a slave that comes back cannot restart the arm by itself. Status LEDs on every board show the state: **solid** = enabled, **5 Hz** = e-stop, **10 Hz** = fault, **1 Hz** = waiting.

## Wiring

- **I2C bus:** connect SDA (GPIO 21) and SCL (GPIO 22) on all five boards, plus a **common ground**. Fit **one** pair of pull-ups (2.2–4.7 kΩ to 3.3 V) for the whole bus. Keep the bus short and twist SDA/SCL with ground through the arm. Every frame carries a CRC, so a noisy bus is detected and does not move anything wrongly, but it can cause faults.
- **Each slave:** motor driver DIR on GPIO 25 and PWM on GPIO 26 (the same driver type as the rover). Encoder A/B go to GPIO 32/33. **Power the goBILDA encoder from 3.3 V or level-shift A/B**: ESP32 inputs are not 5 V tolerant.
- **Master:** USB to the arm computer, plus I2C. It drives no motor.

All of these are placeholders in `firmware_arm/include/config.h` (see [ARM_OPEN_QUESTIONS.md](ARM_OPEN_QUESTIONS.md)).

## Layout

```text
firmware_arm/                      PlatformIO project: master + 4 slaves (DOIT ESP32 DevKit V1, Arduino)
  include/config.h                 pins, I2C addresses, joint table (direction, encoder, max rpm), timeouts, tuning
  lib/arm_core/                    protocols, master and joint state machines, speed control (plain C++, unit-tested)
  src/master/main.cpp              master: USB serial, I2C bus master, LED, watchdog
  src/slave/main.cpp               slave: I2C slave, encoder, motor PWM, LED, watchdog, bench printout
  test/test_native/                unit tests: pio test -e native
ros2_ws/src/arm_pilot_redemption/  ROS 2 Humble package (Python)
  arm_pilot_redemption/            arm_teleop, arm_driver, fake_arm (simulator), protocol, kinematics, logic
  config/arm.yaml                  shared parameters (both machines)
  launch/                          base_station.launch.py, arm.launch.py
  test/                            pytest (protocol, kinematics, e-stop logic, end-to-end, ROS integration)
tools/arm_gen_test_vectors.py      byte-exact vectors shared by the C++ and Python tests
tools/arm_joint_test.py            ROS-free motor identification test for the bench
tools/arm_rehearsal.py             dress rehearsal of the bench test against the simulated arm and PS5
udev/99-arm-master.rules           stable /dev/arm-master name for the master
```

## Setup

### 1. Flash the firmware

From `firmware_arm/`, with each board plugged in by USB in turn:

```bash
pio test -e native                    # optional: unit tests on your computer
pio run -e master -t upload           # the master
pio run -e slave_shoulder -t upload   # each slave gets its own environment: joint + I2C address are built in
pio run -e slave_elbow -t upload
pio run -e slave_wrist_a -t upload
pio run -e slave_wrist_b -t upload
```

**Label each slave board** with its joint. If a board ends up in the wrong place, the master refuses to enable and names it.

A slave's USB port prints one line twice a second, for example `shoulder 0x10 fw 1.1.0  ENABLED  pwm  51  23.4 rpm  pos  1234  bad 0`. Open it with `pio device monitor -e slave_shoulder` to check that board on the bench. The master's serial line is binary: use `tools/arm_joint_test.py` or the driver's logs instead.

### 2. Arm computer (Ubuntu 22.04 + ROS 2 Humble)

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-diagnostic-msgs python3-serial
cd ~/rover_pilot_remastered/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

To give the master a stable port name, do this once with only the master plugged in:

```bash
udevadm info -a -n /dev/ttyUSB0 | grep '{serial}'      # copy the value into udev/99-arm-master.rules
sudo cp ~/rover_pilot_remastered/udev/99-arm-master.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger && ls -l /dev/arm-master
sudo usermod -aG dialout $USER                          # then log out and in
```

Until the rule is installed, pass `port:=/dev/ttyUSB0` to the launch file.

### 3. Base station laptop

Install `ros-humble-joy` and build the same workspace. Then check the controller mapping once:

```bash
ros2 run joy joy_node &          # then: ros2 topic echo /joy
```

- Find the `axes` index that goes to **+1.0** when you push each input up: the left stick (`shoulder_pitch_axis`), the right stick (`elbow_pitch_axis`) and the D-pad (`wrist_pitch_axis`).
- Push the D-pad **left**. The index that goes to +1.0 is `wrist_roll_axis`. Keep `wrist_roll_invert: true` so that D-pad right = auger drilling way.
- Press Circle, Options and L1, and note their `buttons` indices (`estop_button`, `release_button`, `slow_button`).
- If your D-pad shows up as buttons rather than axes, set the joint's `*_axis: -1` and use `*_positive_button` / `*_negative_button` instead.

Put the indices in `config/arm.yaml` (`arm_teleop` section), then rebuild on every machine.

### 4. Every terminal

Use the same `~/.bashrc` lines as the rover (`source` both setups, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, the same `ROS_DOMAIN_ID`). **Rebuild after every change** (`colcon build --symlink-install`): `ros2 launch` runs the installed copy.

## Daily workflow

1. Power on the arm (slaves and master).
2. Base station: `ros2 launch arm_pilot_redemption base_station.launch.py` (add `device_id:=1` if the rover's PS5 is also connected).
3. Arm computer: `ros2 launch arm_pilot_redemption arm.launch.py`.
4. Centre everything and press **Options**.
5. Stop with **Circle**; in an emergency, the kill switch.

The arm log shows every state change and every slave problem by name, for example `arm motors: wrist_b: not answering on I2C`. Run `ros2 topic echo /diagnostics` to see the master, and each motor's state, PWM, rpm, encoder count, faults and I2C error count.

## First test on the bench (arm clear of people and obstacles!)

1. **Motor identity, direction and encoders**, without ROS (stop `arm_driver` first):
   ```bash
   python3 ~/rover_pilot_remastered/tools/arm_joint_test.py /dev/arm-master
   ```
   It waits until the master and all four slaves are ready, then turns each motor positive and back at 20 % for 1 s. It then pitches the wrist up and back, and rolls it positive and back. Every line shows the firmware version (it must say **fw 1.1.0**) and the PWM, rpm and encoder count of each motor. Check each step:
   - **The named motor moves.** If another motor moves, that board got the wrong environment, or its address is wrong.
   - **Positive is the positive direction.** Shoulder and elbow should go up. Wrist pitch should go up. Wrist roll should turn the auger its drilling way. If one is reversed, flip that joint's `forward_level` in `config.h`.
   - **The count rises while the PWM is positive.** If it falls, set `encoder_reversed`. With the encoder reversed and `ENCODER_FAULT_CHECKS 1`, the slave stops itself after 0.3 s with "turning against its PWM".
   - **The rpm is close to `max_rpm` × 20 %.**
   - **Wrist pitch and roll are not swapped.** If "pitch" rolls the wrist, one wrist motor's `forward_level` is wrong.
2. **Joystick:** each input moves its joint the right way, and L1 slows everything.
3. **Stops**, each within the time in the safety table:
   - Circle.
   - Options with a stick pushed (must be refused).
   - Unplugging the PS5.
   - Ctrl-C on each launch.
   - Unplugging the master's USB.
   - Unplugging one slave's I2C wires (FAULT naming it; then Circle, Options).
   - Pressing the master's EN button with a stick pushed (must not move until centred).
4. Watch `/diagnostics`: `rx_bad` and each motor's `i2c_errors` should stay at, or near, 0.

## Configuration

`config/arm.yaml` holds the gearing (shared by both nodes), the joystick mapping and the driver settings. `firmware_arm/include/config.h` holds pins, addresses, the joint table and tuning.

| Setting | Default | Meaning |
|---|---|---|
| `motor_max_rpm` | 117.0 ×4 | Gearmotor output rpm at full PWM. **Must equal `max_rpm` in `kJoints`**; the driver warns if not |
| `shoulder_ratio`, `elbow_ratio` | 1.0 | Gearmotor output turns per joint turn (the worm gearbox and anything after it) |
| `wrist_pitch_ratio`, `wrist_roll_ratio` | 1.0 | Turns of each bevel motor per wrist pitch turn / per roll turn |
| `<joint>_max_velocity` | auto | rad/s at full input; auto = the joint's motors at full PWM |
| `<joint>_axis` / `_invert` / `_positive_button` / `_negative_button` | see yaml | Joystick mapping |
| `deadzone`, `slow_scale` | 0.08, 0.35 | Stick deadzone; slow-mode factor |
| `cmd_timeout_s` / `estop_timeout_s` | 0.3 / 0.5 | Arm side: stale joint command → zero; stale heartbeat → e-stop |
| `kJoints` (config.h) | placeholders | Per joint: I2C address, `forward_level`, `encoder_reversed`, `max_rpm`, `counts_per_rev`, `encoder_fitted`, `closed_loop` |
| `SPEED_KP`, `SPEED_KI` | 0.5, 2.0 | Speed loop, as a PWM fraction per unit of speed error / max rpm (per second for KI) |
| `SPEED_CONTROL`, `ENCODERS_WIRED`, `ENCODER_FAULT_CHECKS` | 0, 1, 0 | Control mode: open loop, encoders read and reported, no action on them |
| `STALL_*`, `REVERSE_*` | 70 % / 5 % / 1 s, 30 % / 20 % / 0.3 s | Stall and wrong-direction detection (when `ENCODER_FAULT_CHECKS 1`) |

The gear ratios only change the units of `/arm/joint_cmd` and `/arm/joint_states`. Full stick always means the motors at full PWM, unless `<joint>_max_velocity` is set. In open loop, `max_rpm` only scales the command to PWM: full stick is PWM 255 whatever the motor really does, as long as `max_rpm` matches `motor_max_rpm`. The switches at the top of `config.h` set the mode for every joint; the last two fields of each `kJoints` entry (`encoder_fitted`, `closed_loop`) can override it per joint. With `ENCODERS_WIRED 0`, slaves report 0 counts per rev and the driver stops publishing `/arm/joint_states`.

## Testing without hardware

```bash
ros2 run arm_pilot_redemption fake_arm --link /tmp/arm-master      # simulated master + 4 slaves (real I2C frames)
ros2 launch arm_pilot_redemption arm.launch.py port:=/tmp/arm-master
ros2 launch arm_pilot_redemption base_station.launch.py

python3 tools/arm_rehearsal.py               # the whole bench test, every stop timed (ROS sourced, workspace built)

cd ros2_ws/src/arm_pilot_redemption && python3 -m pytest test   # protocol, kinematics, logic, end-to-end, ROS integration
cd firmware_arm && pio test -e native                           # firmware state machines, speed control, protocols
python3 tools/arm_gen_test_vectors.py --check                   # C++ and Python vectors agree
```

Add `-p no:launch_testing -p no:launch_ros` to pytest if ROS's own pytest plugins complain.

## Protocols

**USB serial (PC ↔ master):** the rover's framing, `version | type | seq | payload | CRC-16/CCITT-FALSE`, COBS-encoded, `0x00`-terminated, 115200 8N1, little-endian. Motor order everywhere: shoulder, elbow, wrist_a, wrist_b.

| Type | Direction | Rate | Payload |
|---|---|---|---|
| `0x83` ARM_COMMAND | PC → master | 50 Hz | `flags` u8 (bit 0 = e-stop) + 4 × int16 gearmotor targets in 0.1 rpm |
| `0x05` ARM_STATUS | master → PC | 20 Hz | master state, flags (bit 0 = slaves ready), command age, rx good/bad, uptime, reset reason, version; per motor: slave state, slave faults, link faults, PWM, speed (0.1 rpm), encoder count (int32), I2C errors |
| `0x06` ARM_INFO | master → PC | 1 Hz | per motor: max rpm ×10, counts per rev ×100, slave firmware version, slave reset reason |

**I2C (master ↔ each slave), every 20 ms:**
- The master writes an 8-byte command: version, `0x10`, seq, flags, target in 0.1 rpm, CRC-16.
- It then reads a 30-byte status: version, `0x11`, counter, state, faults, joint, PWM, speed, count, rx_bad, command age, max rpm, counts per rev, version, reset reason, CRC-16.
- The 30-byte status fits the ESP32's 32-byte slave FIFO. An ESP32 slave cannot stretch the clock, so each answer is the snapshot it prepared after the previous read, at most one cycle old.
- The counter advances every 10 ms, so a slave whose loop hangs but whose I2C still answers counts as lost.

States: `BOOT`, `NO_COMMAND`, `ESTOP`, `WAIT_NEUTRAL`, `ENABLED`, `FAULT`, `WAIT_SLAVES` (master only). The slaves get the e-stop flag whenever the master is not `ENABLED`, and zero targets until all four have followed it into `ENABLED`, so the two wrist motors always start together.

## Known limits

- **No homing.** Joint positions in `/arm/joint_states` count from where each slave powered up, and there are no soft joint limits. With the encoder checks off (the default), nothing in software stops a motor driving into a hard stop: the operator, the e-stop and the kill switch do. Keep speeds low near the ends of travel.
- Open loop: speed varies with load and battery voltage. If speed control is turned on later, its gains are untuned defaults. Tune them on the real arm (see [ARM_OPEN_QUESTIONS.md](ARM_OPEN_QUESTIONS.md)).
- The auger is on the D-pad, so it runs at a fixed speed (full, or 35 % with L1). Proportional control would need another input.
- The software e-stop depends on the laptops and the link; it does not replace the physical kill switch.
