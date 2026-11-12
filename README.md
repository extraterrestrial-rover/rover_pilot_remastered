# rover_pilot_remastered

Safe drive control for the current rover: **one ESP32 driving four wheel motors**, a PS5 controller at the base station, ROS 2 Humble over the MikroTik link. It replaces the old `rover_pilot` drive path (`drive_node` + `ctrl.py` + the single-ESP32 firmware).

```text
BASE STATION LAPTOP                                    ROVER LAPTOP (192.168.1.68)                     DRIVE ESP32
PS5 ─▶ joy/joy_node ─/joy─▶ base_teleop ─┬─ /cmd_vel (20 Hz) ──────▶ drive_node ───link───▶ firmware ─▶ 4 motor drivers
                                                      └─ /rover/estop (10 Hz) ──▶            DRIVE_COMMAND 50 Hz
                                                                                             ◀── DRIVE_STATUS 10 Hz
```

`/cmd_vel` is the standard ROS velocity topic (m/s, rad/s). On this branch (`visual-odom-serial`) Nav2 can drive the
rover through the same path: **Triangle** switches to AUTO and RViz goals take over ([AUTONOMY.md](AUTONOMY.md)).
The link between the rover laptop and the ESP32 depends on the branch (below); [LINK.md](LINK.md) describes this
branch's.

## Branches

| Branch | Link to the drive ESP32 | Firmware |
|---|---|---|
| `main` | the ESP32's USB port, serial at 115200 baud | 2.0.x |
| `ethernet-udp` | a W5500 Ethernet module on the ESP32, UDP to a dedicated Ethernet port of the rover laptop (the ESP32 is still powered over USB) | 3.0.x |
| `visual-odom-serial` | as `main`, plus autonomous navigation: ZED 2i visual odometry, RTAB-Map, Nav2 ([AUTONOMY.md](AUTONOMY.md)) | 2.0.x |

Everything else is the same on both: the controls, the safety rules, `base_teleop`, `/cmd_vel`, the configuration.

**Switching.** Run `tools/install_hooks.sh` once in each clone (rover laptop, base station, your computer). From then
on every `git switch main` / `git switch ethernet-udp`, and every `git pull`, rebuilds `ros2_ws` by itself and prints
what the branch needs (LINK.md, "On this branch"). Then flash the matching firmware (Setup, step 1) and launch as
usual. The base station's code is the same on both branches, so it never needs to switch.

**Changing code.** Anything that is not about the link goes on `main` first and is then merged into `ethernet-udp`
(`git switch ethernet-udp && git merge main`). Link code lives in its own files (LINK.md, "Files"), so these merges
stay clean.

## What was wrong before, and what this fixes

| Old behaviour | Now |
|---|---|
| The ESP32 kept the last command forever: if `drive_node` died or the radio link dropped, the rover kept driving at the last PWM | The firmware stops the motors when no valid command has arrived for **300 ms**. `drive_node` sends 50 times a second whether or not the joystick moves |
| The firmware read 2 bytes after seeing 1; a missing byte read as −1 → **255 (full speed)**, and one lost byte swapped command and speed forever | Commands are framed (COBS + CRC-16); anything incomplete or corrupted is dropped and can never become a speed |
| `find_port()` opened every USB serial port to look for the text "drive", which can reset ESP32s | The ESP32 has a fixed address (a udev name, or a static IP on `ethernet-udp`); nothing scans ports |
| 5 discrete commands, and turning overrode driving (no arcs), no deadzone | Proportional arcade drive: speed and turn combine; deadzone; slow mode |
| No software stop | Latched joystick e-stop with a heartbeat; see below. The physical kill switch is still the hard stop |
| The firmware printed text on the same serial line; its control loop never yielded (watchdog warnings) | Only protocol packets on the line; non-blocking loop with a watchdog |

## Controls (PS5)

| Control | Action |
|---|---|
| Left stick up / down | Forward / backward (proportional) |
| Right stick left / right | Turn left / right (proportional; combines with the left stick for arcs; alone it spins in place) |
| **L1** (hold) | Slow mode: 35 % of normal speed |
| **Circle** | **E-stop**: latches immediately, the rover stops |
| **Options** | Release the e-stop: only works with **both sticks centred** |
| **Triangle** | AUTO on/off (`visual-odom-serial`): Nav2 drives to RViz goals; a stick push or Circle takes over ([AUTONOMY.md](AUTONOMY.md)) |

The e-stop starts **latched**. After launching, centre the sticks and press Options to start driving.

**Connect the PS5 by USB cable at the base station.** The "controller lost" stop below relies on `joy` noticing the disconnect, which a USB unplug triggers at once. Over Bluetooth, a controller whose battery dies or that goes out of range can stay "connected" for several seconds while `joy` keeps repeating the last stick position, so the rover keeps moving until Linux gives up on it. If you must use Bluetooth, keep the controller charged and close to the base station, and remember Circle and the kill switch.

## Safety behaviour

| Situation | What happens | Time to stop | To drive again |
|---|---|---|---|
| Circle pressed | E-stop latched at the base station, sent to the rover | next command (≤ 20 ms) | Options with sticks centred |
| PS5 unplugged (USB) / `joy` node dies | `base_teleop` latches the e-stop | ≤ 0.6 s (0.5 s joystick timeout + one tick) | reconnect, Options with sticks centred |
| `base_teleop` stopped with Ctrl-C | It sends a final e-stop before exiting | next command (≤ 20 ms) | restart, Options with sticks centred |
| Base station laptop dies or **radio link lost** | E-stop heartbeat goes stale on the rover; `drive_node` sends the e-stop flag | about 0.5 s | link back, Options with sticks centred |
| The same **in AUTO while a goal runs** (`visual-odom-serial`) | The rover goes on to its goal and stops there; then the e-stop is latched until the base station is back. Only the kill switch and the rover's own checks stop it on the way (AUTONOMY.md) | at the goal | link back |
| `drive_node` killed / crashes, rover laptop sleeps | No commands reach the ESP32; firmware timeout | ≤ 0.3 s | restart `drive_node`; the ESP32 waits for centred sticks |
| Link cable unplugged (USB; Ethernet on `ethernet-udp`) | Same as above | ≤ 0.3 s | replug; the ESP32 waits for centred sticks |
| ESP32 resets or powers up | Motors off at boot | immediate | the ESP32 waits for centred sticks |
| ESP32 firmware hangs | The loop watchdog resets it; motors off at boot | ≤ 1 s | the ESP32 waits for centred sticks |
| Commands resume while a stick is pushed | The ESP32 stays stopped (`WAIT_NEUTRAL`) | — | centre the sticks |
| Kill switch | Battery cut | immediate | switch on, then as above |

The rule behind "centred sticks": after any stop, the ESP32 will not move again until it has received an all-zero command. This prevents a lurch when the link comes back with a stick still pushed. The status LED shows the state: **solid** = driving, **fast blink** = e-stop, **slow blink** = waiting for commands or centred sticks.

Kill-switch note: on the Windows/Ubuntu laptop the ESP32 is powered from the laptop's USB, so it stays on when the kill switch cuts the battery. Its motors lose power, but it keeps following commands. **Centre the sticks (or press Circle) before turning the kill switch back on**, or the wheels will resume whatever the stick says. On the Pi 4 / Jetson everything powers off together and the boot interlock covers it.

## Layout

```text
firmware/                      PlatformIO project for the drive ESP32 (DOIT ESP32 DevKit V1, Arduino)
  include/config.h             pins, directions, timeout, speed calibration, slew rate
  lib/drive_core/              protocol + safety state machine (plain C++, unit-tested on the PC)
  src/main.cpp                 ESP32 glue: link, PWM, LED, watchdog
  src/profiler.cpp             CPU, loop-time, heap and stack measurement (profiling build only: pio run -e profile)
  test/test_native/            unit tests: pio test -e native
ros2_ws/src/rover_pilot_remastered/   ROS 2 Humble package (Python)
  rover_pilot_remastered/      base_teleop, drive_node, fake_esp (simulator), protocol, kinematics, logic
  config/drive.yaml            shared parameters (both machines)
  config/wheels/*.yaml         one file per wheel type
  launch/                      base_station.launch.py, rover.launch.py
  test/                        pytest (protocol, kinematics, e-stop logic, end-to-end, ROS integration)
ros2_ws/src/rover_autonomy/    autonomy (AUTONOMY.md): drive_mux, goal_manager, Nav2/EKF/RTAB-Map configs, URDF, launch
tools/gen_test_vectors.py      generates byte-exact test vectors shared by the C++ and Python tests
tools/wheel_test.py            ROS-free wheel identification test for the bench
tools/rehearsal.py             dress rehearsal of the bench test against the simulated ESP32 and PS5
tools/rehearsal_auto.py        dress rehearsal of AUTO mode: real Nav2 against a simulated rover (AUTONOMY.md)
tools/profile.py               measures the ESP32's CPU load and memory under worst-case commands (profiling build)
tools/install_hooks.sh         once per clone: rebuild ros2_ws after every branch switch and pull (tools/git-hooks/)
LINK.md                        this branch's link: wiring, rover laptop setup, launch arguments, simulator, its files
AUTONOMY.md                    AUTO mode: controls, install, measurements, launch, maps, rehearsal
ROVER_LAPTOP.md                a new rover laptop from a blank Ubuntu 22.04: NVIDIA, ZED, ROS, network, tmux
```

## Setup

### 1. Flash the firmware

With PlatformIO (VS Code extension or CLI), from `firmware/`:

```bash
pio test -e native                          # optional: unit tests on your computer
pio run -e esp32doit-devkit-v1 -t upload    # build and flash over USB
```

Settings are in `include/config.h`: pins and directions are copied from the old firmware, and the defaults are a 300 ms timeout, `MAX_WHEEL_RPM` 94.7, a 250 ms ramp to full speed and a 1 s loop watchdog. Which Arduino-ESP32 cores it builds on, and the link's own settings, are in LINK.md.

GPIO 12 (left-back direction) is an ESP32 **strapping pin**. If the board ever fails to boot with the motor drivers connected, move that signal to a free non-strapping GPIO and update `config.h`.

### 2. Rover laptop (Ubuntu 22.04 + ROS 2 Humble)

A new laptop from a blank Ubuntu, with the ZED and the autonomy stack on `visual-odom-serial`: [ROVER_LAPTOP.md](ROVER_LAPTOP.md).

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-diagnostic-msgs python3-serial
cp -r rover_pilot_remastered ~/   # or git clone
cd ~/rover_pilot_remastered/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

Then `cd ~/rover_pilot_remastered && tools/install_hooks.sh`, and set up the link to the ESP32 once: LINK.md, "Rover laptop".

### 3. Base station laptop (Ubuntu 22.04 + ROS 2 Humble)

```bash
sudo apt install ros-humble-joy ros-humble-rmw-cyclonedds-cpp
# copy or clone the folder, then build the same package:
cd ~/rover_pilot_remastered/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

Connect the PS5 controller (USB, or Bluetooth pairing in Ubuntu settings). Check the mapping once:

```bash
ros2 run joy joy_node &          # then: ros2 topic echo /joy
```

- Push the left stick **up**: the `axes` index that goes to **+1.0** is `axis_linear`.
- Push the right stick **left**: the `axes` index that goes to **+1.0** is `axis_angular`.
- Press the buttons you want for e-stop, release and slow mode: their `buttons` indices are `estop_button`, `release_button`, `slow_button`.

Put them in `config/drive.yaml` (base_teleop section; `invert_*` if a direction is reversed), commit and push, then pull on the base station (with the hooks installed, the pull rebuilds; otherwise rebuild by hand).

### 4. Every terminal on both machines

Add to `~/.bashrc` on the base station **and** the rover (same values everywhere):

```bash
source /opt/ros/humble/setup.bash
source ~/rover_pilot_remastered/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=<your team's domain id>
```

Check the link: run `ros2 topic list` on one machine while a node runs on the other.

**Rebuild after every change.** `ros2 launch` runs the installed copy of the package. With some setuptools versions `--symlink-install` still copies the files, so an edited `config/drive.yaml`, a wheel profile or updated code takes effect only after `cd ~/rover_pilot_remastered/ros2_ws && colcon build --symlink-install` (about a second) and a relaunch. The git hooks (`tools/install_hooks.sh`) do this after every branch switch and pull, but not after your own edits. `python3 tools/rehearsal.py` checks it first.

## Daily workflow

1. Kill switch on; the rover laptop boots.
2. Base station: `ros2 launch rover_pilot_remastered base_station.launch.py`
3. `ssh` to the rover (192.168.1.68): `ros2 launch rover_pilot_remastered rover.launch.py`
4. Centre both sticks and press **Options**. Drive.
5. Stop with **Circle**; in an emergency, the kill switch.

Launch arguments:
- `wheels:=wheel_28cm` (default) or `wheels:=wheel_other`, on **both** machines;
- the link's own arguments on the rover (LINK.md), for example to reach the ESP32 at another address;
- `device_id:=1` at the base station if several controllers are connected.

The rover log shows every state change (`stopped: e-stop`, `driving enabled`, `ESP32 state: DRIVING`). `ros2 topic echo /diagnostics` shows the ESP32 status, PWM per wheel and link counters.

## First test on the bench (wheels off the ground!)

1. **Wheel identity and direction**, without ROS (stop `drive_node` first):
   ```bash
   python3 ~/rover_pilot_remastered/tools/wheel_test.py
   ```
   It reaches the ESP32 at this branch's address (LINK.md) and sends neutral until the ESP32 is ready (opening a USB port may reset it), then spins FL, FR, RL and RR forward one at a time, then all forward, then a right turn in place. Each line shows the firmware version: it must be **this branch's** (LINK.md, "On this branch"), otherwise the right firmware is not flashed. Check that each named wheel turns **forward**. If one is wrong, fix its entry (pins or `forward_level`) in `firmware/include/config.h` and reflash.

   **Expect left and right to be swapped.** The pin names in `config.h` come from the old firmware, whose "left turn" drove the `leftFront`/`leftBack` pins forward. If the old rover really turned left that way, those pins drive the **right** wheels. If "FL forward only" spins the front-right wheel, swap the left and right entries in `kDriveMotors` (FL↔FR, RL↔RR) and reflash, rather than using `swap_sides`, so FL/FR/RL/RR mean the same everywhere.
2. **Joystick:**
   - Left stick up = forward.
   - Right stick right = rover turns **right**. If it turns left while forward is correct, set `swap_sides: true` in `config/drive.yaml` (left/right motors wired the other way round). Don't invert the joystick axis: `/cmd_vel` must stay correct for Nav2.
   - L1 = slower.
3. **Stops**; each must stop the wheels within the time in the safety table:
   - Circle (the rover stops within about 0.1 s);
   - pressing Options with a stick pushed (must be refused);
   - unplugging the PS5;
   - `Ctrl-C` on the base station launch;
   - `Ctrl-C` on the rover launch;
   - unplugging the link cable (USB, or the Ethernet cable on `ethernet-udp`);
   - pressing the ESP32's EN (reset) button while a stick is pushed (must not move until centred).
4. Watch `/diagnostics`: `esp_rx_bad` should stay at 0.

## Configuration

`config/drive.yaml` (both machines) and `config/wheels/*.yaml`. Use decimal points for numbers: ROS 2 parameters are typed. After editing, rebuild (`colcon build --symlink-install`) on that machine and relaunch.

| Parameter | Default | Meaning |
|---|---|---|
| `track_width_m` | 0.84 | Distance between left and right wheel centres |
| `max_wheel_rpm` | 94.7 | Wheel rpm at full PWM. **Must equal `MAX_WHEEL_RPM` in the firmware**; the drive node warns if not. 94.7 rpm on a 0.28 m wheel is 5 km/h |
| `turn_track_factor` | 1.0 | Raise above 1 if the rover turns less than commanded (skid-steer slip). Only matters for Nav2 accuracy |
| `wheel_diameter_m` / `wheel_width_m` | 0.28 / 0.14 | Per wheel type, in `config/wheels/` |
| `max_linear_speed` / `max_angular_speed` | auto | Full-stick speed; auto = rover top speed / spin in place at full PWM |
| `deadzone`, `slow_scale` | 0.08, 0.35 | Stick deadzone; slow-mode factor |
| `estop_button`, `release_button`, `slow_button` | 3, 1, 4 | Circle, Options, L1, as `joy_node` numbers this PS5 |
| `auto_button` | 2 | Triangle: AUTO on/off (AUTONOMY.md); -1 disables AUTO |
| `cmd_timeout_s` / `estop_timeout_s` | 0.3 / 0.5 | Rover side: stale `/cmd_vel` → zero; stale e-stop heartbeat → e-stop |
| `require_estop_heartbeat` | true | Set false only for autonomous runs that must continue without the base station link |
| `swap_sides` | false | Swap left and right (wiring fix) |
| link settings | | LINK.md, "Launch arguments" |

**Second wheel type:** measure its diameter, put it in `config/wheels/wheel_other.yaml`, and launch both sides with `wheels:=wheel_other`. The firmware doesn't change: max wheel rpm is a motor property. The top speed follows from the new diameter.

## Testing without hardware

The simulated ESP32 (`fake_esp`) runs on this branch's link; LINK.md, "Simulator", shows how to start it and point the
rover launch at it, then `ros2 launch rover_pilot_remastered base_station.launch.py` as usual.

```bash
# Dress rehearsal of the whole bench test: real launch files, simulated ESP32 and PS5, every stop timed (about 1 min)
python3 tools/rehearsal.py                                          # ROS sourced and the workspace built; exit 1 on any FAIL
python3 tools/rehearsal_auto.py                                     # AUTO mode with the real Nav2 (needs Nav2 installed; about 6 min)

# Test suites
cd ros2_ws/src/rover_pilot_remastered && python3 -m pytest test   # protocol, kinematics, e-stop logic, end-to-end, ROS integration
cd ros2_ws/src/rover_autonomy && python3 -m pytest test           # AUTO rules, mux, goal manager, odometry checks, Nav2 parameters, launch
cd firmware && pio test -e native                                  # firmware state machine and protocol
python3 tools/gen_test_vectors.py --check                          # C++ and Python vectors agree
```

On a machine with ROS sourced, add `-p no:launch_testing -p no:launch_ros` to pytest if ROS's own pytest plugins complain.

The rehearsal and the ROS tests release the e-stop and drive at full speed. So they never use the `ROS_DOMAIN_ID` of
your shell: they run in ROS domains of their own (63 and 87; `--domain`, `ROVER_TEST_DOMAIN`), keep their ROS traffic
on the machine, and refuse to start if another node is already in that domain. A rover that is switched on cannot
hear them.

## Measuring CPU and memory

The profiling build is the same firmware plus a measurement module (`src/profiler.cpp`) that reports once a second in a `PROFILE` packet. The normal build (`esp32doit-devkit-v1`) compiles none of it, so its machine code is unchanged.

- **CPU load per core**: FreeRTOS counts how long each core's idle task runs; load = 1 − idle time / elapsed time, per 100 ms window. It counts everything the chip does (this firmware, the serial and PWM drivers, the watchdog, task switches), not only the drive code. **Peak** is the busiest 100 ms window. The firmware runs on core 1; core 0 runs only system tasks.
- **Loop passes**: how long each pass of `loop()` is busy, from its start to `delay(1)`. A pass starts every 1000 µs, so the longest pass shows the headroom.
- **Memory**: free heap and its lowest value since boot; the most of the loop task's 8 KB stack ever used.
- **The profiler's own time**, so it can be subtracted: the normal firmware is that much lighter.
- **GPIO 21** is high while `loop()` is busy, for a logic analyser or scope (`-DPROFILE_MARKER_PIN=-1` turns it off).

Wheels off the ground, `drive_node` stopped (only one program may talk to the ESP32):

```bash
cd firmware && pio run -e profile -t upload                    # 1. flash the profiling build
python3 tools/profile.py                                        # 2. 120 s of worst-case load, one line a second, then a summary
python3 tools/profile.py --load idle                            #    neutral commands only, for comparison
cd firmware && pio run -e esp32doit-devkit-v1 -t upload         # 3. flash the normal firmware back
```

The worst-case load is what `drive_node` sends (`DRIVE_COMMAND` at 50 Hz) with wheel targets that change on every command (`--percent`, default 30 % of full speed), so the ramp works on all four wheels on every control tick; plus 3 back-to-back commands every 2 s and garbage bytes every 5 s (the decoder's error path). `--seconds` sets the length. The summary ends with the reminder to flash the normal firmware back; `tools/wheel_test.py` then shows this branch's normal firmware version. Flash and static RAM use come from the normal build's `pio run -e esp32doit-devkit-v1` output.

## Protocol

The same packets as the rover's planned CAN-master link: `version | type | seq | payload | CRC-16/CCITT-FALSE`, COBS-encoded, `0x00`-terminated. Multi-byte fields are little-endian. How they travel (a serial byte stream, or UDP datagrams) is in LINK.md.

| Type | Direction | Rate | Payload |
|---|---|---|---|
| `0x82` DRIVE_COMMAND | PC → ESP32 | 50 Hz | `flags` u8 (bit 0 = e-stop) + 4 × int16 wheel targets FL, FR, RL, RR in 0.1 rpm (positive = forward) |
| `0x04` DRIVE_STATUS | ESP32 → PC | 10 Hz | state, command age ms, 4 × int16 applied PWM, rx good/bad, uptime ms, reset reason, `MAX_WHEEL_RPM`×10, firmware version |
| `0x05` PROFILE | ESP32 → PC | 1 Hz | profiling build only: CPU load, loop time, heap, stack (layout in `firmware/src/profiler.cpp`); `drive_node` ignores it |

States: `BOOT`, `NO_COMMAND`, `ESTOP`, `WAIT_NEUTRAL`, `DRIVING`. The firmware is open-loop: it maps target rpm to PWM using `MAX_WHEEL_RPM`, because there are no encoders yet. When the CAN master/slave firmware arrives, the same `/cmd_vel` and e-stop topics will feed the onboard bridge (`telemetry_dashboard/bridge`, `drive:` section of its config) instead of `drive_node`, and the slaves will close the loop on their encoders. The bridge sends the same `DRIVE_COMMAND` (`0x82`) bytes to the CAN master, which applies the same rules (300 ms timeout, neutral interlock, e-stop latch); the firmware contract is `telemetry_dashboard/docs/FIRMWARE_INTERFACE_SPEC.md` (v2.1 on its `ethernet-udp` branch, v1.2 on `usb-serial`). Stop `drive_node` before the bridge drives: only one of them should follow `/cmd_vel`.

## Known limits and warnings

- **`arm_node` still scans every USB serial port** (`ctrl.SerialPortChecker`). Opening the drive ESP32's port that way can reset it or disturb the link while driving: the rover stops, by design, but it is disruptive. Give the arm ESP32 its own udev name and open that directly before running `arm_node` alongside this.
- No encoders, so no wheel odometry: Nav2 runs on the ZED's visual odometry (AUTONOMY.md) until the CAN slaves report wheel speeds.
- Open-loop speed: actual speed varies with battery voltage and terrain.
- The software e-stop depends on the radio link and the laptops; it is not a replacement for the physical kill switch.
