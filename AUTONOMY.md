# Autonomy: visual odometry + Nav2 (`visual-odom-serial`)

This branch is `main` plus autonomous navigation. The rover maps its surroundings all the time, with the ZED 2i and
RTAB-Map, while you drive it with the PS5. Press **Triangle** to switch to AUTO, then click goals in RViz at the base
station, and Nav2 drives the rover to them. The drive path itself doesn't change: `drive_node`, the firmware, the
e-stop and every stop in the README's safety table work as on `main`, with one exception you chose: when the radio
drops while the rover drives to a goal, it goes on to that goal ("Situation in AUTO" below).

The stack follows the colleague's `extraterrestrial-rover/sw_Nav2_Hardware`: ZED visual-inertial odometry, a
robot_localization EKF, RTAB-Map and Nav2 with DWB. It differs where that stack doesn't fit this rover (see
"Compared with sw_Nav2_Hardware").

```text
BASE STATION                                  ROVER LAPTOP (NVIDIA GPU)
PS5 ─▶ base_teleop ─┬─ /cmd_vel ─────────────▶ drive_mux ─/cmd_vel_drive──▶ drive_node ─link─▶ ESP32
                    ├─ /rover/auto (Triangle) ─▶   ▲ ▲ ▲ ─/rover/estop_drive─▶ (the base station's e-stop, passed on)
                    └─ /rover/estop ───────────────┘ │ │
RViz "2D Goal Pose" ─/goal_pose─▶ goal_manager ──────┘ │  /rover/goal_active
                    (only in AUTO)    │ navigate_to_pose │
                                      ▼                  │
                                     Nav2 ─/cmd_vel_auto─┘
ZED 2i ─odom─▶ zed_odometry ─▶ EKF (odom→base_footprint) ─▶ RTAB-Map (map→odom, /map) ─▶ Nav2
       └─point cloud─────────────────────────────────────────────────────────▶ Nav2's costmaps
```

- **`drive_mux`** decides what reaches `drive_node`:
  - the sticks' `/cmd_vel` in manual, passed on as each command arrives;
  - Nav2's `/cmd_vel_auto` in AUTO, and only for a goal `goal_manager` sent. It sends zeros for 0.3 s first, and again
    whenever `drive_node` reports that the ESP32 is not driving, because the ESP32 drives again only after a neutral
    command. It corrects Nav2's commands for the motors' deadband ("Before the first run", step 5);
  - the base station's e-stop, passed on the moment it arrives. Only while the base station is silent does the mux
    decide the e-stop itself (table below).

  It also **stops AUTO by itself**, and the goal is cancelled, when it cannot know where the rover is: no odometry from
  the camera for 0.5 s, or a motion command for 3 s without the odometry moving (tracking lost, wheels stuck, deadband
  set too low). `ros2 topic echo /rover/auto_fault` says why.
- **`goal_manager`** passes RViz goals to Nav2, only in AUTO with the e-stop released, fixed in the map frame when
  they arrive. Nav2's own `/goal_pose` subscription is remapped away, and goals sent to Nav2 any other way are
  ignored by the mux, so a click can't bypass it. It tells the mux whether a goal runs (`/rover/goal_active`).
- **`zed_odometry`** turns the ZED's odometry, which is the camera's pose, into the rover centre's pose
  (`base_footprint`) for the EKF, using the mount in `autonomy.yaml`. Humble's robot_localization would otherwise
  take the camera's pose as the centre, and the rover's footprint would swing round with the camera when it turns.
  It passes on only valid poses while the ZED reports that it tracks, and keeps the odometry continuous: the ZED
  wrapper is started without its "reset the odometry on a loop closure" (`config/zed_override.yaml`), and a jump that
  still comes through is taken out. Such a reset made obstacles appear twice in the costmap when the rover turned.
- **Nav2** never publishes `/cmd_vel`: the controller and the recovery behaviours publish `/cmd_vel_nav`, and the
  velocity smoother turns that into `/cmd_vel_auto` for the mux. `tools/rehearsal_auto.py` checks this on the live
  graph.

## Controls

| Control | Action |
|---|---|
| **Triangle** | AUTO on and off. Turning it on needs the e-stop released and both sticks centred |
| Either stick pushed in AUTO | Back to manual at once; the goal is cancelled |
| **Circle** | E-stop, as always; it also ends AUTO and cancels the goal |
| RViz **2D Goal Pose** | A goal (taken only in AUTO). A new goal replaces the current one |

In AUTO with no goal, the rover holds still. The base station's terminal shows AUTO going on and off (and why it was
refused); `ros2 topic echo /rover/autonomy_events` at the base station shows what the rover did with each goal, for
example why one was ignored.

| Situation in AUTO | What happens | Afterwards |
|---|---|---|
| Goal reached | The rover stops; still in AUTO | click the next goal |
| Nav2 gives up (no path, stuck after its recoveries) | The rover stops; the goal is dropped | click a goal again |
| Radio or base station lost **while a goal runs** | The rover **goes on to its goal** and stops there. If the base station is still not heard then, the e-stop is latched | radio back: the e-stop is released again (still in AUTO); click the next goal |
| Radio or base station lost in manual, or in AUTO without a goal | The rover stops (about 0.5 s, as on `main`) | radio back: drive again (after a stop, centre the sticks) |
| Base station heard again after an outage, in e-stop or manual (restarted, or you pressed Circle or took the sticks) | The goal is cancelled and the rover stops | |
| No odometry, or no motion although commanded (see `drive_mux` above) | AUTO stops itself; the goal is cancelled | find the cause (`/rover/auto_fault`), then a new goal |
| Circle, a stick, or Triangle | Goal cancelled for good | |

**While a goal runs without the base station, nothing at the base station can stop the rover:** not Circle, not a
stick. The kill switch can, and the rover's own checks do (no odometry, no motion). Watch it, and keep the kill
switch in reach. `linkless_limit_s` in `autonomy.yaml` caps that stretch (0, the default, means no limit).

Recoveries: when Nav2 is stuck it clears its costmaps, turns a quarter turn on the spot and waits 5 s, up to 6
times. It never backs up, because the camera can't see behind the rover.

## Install (once)

A new rover laptop from a blank Ubuntu 22.04, step by step (driver, ZED SDK, wrapper, network, power settings,
tmux): [ROVER_LAPTOP.md](ROVER_LAPTOP.md). In short:

**Rover laptop** (Ubuntu 22.04, ROS 2 Humble, the NVIDIA GPU the ZED SDK needs):

```bash
sudo apt install ros-humble-navigation2 ros-humble-robot-localization ros-humble-rtabmap-ros ros-humble-xacro \
                 ros-humble-robot-state-publisher
```

Install the ZED SDK and build `zed-ros2-wrapper` from source (Stereolabs' instructions). Then source its workspace in
`~/.bashrc` **before** this one:

```bash
source /opt/ros/humble/setup.bash
source ~/zed_ws/install/setup.bash          # the ZED wrapper
source ~/rover_pilot_remastered/ros2_ws/install/setup.bash
```

**Base station:** `sudo apt install ros-humble-rviz2 ros-humble-nav2-msgs` (RViz usually comes with ROS desktop).

On both machines: `git switch visual-odom-serial` (the hooks rebuild `ros2_ws`; otherwise run
`colcon build --symlink-install`).

## Before the first run: measure and check

The values in `ros2_ws/src/rover_autonomy/config/autonomy.yaml` marked MEASURE or CHECK are placeholders. Nav2's
footprint, the camera's position and the deadband all come from there. Rebuild after editing.

1. **The rover's size**: `rover_length_m` and `rover_width_m` over the wheels, `rover_height_m` to the highest point.
2. **The ZED's mount**, measured from the ground point midway between the four wheels to the middle of the camera's
   underside (the tripod screw): forward `zed_x_m`, left `zed_y_m`, height `zed_z_m`, and `zed_pitch_rad` if it
   looks down (positive). Check it in RViz: with `TF` on, the point cloud of a wall must stand where the wall is.
3. **The ZED's topics**: with the camera running, `ros2 topic list | grep zed_node`. Wrapper versions name the colour
   image differently (`rgb/color/rect/image` or `rgb/image_rect_color`); copy what you see into the `zed_*_topic`
   settings.
4. **Triangle's number**: `ros2 topic echo /joy` at the base station, press Triangle; the index that goes to 1 is
   `auto_button` in `rover_pilot_remastered/config/drive.yaml` (2 on the controller used so far).
5. **Deadband** (wheels on the ground, open space, `drive_node` stopped): the motors need some power before the rover
   moves at all, much more for turning on the spot. Nav2 asks for slow speeds near goals and when turning, so
   `drive_mux` adds this offset. Find the lowest `--percent` at which the rover just rolls in the "all forward" step,
   and the lowest at which it turns in the "turn right in place" step:
   ```bash
   python3 tools/wheel_test.py --percent 20     # then 15, 25, ... (each step runs 2 s)
   ```
   Then set `min_wheel_rpm_straight` = straight percent × 94.7 / 100 and `min_wheel_rpm_spin` = spin percent ×
   94.7 / 100 (`drive_mux` section). With these, Nav2's speeds are roughly the real speeds. Both ship as 0.0: no
   correction, and `drive_mux` warns at start; the rover may then not move at Nav2's slow commands, and AUTO stops
   itself after 3 s ("no motion"). Too high is worse than too low (it drives and turns faster than Nav2 asked), and a
   value above 80 % of `max_wheel_rpm` is refused as a unit mistake. Each side of the rover gets the correction on
   its own, so arcs keep their shape.

## Daily workflow

1. Base station: `ros2 launch rover_autonomy base_station.launch.py` (the PS5 teleop and RViz; `rviz:=false`
   without RViz).
2. Rover: `ros2 launch rover_autonomy rover_autonomy.launch.py`. It starts the ZED, the EKF, RTAB-Map, Nav2, the mux,
   the goal manager and `drive_node`. It replaces `rover.launch.py`; don't run both. Link arguments work as on
   `main` (`port:=…`).
3. Options, then drive around manually: RTAB-Map builds the map, visible in RViz.
4. **Triangle**, then click **2D Goal Pose** in RViz and drag to set the heading.
5. Take over at any time with a stick, or stop with Circle.

Launch arguments (rover):

| Argument | Default | Meaning |
|---|---|---|
| `continue_map:=true` | false | keep mapping into the last map instead of starting a fresh one |
| `max_speed:=0.3` | `auto_max_speed` (0.5 m/s) | AUTO's top speed for this run |
| `wheels:=` | wheel_28cm | as on `main`; also at the base station |
| `zed:=false` | true | the ZED wrapper runs separately (start it with `publish_tf:=false publish_map_tf:=false`) |
| `slam:=false` | true | no RTAB-Map: something else must provide `/map` and `map → odom` (the rehearsal does) |
| `settings:=` | `config/autonomy.yaml` | another settings file |

**Maps.** RTAB-Map keeps its map in `~/.ros/rover_map.db` (`map_database`). A fresh run moves the previous file to
`rover_map-<date>-<time>.db`, so nothing is lost. With `continue_map:=true` it keeps mapping into the same file and
recognises the old map when the rover sees a known place again. Delete old copies when you no longer need them.

**Speed.** `auto_max_speed` in `autonomy.yaml` (0.5 m/s) is the usual setting. `max_speed:=` overrides it for one
run. Both are checked against the rover's top speed (1.39 m/s with the 28 cm wheels).

## Configuration (`rover_autonomy/config/autonomy.yaml`)

| Setting | Default | Meaning |
|---|---|---|
| `auto_max_speed`, `auto_max_turn_rate` | 0.5 m/s, 0.8 rad/s | AUTO's limits (Nav2's controller, smoother, recoveries) |
| `rover_length_m`, `rover_width_m`, `rover_height_m` | placeholders | the footprint; points higher than the rover are ignored |
| `inflation_radius_m` | 0.7 | how far Nav2 keeps the rover's centre from obstacles where it can |
| `min_obstacle_height_m` | 0.1 | lower points count as ground (raise it on rough ground if Nav2 sees obstacles everywhere) |
| `obstacle_range_m` | 3.0 | how far the depth image marks obstacles (the ZED's depth gets noisy beyond) |
| `zed_x_m` … `zed_pitch_rad` | placeholders | the camera's mount (the URDF and `zed_odometry` use it) |
| `zed_*_topic` | wrapper 5.x names | the ZED's topics, and `zed_status_topic` its tracking status (`''` = not checked) |
| `map_database` | `~/.ros/rover_map.db` | RTAB-Map's map |
| `drive_mux`: `min_wheel_rpm_straight`, `min_wheel_rpm_spin` | 0.0 (not measured) | the deadband |
| `drive_mux`: `neutral_hold_s` | 0.3 | zeros first when AUTO starts, and when the ESP32 is not driving (`rearm_interval_s` 1.5) |
| `drive_mux`: `mode_timeout_s` | 0.5 | base station silent this long: lost |
| `drive_mux`: `odometry_timeout_s`, `no_motion_timeout_s` | 0.5, 3.0 | AUTO stops itself (no odometry; no motion although commanded, 0 = not checked) |
| `drive_mux`: `linkless_limit_s` | 0.0 (no limit) | the longest a goal may run without the base station |
| `goal_manager`: `link_timeout_s` | 0.5 | base station silent this long: lost (the goal goes on) |

Everything else Nav2 uses is in `config/nav2_params.yaml` (Humble's names; the values marked "from autonomy.yaml" are
overwritten at launch). The launch prints where the final parameter file is (removed when the launch ends). What the
ZED wrapper must do differently from its defaults is in `config/zed_override.yaml`.

## Frames and topics

`map` (RTAB-Map) → `odom` (EKF) → `base_footprint` (ground, centre of the four wheels) → `base_link` (axle height) →
`zed_camera_link` (the mount; the ZED wrapper publishes the camera's own frames below it). The ZED itself publishes
no transform (`publish_tf:=false`, `publish_map_tf:=false`). Its odometry carries only a pose (the camera's);
`zed_odometry` moves it to `base_footprint` and the EKF fuses x, y and yaw. The odometry must never jump: the costmaps
place what the camera sees with it, and RTAB-Map corrects the `map` frame instead.

| Topic | From → to |
|---|---|
| `/cmd_vel` | base_teleop → drive_mux (manual) |
| `/rover/auto`, `/rover/estop` | base_teleop → drive_mux, goal_manager |
| `/goal_pose` | RViz → goal_manager |
| `/cmd_vel_nav` → `/cmd_vel_auto` | Nav2 controller and behaviours → velocity_smoother → drive_mux |
| `/cmd_vel_drive`, `/rover/estop_drive` | drive_mux → drive_node (its `/cmd_vel` and `/rover/estop`, remapped) |
| `/rover/goal_active` | goal_manager → drive_mux: a goal of its own runs |
| `/rover/auto_fault` | drive_mux → goal_manager, you: why AUTO stopped itself (empty while it has not) |
| `/zed/zed_node/odom` → `/zed/odom_base` | ZED (camera pose) → zed_odometry (rover pose) → EKF |
| `/zed/zed_node/pose/status` | ZED → zed_odometry: whether it tracks |
| `/odometry/filtered` | EKF → Nav2 |
| `/map` | RTAB-Map → Nav2's global costmap, RViz |
| `/rover/autonomy_events` | goal_manager → you |

## Rehearsal without hardware

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 tools/rehearsal_auto.py      # about 6 minutes; PASS/FAIL table, exit code 1 on any failure
```

The rehearsal runs the real launch files, Nav2 included, against the simulated ESP32. It simulates the rover's motion
with a deadband on each side's motors, publishes it as the ZED's odometry, and puts a post in front of the rover. The
base station runs in its own ROS domain behind a relay, so the script can cut the "radio". Both domains are its own
(61, 62), and it refuses to start next to another node in either. It checks:

- goals are taken only in AUTO, are fixed in the map when given in a moving frame, and Nav2 goals sent around
  `goal_manager` don't move the rover;
- Nav2 drives around the post and the EKF's pose matches the simulated rover;
- radio loss while a goal runs: the rover goes on to its goal, then the e-stop latches until the radio is back;
- a base-station restart while a goal runs: the rover goes on until the restarted base station (e-stopped) is heard,
  then stops, and the goal is gone;
- Circle and a stick take over;
- nothing but the base station publishes `/cmd_vel`, and `drive_node` takes its e-stop from the mux.

No camera or RTAB-Map runs in it (`zed:=false slam:=false`).

## Branches

`visual-odom-serial` is `main` + this package + the Triangle button in `base_teleop`. Changes to the drive path go on
`main` first and are merged in (`git switch visual-odom-serial && git merge main`). The autonomy code lives only in
`ros2_ws/src/rover_autonomy/`, `tools/rehearsal_auto.py` and this file, plus the AUTO button in `teleop_logic.py` /
`base_teleop.py`. Nothing in it depends on the link, so an Ethernet variant is `ethernet-udp` with this branch merged
in.

## Compared with sw_Nav2_Hardware

| sw_Nav2_Hardware | Here |
|---|---|
| `auto_node` turns `/cmd_vel_nav` into the old 2-byte protocol (direction + PWM, no arcs; spinning forced to PWM 200) | Nav2's commands go through `drive_node` and the framed protocol like the sticks', with arcs; `drive_mux` corrects the deadband for every mix of rolling and turning |
| Nav2 publishes `/cmd_vel` directly; nothing stops it in manual | Nav2 drives only in AUTO, for a goal of `goal_manager`'s, through the mux; the e-stop and the firmware's rules stay |
| RViz goals go straight to Nav2 | `goal_manager`: only in AUTO, fixed in the map; finished without the radio, then the e-stop latches |
| The ZED wrapper's defaults (odometry reset on loop closures) | no reset, and jumps taken out: the costmaps stay right when the rover turns |
| EKF fuses the ZED's odometry pose **and** twist, as the rover's pose | the ZED wrapper publishes no twist (all zeros), so only the pose is fused, after moving it from the camera to the rover's centre |
| Default behaviour tree: backs up as a recovery | A tree without backing up (the camera can't see behind) |
| URDF: a placeholder robot with mock ros2_control | the rover's own frames from `autonomy.yaml`; no ros2_control |
| RTAB-Map database always the same file | fresh or continued by launch argument, old maps kept |

## Known limits

- **The camera only looks forward.** Nav2 never reverses, and nothing sees obstacles beside or behind the rover. It
  turns on the spot to look around.
- **Odometry is visual only.** The motors have no encoders that reach ROS. A featureless view (a blank wall, glare,
  darkness) or a covered lens makes the ZED lose track. AUTO then stops itself (no odometry, or no motion within
  3 s), but in those seconds the rover drives on what it last knew. In manual nothing stops: the map and the
  costmaps can then get things in the wrong place until the ZED tracks again.
- Speeds are open loop. With the deadband measured, AUTO's speeds are roughly right; uphill they're lower. Nav2 corrects
  from the odometry.
- RViz over the radio: the costmaps and the map are a few hundred kB/s. The ZED point cloud display is off by default
  for this reason.
- **Multicast must stay off the radio** (README, setup step 4: `AllowMulticast` `spdp` on both machines). Otherwise
  CycloneDDS sends the ZED's point cloud as multicast, about 150 Mbit/s, and the two radios lose each other as soon
  as the stack starts. The launch prints a warning when the setting is missing.
- Don't use the Nav2 RViz panel or another action client: it sends goals around `goal_manager`, which the mux
  ignores, so they do nothing (Nav2 still plans them).
- The costmaps clear what has gone with a second pass over the ZED's point cloud (`zed_ground`). It doubles that
  work for the CPU; if Nav2's controller falls behind (`Control loop missed its desired rate` in its log), lower the
  ZED's point cloud rate or resolution in `config/zed_override.yaml`.
