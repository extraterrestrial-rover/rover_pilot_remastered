# Autonomy: visual odometry + Nav2 (`visual-odom-serial`)

This branch is `main` plus autonomous navigation. The rover maps its surroundings all the time, with the ZED 2i and
RTAB-Map, while you drive it with the PS5. Press **Triangle** to switch to AUTO, then click goals in RViz at the base
station, and Nav2 drives the rover to them. The drive path itself doesn't change: `drive_node`, the firmware, the
e-stop and every stop in the README's safety table work as on `main`.

The stack follows the colleague's `extraterrestrial-rover/sw_Nav2_Hardware`: ZED visual-inertial odometry, a
robot_localization EKF, RTAB-Map and Nav2 with DWB. It differs where that stack doesn't fit this rover (see
"Compared with sw_Nav2_Hardware").

```text
BASE STATION                                  ROVER LAPTOP (NVIDIA GPU)
PS5 ─▶ base_teleop ─┬─ /cmd_vel ─────────────▶ drive_mux ─/cmd_vel_drive─▶ drive_node ─link─▶ ESP32
                    ├─ /rover/auto (Triangle) ─▶   ▲   ▲                       ▲
                    └─ /rover/estop ───────────────┼───┼──────────────────────┘ (e-stop goes straight to drive_node)
RViz "2D Goal Pose" ─/goal_pose─▶ goal_manager ─navigate_to_pose─▶ Nav2 ─/cmd_vel_auto─┘
                                     (only in AUTO)                  ▲
ZED 2i ─odom─▶ zed_odometry ─▶ EKF (odom→base_footprint) ─▶ RTAB-Map (map→odom, /map) ┤
       └─point cloud──────────────────────────────────────▶ costmaps ┘
```

- **`drive_mux`** decides what reaches `drive_node`: the sticks' `/cmd_vel` in manual, or Nav2's `/cmd_vel_auto` in
  AUTO. In AUTO it sends zeros for 0.3 s first, because the ESP32 drives again only after a neutral command. Then it
  corrects Nav2's commands for the motors' deadband ("Deadband" below). With the base station silent it passes
  nothing on, so `drive_node` stops the rover exactly as on `main`.
- **`goal_manager`** passes RViz goals to Nav2, only in AUTO with the e-stop released. Nav2's own `/goal_pose`
  subscription is remapped away, so a click can't bypass it.
- **`zed_odometry`** turns the ZED's odometry, which is the camera's pose, into the rover centre's pose
  (`base_footprint`) for the EKF, using the mount in `autonomy.yaml`. Humble's robot_localization would otherwise
  take the camera's pose as the centre, and the rover's footprint would swing round with the camera when it turns.
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
| Radio or base station lost | The rover stops (about 0.5 s, as on `main`); the goal is **paused** | radio back: the goal **resumes** by itself |
| Base station restarted (it starts e-stopped and in manual) | The goal stays paused | Options, then Triangle: it resumes. Circle instead drops it |
| Circle, a stick, or Triangle | Goal cancelled for good | |

Recoveries: when Nav2 is stuck it clears its costmaps, turns a quarter turn on the spot and waits 5 s, up to 6
times. It never backs up, because the camera can't see behind the rover.

## Install (once)

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
   94.7 / 100 (`drive_mux` section). With these, Nav2's speeds are roughly the real speeds; 0.0 turns the correction
   off.

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
| `zed_*_topic` | wrapper 5.x names | the ZED's topics |
| `map_database` | `~/.ros/rover_map.db` | RTAB-Map's map |
| `drive_mux`: `min_wheel_rpm_straight`, `min_wheel_rpm_spin` | 20, 60 (placeholders) | the deadband |
| `drive_mux`: `neutral_hold_s` | 0.3 | zeros first when AUTO starts or resumes |
| `goal_manager`: `link_timeout_s` | 0.5 | base station silent this long: the goal pauses |

Everything else Nav2 uses is in `config/nav2_params.yaml` (Humble's names; the values marked "from autonomy.yaml" are
overwritten at launch). The launch prints where the final parameter file is.

## Frames and topics

`map` (RTAB-Map) → `odom` (EKF) → `base_footprint` (ground, centre of the four wheels) → `base_link` (axle height) →
`zed_camera_link` (the mount; the ZED wrapper publishes the camera's own frames below it). The ZED itself publishes
no transform (`publish_tf:=false`, `publish_map_tf:=false`). Its odometry carries only a pose (the camera's);
`zed_odometry` moves it to `base_footprint` and the EKF fuses x, y and yaw.

| Topic | From → to |
|---|---|
| `/cmd_vel` | base_teleop → drive_mux (manual) |
| `/rover/auto`, `/rover/estop` | base_teleop → drive_mux, goal_manager; `/rover/estop` also → drive_node |
| `/goal_pose` | RViz → goal_manager |
| `/cmd_vel_nav` → `/cmd_vel_auto` | Nav2 controller and behaviours → velocity_smoother → drive_mux |
| `/cmd_vel_drive` | drive_mux → drive_node |
| `/zed/zed_node/odom` → `/zed/odom_base` | ZED (camera pose) → zed_odometry (rover pose) → EKF |
| `/odometry/filtered` | EKF → Nav2 |
| `/map` | RTAB-Map → Nav2's global costmap, RViz |
| `/rover/autonomy_events` | goal_manager → you |

## Rehearsal without hardware

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 tools/rehearsal_auto.py      # about 5 minutes; PASS/FAIL table, exit code 1 on any failure
```

The rehearsal runs the real launch files, Nav2 included, against the simulated ESP32. It simulates the rover's motion
with a motor deadband, publishes it as the ZED's odometry, and puts a post in front of the rover. The base station
runs in its own ROS domain behind a relay, so the script can cut the "radio". It checks:

- goals are taken only in AUTO;
- Nav2 drives around the post and the EKF's pose matches the simulated rover;
- radio loss stops the rover and the goal then resumes;
- after a base-station restart the goal waits for Options + Triangle;
- Circle and a stick take over;
- nothing but the base station publishes `/cmd_vel`.

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
| Nav2 publishes `/cmd_vel` directly; nothing stops it in manual | Nav2 drives only in AUTO, through the mux; the e-stop and link-loss rules stay in `drive_node` and the firmware |
| RViz goals go straight to Nav2 | `goal_manager`: only in AUTO; paused on link loss and resumed |
| EKF fuses the ZED's odometry pose **and** twist, as the rover's pose | the ZED wrapper publishes no twist (all zeros), so only the pose is fused, after moving it from the camera to the rover's centre |
| Default behaviour tree: backs up as a recovery | A tree without backing up (the camera can't see behind) |
| URDF: a placeholder robot with mock ros2_control | the rover's own frames from `autonomy.yaml`; no ros2_control |
| RTAB-Map database always the same file | fresh or continued by launch argument, old maps kept |

## Known limits

- **The camera only looks forward.** Nav2 never reverses, and nothing sees obstacles beside or behind the rover. It
  turns on the spot to look around.
- **Odometry is visual only.** The motors have no encoders that reach ROS. A featureless view (a blank wall, glare,
  darkness) or a covered lens makes the ZED lose track: Nav2 then stops or wanders. Keep an eye on it, with a stick or
  Circle ready.
- Speeds are open loop. With the deadband measured, AUTO's speeds are roughly right; uphill they're lower. Nav2 corrects
  from the odometry.
- RViz over the radio: the costmaps and the map are a few hundred kB/s. The ZED point cloud display is off by default
  for this reason.
- Don't use the Nav2 RViz panel or another action client: it would send goals around `goal_manager`. The mux still
  ignores Nav2 outside AUTO, and the e-stop always works.
