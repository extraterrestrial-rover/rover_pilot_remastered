# Autonomy: visual odometry + Nav2 (`visual-odom-serial`)

This branch is `main` plus autonomous navigation. The rover maps its surroundings all the time, with the ZED 2i and
RTAB-Map, while you drive it with the PS5. Press **Triangle** to switch to AUTO, then click goals in RViz at the base
station (or send GPS waypoints, in the GPS mode: "GPS" below), and Nav2 drives the rover to them, rolling through
curves. The drive path itself doesn't change: `drive_node`, the firmware, the
e-stop and every stop in the README's safety table work as on `main`, with one exception you chose: when the radio
drops while the rover drives to a goal, it goes on to that goal ("Situation in AUTO" below).

The stack follows the colleague's `extraterrestrial-rover/sw_Nav2_Hardware`: ZED visual-inertial odometry, a
robot_localization EKF, RTAB-Map and Nav2 (with Regulated Pure Pursuit instead of DWB). It differs where that stack
doesn't fit this rover (see
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
       └─point cloud─▶ obstacles_detection ─/rover/obstacles, /rover/ground──▶ Nav2's costmaps
```

- **`drive_mux`** decides what reaches `drive_node`:
  - the sticks' `/cmd_vel` in manual, passed on as each command arrives;
  - Nav2's `/cmd_vel_auto` in AUTO, and only for a goal `goal_manager` sent. It sends zeros for 0.3 s first, and again
    whenever `drive_node` reports that the ESP32 is not driving, because the ESP32 drives again only after a neutral
    command. It corrects Nav2's commands for the motors' deadband ("Before the first run", step 5), and when the
    rover stands for a second although commanded, it raises its output until the rover moves (the stall kick);
  - the base station's e-stop, passed on the moment it arrives. Only while the base station is silent does the mux
    decide the e-stop itself (table below).

  It also **stops AUTO by itself**, and the goal is cancelled, when the rover would drive blind:
  - no odometry from the camera for 0.5 s;
  - a motion command for 3 s without the odometry moving (tracking lost, wheels stuck);
  - no obstacle data from the camera for 1 s (its point cloud or `obstacles_detection` stopped);
  - one of Nav2's two costmaps has lost track of the rover: the transform it places the rover with is over 1 s old,
    or the costmap is not heard for 3 s. Each costmap keeps its own copy of the transforms, and such a copy can
    stop ("Known limits", tf2).

  `ros2 topic echo /rover/auto_fault` says why.
- **`obstacles_detection`** (RTAB-Map's) splits the ZED's point cloud into ground and obstacles by the direction
  each surface faces, not by its height: a tilted rover or a slope ahead doesn't become a wall of obstacles. Nav2's
  costmaps take those two clouds into a voxel layer, which keeps and clears obstacles in 3D, so a low obstacle stays
  when the camera, close to it, looks over it.
- **`mount_check`** runs once at the start: it fits the ground in the camera's first point clouds and compares the
  camera's real height and tilt with `zed_z_m` and `zed_pitch_rad`. If they differ it prints the values to set.
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
- **Turning: arcs.** The controller is Regulated Pure Pursuit: it steers towards a point about a metre ahead on the
  planned path, so the rover rolls through curves like it does on the sticks. It turns on the spot only when the path
  starts behind it (more than `pivot_above_deg`, 115°, off its heading), and at the goal to the arrow you dragged in
  RViz (to within `goal_heading_tolerance_deg`, 15°). It slows in tight curves and near obstacles, and stops before
  the arc ahead would touch one; Nav2 then re-plans. `auto_driving: pivot` brings back the controller before arcs
  (DWB), which stops and turns on the spot wherever the path bends.

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
| No odometry, no motion although commanded, no obstacle data, or Nav2 has lost track of the rover (see `drive_mux` above) | AUTO stops itself; the goal is cancelled | find the cause (`/rover/auto_fault`), then a new goal |
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
   looks down (positive). Everything the camera sees is placed with these: a pitch 3° off puts the ground 2 m ahead
   10 cm too high or too low. So the launch checks height and pitch each time it starts, with the rover standing on
   flat ground and open floor in view. Look for the `camera mount check:` lines: either "as the settings say", or
   the values to put into `autonomy.yaml`. To run the check alone:
   ```bash
   ros2 run rover_autonomy mount_check --ros-args --params-file ros2_ws/src/rover_autonomy/config/autonomy.yaml
   ```
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

   Ground differs, so the measured value is only right where you measured it. When the rover stands for 1 s although
   Nav2 commands it, or crawls at under half of what was asked, `drive_mux` raises the deadband by up to 20 rpm. It
   keeps the extra while the ground needs it and takes it back when the rover runs faster than commanded (the log
   says "the rover does not start: raising the output"). If you see that often, measure again on that ground.

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
| `localization:=gps` | slam | the GPS mode ("GPS"): the GPS and `ekf_global` correct the drift, RTAB-Map only records |
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
| `auto_driving` | arcs | `arcs`: roll through curves (Pure Pursuit); `pivot`: stop and turn on the spot (DWB, before arcs) |
| `pivot_above_deg` | 115 | arcs: turn on the spot only when the path starts more than this far off the rover's heading |
| `turn_to_goal_heading`, `goal_heading_tolerance_deg` | true, 15 | at the goal, turn to the RViz arrow, to within this many degrees (false: arrive in any heading) |
| `rover_length_m`, `rover_width_m`, `rover_height_m` | placeholders | the footprint; points higher than the rover are ignored |
| `inflation_radius_m` | 1.0 | how far Nav2 plans the rover's centre from obstacles where there is room; keep it at least 0.2 m above the distance from the centre to a corner of the outline (the launch warns) |
| `min_obstacle_height_m` | 0.1 | obstacles lower than this are driven over (the costmaps leave them out; the map still shows them) |
| `max_ground_slope_deg` | 45 | a surface steeper than this is an obstacle, anything flatter is ground |
| `obstacle_min_points` | 10 | an obstacle needs this many points 5 cm apart; fewer is noise (lower it for thin stakes) |
| `obstacle_range_m` | 3.0 | how far the depth image marks obstacles (the ZED's depth gets noisy beyond) |
| `zed_x_m` … `zed_pitch_rad` | placeholders | the camera's mount (the URDF and `zed_odometry` use it) |
| `zed_*_topic` | wrapper 5.x names | the ZED's topics, and `zed_status_topic` its tracking status (`''` = not checked) |
| `map_database` | `~/.ros/rover_map.db` | RTAB-Map's map |
| `gps_x_m`, `gps_y_m`, `gps_z_m` | placeholders | the GPS antenna's position (the top of the L89HA board) from the rover's centre point on the ground |
| `gps_window_m` | 100 | GPS mode: the square around the rover Nav2 plans in |
| `gps_node`: `gps_port`, `gps_baud` | `/dev/rover-gps`, 9600 | the GPS board's port and speed (the L89HA's factory setting) |
| `gps_node`: `gps_uere_m`, `gps_min_satellites`, `gps_max_hdop` | 1.5, 5, 3.0 | a fix's standard deviation at HDOP 1; fixes with fewer satellites or a larger HDOP are not used |
| `gps_node`: `gps_setup` | `''` | commands sent to the L89HA at every start, separated by `;` (see "GPS") |
| `gps_alignment`: `min_track_m`, `refit_every_m`, `freeze_track_m` | 15, 10, 100 | north from the track: after this much driving, worked out again every 10 m, kept from 100 m on |
| `goal_manager`: `leg_m`, `max_gps_goal_distance_m` | 40, 1000 | GPS waypoints further than 40 m go in legs; one further than 1 km is refused (swapped latitude and longitude?) |
| `drive_mux`: `min_wheel_rpm_straight`, `min_wheel_rpm_spin` | 0.0 (not measured) | the deadband |
| `drive_mux`: `neutral_hold_s` | 0.3 | zeros first when AUTO starts, and when the ESP32 is not driving (`rearm_interval_s` 1.5) |
| `drive_mux`: `mode_timeout_s` | 0.5 | base station silent this long: lost |
| `drive_mux`: `odometry_timeout_s`, `no_motion_timeout_s` | 0.5, 3.0 | AUTO stops itself (no odometry; no motion although commanded, 0 = not checked) |
| `drive_mux`: `obstacle_timeout_s` | 1.0 | AUTO stops itself: no obstacle cloud from the camera (0 = not checked) |
| `drive_mux`: `nav_pose_max_age_s`, `nav_pose_timeout_s` | 1.0, 3.0 | AUTO stops itself: a Nav2 costmap works with a transform older than this, or is not heard (0 = not checked) |
| `drive_mux`: `stall_kick_delay_s`, `stall_kick_rpm_per_s`, `stall_kick_max_rpm` | 1.0, 12, 20 | the stall kick: standing (or crawling) this long although commanded, the deadband is raised this fast, by at most this much (rate 0 = never) |
| `drive_mux`: `linkless_limit_s` | 0.0 (no limit) | the longest a goal may run without the base station |
| `goal_manager`: `link_timeout_s` | 0.5 | base station silent this long: lost (the goal goes on) |

Everything else Nav2 uses is in `config/nav2_params.yaml` (Humble's names; the values marked "from autonomy.yaml" are
overwritten at launch). The launch prints where the final parameter file is (removed when the launch ends). What the
ZED wrapper must do differently from its defaults is in `config/zed_override.yaml`: no odometry resets, and colour
and depth images at 15 Hz instead of the camera's 60 (RTAB-Map takes one pair a second; at 60 Hz that was hundreds
of MB/s copied between processes).

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
| `/zed/zed_node/point_cloud/cloud_registered` → `/rover/obstacles`, `/rover/ground` | ZED → obstacles_detection → Nav2's costmaps, drive_mux (that they keep coming) |
| `/local_costmap/published_footprint`, `/global_costmap/published_footprint` | Nav2 → drive_mux: how old the transform is that each costmap places the rover with |
| `/odometry/filtered` | EKF → Nav2 |
| `/map` | RTAB-Map → Nav2's global costmap, RViz |
| `/rover/autonomy_events` | goal_manager → you |
| `/gps/fix` | gps_node → gps_alignment, RTAB-Map (GPS mode) |
| `/rover/gps_status`, `/rover/gps_datum`, TF `map → enu` | gps_alignment → goal_manager, you: north found yet?, the GPS origin, east-north in the map |
| `/odometry/gps` → `/odometry/global` | gps_alignment (fixes in the map frame) → ekf_global (publishes `map → odom` in the GPS mode) |
| `/rover/gps_command` | gps_goals (base station) → goal_manager: waypoint lists, `next`, `cancel` |
| `/rover/gps_waypoints` | goal_manager → RViz |

## GPS (Quectel L89HA)

The GPS mode is for long runs outdoors (competition traverses up to about 500 m). The camera's odometry is precise
over metres but drifts about 1-2 % of the distance; GPS is a metre or two off everywhere but never drifts. In the GPS
mode a second EKF, `ekf_global`, combines the two and corrects the drift (`map -> odom`), and you can send goals as GPS
coordinates. RTAB-Map then only records (its map and the GPS fixes stay in its database); Nav2 plans in a
`gps_window_m` (100 m) square around the rover, from what the camera has seen. The RTAB-Map mode (the default) stays as
it was, for small areas and indoors.

**The receiver.** The L89HA (dual-band L1 + L5: GPS, Galileo, NavIC by default; GLONASS and BeiDou can be switched on;
SBAS including GAGAN) is about 1.5 m off at HDOP 1 (Quectel: under 1.8 m CEP). Its board has its own CP2102 USB-serial
bridge. From the factory it sends at 9600 baud, once a second.

**Mounting.** On top, at the highest point, with nothing above it; the antennas are on the board's top face: keep it
level, face up. At least 30 cm from the ZED and its USB 3 cable (USB 3 jams GPS), 50 cm from the MikroTik and its
antenna, away from the laptop and the motor drivers; a metal plate of about 10 x 10 cm under it helps. Measure where it
sits from the rover's centre point on the ground and enter `gps_x_m`, `gps_y_m`, `gps_z_m`.

**A name for its port (once).** The GPS board and the drive ESP32 both came with the CP2102 serial number `0001`, so
the drive's udev rule cannot tell them apart: with both plugged in, `/dev/rover-drive` can land on the GPS (the rover
then just doesn't drive). Give the GPS board's CP2102 a serial number of its own, with only the GPS board plugged in:

```bash
sudo apt install git python3-usb
git clone https://github.com/VCTLabs/cp210x-program.git ~/cp210x-program && cd ~/cp210x-program
lsusb | grep -c 10c4:ea60                      # must be 1: unplug the drive ESP32 (and any other CP2102) first
sudo PYTHONPATH=. python3 scripts/cp210x-program.py --read-cp210x -f ~/gps-cp2102-backup.hex   # a backup first
sudo PYTHONPATH=. python3 scripts/cp210x-program.py --write-cp210x --set-serial-number=ROVERGPS --reset-device
```

Unplug and replug the GPS board; `udevadm info -q property -n /dev/ttyUSB0 | grep ID_SERIAL=` must end in `ROVERGPS`.
Then install the rule and plug the ESP32 back in:

```bash
sudo cp ~/rover_pilot_remastered/udev/99-rover-gps.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
ls -l /dev/rover-gps /dev/rover-drive          # two different ttyUSB devices
```

The tool only changes the serial number; the backup restores the chip
(`--write-cp210x -F ~/gps-cp2102-backup.hex`) if anything goes wrong. Only the GPS board is reprogrammed; the drive
ESP32's rule and chip stay as they are.

**Check it** under open sky, with the rover's launch stopped (it holds the port):

```bash
python3 tools/gps_check.py --seconds 300       # once a second: satellites per system, HDOP, position
```

At the end it prints how far 5 minutes of fixes wandered while standing: that is about how close the rover can get to
a GPS waypoint at that spot. `--send 'PAIR410,1'` sends a command and prints the receiver's answer (the `$` and the
checksum are added). Commands that work can go into `gps_setup` in `autonomy.yaml`, separated by `;`, to be sent at
every start. Check each with `--send` first: the L89HA's command set (Airoha `$PAIR`) is not tried on this rover yet.

**A GPS run.**

1. Rover: `ros2 launch rover_autonomy rover_autonomy.launch.py localization:=gps`. The launch says GPS mode;
   `ros2 topic echo /rover/gps_status` says what the GPS is waiting for.
2. Drive about 15 m, by hand or to an RViz goal, straight or in curves. A single antenna cannot tell which way the
   rover faces while it stands; the rover works out north from its own track against the GPS's. `/rover/gps_status`
   then says `GPS ready: heading known to about N degrees`, and RViz shows east (red) and north (green). It refines
   that every 10 m up to 100 m, then keeps it.
3. Triangle (AUTO), then at the base station:

   ```bash
   ros2 run rover_autonomy gps_goals "28.750123, 77.117456" "28°45'02.10\"N 77°07'05.30\"E"   # in turn
   ros2 run rover_autonomy gps_goals --file waypoints.txt --mode pause --pause 15                # 15 s at each
   ros2 run rover_autonomy gps_goals --file waypoints.txt --mode wait                            # until 'next'
   ros2 run rover_autonomy gps_goals next
   ros2 run rover_autonomy gps_goals cancel
   ```

   Positions in any usual form: decimal degrees (`28.750123, 77.117456`, or with N/S/E/W), degrees minutes seconds
   (`28°45'00.44"N 77°07'02.84"E`, `28 45 00.44 N, 77 7 2.84 E`), degrees decimal minutes (`28°45.0074'N
   77°07.0474'E`); latitude first. A file has one position per line, `#` starts a comment. Each is checked and
   printed (with its distance from the rover) before anything is sent; anything ambiguous is refused with why. Then the
   command prints what the rover does, with a beep at each waypoint, until the list is done (Ctrl-C stops only the
   command).

The modes: `continue` drives the whole list and stops at the last; `pause` stops `--pause` seconds at each, then goes on
by itself; `wait` stops at each until `gps_goals next`. A waypoint further than `leg_m` (40 m) is driven in legs, each
sent before the rover reaches the last, so it doesn't stop on the way. A waypoint is reached when the rover is within
0.3 m of where the GPS places it, so within the GPS's own error of the real spot. The same rules as for RViz goals
apply: only in AUTO, Circle or a stick ends the list. If the base station falls silent, the rover finishes the
waypoint it is driving to (all its legs) and stops there; the list does not go on without the base station. If the
GPS drops out, the rover goes on with the camera's odometry, and `/rover/gps_status` says for how long.

## Rehearsal without hardware

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 tools/rehearsal_auto.py      # about 8 minutes; PASS/FAIL table, exit code 1 on any failure
```

The rehearsal runs the real launch files, Nav2 included, against the simulated ESP32. It simulates the rover's motion
with a deadband on each side's motors and publishes it as the ZED's odometry. The ZED's point cloud is ray-cast with
the camera's field of view: flat ground and a post in front of the rover, so the obstacle detector and the costmaps
work on a real view. The base station runs in its own ROS domain behind a relay, so the script can cut the "radio".
Both domains are its own (61, 62), and it refuses to start next to another node in either. It checks:

- goals are taken only in AUTO, are fixed in the map when given in a moving frame, and Nav2 goals sent around
  `goal_manager` don't move the rover;
- the post is in the costmap and the ground before it is free, also with the rover tilted 4°; a camera mounted 4° off
  is reported with the value to set;
- Nav2 drives around the post and the EKF's pose matches the simulated rover;
- AUTO stops when the point cloud stops, and when Nav2's transforms stop (the EKF frozen);
- on ground that needs more than the measured deadband the rover still starts (the stall kick);
- radio loss while a goal runs: the rover goes on to its goal, then the e-stop latches until the radio is back;
- a base-station restart while a goal runs: the rover goes on until the restarted base station (e-stopped) is heard,
  then stops, and the goal is gone;
- Circle and a stick take over;
- nothing but the base station publishes `/cmd_vel`, and `drive_node` takes its e-stop from the mux.

No camera or RTAB-Map runs in it (`zed:=false slam:=false`). It needs `rtabmap_util` (part of
`ros-humble-rtabmap-ros`) for the obstacle detector. It also checks arcs: a goal off to the side is reached without
turning on the spot on the way, then the rover turns to the arrow.

`python3 tools/rehearsal_gps.py` (about 12 minutes, domains 65 and 66) does the same in the GPS mode, with a simulated
L89HA on a pseudo-terminal: the simulated world faces 30° north of east, the GPS wanders by up to about 1.5 m, and the
camera's odometry overstates every distance by 4 %. It checks that GPS waypoints are refused until the rover has found
north by driving, that it finds it to within 5°, that the GPS keeps the position right although the odometry drifts,
the three waypoint modes, a far waypoint in legs typed in degrees-minutes-seconds, a GPS outage on the way, and the
e-stop stopping a list.

## When a run goes wrong

Record it, so there is more to go by than the console:

```bash
tools/record_run.sh barrier-test          # on the rover laptop, in a second tmux window; Ctrl-C stops it
```

It writes `~/rover_runs/<date>-barrier-test/`: a rosbag of the commands at every stage, poses, transforms, Nav2's
plans and costmaps, the obstacle clouds, the events and faults; the launch log; and the installed package versions.
That is tens of MB a minute. `--cloud` adds the ZED's raw point cloud, about 1 GB a minute: use it for a minute when
the question is what counts as an obstacle. When the map is the problem, copy `~/.ros/rover_map.db` along.

What to look at first:

| Seen | Look at |
|---|---|
| The goal was cancelled | `ros2 topic echo /rover/autonomy_events` and `/rover/auto_fault`: the reason is in words |
| Obstacles where there are none, or the map's floor full of them | the `camera mount check:` lines of the launch (step 2 above); in RViz the "Obstacles" cloud shows what Nav2 avoids |
| A real obstacle is not in the costmap | lower than `min_obstacle_height_m`, or too few points (`obstacle_min_points`), or beyond `obstacle_range_m` |
| Nav2's log repeats `Transform data too old` | `/rover/auto_fault` says which costmap lost track; check the tf2 version ("Known limits") |
| The rover passes too close to things | `inflation_radius_m` |

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
- The stall kick helps a rover that does not start straight ahead or on the spot. It lifts no command more than
  20 rpm above the deadband for that command, so in a tight arc on ground that needs much more than measured the
  rover can still stand; AUTO then stops after 3 s ("has not moved"). It is tested in simulation only: if the rover
  lurches when it starts, switch it off with `stall_kick_rpm_per_s: 0.0` and record a run with it on
  (`tools/record_run.sh`).
- RViz over the radio: the costmaps and the map are a few hundred kB/s. The ZED point cloud display is off by default
  for this reason.
- **Multicast must stay off the radio** (README, setup step 4: `AllowMulticast` `spdp` on both machines). Otherwise
  CycloneDDS sends the ZED's point cloud as multicast, about 150 Mbit/s, and the two radios lose each other as soon
  as the stack starts. The launch prints a warning when the setting is missing.
- Don't use the Nav2 RViz panel or another action client: it sends goals around `goal_manager`, which the mux
  ignores, so they do nothing (Nav2 still plans them).
- **tf2 0.25.23 freezes Nav2.** ROS Humble's packages from 2026-09-14 to 2026-10-12 carry tf2 0.25.23, which
  deadlocks inside Nav2's costmaps within a minute or so (fixed in 0.25.24). The costmap then gets no transforms:
  Nav2 steers from an old position, and its log repeats `Transform data too old when converting from map to odom`.
  The launch warns when that version is installed, and AUTO stops itself when a costmap loses track. Check with
  `dpkg -s ros-humble-tf2-ros | grep Version`; fix with
  `sudo apt update && sudo apt install --only-upgrade 'ros-humble-tf2*'` (both `tf2` and `tf2_ros`).
- What counts as an obstacle is decided by shape: a surface steeper than `max_ground_slope_deg` with at least
  `obstacle_min_points` points. A thin stake has few points and is seen later than a wall; holes and drops are not
  seen at all. In the costmaps, obstacles lower than `min_obstacle_height_m` are left out; that height is taken
  from where the rover stands, so on a tilted rover it is off by a few cm per metre of distance.
- The mount check assumes flat ground at the start. On a slope, or facing a wall, its warning means nothing.
- Arcs: Pure Pursuit follows the planned path closely and avoids obstacles by stopping and re-planning (Nav2 plans
  once a second), where DWB used to swerve on its own. If the rover often stops in front of obstacles, try
  `auto_driving: pivot` and tell me what you saw.
- GPS: north is found from the first 15 m or so driven; until then no GPS goal is taken. In the GPS mode the map
  frame is the rover's start pose; RTAB-Map's map is recorded but not used to plan or to correct the position.
  The L89HA's setup commands (`gps_setup`) are not tried on the rover yet. All of the GPS mode is tested in
  simulation only (`tools/rehearsal_gps.py`).
