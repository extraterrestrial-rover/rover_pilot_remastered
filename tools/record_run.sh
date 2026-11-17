#!/usr/bin/env bash
# Records one autonomy run on the rover laptop, to look at afterwards (AUTONOMY.md, "When a run goes wrong").
#
#     tools/record_run.sh [--cloud] [name]
#
# Run it in a second terminal on the rover laptop (ROS sourced) while rover_autonomy.launch.py runs, and stop it with
# Ctrl-C. It writes ~/rover_runs/<date>-<name>/ with:
#   bag/          the small topics: the commands at every stage, poses and transforms, Nav2's plans and costmaps, the
#                 obstacle clouds, the events and faults (tens of MB a minute, mostly the map and the costmaps)
#   launch.log    the launch's own log, copied when the recording stops
#   versions.txt  the installed versions of the packages that matter, and this repository's commit
# --cloud also records the ZED's raw point cloud, about 1 GB a minute: for a minute or so, to tune what counts as an
# obstacle. The map itself is ~/.ros/rover_map.db: copy it along when the map is the problem.
set -u

TOPICS=(
  /tf /tf_static /diagnostics /rosout
  /rover/auto /rover/estop /rover/estop_drive /rover/goal_active /rover/auto_fault /rover/autonomy_events
  /goal_pose /cmd_vel /cmd_vel_nav /cmd_vel_auto /cmd_vel_drive
  /odometry/filtered /zed/odom_base /zed/zed_node/odom /zed/zed_node/pose/status
  /rover/obstacles /rover/ground
  /plan /local_plan /map
  /local_costmap/costmap /local_costmap/published_footprint
  /global_costmap/costmap /global_costmap/published_footprint
  /gps/fix /odometry/gps /odometry/global /rover/gps_status /rover/gps_datum /rover/gps_command /rover/gps_waypoints
)
CLOUD=/zed/zed_node/point_cloud/cloud_registered
PACKAGES='ros-humble-tf2 ros-humble-tf2-ros ros-humble-navigation2 ros-humble-nav2-costmap-2d ros-humble-rtabmap ros-humble-rtabmap-ros ros-humble-robot-localization ros-humble-rmw-cyclonedds-cpp ros-humble-cyclonedds'

name=run
for argument in "$@"; do
  case "$argument" in
    --cloud) TOPICS+=("$CLOUD") ;;
    -h|--help) sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) echo "unknown option $argument (see --help)" >&2; exit 2 ;;
    *) name=$argument ;;
  esac
done
if ! command -v ros2 >/dev/null; then
  echo 'ros2 is not on the PATH: source /opt/ros/humble/setup.bash first' >&2
  exit 1
fi

repository=$(cd "$(dirname "$0")/.." && pwd)
directory=~/rover_runs/$(date +%Y%m%d-%H%M%S)-$name
mkdir -p "$directory" || exit 1
{
  date
  # shellcheck disable=SC2086  # the package names are separate words
  dpkg-query -W -f '${Package} ${Version}\n' $PACKAGES 2>/dev/null
  echo "repository: $(git -C "$repository" rev-parse --abbrev-ref HEAD 2>/dev/null) $(git -C "$repository" rev-parse HEAD 2>/dev/null)"
  git -C "$repository" status --short 2>/dev/null | sed 's/^/  changed: /'
} > "$directory/versions.txt"

echo "recording into $directory (Ctrl-C to stop)"
trap '' INT  # Ctrl-C goes to ros2 bag, which closes its file; this script then finishes below
ros2 bag record -o "$directory/bag" "${TOPICS[@]}"
trap - INT

log=$(ls -td ~/.ros/log/*/ 2>/dev/null | while read -r run; do [ -f "$run/launch.log" ] && echo "$run/launch.log" && break; done)
if [ -n "$log" ]; then
  cp "$log" "$directory/launch.log"
fi
echo
echo "recorded: $directory ($(du -sh "$directory" | cut -f1))"
echo "to look at the map as well, copy it along: cp ~/.ros/rover_map.db $directory/"
