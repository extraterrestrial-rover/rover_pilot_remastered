#!/usr/bin/env bash
# Shared by the post-checkout and post-merge hooks (and tools/install_hooks.sh): rebuild ros2_ws, because `ros2 launch`
# runs the installed copy and colcon may copy files even with --symlink-install, then print this branch's summary
# (the "On this branch" section of LINK.md). Never fails the git command that ran it.
set -u
root=$(git rev-parse --show-toplevel)
echo "rover_pilot_remastered: on branch '$(git rev-parse --abbrev-ref HEAD)'"

if [ -z "${ROS_DISTRO:-}" ] && [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash >/dev/null 2>&1
fi
if [ -n "${ROS_DISTRO:-}" ] && command -v colcon >/dev/null 2>&1; then
  log=$(mktemp)
  if (cd "$root/ros2_ws" && colcon build --symlink-install) >"$log" 2>&1; then
    echo "  ros2_ws rebuilt"
  else
    echo "  ros2_ws build FAILED (cd ros2_ws && colcon build --symlink-install):"
    tail -n 20 "$log" | sed 's/^/    /'
  fi
  rm -f "$log"
else
  echo "  no ROS 2 here (no /opt/ros/humble): ros2_ws not rebuilt"
fi

if [ -f "$root/LINK.md" ]; then
  sed -n '/^## On this branch/,/^## /{/^## /!p;}' "$root/LINK.md" | sed '/^$/d'
fi
if pgrep -f 'rover_pilot_remastered/drive_node' >/dev/null 2>&1; then
  echo "  drive_node is still running the previous build: Ctrl-C rover.launch.py and launch it again"
fi
exit 0
