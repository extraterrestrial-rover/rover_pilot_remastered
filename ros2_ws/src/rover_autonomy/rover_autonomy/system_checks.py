"""What the autonomy launch checks about the installed ROS packages before it starts. No ROS, so it is unit-tested.

tf2 0.25.23 (ROS Humble's packages from 2026-09-14 to 2026-10-12) can deadlock: tf2_ros' Buffer::waitForTransform
holds its timer lock while it asks tf2 for a transform, and tf2's BufferCore::testTransformableRequests calls back
into tf2_ros while it holds its request lock (ros2/geometry2 #990, fixed in tf2 0.25.24). Nav2's costmaps do both for
every point cloud. Within a minute or so one costmap's two threads lock each other: that costmap gets no transforms
any more, takes in no new obstacles, and Nav2 steers from the position it last knew. Nav2's log then repeats
"Transform data too old when converting from map to odom".
"""
from __future__ import annotations

import re

FIRST_LOCKING_TF2_ROS = (0, 25, 23)  # waitForTransform holds its lock across the request from this version on
FIRST_FIXED_TF2 = (0, 25, 24)  # tf2 calls back without its lock from this version on


def package_version(package_xml: str) -> str:
    """The <version> of a package.xml's text ('' if it has none)."""
    match = re.search(r'<version>\s*([^<\s]+)\s*</version>', package_xml)
    return match.group(1) if match else ''


def _numbers(version: str) -> tuple[int, ...] | None:
    parts = version.strip().split('.')
    if len(parts) != 3 or not all(part.isdigit() for part in parts):
        return None
    return tuple(int(part) for part in parts)


def tf2_problem(tf2_version: str, tf2_ros_version: str) -> str:
    """'' unless this pair of installed tf2 and tf2_ros versions can deadlock Nav2's costmaps. Only Humble's 0.25
    series is judged: other series have their own version numbers for the same change."""
    tf2, tf2_ros = _numbers(tf2_version), _numbers(tf2_ros_version)
    if tf2 is None or tf2_ros is None or tf2[:2] != (0, 25) or tf2_ros[:2] != (0, 25):
        return ''
    if tf2_ros < FIRST_LOCKING_TF2_ROS or tf2 >= FIRST_FIXED_TF2:
        return ''
    return (f"tf2 {tf2_version} with tf2_ros {tf2_ros_version} is installed: this pair can deadlock inside Nav2's "
            'costmaps within minutes. Nav2 then gets no transforms and steers from an old position (AUTO stops itself '
            'when it sees that). Upgrade both before using AUTO: '
            "sudo apt update && sudo apt install --only-upgrade 'ros-humble-tf2*'")
