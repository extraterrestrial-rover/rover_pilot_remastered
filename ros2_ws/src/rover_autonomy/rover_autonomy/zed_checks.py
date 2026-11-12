"""What zed_odometry checks before it passes the ZED's odometry on. No ROS, so it is unit-tested.

The EKF and Nav2 believe whatever arrives. The ZED wrapper keeps publishing its last pose when it loses tracking, and
with nothing arriving at all the EKF keeps predicting at the last speed: either way the rover would drive blind.
zed_odometry therefore passes on only real poses from a camera that tracks; drive_mux stops AUTO when none arrive.
"""
from __future__ import annotations

import math

from rover_autonomy.frames import Pose

# zed_msgs/msg/PosTrackStatus
ODOMETRY_OK = 0
ODOMETRY_PROBLEMS = {1: 'unavailable', 2: 'not enough features in view'}
SPATIAL_MEMORY_LOST = 7


def checked_pose(pose) -> Pose | None:
    """`pose` (x, y, z, qx, qy, qz, qw) with its quaternion normalised, or None if it is not a pose: a number that
    is not finite, or a quaternion far from unit length (an empty message has all zeros)."""
    values = tuple(float(value) for value in pose)
    if len(values) != 7 or not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values[3:]))
    if not 0.9 <= norm <= 1.1:
        return None
    return (*values[:3], *(value / norm for value in values[3:]))


def tracking_problem(odometry_status: int, spatial_memory_status: int) -> str:
    """'' while the ZED tracks, else what it reports."""
    if odometry_status != ODOMETRY_OK:
        return ODOMETRY_PROBLEMS.get(odometry_status, f'odometry status {odometry_status}')
    if spatial_memory_status == SPATIAL_MEMORY_LOST:
        return 'lost'
    return ''
