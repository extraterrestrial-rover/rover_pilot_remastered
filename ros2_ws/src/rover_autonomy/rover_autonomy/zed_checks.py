"""What zed_odometry checks before it passes the ZED's odometry on. No ROS, so it is unit-tested.

The EKF and Nav2 believe whatever arrives. The ZED wrapper keeps publishing its last pose when it loses tracking, and
with nothing arriving at all the EKF keeps predicting at the last speed: either way the rover would drive blind.
zed_odometry therefore passes on only real poses from a camera that tracks; drive_mux stops AUTO when none arrive.
"""
from __future__ import annotations

import math

from rover_autonomy.frames import Pose, compose, inverse, yaw_of

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


class ContinuousOdometry:
    """Keeps the camera's odometry continuous, as the EKF and the costmaps need it (REP 105: odom never jumps).

    The ZED wrapper's defaults reset its odometry to where it started whenever it recognises a place it has seen (a
    loop closure). Turning on the spot in front of a wall is exactly that. The rover's heading then snaps back while
    the rover really faces elsewhere, and what the camera sees from there lands in the wrong place: obstacles where
    there are none, which stay in the costmap. The launch turns that reset off (config/zed_override.yaml); this
    catches it anyway, and any other jump: a step that is faster than the rover can move is not motion. The pose
    then carries on from where it was, and only real motion is added from there.
    """

    def __init__(self, max_speed: float = 2.0, max_turn_rate: float = 4.0, slack_m: float = 0.15,
                 slack_rad: float = 0.2) -> None:
        self.max_speed, self.max_turn_rate = max_speed, max_turn_rate  # the rover's top speeds, with margin
        self.slack_m, self.slack_rad = slack_m, slack_rad  # for tracking noise and timestamp jitter
        self._offset: Pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        self._last_in: Pose | None = None
        self._last_stamp: float | None = None
        self.jumps = 0

    def update(self, camera: Pose, stamp: float) -> tuple[Pose, bool]:
        """(the continuous camera pose, whether this step was a jump that was taken out)."""
        jumped = False
        if self._last_in is not None:
            dt = min(max(stamp - self._last_stamp, 0.0), 0.5)
            step = compose(inverse(self._last_in), camera)  # this step's motion, seen from the camera
            distance = math.sqrt(sum(value * value for value in step[:3]))
            turned = abs(yaw_of(step))
            if distance > self.slack_m + self.max_speed * dt or turned > self.slack_rad + self.max_turn_rate * dt:
                # Carry on from the last continuous pose: offset . camera == offset_before . last_in
                self._offset = compose(compose(self._offset, self._last_in), inverse(camera))
                self.jumps += 1
                jumped = True
        self._last_in, self._last_stamp = camera, stamp
        return compose(self._offset, camera), jumped

