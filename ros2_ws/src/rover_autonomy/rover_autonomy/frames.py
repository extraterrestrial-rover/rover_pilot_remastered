"""Rigid-body poses for zed_odometry, without ROS or numpy so they are unit-tested.

A pose is (x, y, z, qx, qy, qz, qw): a position and a unit quaternion.
"""
from __future__ import annotations

import math

Pose = tuple[float, float, float, float, float, float, float]


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rotate(q, v):
    x, y, z, _ = quaternion_multiply(quaternion_multiply(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return x, y, z


def compose(a: Pose, b: Pose) -> Pose:
    """a then b: the pose b given in a's frame, expressed in a's parent frame."""
    offset = rotate(a[3:], b[:3])
    return (a[0] + offset[0], a[1] + offset[1], a[2] + offset[2], *quaternion_multiply(a[3:], b[3:]))


def inverse(a: Pose) -> Pose:
    conjugate = (-a[3], -a[4], -a[5], a[6])
    x, y, z = rotate(conjugate, a[:3])
    return (-x, -y, -z, *conjugate)


def from_xyz_rpy(x: float, y: float, z: float, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> Pose:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (x, y, z, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def yaw_of(pose: Pose) -> float:
    qx, qy, qz, qw = pose[3:]
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def camera_to_base(camera_in_odom: Pose, camera_on_base: Pose) -> Pose:
    """The ZED reports where its camera is; the EKF and Nav2 need where the rover's centre is."""
    return compose(camera_in_odom, inverse(camera_on_base))
