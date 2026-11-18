"""The arm gearing parameters (shared section of config/arm.yaml), declared the same way by both nodes."""
from __future__ import annotations

from rclpy.node import Node

from arm_pilot_redemption.kinematics import ArmGeometry


def declare_geometry(node: Node) -> ArmGeometry:
    param = node.declare_parameter
    return ArmGeometry(tuple(param('motor_max_rpm', [117.0, 117.0, 117.0, 117.0]).value),
                       param('shoulder_ratio', 1.0).value, param('elbow_ratio', 1.0).value,
                       param('wrist_pitch_ratio', 1.0).value, param('wrist_roll_ratio', 1.0).value)
