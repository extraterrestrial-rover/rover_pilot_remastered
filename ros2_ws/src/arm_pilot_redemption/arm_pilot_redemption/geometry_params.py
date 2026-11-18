"""The arm gearing and PWM limit parameters (shared section of config/arm.yaml), declared the same way by both nodes."""
from __future__ import annotations

from rclpy.node import Node

from arm_pilot_redemption.kinematics import ArmGeometry, with_pwm_limit


def declare_geometry(node: Node) -> ArmGeometry:
    param = node.declare_parameter
    return ArmGeometry(tuple(param('motor_max_rpm', [117.0, 117.0, 117.0, 117.0]).value),
                       param('shoulder_ratio', 1.0).value, param('elbow_ratio', 1.0).value,
                       param('wrist_pitch_ratio', 1.0).value, param('wrist_roll_ratio', 1.0).value)


def declare_limited_geometry(node: Node, geometry: ArmGeometry) -> tuple[ArmGeometry, float]:
    """(geometry with every motor capped at pwm_limit, pwm_limit). Commands are computed with the capped geometry, so
    full stick is pwm_limit of full PWM. The firmware clamps to PWM_LIMIT_PERCENT as well."""
    pwm_limit = node.declare_parameter('pwm_limit', 0.35).value
    return with_pwm_limit(geometry, pwm_limit), pwm_limit
