"""Joint <-> gearmotor conversions for the arm: two independent pitch joints and a bevel-gear differential wrist.

Joints (rad, rad/s; order everywhere): shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll.
Motors (gearmotor output shaft, rpm or revolutions): shoulder, elbow, wrist_a, wrist_b.

The wrist's two motors drive the bevel gears from either side: turning them the same way pitches the wrist, turning
them opposite ways rolls it (the auger, later the gripper):
    wrist_a = wrist_pitch_ratio * pitch + wrist_roll_ratio * roll
    wrist_b = wrist_pitch_ratio * pitch - wrist_roll_ratio * roll
so  pitch = (wrist_a + wrist_b) / (2 * wrist_pitch_ratio),  roll = (wrist_a - wrist_b) / (2 * wrist_roll_ratio).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

JOINTS = ('shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll')
RPM_PER_RAD_S = 60.0 / (2.0 * math.pi)
ZERO = (0.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class ArmGeometry:
    motor_max_rpm: tuple[float, float, float, float]  # gearmotor output rpm at full PWM (firmware max_rpm), per motor
    shoulder_ratio: float  # gearmotor output revolutions per shoulder revolution (the worm gearbox and anything after it)
    elbow_ratio: float  # the same for the elbow
    wrist_pitch_ratio: float  # revolutions of each wrist motor per wrist pitch revolution (both motors the same way)
    wrist_roll_ratio: float  # revolutions of each wrist motor per wrist roll revolution (motors opposite ways)

    def __post_init__(self) -> None:
        if len(self.motor_max_rpm) != 4:
            raise ValueError('motor_max_rpm needs four values (shoulder, elbow, wrist_a, wrist_b)')
        values = {'motor_max_rpm': min(self.motor_max_rpm, default=0.0), 'shoulder_ratio': self.shoulder_ratio,
                  'elbow_ratio': self.elbow_ratio, 'wrist_pitch_ratio': self.wrist_pitch_ratio,
                  'wrist_roll_ratio': self.wrist_roll_ratio}
        for name, value in values.items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive numbers, got {value}')

    @property
    def max_joint_velocity(self) -> tuple[float, float, float, float]:
        """Fastest rate of each joint on its own (rad/s), with its motors at full PWM."""
        shoulder, elbow, wrist_a, wrist_b = self.motor_max_rpm
        wrist = min(wrist_a, wrist_b)
        return (shoulder / self.shoulder_ratio / RPM_PER_RAD_S, elbow / self.elbow_ratio / RPM_PER_RAD_S,
                wrist / self.wrist_pitch_ratio / RPM_PER_RAD_S, wrist / self.wrist_roll_ratio / RPM_PER_RAD_S)


def joint_velocity_to_motor_rpm(velocities: tuple[float, ...] | list[float], geometry: ArmGeometry) -> tuple[float, float, float, float]:
    """Joint rates (rad/s) -> gearmotor targets (rpm, shoulder, elbow, wrist_a, wrist_b).

    The shoulder and elbow are clamped to their motor's max rpm. If a wrist motor would exceed its max, both wrist
    motors are scaled down together, which keeps the ratio of wrist pitch to roll. Non-finite inputs, or inputs so large
    that the motor speeds overflow, give zero for every motor.
    """
    if len(velocities) != 4 or not all(math.isfinite(value) for value in velocities):
        return ZERO
    shoulder_rate, elbow_rate, pitch_rate, roll_rate = velocities
    shoulder_max, elbow_max, wrist_a_max, wrist_b_max = geometry.motor_max_rpm
    shoulder = shoulder_rate * RPM_PER_RAD_S * geometry.shoulder_ratio
    elbow = elbow_rate * RPM_PER_RAD_S * geometry.elbow_ratio
    pitch = pitch_rate * RPM_PER_RAD_S * geometry.wrist_pitch_ratio
    roll = roll_rate * RPM_PER_RAD_S * geometry.wrist_roll_ratio
    wrist_a, wrist_b = pitch + roll, pitch - roll
    if not all(math.isfinite(value) for value in (shoulder, elbow, wrist_a, wrist_b)):
        return ZERO  # e.g. 1e308 rad/s: scaling inf down would give NaN
    shoulder = max(-shoulder_max, min(shoulder_max, shoulder))
    elbow = max(-elbow_max, min(elbow_max, elbow))
    peak = max(abs(wrist_a) / wrist_a_max, abs(wrist_b) / wrist_b_max)
    if peak > 1.0:
        wrist_a, wrist_b = wrist_a / peak, wrist_b / peak
    return (shoulder, elbow, wrist_a, wrist_b)


def motor_to_joint(motor_values: tuple[float, ...] | list[float], geometry: ArmGeometry) -> tuple[float, float, float, float]:
    """Gearmotor revolutions (or rpm) -> joint revolutions (or revolutions per minute): the inverse of the gearing."""
    shoulder, elbow, wrist_a, wrist_b = motor_values
    return (shoulder / geometry.shoulder_ratio, elbow / geometry.elbow_ratio,
            (wrist_a + wrist_b) / (2.0 * geometry.wrist_pitch_ratio), (wrist_a - wrist_b) / (2.0 * geometry.wrist_roll_ratio))


def motor_revolutions_to_joint_rad(revolutions: tuple[float, ...] | list[float], geometry: ArmGeometry) -> tuple[float, ...]:
    return tuple(value * 2.0 * math.pi for value in motor_to_joint(revolutions, geometry))


def motor_rpm_to_joint_rad_s(rpm: tuple[float, ...] | list[float], geometry: ArmGeometry) -> tuple[float, ...]:
    return tuple(value / RPM_PER_RAD_S for value in motor_to_joint(rpm, geometry))
