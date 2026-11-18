import math

import pytest

from arm_pilot_redemption.kinematics import (RPM_PER_RAD_S, ArmGeometry, joint_velocity_to_motor_rpm, motor_revolutions_to_joint_rad,
                                             motor_rpm_to_joint_rad_s)

ARM = ArmGeometry((117.0, 117.0, 117.0, 117.0), shoulder_ratio=30.0, elbow_ratio=20.0, wrist_pitch_ratio=2.0, wrist_roll_ratio=1.5)


def test_pitch_joints_follow_their_ratio_and_clamp():
    shoulder, elbow, wrist_a, wrist_b = joint_velocity_to_motor_rpm((0.1, -0.2, 0.0, 0.0), ARM)
    assert shoulder == pytest.approx(0.1 * RPM_PER_RAD_S * 30.0)
    assert elbow == pytest.approx(-0.2 * RPM_PER_RAD_S * 20.0)
    assert (wrist_a, wrist_b) == (0.0, 0.0)
    assert joint_velocity_to_motor_rpm((100.0, -100.0, 0.0, 0.0), ARM)[:2] == (117.0, -117.0)


def test_wrist_pitch_turns_both_motors_the_same_way_and_roll_opposite():
    _, _, a, b = joint_velocity_to_motor_rpm((0.0, 0.0, 1.0, 0.0), ARM)
    assert a == pytest.approx(b) and a > 0
    _, _, a, b = joint_velocity_to_motor_rpm((0.0, 0.0, 0.0, 1.0), ARM)
    assert a == pytest.approx(-b) and a > 0


def test_wrist_saturation_keeps_the_pitch_to_roll_ratio():
    pitch, roll = 4.0, 6.0  # far beyond what the motors can do together
    _, _, a, b = joint_velocity_to_motor_rpm((0.0, 0.0, pitch, roll), ARM)
    assert max(abs(a), abs(b)) == pytest.approx(117.0)
    back = motor_rpm_to_joint_rad_s((0.0, 0.0, a, b), ARM)
    assert back[3] / back[2] == pytest.approx(roll / pitch)


def test_motor_to_joint_inverts_the_gearing():
    velocities = (0.05, -0.1, 0.3, -2.0)
    rpm = joint_velocity_to_motor_rpm(velocities, ARM)
    assert motor_rpm_to_joint_rad_s(rpm, ARM) == pytest.approx(velocities)
    # One shoulder-motor revolution = 1/30 shoulder revolution; wrist_a and wrist_b one turn each way = pure roll.
    joints = motor_revolutions_to_joint_rad((30.0, 0.0, 1.5, -1.5), ARM)
    assert joints == pytest.approx((2 * math.pi, 0.0, 0.0, 2 * math.pi))


def test_top_speeds_put_the_motors_at_full_pwm():
    top = ARM.max_joint_velocity
    for joint in range(4):
        command = [0.0] * 4
        command[joint] = top[joint]
        assert max(abs(rpm) for rpm in joint_velocity_to_motor_rpm(command, ARM)) == pytest.approx(117.0)


def test_bad_inputs_give_zero():
    assert joint_velocity_to_motor_rpm((float('nan'), 0.0, 0.0, 0.0), ARM) == (0.0, 0.0, 0.0, 0.0)
    assert joint_velocity_to_motor_rpm((0.0, 0.0, 1e308, 1e308), ARM) == (0.0, 0.0, 0.0, 0.0)
    assert joint_velocity_to_motor_rpm((1.0, 1.0), ARM) == (0.0, 0.0, 0.0, 0.0)


def test_geometry_is_validated():
    with pytest.raises(ValueError):
        ArmGeometry((117.0, 117.0, 117.0), 1.0, 1.0, 1.0, 1.0)
    with pytest.raises(ValueError):
        ArmGeometry((117.0, 117.0, 117.0, 117.0), 0.0, 1.0, 1.0, 1.0)
    with pytest.raises(ValueError):
        ArmGeometry((117.0, 117.0, -1.0, 117.0), 1.0, 1.0, 1.0, 1.0)
