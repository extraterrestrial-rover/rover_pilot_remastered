"""zed_checks: what zed_odometry passes on to the EKF."""
import math

import pytest

from rover_autonomy.frames import compose, from_xyz_rpy, yaw_of
from rover_autonomy.zed_checks import ContinuousOdometry, checked_pose, tracking_problem


def test_a_real_pose_is_passed_on_with_a_unit_quaternion():
    assert checked_pose((1.0, 2.0, 0.5, 0.0, 0.0, 0.0, 1.0)) == (1.0, 2.0, 0.5, 0.0, 0.0, 0.0, 1.0)
    pose = checked_pose((0.0, 0.0, 0.0, 0.0, 0.0, 0.7, 0.7))  # rounded by the sender: norm 0.99
    assert math.sqrt(sum(value * value for value in pose[3:])) == pytest.approx(1.0)


@pytest.mark.parametrize('bad', [
    (float('nan'), 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    (0.0, float('inf'), 0.0, 0.0, 0.0, 0.0, 1.0),
    (0.0, 0.0, 0.0, 0.0, 0.0, float('nan'), 1.0),
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),  # an empty message
    (0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 2.0),
    (0.0, 0.0, 0.0, 1.0),
])
def test_what_is_not_a_pose_is_withheld(bad):
    assert checked_pose(bad) is None


def test_odometry_counts_only_while_the_zed_tracks():
    assert tracking_problem(0, 3) == ''  # tracking; spatial memory off, as this stack starts it
    assert tracking_problem(0, 6) == ''
    assert tracking_problem(1, 3) == 'unavailable'
    assert 'features' in tracking_problem(2, 3)
    assert tracking_problem(0, 7) == 'lost'
    assert tracking_problem(9, 3) == 'odometry status 9'


def test_real_motion_passes_through_unchanged():
    odometry = ContinuousOdometry()
    for step in range(100):  # 1.0 m/s and 1.5 rad/s at 30 Hz
        t = step / 30
        pose = from_xyz_rpy(t, 0.2 * t, 0.0, yaw=1.5 * t)
        out, jumped = odometry.update(pose, t)
        assert not jumped and out == pytest.approx(pose)


def test_a_reset_to_the_start_is_taken_out():
    # The rover turned 90 degrees right on the spot; the ZED recognises the place and resets its odometry to the start.
    odometry = ContinuousOdometry()
    for step in range(31):
        odometry.update(from_xyz_rpy(0.0, 0.0, 0.0, yaw=-math.pi / 2 * step / 30), step / 30)
    out, jumped = odometry.update(from_xyz_rpy(0.0, 0.0, 0.0), 31 / 30)  # identity: the reset
    assert jumped and odometry.jumps == 1
    assert yaw_of(out) == pytest.approx(-math.pi / 2, abs=0.06)  # still facing right, not snapped back
    # From there on, real motion is added to the continuous pose.
    out, jumped = odometry.update(from_xyz_rpy(0.0, 0.0, 0.0, yaw=0.3), 2.0)
    assert not jumped and yaw_of(out) == pytest.approx(-math.pi / 2 + 0.3, abs=0.06)
    out, _ = odometry.update(compose(from_xyz_rpy(0.0, 0.0, 0.0, yaw=0.3), from_xyz_rpy(0.5, 0.0, 0.0)), 2.5)
    heading = -math.pi / 2 + 0.3
    assert out[:2] == pytest.approx((0.5 * math.cos(heading), 0.5 * math.sin(heading)), abs=0.06)


def test_a_position_jump_is_taken_out_too():
    odometry = ContinuousOdometry()
    odometry.update(from_xyz_rpy(5.0, 2.0, 0.0, yaw=1.0), 0.0)
    out, jumped = odometry.update(from_xyz_rpy(0.0, 0.0, 0.0), 0.05)
    assert jumped and out == pytest.approx(from_xyz_rpy(5.0, 2.0, 0.0, yaw=1.0))
    out, jumped = odometry.update(from_xyz_rpy(0.04, 0.0, 0.0), 0.1)  # 4 cm forward in 50 ms: real
    assert not jumped and out[:2] == pytest.approx((5.0 + 0.04 * math.cos(1.0), 2.0 + 0.04 * math.sin(1.0)))

