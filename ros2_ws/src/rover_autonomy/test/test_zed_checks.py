"""zed_checks: what zed_odometry passes on to the EKF."""
import math

import pytest

from rover_autonomy.zed_checks import checked_pose, tracking_problem


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
