"""frames: the camera's pose -> the rover's, as zed_odometry does it."""
import math

import pytest

from rover_autonomy.frames import camera_to_base, compose, from_xyz_rpy, inverse, yaw_of


def close(a, b):
    # A quaternion and its negative are the same rotation.
    sign = 1.0 if sum(x * y for x, y in zip(a[3:], b[3:])) >= 0 else -1.0
    return a[:3] == pytest.approx(b[:3], abs=1e-9) and tuple(sign * x for x in a[3:]) == pytest.approx(b[3:], abs=1e-9)


def test_inverse_and_compose():
    pose = from_xyz_rpy(1.0, -2.0, 0.5, 0.1, -0.2, 2.5)
    assert close(compose(pose, inverse(pose)), (0, 0, 0, 0, 0, 0, 1))
    assert yaw_of(from_xyz_rpy(0, 0, 0, yaw=-1.2)) == pytest.approx(-1.2)


def test_the_camera_at_the_start_puts_the_rover_behind_it():
    mount = from_xyz_rpy(0.4, 0.0, 0.6)
    assert close(camera_to_base(from_xyz_rpy(0, 0, 0), mount), from_xyz_rpy(-0.4, 0.0, -0.6))


def test_turning_on_the_spot_keeps_the_rover_where_it_is():
    mount = from_xyz_rpy(0.4, 0.1, 0.6)
    start = camera_to_base(from_xyz_rpy(0, 0, 0), mount)
    for yaw in (0.5, 1.6, 3.0, -2.0):
        rover = compose(start, from_xyz_rpy(0, 0, 0, yaw=yaw))  # the rover turned in place by yaw
        camera = compose(rover, mount)  # where the ZED then is (it swings round)
        assert math.hypot(camera[0] - start[0], camera[1] - start[1]) > 0.4
        assert close(camera_to_base(camera, mount), rover)


def test_a_camera_looking_down_still_gives_a_level_rover():
    mount = from_xyz_rpy(0.4, 0.0, 0.6, pitch=0.2)
    rover = from_xyz_rpy(3.0, 1.0, 0.0, yaw=0.7)
    base = camera_to_base(compose(rover, mount), mount)
    assert close(base, rover) and yaw_of(base) == pytest.approx(0.7)
