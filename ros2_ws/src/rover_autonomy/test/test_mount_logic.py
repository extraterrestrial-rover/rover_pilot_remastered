"""The camera's real mount from the ground it sees (mount_logic)."""
import math
import struct

import numpy as np
import pytest

from rover_autonomy.mount_logic import compare, fit_ground, points_from_cloud, tilt_of, up_in_frame


def ground_seen(height, pitch_deg=0.0, roll_deg=0.0, wall_at=None, noise=0.005, seed=1):
    """Points of flat ground, 0.8 to 3.5 m ahead, as a camera `height` above it sees them when it is pitched down and
    rolled by these angles; with wall_at, also a wall across the view at that distance."""
    random = np.random.default_rng(seed)
    x, y = random.uniform(0.8, 3.5, 4000), random.uniform(-1.5, 1.5, 4000)
    world = np.stack([x, y, np.full_like(x, -height)], axis=1)
    if wall_at is not None:
        wall = np.stack([np.full(1500, wall_at), random.uniform(-1.5, 1.5, 1500), random.uniform(-height, 0.6, 1500)], axis=1)
        world = np.concatenate([world, wall])
    world += random.normal(0.0, noise, world.shape)
    pitch, roll = math.radians(pitch_deg), math.radians(roll_deg)
    pitch_matrix = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    roll_matrix = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    return np.einsum('ij,jk->ik', world, np.einsum('ij,jk->ik', pitch_matrix, roll_matrix))  # camera = R^T . world, for row vectors


@pytest.mark.parametrize('height, pitch, roll', [(0.6, 0.0, 0.0), (0.6, 10.0, 0.0), (0.45, -4.0, 0.0), (0.8, 15.0, 3.0),
                                                 (0.6, 0.0, -5.0)])
def test_the_ground_gives_the_cameras_height_and_tilt(height, pitch, roll):
    normal, found_height, share = fit_ground(ground_seen(height, pitch, roll))
    found_pitch, found_roll = tilt_of(normal)
    assert found_height == pytest.approx(height, abs=0.01)
    assert math.degrees(found_pitch) == pytest.approx(pitch, abs=0.5)
    assert math.degrees(found_roll) == pytest.approx(roll, abs=0.5)
    assert share > 0.9


def test_a_wall_in_view_is_not_the_ground():
    normal, height, share = fit_ground(ground_seen(0.6, 8.0, wall_at=2.5))
    assert height == pytest.approx(0.6, abs=0.01) and math.degrees(tilt_of(normal)[0]) == pytest.approx(8.0, abs=0.5)
    assert 0.5 < share < 0.9  # the wall's points are left out
    only_wall = ground_seen(0.6, wall_at=2.0)[4000:]
    assert fit_ground(only_wall) is None  # nothing level in view: no answer rather than a wrong one
    assert fit_ground(ground_seen(0.6)[:100]) is None  # a handful of points is no floor


def test_a_mount_that_differs_from_the_settings_is_reported_with_the_value_to_set():
    """The settings ship with placeholders (zed_pitch_rad 0.0). A camera really tilted 10 degrees down put the ground
    2 m ahead 35 cm too low for the costmaps, and obstacles with it."""
    agree = compare((0.61, math.radians(0.8), math.radians(-0.5)), (0.6, 0.0, 0.0), zed_z_m=0.6, zed_pitch_rad=0.0)
    assert agree == []
    off = compare((0.615, math.radians(10.0), 0.0), (0.615, 0.0, 0.0), zed_z_m=0.6, zed_pitch_rad=0.0)
    assert len(off) == 1 and '10.0 degrees further down' in off[0] and 'set zed_pitch_rad: 0.175' in off[0]
    up = compare((0.6, math.radians(5.0), 0.0), (0.6, math.radians(10.0), 0.0), zed_z_m=0.6, zed_pitch_rad=0.175)
    assert '5.0 degrees further up' in up[0] and 'set zed_pitch_rad: 0.088' in up[0]
    low = compare((0.465, 0.0, 0.0), (0.615, 0.0, 0.0), zed_z_m=0.6, zed_pitch_rad=0.0)
    assert len(low) == 1 and '15 cm lower' in low[0] and 'set zed_z_m: 0.45' in low[0]
    rolled = compare((0.6, 0.0, math.radians(4.0)), (0.6, 0.0, 0.0), zed_z_m=0.6, zed_pitch_rad=0.0)
    assert len(rolled) == 1 and 'rolled 4.0 degrees' in rolled[0] and 'right side down' in rolled[0]


def test_the_settings_expected_view_of_the_ground():
    assert up_in_frame((0.0, 0.0, 0.0, 1.0)) == (0.0, 0.0, 1.0)
    pitch = math.radians(10.0)  # the URDF's mount: rpy = (0, pitch, 0)
    found_pitch, found_roll = tilt_of(up_in_frame((0.0, math.sin(pitch / 2), 0.0, math.cos(pitch / 2))))
    assert found_pitch == pytest.approx(pitch) and found_roll == pytest.approx(0.0)
    # ... and that is what the ground looks like to a camera pitched down by that much.
    measured_pitch, _ = tilt_of(fit_ground(ground_seen(0.6, 10.0))[0])
    assert measured_pitch == pytest.approx(pitch, abs=math.radians(0.5))


def test_points_are_read_from_a_cloud_like_the_zeds():
    rows = [(1.0, 0.5, -0.6, 0.0), (float('nan'),) * 4, (2.0, -0.5, -0.6, 0.0), (9.0, 0.0, -0.6, 0.0), (0.1, 0.0, 0.0, 0.0)]
    data = b''.join(struct.pack('<ffff', *row) for row in rows)
    points = points_from_cloud(data, 16, {'x': 0, 'y': 4, 'z': 8}, len(rows))
    assert points.tolist() == [[1.0, 0.5, pytest.approx(-0.6)], [2.0, -0.5, pytest.approx(-0.6)]]  # real, 0.5 to 4 m away
    many = b''.join(struct.pack('<ffff', 1.0 + 0.0001 * i, 0.0, -0.6, 0.0) for i in range(5000))
    assert len(points_from_cloud(many, 16, {'x': 0, 'y': 4, 'z': 8}, 5000, at_most=1000)) == 1000
