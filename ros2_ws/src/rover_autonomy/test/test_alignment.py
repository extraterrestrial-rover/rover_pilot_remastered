"""Finding north from the rover's own track (alignment_logic)."""
import math
import random

import pytest

from rover_autonomy.alignment_logic import Alignment, AlignmentConfig, rigid_fit


def drive(alignment, heading_offset_deg, path, *, noise=0.0, wander=0.0, seed=1, start=(3.0, -7.0), bad=()):
    """Feed the alignment the antenna's track along `path` (odometry x, y points), as the GPS (east, north) sees it:
    the odometry frame is turned by heading_offset_deg from east and starts at `start`. noise: white, per fix;
    wander: a slowly drifting bias (the usual GPS error). bad: indexes of fixes 15 m off (multipath)."""
    rng = random.Random(seed)
    theta = math.radians(heading_offset_deg)
    bias = [0.0, 0.0]
    changed = 0
    for i, (x, y) in enumerate(path):
        bias = [b + rng.gauss(0.0, wander) for b in bias]
        east = math.cos(theta) * x - math.sin(theta) * y + start[0] + bias[0] + rng.gauss(0.0, noise)
        north = math.sin(theta) * x + math.cos(theta) * y + start[1] + bias[1] + rng.gauss(0.0, noise)
        if i in bad:
            east += 15.0
        changed += alignment.add((x, y), (east, north))
    return changed


def straight(length, step=0.5):
    return [(i * step, 0.0) for i in range(int(length / step) + 1)]


def test_the_fit_finds_the_rotation_and_shift():
    theta, tx, ty, rms = rigid_fit([(0, 0), (1, 0), (0, 2)], [(5, 5), (5, 6), (3, 5)])
    assert math.degrees(theta) == pytest.approx(90.0) and (tx, ty) == (pytest.approx(5.0), pytest.approx(5.0)) and rms < 1e-9


def test_no_heading_before_the_rover_has_driven_far_enough():
    alignment = Alignment()
    assert alignment.status() == 'GPS heading unknown: no GPS fix yet'
    drive(alignment, 40.0, [(0.0, 0.0)] * 30)  # standing: the same pair again and again
    assert len(alignment.odom) == 1 and not alignment.ready
    drive(alignment, 40.0, straight(12.0))
    assert not alignment.ready and 'drive 3 m more' in alignment.status()


@pytest.mark.parametrize('offset', [0.0, 37.0, -120.0, 179.0])
def test_after_15_m_the_heading_is_known(offset):
    alignment = Alignment()
    drive(alignment, offset, straight(16.0), noise=0.3, wander=0.05)
    assert alignment.ready
    error = math.degrees(math.remainder(alignment.fit.theta - math.radians(offset), 2 * math.pi))
    assert abs(error) < 4.0, error
    assert 'GPS ready' in alignment.status()


def test_a_curved_track_works_as_well_and_goals_land_in_place():
    alignment = Alignment()
    path = [(10.0 * math.sin(a / 20), 10.0 * (1 - math.cos(a / 20))) for a in range(0, 50)]  # 10 m radius, 140 degrees
    drive(alignment, 75.0, path, noise=0.3, wander=0.05)
    assert alignment.ready
    # A GPS goal 50 m east, 20 m north of the start, back in the odometry frame:
    theta, start = math.radians(75.0), (3.0, -7.0)
    gx, gy = alignment.fit.enu_to_odom(start[0] + 50.0, start[1] + 20.0)
    truth = (math.cos(theta) * 50.0 + math.sin(theta) * 20.0, -math.sin(theta) * 50.0 + math.cos(theta) * 20.0)
    assert math.hypot(gx - truth[0], gy - truth[1]) < 3.5  # about the GPS's own error at that distance


def test_refits_while_short_then_frozen():
    alignment = Alignment(AlignmentConfig(min_track_m=15.0, refit_every_m=10.0, freeze_track_m=40.0))
    changes = drive(alignment, 20.0, straight(60.0), noise=0.2)
    assert changes == 4 and alignment.frozen  # at 15, 25, 35 and 45 m, the last past 40 m: kept from then on
    before = alignment.fit
    assert not alignment.add((100.0, 0.0), (0.0, 0.0)) and alignment.fit == before


def test_multipath_fixes_are_left_out():
    alignment = Alignment()
    drive(alignment, 30.0, straight(16.0), noise=0.2, bad=(10, 11, 12))
    error = math.degrees(math.remainder(alignment.fit.theta - math.radians(30.0), 2 * math.pi))
    assert abs(error) < 4.0 and alignment.fit.points == 31 - 3  # 31 fixes over the first 15 m
