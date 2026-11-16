"""Where the camera really sits, from what it sees of the ground. numpy but no ROS, so it is unit-tested.

The map and the costmaps place every point the camera sees with the mount from autonomy.yaml: its height and its pitch.
A pitch that is 3 degrees off puts the ground 2 m ahead 10 cm too high or too low, and obstacles with it. The camera
sees the ground, though: the plane through those points gives the camera's real height above the ground and its real
tilt towards it. mount_check compares that with the settings when the rover starts on flat, open ground.

Frames as in REP 103: x forward, y left, z up. Pitch is positive nose down, roll positive right side down.
"""
from __future__ import annotations

import math

import numpy as np

MIN_POINTS = 300  # fewer points on the plane than this is not a floor in view
MAX_TILT_DEG = 40.0  # a plane tilted more than this against the camera is a wall or a ramp, not the ground it stands on


def fit_ground(points: np.ndarray, tolerance_m: float = 0.03, iterations: int = 150,
               seed: int = 0) -> tuple[np.ndarray, float, float] | None:
    """The ground plane in `points` (N x 3): (its unit normal, pointing up; the height of the frame's origin above
    it; the share of the points that lie on it), or None if no such plane is in view. The plane most points lie on
    within tolerance_m wins (RANSAC), then it is fitted to all of those points."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < MIN_POINTS:
        return None
    random = np.random.default_rng(seed)
    upright = math.cos(math.radians(MAX_TILT_DEG))
    best = None
    for _ in range(iterations):
        a, b, c = points[random.choice(len(points), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1e-9:
            continue
        normal = normal / length
        if normal[2] < 0:
            normal = -normal
        if normal[2] < upright:
            continue
        # einsum, not @: macOS' BLAS raises floating-point warnings on a plain matrix product of this shape.
        on_plane = np.abs(np.einsum('ij,j->i', points - a, normal)) <= tolerance_m
        if best is None or on_plane.sum() > best.sum():
            best = on_plane
    if best is None or best.sum() < MIN_POINTS:
        return None
    ground = points[best]
    centre = ground.mean(axis=0)
    normal = np.linalg.svd(ground - centre, full_matrices=False)[2][-1]  # the direction the points spread least in
    if normal[2] < 0:
        normal = -normal
    height = float(-np.dot(normal, centre))
    if normal[2] < upright or height <= 0:
        return None
    return normal, height, float(best.sum() / len(points))


def tilt_of(normal) -> tuple[float, float]:
    """(pitch, roll) in radians of a frame in which the ground's upward normal is `normal`."""
    return math.atan2(-normal[0], normal[2]), math.atan2(normal[1], normal[2])


def compare(measured: tuple[float, float, float], expected: tuple[float, float, float], zed_z_m: float,
            zed_pitch_rad: float, angle_tolerance_deg: float = 1.5, height_tolerance_m: float = 0.05) -> list[str]:
    """What is off between the camera's (height, pitch, roll) as measured on the ground and as the settings place
    it: one line per problem, with the value to put into autonomy.yaml; none if they agree."""
    height, pitch, roll = measured
    set_height, set_pitch, set_roll = expected
    problems = []
    if abs(math.degrees(pitch - set_pitch)) > angle_tolerance_deg:
        problems.append(f'the camera looks {abs(math.degrees(pitch - set_pitch)):.1f} degrees further '
                        f'{"down" if pitch > set_pitch else "up"} than the settings say: set zed_pitch_rad: '
                        f'{zed_pitch_rad + pitch - set_pitch:.3f} (it is {zed_pitch_rad:g})')
    if abs(height - set_height) > height_tolerance_m:
        problems.append(f'the camera is {abs(height - set_height) * 100:.0f} cm {"higher" if height > set_height else "lower"} '
                        f'above the ground than the settings say: set zed_z_m: {zed_z_m + height - set_height:.2f} '
                        f'(it is {zed_z_m:g})')
    if abs(math.degrees(roll - set_roll)) > angle_tolerance_deg:
        problems.append(f'the camera is rolled {abs(math.degrees(roll - set_roll)):.1f} degrees, its '
                        f'{"right" if roll > set_roll else "left"} side down: level it on its mount (there is no '
                        'setting for roll)')
    return problems


def up_in_frame(quaternion) -> tuple[float, float, float]:
    """The parent frame's z axis seen from a child frame whose orientation in the parent is `quaternion` (x, y, z, w):
    the ground's normal as the settings expect the camera to see it."""
    x, y, z, w = quaternion
    return 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)


def points_from_cloud(data, point_step: int, offsets: dict[str, int], count: int, big_endian: bool = False,
                      nearest_m: float = 0.5, furthest_m: float = 4.0, at_most: int = 20000) -> np.ndarray:
    """The x, y, z (float32 at `offsets`) of a PointCloud2's data as an N x 3 array: only real points between
    nearest_m and furthest_m from the camera, thinned out evenly to at_most."""
    kind = '>f4' if big_endian else '<f4'
    layout = np.dtype({'names': ['x', 'y', 'z'], 'formats': [kind] * 3, 'offsets': [offsets[name] for name in 'xyz'],
                       'itemsize': point_step})
    raw = np.frombuffer(data, dtype=layout, count=count)
    points = np.stack([raw['x'], raw['y'], raw['z']], axis=1).astype(np.float64)
    points = points[np.isfinite(points).all(axis=1)]
    distance = np.linalg.norm(points, axis=1)
    points = points[(distance >= nearest_m) & (distance <= furthest_m)]
    if len(points) > at_most:
        points = points[::math.ceil(len(points) / at_most)]
    return points
