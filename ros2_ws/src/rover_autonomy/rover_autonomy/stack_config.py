"""Builds the autonomy stack's parameters from config/autonomy.yaml, at launch time. No ROS, so it is unit-tested.

autonomy.yaml holds what an operator changes (AUTO speed, the rover's outline, the ZED's mount and topics, the map
file); Nav2's, RTAB-Map's and the URDF's own settings are derived from it here, so a value is never kept in two
places.
"""
from __future__ import annotations

import copy
import math
import shutil
import time
from pathlib import Path
from typing import Any

import yaml

SETTINGS = (
    'auto_max_speed', 'auto_max_turn_rate', 'rover_length_m', 'rover_width_m', 'rover_height_m', 'inflation_radius_m',
    'min_obstacle_height_m', 'obstacle_range_m', 'zed_x_m', 'zed_y_m', 'zed_z_m', 'zed_pitch_rad', 'zed_camera_name',
    'zed_odom_topic', 'zed_cloud_topic', 'zed_rgb_topic', 'zed_rgb_info_topic', 'zed_depth_topic', 'map_database',
)
NUMBERS = {name for name in SETTINGS if name.endswith(('_m', '_rad', '_speed', '_rate'))}


def load_settings(path: str | Path) -> dict[str, Any]:
    """The /** section of autonomy.yaml, checked."""
    document = yaml.safe_load(Path(path).read_text())
    settings = dict(document['/**']['ros__parameters'])
    missing = [name for name in SETTINGS if name not in settings]
    if missing:
        raise ValueError(f'{path}: missing {", ".join(missing)}')
    for name in NUMBERS:
        value = settings[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{path}: {name} must be a number, got {value!r}')
    for name in ('auto_max_speed', 'auto_max_turn_rate', 'rover_length_m', 'rover_width_m', 'rover_height_m',
                 'obstacle_range_m'):
        if settings[name] <= 0:
            raise ValueError(f'{path}: {name} must be positive, got {settings[name]}')
    if not 0 <= settings['min_obstacle_height_m'] < settings['rover_height_m']:
        raise ValueError(f'{path}: min_obstacle_height_m must be between 0 and rover_height_m')
    return settings


def footprint(length_m: float, width_m: float) -> str:
    """The rover's outline for the costmaps: a rectangle centred on base_footprint, x forward, y left."""
    x, y = length_m / 2.0, width_m / 2.0
    return f'[[{x:g}, {y:g}], [{x:g}, {-y:g}], [{-x:g}, {-y:g}], [{-x:g}, {y:g}]]'


def checked_limits(settings: dict[str, Any], max_speed: float | None, top_speed: float, top_turn_rate: float) -> tuple[float, float]:
    """AUTO's speed (the launch argument wins over autonomy.yaml) and turn rate, within what the rover can do."""
    speed = settings['auto_max_speed'] if max_speed is None else max_speed
    turn_rate = settings['auto_max_turn_rate']
    if not (math.isfinite(speed) and 0 < speed <= top_speed):
        raise ValueError(f'AUTO max speed {speed} m/s: must be above 0 and at most the rover\'s top speed {top_speed:.2f} m/s')
    if not 0 < turn_rate <= top_turn_rate:
        raise ValueError(f'auto_max_turn_rate {turn_rate} rad/s: must be above 0 and at most {top_turn_rate:.2f} rad/s')
    return speed, turn_rate


def nav2_parameters(template: dict, settings: dict[str, Any], speed: float, turn_rate: float, trees: str) -> dict:
    """config/nav2_params.yaml with the values that come from autonomy.yaml filled in; `trees` is the
    behavior_trees folder (bt_navigator loads both default trees at start, so neither may need BackUp)."""
    params = copy.deepcopy(template)

    def node(name: str) -> dict:
        return params[name]['ros__parameters']

    follow = node('controller_server')['FollowPath']
    follow['max_vel_x'] = follow['max_speed_xy'] = speed
    follow['max_vel_theta'] = turn_rate
    smoother = node('velocity_smoother')
    smoother['max_velocity'] = [speed, 0.0, turn_rate]
    smoother['min_velocity'] = [0.0, 0.0, -turn_rate]
    node('behavior_server')['max_rotational_vel'] = turn_rate
    node('bt_navigator')['default_nav_to_pose_bt_xml'] = f'{trees}/navigate_to_pose.xml'
    node('bt_navigator')['default_nav_through_poses_bt_xml'] = f'{trees}/navigate_through_poses.xml'
    outline = footprint(settings['rover_length_m'], settings['rover_width_m'])
    for costmap in ('local_costmap', 'global_costmap'):
        costmap_params = params[costmap][costmap]['ros__parameters']
        costmap_params['footprint'] = outline
        costmap_params['inflation_layer']['inflation_radius'] = settings['inflation_radius_m']
        layer = costmap_params['obstacle_layer']
        source = layer['zed']
        source['topic'] = settings['zed_cloud_topic']
        source['min_obstacle_height'] = settings['min_obstacle_height_m']
        source['max_obstacle_height'] = settings['rover_height_m']
        source['obstacle_max_range'] = settings['obstacle_range_m']
        source['raytrace_max_range'] = settings['obstacle_range_m'] + 0.5
        ground = layer['zed_ground']  # clearing only, through the ground points as well
        ground['topic'] = settings['zed_cloud_topic']
        ground['max_obstacle_height'] = settings['rover_height_m']
        ground['raytrace_max_range'] = settings['obstacle_range_m'] + 0.5
    return params


def rtabmap_parameters(settings: dict[str, Any], database: str) -> dict[str, Any]:
    """RTAB-Map maps continuously (manual and AUTO) from the ZED's colour and depth, odometry from TF (the EKF)."""
    return {
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_odom_info': False,
        'approx_sync': True,
        'sync_queue_size': 30,
        'topic_queue_size': 30,
        'qos_image': 2,  # best effort: receives from the ZED whatever its QoS
        'qos_camera_info': 2,
        'wait_for_transform': 0.5,
        'database_path': database,
        'publish_tf': True,  # map -> odom
        'Reg/Force3DoF': 'true',  # a ground rover: x, y, yaw
        'RGBD/NeighborLinkRefining': 'true',
        'Grid/FromDepth': 'true',
        'Grid/RangeMax': f'{settings["obstacle_range_m"] + 2.0:g}',
        'Grid/MinGroundHeight': '-0.5',
        'Grid/MaxGroundHeight': f'{settings["min_obstacle_height_m"]:g}',
        'Grid/MaxObstacleHeight': f'{settings["rover_height_m"]:g}',
        'Grid/RayTracing': 'true',
        'Vis/MaxFeatures': '600',
    }


def urdf_mappings(settings: dict[str, Any], wheel_diameter_m: float) -> dict[str, str]:
    """xacro arguments for urdf/rover.urdf.xacro."""
    return {
        'wheel_radius': f'{wheel_diameter_m / 2.0:g}',
        'length': f'{settings["rover_length_m"]:g}',
        'width': f'{settings["rover_width_m"]:g}',
        'zed_name': str(settings['zed_camera_name']),
        'zed_x': f'{settings["zed_x_m"]:g}',
        'zed_y': f'{settings["zed_y_m"]:g}',
        'zed_z': f'{settings["zed_z_m"]:g}',
        'zed_pitch': f'{settings["zed_pitch_rad"]:g}',
    }


def prepare_map(database: str | Path, continue_map: bool, now: float | None = None) -> tuple[Path, Path | None]:
    """A fresh run moves the previous map aside (rover_map-<time>.db) instead of deleting it; continuing keeps it."""
    path = Path(database).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    if continue_map or not path.exists():
        return path, None
    stamp = time.strftime('%Y%m%d-%H%M%S', time.localtime(time.time() if now is None else now))
    kept = path.with_name(f'{path.stem}-{stamp}{path.suffix}')
    shutil.move(str(path), str(kept))
    return path, kept


def write_yaml(document: dict, path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(document, sort_keys=False))
    return path
