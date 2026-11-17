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
NUMBERS = {name for name in SETTINGS if name.endswith(('_m', '_rad', '_speed', '_rate'))} | {'gps_x_m', 'gps_y_m', 'gps_z_m'}
OPTIONAL = {  # settings with a default, so that an autonomy.yaml from before they existed still loads
    'max_ground_slope_deg': 45.0,
    'obstacle_min_points': 10,
    'auto_driving': 'arcs',
    'pivot_above_deg': 115.0,
    'turn_to_goal_heading': True,
    'goal_heading_tolerance_deg': 15.0,
    'gps_x_m': 0.0,
    'gps_y_m': 0.0,
    'gps_z_m': 0.9,
    'gps_window_m': 100.0,
}
LOCALIZATIONS = ('slam', 'gps')  # who corrects the odometry's drift: RTAB-Map, or the GPS with ekf_global
GPS_WINDOW_RESOLUTION_M = 0.1  # the GPS mode's global costmap: coarser, it covers a much larger area
DRIVING_STYLES = ('arcs', 'pivot')  # auto_driving: Pure Pursuit (arcs), or DWB turning on the spot (before arcs)
OBSTACLES_TOPIC, GROUND_TOPIC = '/rover/obstacles', '/rover/ground'  # obstacles_detection's clouds, for the costmaps
GROUND_LOWEST_M = -1.0  # points further below the rover than this are not looked at
VOXEL_HEIGHT_M, MAX_VOXELS = 0.1, 16  # the costmaps' voxel layer (Nav2 allows 16 voxels in a column)
INFLATION_MARGIN_M = 0.2  # inflation_radius_m should be at least this much above the rover's reach


def load_settings(path: str | Path) -> dict[str, Any]:
    """The /** section of autonomy.yaml, checked."""
    document = yaml.safe_load(Path(path).read_text())
    settings = {**OPTIONAL, **document['/**']['ros__parameters']}
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
    slope = settings['max_ground_slope_deg']
    if isinstance(slope, bool) or not isinstance(slope, (int, float)) or not 5 <= slope <= 85:
        raise ValueError(f'{path}: max_ground_slope_deg must be between 5 and 85, got {slope!r}')
    points = settings['obstacle_min_points']
    if isinstance(points, bool) or not isinstance(points, int) or points < 1:
        raise ValueError(f'{path}: obstacle_min_points must be a whole number, 1 or more, got {points!r}')
    window = settings['gps_window_m']
    if isinstance(window, bool) or not isinstance(window, (int, float)) or not 20 <= window <= 300:
        raise ValueError(f'{path}: gps_window_m must be between 20 and 300, got {window!r}')
    if settings['auto_driving'] not in DRIVING_STYLES:
        raise ValueError(f"{path}: auto_driving must be {' or '.join(DRIVING_STYLES)}, got {settings['auto_driving']!r}")
    pivot = settings['pivot_above_deg']
    if isinstance(pivot, bool) or not isinstance(pivot, (int, float)) or not 30 <= pivot <= 180:
        raise ValueError(f'{path}: pivot_above_deg must be between 30 and 180, got {pivot!r}')
    if not isinstance(settings['turn_to_goal_heading'], bool):
        raise ValueError(f"{path}: turn_to_goal_heading must be true or false, got {settings['turn_to_goal_heading']!r}")
    tolerance = settings['goal_heading_tolerance_deg']
    if isinstance(tolerance, bool) or not isinstance(tolerance, (int, float)) or not 3 <= tolerance <= 90:
        raise ValueError(f'{path}: goal_heading_tolerance_deg must be between 3 and 90, got {tolerance!r}')
    if max(settings['rover_height_m'], settings['zed_z_m']) > VOXEL_HEIGHT_M * MAX_VOXELS:
        raise ValueError(f'{path}: rover_height_m and zed_z_m must be at most {VOXEL_HEIGHT_M * MAX_VOXELS:g} m (the '
                         "costmaps' obstacle layer reaches that high)")
    return settings


def settings_warnings(settings: dict[str, Any]) -> list[str]:
    """What load_settings accepts but is probably not meant: the launch prints these."""
    warnings = []
    reach = math.hypot(settings['rover_length_m'], settings['rover_width_m']) / 2.0  # the centre to a corner
    if settings['inflation_radius_m'] < reach + INFLATION_MARGIN_M:
        warnings.append(f"inflation_radius_m is {settings['inflation_radius_m']:g}, and the rover's outline reaches "
                        f"{reach:.2f} m from its centre: Nav2 will plan paths that graze obstacles. Use "
                        f"{reach + 0.3:.1f} or more")
    return warnings


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


def nav2_parameters(template: dict, settings: dict[str, Any], speed: float, turn_rate: float, trees: str,
                    localization: str = 'slam') -> dict:
    """config/nav2_params.yaml with the values that come from autonomy.yaml filled in; `trees` is the
    behavior_trees folder (bt_navigator loads both default trees at start, so neither may need BackUp).

    localization 'gps': no RTAB-Map map to plan on (RTAB-Map only records). The global costmap is a window of
    gps_window_m around the rover, built from what the camera has seen, and Nav2 plans through unknown space; goals
    further away are reached in stages (goal_manager)."""
    if localization not in LOCALIZATIONS:
        raise ValueError(f"localization:={localization}: use {' or '.join(LOCALIZATIONS)}")
    params = copy.deepcopy(template)

    def node(name: str) -> dict:
        return params[name]['ros__parameters']

    controller = node('controller_server')
    pivot = controller.pop('PivotController')
    if settings['auto_driving'] == 'pivot':
        controller['FollowPath'] = pivot  # DWB, under the name the behaviour trees use
        follow = controller['FollowPath']
        follow['max_vel_x'] = follow['max_speed_xy'] = speed
        follow['max_vel_theta'] = turn_rate
    else:
        follow = controller['FollowPath']
        follow['desired_linear_vel'] = speed
        follow['rotate_to_heading_angular_vel'] = turn_rate
        follow['rotate_to_heading_min_angle'] = math.radians(settings['pivot_above_deg'])
        follow['use_rotate_to_heading'] = True  # for a path that starts behind, and the turn to the arrow at the goal
    # The arrow clicked in RViz: turn to it at the goal, or arrive in whatever heading the rover came in.
    checker = controller['general_goal_checker']
    checker['yaw_goal_tolerance'] = (math.radians(settings['goal_heading_tolerance_deg'])
                                     if settings['turn_to_goal_heading'] else 3.15)
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
        if costmap == 'local_costmap' and settings['auto_driving'] == 'arcs':  # Pure Pursuit reads the local one
            follow['inflation_cost_scaling_factor'] = costmap_params['inflation_layer']['cost_scaling_factor']
        layer = costmap_params['obstacle_layer']
        height = settings['rover_height_m']
        # The voxels reach the rover's height, and above the camera: Nav2 cannot clear from a camera outside them.
        top = max(height, settings['zed_z_m'] + VOXEL_HEIGHT_M)
        layer['z_resolution'] = VOXEL_HEIGHT_M
        layer['z_voxels'] = min(MAX_VOXELS, math.ceil(top / VOXEL_HEIGHT_M - 1e-9))
        layer['max_obstacle_height'] = height
        source = layer['obstacles']
        source['topic'] = OBSTACLES_TOPIC
        source['min_obstacle_height'] = settings['min_obstacle_height_m']
        source['max_obstacle_height'] = height
        source['obstacle_max_range'] = settings['obstacle_range_m']
        source['raytrace_max_range'] = settings['obstacle_range_m'] + 0.5
        ground = layer['ground']  # clearing only, down to the ground
        ground['topic'] = GROUND_TOPIC
        ground['max_obstacle_height'] = height
        ground['raytrace_max_range'] = settings['obstacle_range_m'] + 0.5
    if localization == 'gps':
        global_costmap = params['global_costmap']['global_costmap']['ros__parameters']
        global_costmap['plugins'] = [plugin for plugin in global_costmap['plugins'] if plugin != 'static_layer']
        global_costmap.pop('static_layer', None)
        global_costmap['rolling_window'] = True
        global_costmap['width'] = global_costmap['height'] = int(settings['gps_window_m'])
        global_costmap['resolution'] = GPS_WINDOW_RESOLUTION_M
    return params


def segmentation_parameters(settings: dict[str, Any]) -> dict[str, str]:
    """How RTAB-Map's code tells ground from obstacles in the camera's points, for the map (rtabmap) and for the
    costmaps (obstacles_detection) alike: a surface facing up within max_ground_slope_deg is ground, the rest is an
    obstacle if at least obstacle_min_points points of it hang together. Heights are from base_footprint.

    Grid/MaxGroundHeight stays 0 (off). It does two things at once: the ground is taken only if it begins below it,
    and every obstacle point below it is dropped. Set to min_obstacle_height_m (0.1), a rover tilted 9 degrees nose
    down, or a camera pitched down that much more than zed_pitch_rad says, has no ground left: the nearest ground in
    view is above 0.1 m then, and the floor goes into the map as an obstacle (in the lab, 390 of the 512 cells before
    a box at 9 degrees; none with 0, also at 12). Set higher, low obstacles vanish. The costmaps leave out what is
    lower than min_obstacle_height_m themselves (their obstacle source's height window)."""
    return {
        'Grid/NormalsSegmentation': 'true',
        'Grid/MaxGroundAngle': f'{settings["max_ground_slope_deg"]:g}',
        'Grid/MinClusterSize': str(settings['obstacle_min_points']),
        'Grid/MinGroundHeight': f'{GROUND_LOWEST_M:g}',
        'Grid/MaxGroundHeight': '0',
        'Grid/MaxObstacleHeight': f'{settings["rover_height_m"]:g}',
        # The rover's own parts in the camera's view (a wheel, the nose, when it is tilted down) are not obstacles.
        'Grid/FootprintLength': f'{settings["rover_length_m"]:g}',
        'Grid/FootprintWidth': f'{settings["rover_width_m"]:g}',
        'Grid/FootprintHeight': f'{settings["rover_height_m"]:g}',
    }


def obstacle_parameters(settings: dict[str, Any]) -> dict[str, Any]:
    """rtabmap_util's obstacles_detection: the ZED's point cloud split into ground and obstacles for the costmaps. It
    publishes both clouds for every cloud it gets (empty ones too), which is what drive_mux and the costmaps'
    expected_update_rate watch."""
    return {
        'frame_id': 'base_footprint',
        'wait_for_transform': 0.2,
        'qos': 2,  # best effort, in and out: takes the ZED's cloud whatever its QoS, as the costmaps' subscriptions do
        'Grid/RangeMax': f'{settings["obstacle_range_m"] + 0.5:g}',
        'Grid/CellSize': '0.05',
        **segmentation_parameters(settings),
    }


def rtabmap_parameters(settings: dict[str, Any], database: str, localization: str = 'slam') -> dict[str, Any]:
    """RTAB-Map maps continuously (manual and AUTO) from the ZED's colour and depth, odometry from TF (the EKF).
    In the GPS mode it only records (the map and the GPS fixes, in its database): ekf_global corrects the drift."""
    return {
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_odom_info': False,
        # The ZED stamps a colour image, its depth image and its camera info with the same time: pair them by exactly
        # that time (as RTAB-Map's own ZED example does). Pairing by nearest time puts the depth of one frame on the
        # colour of another whenever a message is lost, and at 7.4 MB a pair they are.
        'approx_sync': False,
        'sync_queue_size': 10,
        'topic_queue_size': 10,
        'qos_image': 2,  # best effort: receives from the ZED whatever its QoS
        'qos_camera_info': 2,
        'wait_for_transform': 0.5,
        'database_path': database,
        'publish_tf': localization == 'slam',  # map -> odom; in the GPS mode ekf_global publishes it
        'Reg/Force3DoF': 'true',  # a ground rover: x, y, yaw
        'RGBD/NeighborLinkRefining': 'true',
        'Grid/Sensor': '1',  # the grid from the depth image (Grid/FromDepth before 0.21; the old name only warns)
        'Grid/RangeMax': f'{settings["obstacle_range_m"] + 2.0:g}',
        **segmentation_parameters(settings),
        'Grid/RayTracing': 'true',
        # The map is at least this wide (unknown space around what has been seen), so Nav2 can plan to a goal the
        # rover has not mapped yet; its global costmap is the size of this map.
        'GridGlobal/MinSize': '40',
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
        'gps_x': f'{settings["gps_x_m"]:g}',
        'gps_y': f'{settings["gps_y_m"]:g}',
        'gps_z': f'{settings["gps_z_m"]:g}',
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
