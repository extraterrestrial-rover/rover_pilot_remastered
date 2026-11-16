"""stack_config: the autonomy stack's parameters built from config/autonomy.yaml at launch time."""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from rover_autonomy import stack_config
from rover_autonomy.mux_logic import MuxConfig

PACKAGE = Path(__file__).resolve().parents[1]
TEMPLATE = yaml.safe_load((PACKAGE / 'config' / 'nav2_params.yaml').read_text())
SETTINGS = stack_config.load_settings(PACKAGE / 'config' / 'autonomy.yaml')
AUTONOMY = yaml.safe_load((PACKAGE / 'config' / 'autonomy.yaml').read_text())


def build(speed=0.5, turn_rate=0.8, **changes):
    return stack_config.nav2_parameters(TEMPLATE, dict(SETTINGS, **changes), speed, turn_rate, '/trees')


def test_auto_speed_and_turn_rate_reach_every_nav2_limit():
    params = build(speed=0.3, turn_rate=0.6)
    follow = params['controller_server']['ros__parameters']['FollowPath']
    assert follow['max_vel_x'] == follow['max_speed_xy'] == 0.3 and follow['max_vel_theta'] == 0.6
    assert follow['min_vel_x'] == 0.0  # never reverses
    smoother = params['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'] == [0.3, 0.0, 0.6] and smoother['min_velocity'] == [0.0, 0.0, -0.6]
    assert params['behavior_server']['ros__parameters']['max_rotational_vel'] == 0.6
    navigator = params['bt_navigator']['ros__parameters']
    assert navigator['default_nav_to_pose_bt_xml'] == '/trees/navigate_to_pose.xml'
    assert navigator['default_nav_through_poses_bt_xml'] == '/trees/navigate_through_poses.xml'
    assert TEMPLATE['controller_server']['ros__parameters']['FollowPath']['max_vel_x'] == 0.5  # template untouched


def test_the_outline_and_the_obstacles_reach_both_costmaps():
    params = build(rover_length_m=1.2, rover_width_m=0.9, rover_height_m=0.7, obstacle_range_m=2.5,
                   min_obstacle_height_m=0.15, inflation_radius_m=0.8)
    for name in ('local_costmap', 'global_costmap'):
        costmap = params[name][name]['ros__parameters']
        assert yaml.safe_load(costmap['footprint']) == [[0.6, 0.45], [0.6, -0.45], [-0.6, -0.45], [-0.6, 0.45]]
        assert costmap['inflation_layer']['inflation_radius'] == 0.8
        obstacles = costmap['obstacle_layer']['obstacles']
        assert obstacles['topic'] == stack_config.OBSTACLES_TOPIC and obstacles['marking'] and obstacles['clearing']
        assert (obstacles['min_obstacle_height'], obstacles['max_obstacle_height']) == (0.15, 0.7)
        assert obstacles['obstacle_max_range'] == 2.5 and obstacles['raytrace_max_range'] == 3.0
        assert costmap['robot_base_frame'] == 'base_footprint'


def test_the_costmaps_do_not_tell_ground_from_obstacles_by_height():
    """They took the ZED's whole cloud and called every point above 0.1 m an obstacle. With the rover tilted 3 degrees
    nose down (braking, a wheel on a stone) or the camera's pitch setting that far off, the ground 2 m ahead is above
    0.1 m: in the lab that put 616 lethal cells on open ground in the local costmap, 1816 at 5 degrees. They now get
    the clouds of obstacles_detection, which decides by the direction a surface faces."""
    params = build()
    for name in ('local_costmap', 'global_costmap'):
        layer = params[name][name]['ros__parameters']['obstacle_layer']
        topics = {layer[source]['topic'] for source in layer['observation_sources'].split()}
        assert topics == {stack_config.OBSTACLES_TOPIC, stack_config.GROUND_TOPIC}
        assert SETTINGS['zed_cloud_topic'] not in topics
    detector = stack_config.obstacle_parameters(dict(SETTINGS, max_ground_slope_deg=30.0, obstacle_min_points=6,
                                                     rover_height_m=0.7, obstacle_range_m=2.5))
    assert detector['Grid/NormalsSegmentation'] == 'true' and detector['frame_id'] == 'base_footprint'
    assert (detector['Grid/MaxGroundAngle'], detector['Grid/MinClusterSize']) == ('30', '6')
    assert (detector['Grid/MaxObstacleHeight'], detector['Grid/RangeMax']) == ('0.7', '3')
    assert all(isinstance(value, str) for key, value in detector.items() if key.startswith('Grid/'))  # RTAB-Map's own


def test_the_ground_is_told_by_its_direction_alone():
    """RTAB-Map's Grid/MaxGroundHeight does two things: the ground is taken only if it begins below it, and every
    obstacle point below it is dropped. At 0.1 m, a rover tilted 5 degrees nose down saw no ground at all (the nearest
    ground in view is above 0.1 m then), and every level surface in view, the floor, became an obstacle. At 0.3 m, a
    0.3 m stake or plank was no obstacle any more: in the lab neither reached the costmaps. 0 switches both off."""
    for parameters in (stack_config.rtabmap_parameters(SETTINGS, '/maps/a.db'), stack_config.obstacle_parameters(SETTINGS)):
        assert float(parameters['Grid/MaxGroundHeight']) == 0.0
        assert float(parameters['Grid/MinGroundHeight']) <= -1.0  # and a tilt the other way does not hide the ground
    # What is too low to matter is left out by the costmaps instead, in their obstacle source's height window.
    costmap = build(min_obstacle_height_m=0.15)['local_costmap']['local_costmap']['ros__parameters']
    assert costmap['obstacle_layer']['obstacles']['min_obstacle_height'] == 0.15
    # The map and the costmaps tell ground from obstacles in the same way.
    shared = stack_config.segmentation_parameters(SETTINGS)
    assert shared.items() <= stack_config.rtabmap_parameters(SETTINGS, '/maps/a.db').items()
    assert shared.items() <= stack_config.obstacle_parameters(SETTINGS).items()


def test_obstacles_are_kept_and_cleared_in_3d():
    """With the 2D obstacle layer a 0.3 m stake vanished from both costmaps once the camera, 0.25 m from it, looked
    over it: the rays to the ground behind it cleared its cells. A voxel layer clears only what a ray really crosses."""
    params = build(rover_height_m=0.7, zed_z_m=0.9)
    for name in ('local_costmap', 'global_costmap'):
        layer = params[name][name]['ros__parameters']['obstacle_layer']
        assert layer['plugin'] == 'nav2_costmap_2d::VoxelLayer'
        top = layer['origin_z'] + layer['z_voxels'] * layer['z_resolution']
        assert top >= 0.9 + 0.1 - 1e-9  # above the camera: Nav2 does not clear from a camera outside the voxels
        assert layer['z_voxels'] <= 16 and layer['max_obstacle_height'] == 0.7
    low = build(rover_height_m=0.8, zed_z_m=0.6)['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']
    assert low['z_voxels'] == 8


def test_obstacles_that_have_gone_are_cleared_in_open_ground():
    """Clearing only along rays to obstacle points left anything that had gone in open ground (nothing behind it)
    in the costmap for good. A second source clears along rays to the ground points, and never marks."""
    params = build(rover_height_m=0.7, obstacle_range_m=2.5)
    for name in ('local_costmap', 'global_costmap'):
        layer = params[name][name]['ros__parameters']['obstacle_layer']
        assert layer['observation_sources'].split() == ['obstacles', 'ground']
        ground = layer['ground']
        assert ground['topic'] == stack_config.GROUND_TOPIC and ground['marking'] is False and ground['clearing'] is True
        assert ground['min_obstacle_height'] < 0.0 and ground['max_obstacle_height'] == 0.7
        assert ground['raytrace_max_range'] == 3.0


def test_nav2_waits_when_the_camera_stops_sending():
    """A costmap source without expected_update_rate counts as current for ever: Nav2 would plan and drive on the
    obstacles it last saw."""
    for name in ('local_costmap', 'global_costmap'):
        layer = TEMPLATE[name][name]['ros__parameters']['obstacle_layer']
        for source in layer['observation_sources'].split():
            assert 0.5 <= layer[source]['expected_update_rate'] <= 3.0, (name, source)


def test_paths_keep_a_distance_from_obstacles():
    """With inflation_radius_m 0.7, below the 1 x 1 m outline's reach of 0.71 m, Nav2 took the rover round the end of a
    barrier with 13 cm to spare (in the lab; 1.0 gave 46 cm)."""
    reach = (SETTINGS['rover_length_m'] ** 2 + SETTINGS['rover_width_m'] ** 2) ** 0.5 / 2
    assert SETTINGS['inflation_radius_m'] >= reach + 0.25
    assert stack_config.settings_warnings(SETTINGS) == []
    tight = stack_config.settings_warnings(dict(SETTINGS, inflation_radius_m=0.7))
    assert len(tight) == 1 and 'inflation_radius_m is 0.7' in tight[0] and 'Use 1.0 or more' in tight[0]
    assert stack_config.settings_warnings(dict(SETTINGS, rover_length_m=0.6, rover_width_m=0.5, inflation_radius_m=0.7)) == []


def test_the_whole_outline_is_checked_against_obstacles():
    critics = TEMPLATE['controller_server']['ros__parameters']['FollowPath']['critics']
    assert 'ObstacleFootprint' in critics and 'BaseObstacle' not in critics  # BaseObstacle: only the centre cell


def test_speed_limits_are_checked():
    assert stack_config.checked_limits(SETTINGS, None, 1.39, 3.3) == (0.5, 0.8)
    assert stack_config.checked_limits(SETTINGS, 0.3, 1.39, 3.3) == (0.3, 0.8)  # max_speed:= wins
    for bad in (0.0, -0.1, 2.0, float('nan')):
        with pytest.raises(ValueError, match='AUTO max speed'):
            stack_config.checked_limits(SETTINGS, bad, 1.39, 3.3)
    with pytest.raises(ValueError, match='auto_max_turn_rate'):
        stack_config.checked_limits(dict(SETTINGS, auto_max_turn_rate=5.0), None, 1.39, 3.3)


def test_settings_are_checked(tmp_path):
    def write(**changes):
        document = yaml.safe_load((PACKAGE / 'config' / 'autonomy.yaml').read_text())
        document['/**']['ros__parameters'].update(changes)
        for key, value in changes.items():
            if value is None:
                del document['/**']['ros__parameters'][key]
        path = tmp_path / 'autonomy.yaml'
        path.write_text(yaml.safe_dump(document))
        return path
    with pytest.raises(ValueError, match='missing zed_odom_topic'):
        stack_config.load_settings(write(zed_odom_topic=None))
    with pytest.raises(ValueError, match='rover_width_m must be a number'):
        stack_config.load_settings(write(rover_width_m='wide'))
    with pytest.raises(ValueError, match='rover_length_m must be positive'):
        stack_config.load_settings(write(rover_length_m=0.0))
    with pytest.raises(ValueError, match='min_obstacle_height_m'):
        stack_config.load_settings(write(min_obstacle_height_m=2.0))
    with pytest.raises(ValueError, match='max_ground_slope_deg'):
        stack_config.load_settings(write(max_ground_slope_deg=0.5))  # radians typed for degrees
    with pytest.raises(ValueError, match='obstacle_min_points'):
        stack_config.load_settings(write(obstacle_min_points=0))
    with pytest.raises(ValueError, match='at most 1.6 m'):
        stack_config.load_settings(write(rover_height_m=2.0, zed_z_m=1.8))
    # A settings file from before these two existed (a copy kept on the rover) still loads, with the defaults.
    older = stack_config.load_settings(write(max_ground_slope_deg=None, obstacle_min_points=None))
    assert (older['max_ground_slope_deg'], older['obstacle_min_points']) == (45.0, 10)


def test_a_fresh_map_keeps_the_previous_one(tmp_path):
    database = tmp_path / 'maps' / 'rover_map.db'
    assert stack_config.prepare_map(database, continue_map=False) == (database, None)  # nothing yet
    database.write_text('old map')
    path, kept = stack_config.prepare_map(database, continue_map=True)
    assert path == database and kept is None and database.read_text() == 'old map'
    path, kept = stack_config.prepare_map(database, continue_map=False, now=0.0)
    assert not database.exists() and kept.read_text() == 'old map'
    assert kept.name.startswith('rover_map-') and kept.suffix == '.db'


def test_rtabmap_and_urdf_follow_the_settings():
    rtabmap = stack_config.rtabmap_parameters(SETTINGS, '/maps/a.db')
    assert rtabmap['database_path'] == '/maps/a.db' and rtabmap['frame_id'] == 'base_footprint'
    assert rtabmap['Reg/Force3DoF'] == 'true' and all(isinstance(v, (str, bool, int, float)) for v in rtabmap.values())
    assert rtabmap['Grid/Sensor'] == '1' and 'Grid/FromDepth' not in rtabmap  # renamed in RTAB-Map 0.21
    assert float(rtabmap['GridGlobal/MinSize']) >= 20.0  # room to plan beyond what has been mapped
    assert rtabmap['approx_sync'] is False  # colour and depth of the same frame, never of neighbouring ones
    urdf = stack_config.urdf_mappings(SETTINGS, 0.28)
    assert urdf['wheel_radius'] == '0.14' and urdf['zed_name'] == 'zed'


def test_the_behaviour_tree_uses_only_loaded_behaviours():
    navigator = TEMPLATE['bt_navigator']['ros__parameters']
    assert {'nav2_spin_action_bt_node', 'nav2_wait_action_bt_node'} <= set(navigator['plugin_lib_names'])
    behaviors = TEMPLATE['behavior_server']['ros__parameters']['behavior_plugins']
    for name in ('navigate_to_pose.xml', 'navigate_through_poses.xml'):  # bt_navigator loads both at start
        used = {element.tag for element in ET.parse(PACKAGE / 'behavior_trees' / name).iter()}
        assert 'BackUp' not in used and {'Spin', 'Wait'} <= used, name
    assert {'spin': 'Spin', 'wait': 'Wait'} == {name: TEMPLATE['behavior_server']['ros__parameters'][name]['plugin'].split('/')[1] for name in behaviors}


def test_drive_mux_settings_match_its_config():
    mux = AUTONOMY['drive_mux']['ros__parameters']
    assert set(mux) - {'rate_hz'} == set(MuxConfig.__dataclass_fields__)
    assert all(isinstance(value, float) for value in mux.values())  # typed ROS parameters: decimal points


def test_the_deadband_is_off_until_it_is_measured():
    """A guessed deadband above the real one makes the rover drive and turn faster than Nav2 asked (the first version
    shipped 20 and 60 rpm as placeholders: a 0.3 rad/s turn went out at 67 % PWM)."""
    mux = AUTONOMY['drive_mux']['ros__parameters']
    assert (mux['min_wheel_rpm_straight'], mux['min_wheel_rpm_spin']) == (0.0, 0.0)
