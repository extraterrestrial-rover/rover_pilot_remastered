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


def test_the_outline_and_the_zed_reach_both_costmaps():
    params = build(rover_length_m=1.2, rover_width_m=0.9, rover_height_m=0.7, zed_cloud_topic='/cloud',
                   obstacle_range_m=2.5, min_obstacle_height_m=0.15, inflation_radius_m=0.8)
    for name in ('local_costmap', 'global_costmap'):
        costmap = params[name][name]['ros__parameters']
        assert yaml.safe_load(costmap['footprint']) == [[0.6, 0.45], [0.6, -0.45], [-0.6, -0.45], [-0.6, 0.45]]
        assert costmap['inflation_layer']['inflation_radius'] == 0.8
        zed = costmap['obstacle_layer']['zed']
        assert (zed['topic'], zed['min_obstacle_height'], zed['max_obstacle_height']) == ('/cloud', 0.15, 0.7)
        assert zed['obstacle_max_range'] == 2.5 and zed['raytrace_max_range'] == 3.0
        assert costmap['robot_base_frame'] == 'base_footprint'


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
