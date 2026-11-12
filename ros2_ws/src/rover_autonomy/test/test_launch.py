"""The autonomy launch file's safety-relevant remaps, read from its source (no ROS needed)."""
import ast
from pathlib import Path

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE / 'launch' / 'rover_autonomy.launch.py').read_text()
REMAPS = {}  # the module's remap constants, evaluated on their own (they are plain literals)
for statement in ast.parse(SOURCE).body:
    if isinstance(statement, ast.Assign) and getattr(statement.targets[0], 'id', '') in (
            'NAV2_CMD_VEL', 'SMOOTHER', 'NAVIGATOR', 'NAV2_NODES'):
        exec(compile(ast.Module([statement], []), 'rover_autonomy.launch.py', 'exec'), REMAPS)  # noqa: S102


def constant(name):
    return REMAPS[name]


def test_no_nav2_node_publishes_cmd_vel():
    nodes = constant('NAV2_NODES')
    publishers = {'controller_server', 'behavior_server', 'velocity_smoother'}  # Humble nodes that publish cmd_vel
    for _, executable, remappings in nodes:
        mapped = dict(remappings)
        if executable in publishers:
            assert mapped.get('/cmd_vel') == '/cmd_vel_nav', executable
    smoother = dict(next(r for _, e, r in nodes if e == 'velocity_smoother'))
    assert smoother['/cmd_vel_smoothed'] == '/cmd_vel_auto'


def test_rviz_goals_reach_nav2_only_through_goal_manager():
    navigator = dict(next(r for _, e, r in constant('NAV2_NODES') if e == 'bt_navigator'))
    assert navigator['/goal_pose'] != '/goal_pose'


def test_drive_node_listens_to_the_mux():
    assert "SetRemap(src='/cmd_vel', dst='/cmd_vel_drive')" in SOURCE
    # ... for its e-stop too: the mux passes the base station's on, and holds it released only while a goal is
    # finished without the base station.
    assert "SetRemap(src='/rover/estop', dst='/rover/estop_drive')" in SOURCE


def test_lifecycle_manager_manages_every_nav2_node():
    template = yaml.safe_load((PACKAGE / 'config' / 'nav2_params.yaml').read_text())
    managed = template['lifecycle_manager_navigation']['ros__parameters']['node_names']
    assert managed == [executable for _, executable, _ in constant('NAV2_NODES')]
