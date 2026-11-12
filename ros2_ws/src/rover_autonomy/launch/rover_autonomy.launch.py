"""Rover with autonomy: rover.launch.py's drive_node (unchanged, on its usual link) plus the ZED 2i, the EKF,
RTAB-Map, Nav2, drive_mux and goal_manager (AUTONOMY.md).

    ros2 launch rover_autonomy rover_autonomy.launch.py [continue_map:=true] [max_speed:=0.3] [wheels:=wheel_28cm]
        [port:=/dev/rover-drive] [zed:=true] [slam:=true] [settings:=<autonomy.yaml>]

Link arguments (port:= on this branch) are passed through to rover.launch.py. drive_node's /cmd_vel and /rover/estop
are remapped to /cmd_vel_drive and /rover/estop_drive, drive_mux's outputs: it passes the base station's e-stop on as
it arrives, and decides only while the base station is silent (mux_logic). Every Nav2 velocity output
is remapped to /cmd_vel_nav -> velocity_smoother -> /cmd_vel_auto, and bt_navigator's own /goal_pose subscription
away from RViz's topic, so only goal_manager starts goals, and only in AUTO.
"""
from __future__ import annotations

import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

from rover_autonomy import stack_config
from rover_pilot_remastered.kinematics import DriveGeometry

# Remaps that keep Nav2 off /cmd_vel and RViz's /goal_pose (tested in test/test_launch.py).
NAV2_CMD_VEL = [('/cmd_vel', '/cmd_vel_nav')]
SMOOTHER = [('/cmd_vel', '/cmd_vel_nav'), ('/cmd_vel_smoothed', '/cmd_vel_auto')]
NAVIGATOR = [('/goal_pose', '/bt_navigator/goal_pose_unused')]
NAV2_NODES = (  # (package, executable, remappings), all managed by lifecycle_manager_navigation
    ('nav2_controller', 'controller_server', NAV2_CMD_VEL),
    ('nav2_planner', 'planner_server', []),
    ('nav2_behaviors', 'behavior_server', NAV2_CMD_VEL),
    ('nav2_bt_navigator', 'bt_navigator', NAVIGATOR),
    ('nav2_velocity_smoother', 'velocity_smoother', SMOOTHER),
)


def flag(context, name: str) -> bool:
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in ('true', 'false'):
        raise ValueError(f'{name}:={value}: use true or false')
    return value == 'true'


def launch_setup(context):
    share = Path(get_package_share_directory('rover_autonomy'))
    pilot = Path(get_package_share_directory('rover_pilot_remastered'))
    settings_file = LaunchConfiguration('settings').perform(context)
    settings = stack_config.load_settings(settings_file)
    wheels = LaunchConfiguration('wheels').perform(context)
    wheel_file = pilot / 'config' / 'wheels' / f'{wheels}.yaml'
    drive = yaml.safe_load((pilot / 'config' / 'drive.yaml').read_text())['/**']['ros__parameters']
    wheel = yaml.safe_load(wheel_file.read_text())['/**']['ros__parameters']
    geometry = DriveGeometry(wheel['wheel_diameter_m'], drive['track_width_m'], drive['max_wheel_rpm'],
                             drive.get('turn_track_factor', 1.0))
    max_speed = LaunchConfiguration('max_speed').perform(context).strip()
    speed, turn_rate = stack_config.checked_limits(settings, float(max_speed) if max_speed else None,
                                                   geometry.max_linear_speed, geometry.max_angular_speed)
    template = yaml.safe_load((share / 'config' / 'nav2_params.yaml').read_text())
    nav2 = stack_config.nav2_parameters(template, settings, speed, turn_rate, str(share / 'behavior_trees'))
    handle, nav2_file = tempfile.mkstemp(prefix='rover_autonomy_nav2_', suffix='.yaml')
    os.close(handle)
    stack_config.write_yaml(nav2, nav2_file)
    actions = [LogInfo(msg=f'AUTO: up to {speed:g} m/s and {turn_rate:g} rad/s; Nav2 parameters in {nav2_file}')]

    if flag(context, 'zed'):
        try:
            zed_launch = Path(get_package_share_directory('zed_wrapper')) / 'launch' / 'zed_camera.launch.py'
        except PackageNotFoundError as error:
            raise RuntimeError('zed_wrapper is not installed or not sourced (source its workspace first), '
                               'or run the camera separately with zed:=false (AUTONOMY.md)') from error
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(zed_launch)), launch_arguments={
            'camera_model': 'zed2i', 'camera_name': str(settings['zed_camera_name']),
            'publish_tf': 'false', 'publish_map_tf': 'false',  # the EKF publishes odom -> rover, RTAB-Map map -> odom
            'ros_params_override_path': str(share / 'config' / 'zed_override.yaml'),  # continuous odometry
        }.items()))

    import xacro  # noqa: PLC0415  (only needed here; keeps the module importable for the tests without xacro)
    description = xacro.process_file(str(share / 'urdf' / 'rover.urdf.xacro'),
                                     mappings=stack_config.urdf_mappings(settings, wheel['wheel_diameter_m'])).toxml()
    actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
                        parameters=[{'robot_description': description}]))
    actions.append(Node(package='rover_autonomy', executable='zed_odometry', name='zed_odometry', output='screen',
                        respawn=True, respawn_delay=1.0, parameters=[settings_file]))  # the camera's pose -> the rover's
    actions.append(Node(package='robot_localization', executable='ekf_node', name='ekf_local', output='screen',
                        parameters=[str(share / 'config' / 'ekf.yaml')]))

    if flag(context, 'slam'):
        database, kept = stack_config.prepare_map(settings['map_database'], flag(context, 'continue_map'))
        actions.append(LogInfo(msg=f'map: continuing {database}' if flag(context, 'continue_map') else
                               f'map: fresh in {database}' + (f' (the previous one is now {kept})' if kept else '')))
        actions.append(Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
                            parameters=[stack_config.rtabmap_parameters(settings, str(database))],
                            remappings=[('rgb/image', settings['zed_rgb_topic']),
                                        ('rgb/camera_info', settings['zed_rgb_info_topic']),
                                        ('depth/image', settings['zed_depth_topic'])]))

    for package, executable, remappings in NAV2_NODES:
        actions.append(Node(package=package, executable=executable, name=executable, output='screen',
                            parameters=[nav2_file], remappings=remappings))
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                        name='lifecycle_manager_navigation', output='screen', parameters=[nav2_file]))

    autonomy = settings_file
    actions.append(Node(package='rover_autonomy', executable='drive_mux', name='drive_mux', output='screen',
                        respawn=True, respawn_delay=1.0,
                        parameters=[str(pilot / 'config' / 'drive.yaml'), str(wheel_file), autonomy]))
    actions.append(Node(package='rover_autonomy', executable='goal_manager', name='goal_manager', output='screen',
                        respawn=True, respawn_delay=1.0, parameters=[autonomy]))
    actions.append(GroupAction([
        SetRemap(src='/cmd_vel', dst='/cmd_vel_drive'),
        SetRemap(src='/rover/estop', dst='/rover/estop_drive'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(pilot / 'launch' / 'rover.launch.py')),
                                 launch_arguments={'wheels': wheels}.items()),
    ]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('rover_autonomy'))
    return LaunchDescription([
        DeclareLaunchArgument('wheels', default_value='wheel_28cm', description='wheel profile in rover_pilot_remastered config/wheels'),
        DeclareLaunchArgument('continue_map', default_value='false',
                              description='true: keep mapping into the last map; false: a fresh map (the old one is kept aside)'),
        DeclareLaunchArgument('max_speed', default_value='', description="AUTO's top speed in m/s for this run (default: autonomy.yaml)"),
        DeclareLaunchArgument('zed', default_value='true', description='start the ZED wrapper (false: it runs separately)'),
        DeclareLaunchArgument('slam', default_value='true', description='start RTAB-Map (false: something else provides /map and map -> odom)'),
        DeclareLaunchArgument('settings', default_value=str(share / 'config' / 'autonomy.yaml'), description='autonomy settings file'),
        OpaqueFunction(function=launch_setup),
    ])
