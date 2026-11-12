"""Rover: /cmd_vel + e-stop heartbeat -> drive ESP32.

    ros2 launch rover_pilot_remastered rover.launch.py [wheels:=wheel_28cm] [master:=192.168.77.2] [interface:=enx...]
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from rover_pilot_remastered.dds_check import multicast_problem


def generate_launch_description():
    share = Path(get_package_share_directory('rover_pilot_remastered'))
    wheels = PathJoinSubstitution([str(share / 'config' / 'wheels'), [LaunchConfiguration('wheels'), '.yaml']])
    problem = multicast_problem(os.environ)
    return LaunchDescription([LogInfo(msg=f'WARNING: {problem}')] * bool(problem) + [
        DeclareLaunchArgument('wheels', default_value='wheel_28cm', description='wheel profile in config/wheels'),
        DeclareLaunchArgument('master', default_value='192.168.77.2', description="drive ESP32's address (LINK.md)"),
        DeclareLaunchArgument('interface', default_value='',
                              description='the PC port the ESP32 is plugged into: send only through it (Linux)'),
        Node(package='rover_pilot_remastered', executable='drive_node', name='drive_node', output='screen',
             parameters=[str(share / 'config' / 'drive.yaml'), wheels,
                         {'master_host': LaunchConfiguration('master'), 'interface': LaunchConfiguration('interface')}]),
    ])
