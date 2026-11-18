"""Arm computer: /arm/joint_cmd + e-stop heartbeat -> arm master ESP32; /arm/joint_states and /diagnostics back.

    ros2 launch arm_pilot_redemption arm.launch.py [port:=/dev/arm-master]
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('arm_pilot_redemption'))
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/arm-master', description='arm master ESP32 serial port'),
        Node(package='arm_pilot_redemption', executable='arm_driver', name='arm_driver', output='screen',
             parameters=[str(share / 'config' / 'arm.yaml'), {'port': LaunchConfiguration('port')}]),
    ])
