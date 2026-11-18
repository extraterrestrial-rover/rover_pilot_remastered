"""Base station: PS5 controller -> /arm/joint_cmd + e-stop heartbeat /arm/estop.

    ros2 launch arm_pilot_redemption base_station.launch.py [device_id:=0]
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
        DeclareLaunchArgument('device_id', default_value='0', description='joystick index for SDL'),
        # joy_node (the raw SDL joystick), as for the rover. deadzone 0 keeps /joy flowing while the sticks rest (joy only
        # autorepeats when no event arrives, and stick jitter never stops), so the teleop never thinks the PS5 is lost.
        Node(package='joy', executable='joy_node', name='joy', output='screen',
             parameters=[{'device_id': LaunchConfiguration('device_id'), 'deadzone': 0.0, 'autorepeat_rate': 20.0}]),
        Node(package='arm_pilot_redemption', executable='arm_teleop', name='arm_teleop', output='screen',
             parameters=[str(share / 'config' / 'arm.yaml')]),
    ])
