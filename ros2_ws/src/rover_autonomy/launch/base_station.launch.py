"""Base station with autonomy: rover_pilot_remastered's base_station.launch.py (PS5 -> /cmd_vel, the e-stop and AUTO
heartbeats) plus RViz with the map, the costmaps, Nav2's path, and the "2D Goal Pose" tool on /goal_pose.

    ros2 launch rover_autonomy base_station.launch.py [wheels:=wheel_28cm] [device_id:=0] [rviz:=true]

Triangle switches AUTO on and off; goals clicked in RViz are taken only in AUTO (AUTONOMY.md). Watch the rover's
answers with `ros2 topic echo /rover/autonomy_events`.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('rover_autonomy'))
    pilot = Path(get_package_share_directory('rover_pilot_remastered'))
    return LaunchDescription([
        DeclareLaunchArgument('wheels', default_value='wheel_28cm', description='wheel profile in rover_pilot_remastered config/wheels'),
        DeclareLaunchArgument('device_id', default_value='0', description='joystick index for SDL'),
        DeclareLaunchArgument('rviz', default_value='true', description='start RViz'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(pilot / 'launch' / 'base_station.launch.py')),
                                 launch_arguments={'wheels': LaunchConfiguration('wheels'),
                                                   'device_id': LaunchConfiguration('device_id')}.items()),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(share / 'rviz' / 'base_station.rviz')]),
    ])
