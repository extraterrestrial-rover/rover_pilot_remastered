from glob import glob

from setuptools import find_packages, setup

package_name = 'rover_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/behavior_trees', glob('behavior_trees/*.xml')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bruce',
    maintainer_email='diveshkumar_23ee078@dtu.ac.in',
    description='Autonomous navigation: ZED 2i VIO, RTAB-Map, Nav2, AUTO mode mux and RViz goals.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'drive_mux = rover_autonomy.drive_mux:main',
            'goal_manager = rover_autonomy.goal_manager:main',
            'zed_odometry = rover_autonomy.zed_odometry:main',
        ],
    },
)
