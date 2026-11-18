from glob import glob

from setuptools import find_packages, setup

package_name = 'arm_pilot_redemption'

setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bruce',
    maintainer_email='diveshkumar_23ee078@dtu.ac.in',
    description='Arm manual control: PS5 teleop with a latched e-stop, joint commands to the arm master ESP32.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'arm_driver = arm_pilot_redemption.arm_driver:main',
            'arm_teleop = arm_pilot_redemption.arm_teleop:main',
            'fake_arm = arm_pilot_redemption.fake_arm:main',
        ],
    },
)
