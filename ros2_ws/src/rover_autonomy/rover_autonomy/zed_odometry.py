"""Rover-side: the ZED's odometry (the camera's pose) -> the rover's (base_footprint's), for the EKF.

The ZED wrapper reports the pose of zed_camera_link, and Humble's robot_localization uses an odometry pose as the
pose of its base frame whatever the message's child frame. Fused directly, the rover's centre would sit at the
camera, which swings round when the rover turns on the spot. This node moves the pose to base_footprint with the
mount from autonomy.yaml (the same values the URDF is built from). The covariance is passed on unchanged.
"""
from __future__ import annotations

import signal

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from rover_autonomy.frames import camera_to_base, from_xyz_rpy


class ZedOdometry(Node):
    def __init__(self) -> None:
        super().__init__('zed_odometry')
        param = self.declare_parameter
        self.mount = from_xyz_rpy(param('zed_x_m', 0.4).value, param('zed_y_m', 0.0).value, param('zed_z_m', 0.6).value,
                                  pitch=param('zed_pitch_rad', 0.0).value)
        self.base_frame = param('base_frame', 'base_footprint').value
        source = param('zed_odom_topic', '/zed/zed_node/odom').value
        target = param('output_topic', '/zed/odom_base').value
        self.output = self.create_publisher(Odometry, target, 10)
        # The ZED wrapper's publishers are reliable or best effort depending on its settings: best effort takes both.
        self.create_subscription(Odometry, source, self.on_odometry, qos_profile_sensor_data)
        self.get_logger().info(f'{source} (the camera) -> {target} ({self.base_frame})')

    def on_odometry(self, message: Odometry) -> None:
        p, q = message.pose.pose.position, message.pose.pose.orientation
        base = camera_to_base((p.x, p.y, p.z, q.x, q.y, q.z, q.w), self.mount)
        out = Odometry()
        out.header = message.header
        out.child_frame_id = self.base_frame
        position, orientation = out.pose.pose.position, out.pose.pose.orientation
        position.x, position.y, position.z = base[:3]
        orientation.x, orientation.y, orientation.z, orientation.w = base[3:]
        out.pose.covariance = message.pose.covariance
        self.output.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ZedOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # One Ctrl-C on `ros2 launch` delivers SIGINT twice (the terminal's, then launch's): the second must not
        # interrupt the shutdown below.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
