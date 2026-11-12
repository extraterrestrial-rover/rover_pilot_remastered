"""Rover-side: the ZED's odometry (the camera's pose) -> the rover's (base_footprint's), for the EKF.

The ZED wrapper reports the pose of zed_camera_link, and Humble's robot_localization uses an odometry pose as the
pose of its base frame whatever the message's child frame. Fused directly, the rover's centre would sit at the
camera, which swings round when the rover turns on the spot. This node moves the pose to base_footprint with the
mount from autonomy.yaml (the same values the URDF is built from). The covariance is passed on unchanged.

It passes on only what can be trusted (zed_checks): real poses, and only while the ZED reports that it tracks. When it
loses tracking the wrapper keeps publishing its last pose, which would freeze the rover in place for the EKF and Nav2
while it drives on. drive_mux stops AUTO when nothing arrives here.
"""
from __future__ import annotations

import signal

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from rover_autonomy.frames import camera_to_base, from_xyz_rpy
from rover_autonomy.zed_checks import checked_pose, tracking_problem

try:
    from zed_msgs.msg import PosTrackStatus
except ImportError:  # no ZED packages here (the rehearsal's computer): the tracking status cannot be read
    PosTrackStatus = None


class ZedOdometry(Node):
    def __init__(self) -> None:
        super().__init__('zed_odometry')
        param = self.declare_parameter
        self.mount = from_xyz_rpy(param('zed_x_m', 0.4).value, param('zed_y_m', 0.0).value, param('zed_z_m', 0.6).value,
                                  pitch=param('zed_pitch_rad', 0.0).value)
        self.base_frame = param('base_frame', 'base_footprint').value
        source = param('zed_odom_topic', '/zed/zed_node/odom').value
        target = param('output_topic', '/zed/odom_base').value
        status_topic = param('zed_status_topic', '/zed/zed_node/pose/status').value  # '' = not checked
        self.output = self.create_publisher(Odometry, target, 10)
        self.tracking = ''  # what the ZED reports when it does not track
        self.withheld = ''  # why odometry is not passed on right now
        # The ZED wrapper's publishers are reliable or best effort depending on its settings: best effort takes both.
        self.create_subscription(Odometry, source, self.on_odometry, qos_profile_sensor_data)
        if status_topic and PosTrackStatus is not None:
            self.create_subscription(PosTrackStatus, status_topic, self.on_status, qos_profile_sensor_data)
        elif status_topic:
            self.get_logger().warn("zed_msgs is not installed: the ZED's tracking status is not checked, only missing "
                                   'or invalid odometry stops AUTO')
        self.get_logger().info(f'{source} (the camera) -> {target} ({self.base_frame})')

    def on_status(self, message) -> None:
        self.tracking = tracking_problem(message.odometry_status, message.spatial_memory_status)

    def on_odometry(self, message: Odometry) -> None:
        p, q = message.pose.pose.position, message.pose.pose.orientation
        camera = checked_pose((p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        if camera is None:
            withheld = 'the ZED sent a pose that is not one (not a number, or an empty quaternion)'
        elif self.tracking:
            withheld = f'the ZED reports its tracking as {self.tracking}'
        else:
            withheld = ''
        if withheld != self.withheld:
            if withheld:
                self.get_logger().warn(f'odometry withheld, AUTO stops: {withheld}')
            else:
                self.get_logger().info('odometry passed on again')
            self.withheld = withheld
        if withheld:
            return
        base = camera_to_base(camera, self.mount)
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
