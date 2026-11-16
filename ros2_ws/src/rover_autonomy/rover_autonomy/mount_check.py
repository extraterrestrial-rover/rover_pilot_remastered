"""Checks the camera's mount against what the camera sees of the ground, once, when the rover starts.

    ros2 run rover_autonomy mount_check --ros-args --params-file <autonomy.yaml>     # or let the launch run it

The map and the costmaps place every point of the camera with zed_z_m and zed_pitch_rad from autonomy.yaml (through
the URDF). This node fits the ground plane in the ZED's first point clouds (mount_logic), compares the camera's real
height and tilt with those settings, says what to change, and exits. It assumes the rover stands on flat ground with
open floor in view; a rover parked on a slope or facing a wall gets a warning that means nothing.
"""
from __future__ import annotations

import math
import statistics
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rover_autonomy.mount_logic import compare, fit_ground, points_from_cloud, tilt_of, up_in_frame


class MountCheck(Node):
    def __init__(self) -> None:
        super().__init__('mount_check')
        param = self.declare_parameter
        self.base_frame = param('base_frame', 'base_footprint').value
        self.zed_z_m, self.zed_pitch_rad = param('zed_z_m', 0.6).value, param('zed_pitch_rad', 0.0).value
        self.wanted = param('clouds', 5).value
        self.deadline = time.monotonic() + param('timeout_s', 60.0).value
        self.measured: list[tuple[float, float, float]] = []
        self.expected: tuple[float, float, float] | None = None
        self.seen = 0
        self.done = False
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        topic = param('zed_cloud_topic', '/zed/zed_node/point_cloud/cloud_registered').value
        self.create_subscription(PointCloud2, topic, self.on_cloud, qos_profile_sensor_data)
        self.create_timer(1.0, self.on_timer)

    def on_cloud(self, message: PointCloud2) -> None:
        if self.done:
            return
        try:  # where the settings put the cloud's frame on the rover (the URDF and the camera's own frames)
            mount = self.tf.lookup_transform(self.base_frame, message.header.frame_id, Time()).transform
        except TransformException:
            return
        offsets = {field.name: field.offset for field in message.fields
                   if field.name in ('x', 'y', 'z') and field.datatype == PointField.FLOAT32}
        if len(offsets) != 3:
            return
        self.seen += 1
        points = points_from_cloud(message.data, message.point_step, offsets, message.width * message.height,
                                   message.is_bigendian)
        ground = fit_ground(points)
        if ground is None:
            return
        normal, height, _ = ground
        rotation = mount.rotation
        self.expected = (mount.translation.z, *tilt_of(up_in_frame((rotation.x, rotation.y, rotation.z, rotation.w))))
        self.measured.append((height, *tilt_of(normal)))
        if len(self.measured) >= self.wanted:
            self.report()

    def on_timer(self) -> None:
        if not self.done and time.monotonic() > self.deadline:
            self.report()

    def report(self) -> None:
        self.done = True
        log = self.get_logger()
        if not self.measured:
            log.warn(f'camera mount check: not done, {"no flat ground in view" if self.seen else "no point cloud from the camera"}'
                     ' (zed_z_m and zed_pitch_rad in autonomy.yaml stay unchecked)')
            return
        measured = tuple(statistics.median(sample[i] for sample in self.measured) for i in range(3))
        problems = compare(measured, self.expected, self.zed_z_m, self.zed_pitch_rad)
        seen = (f'the ground is {measured[0]:.2f} m below the camera, which looks {math.degrees(measured[1]):+.1f} degrees '
                f'down and is rolled {math.degrees(measured[2]):+.1f} degrees')
        if not problems:
            log.info(f'camera mount check: {seen}, as the settings say')
            return
        log.warn(f'camera mount check: {seen}. If the rover stands on flat ground, the settings are off, and the map and '
                 'the costmaps put the ground and obstacles in the wrong place until they are corrected:')
        for problem in problems:
            log.warn(f'camera mount check: {problem}')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MountCheck()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
