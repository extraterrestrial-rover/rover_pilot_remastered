"""Rover-side, GPS mode: lines the GPS up with the rover's map, and feeds it to the global EKF (AUTONOMY.md, "GPS").

The first usable fix becomes the datum (the origin of local east-north metres). The node collects the GPS antenna's
positions twice, by GPS and by the camera's odometry (TF odom -> gps_link), and fits how one lies on the other
(alignment_logic): that is the map's heading against north, found by driving, with no compass. From then on it
  - publishes every fix as the rover's position in the map frame (/odometry/gps, corrected for the antenna's offset
    from the rover's centre), which ekf_global fuses with the camera's odometry: the map -> odom correction;
  - publishes TF map -> enu (east-north-up at the datum) and the datum (/rover/gps_datum), with which goal_manager
    turns GPS goals into map goals.
Until then nothing goes to the EKF, and /rover/gps_status says how much more to drive.
"""
from __future__ import annotations

import math
import signal
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rover_autonomy import geo
from rover_autonomy.alignment_logic import Alignment, AlignmentConfig
from rover_autonomy.frames import yaw_of

LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
GPS_LOST_S = 3.0  # no fix for this long: said so in the status


class GpsAlignment(Node):
    def __init__(self) -> None:
        super().__init__('gps_alignment')
        param = self.declare_parameter
        defaults = AlignmentConfig()
        self.alignment = Alignment(AlignmentConfig(
            min_track_m=param('min_track_m', defaults.min_track_m).value,
            refit_every_m=param('refit_every_m', defaults.refit_every_m).value,
            freeze_track_m=param('freeze_track_m', defaults.freeze_track_m).value))
        self.map_frame = param('map_frame', 'map').value
        self.odom_frame = param('odom_frame', 'odom').value
        self.base_frame = param('base_frame', 'base_footprint').value
        self.gps_frame = param('gps_frame', 'gps_link').value
        self.enu_frame = param('enu_frame', 'enu').value
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.odometry = self.create_publisher(Odometry, param('gps_odometry_topic', '/odometry/gps').value, 10)
        self.datum_pub = self.create_publisher(NavSatFix, param('datum_topic', '/rover/gps_datum').value, LATCHED)
        self.status_pub = self.create_publisher(String, param('status_topic', '/rover/gps_status').value, LATCHED)
        self.create_subscription(NavSatFix, param('gps_fix_topic', '/gps/fix').value, self.on_fix, qos_profile_sensor_data)
        self.create_timer(1.0, self.report)
        self.datum: tuple[float, float] | None = None
        self.last_fix_time: float | None = None
        self.last_status = ''

    def lookup(self, target: str, source: str):
        try:
            return self.tf.lookup_transform(target, source, Time())
        except TransformException:
            return None

    def on_fix(self, fix: NavSatFix) -> None:
        if not (math.isfinite(fix.latitude) and math.isfinite(fix.longitude)):
            return
        self.last_fix_time = time.monotonic()
        if self.datum is None:
            self.datum = (fix.latitude, fix.longitude)
            datum = NavSatFix()
            datum.header.stamp, datum.header.frame_id = fix.header.stamp, self.enu_frame
            datum.latitude, datum.longitude, datum.altitude = fix.latitude, fix.longitude, fix.altitude
            datum.status = fix.status
            self.datum_pub.publish(datum)
            self.get_logger().info(f'GPS datum (origin of the east-north frame): {geo.format_position(*self.datum)}')
        east, north, _ = geo.enu(fix.latitude, fix.longitude, self.datum)
        antenna = self.lookup(self.odom_frame, self.gps_frame)
        if antenna is None:
            return
        if self.alignment.add((antenna.transform.translation.x, antenna.transform.translation.y), (east, north)):
            self.publish_enu_frame()
            self.get_logger().info(self.alignment.status())
        if self.alignment.ready:
            self.publish_odometry(fix, east, north)

    def publish_enu_frame(self) -> None:
        """TF map -> enu: the east-north frame at the datum, as the fit places it in the map."""
        fit = self.alignment.fit
        x, y = fit.enu_to_odom(0.0, 0.0)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id, transform.child_frame_id = self.map_frame, self.enu_frame
        transform.transform.translation.x, transform.transform.translation.y = x, y
        transform.transform.rotation.z, transform.transform.rotation.w = math.sin(-fit.theta / 2), math.cos(-fit.theta / 2)
        self.static_tf.sendTransform(transform)

    def publish_odometry(self, fix: NavSatFix, east: float, north: float) -> None:
        fit = self.alignment.fit
        x, y = fit.enu_to_odom(east, north)  # the antenna in the map frame
        base = self.lookup(self.map_frame, self.base_frame)
        mount = self.lookup(self.base_frame, self.gps_frame)
        if base is None or mount is None:
            return
        r = base.transform.rotation
        yaw = yaw_of((0.0, 0.0, 0.0, r.x, r.y, r.z, r.w))
        ox, oy = mount.transform.translation.x, mount.transform.translation.y
        x -= math.cos(yaw) * ox - math.sin(yaw) * oy  # from the antenna to the rover's centre
        y -= math.sin(yaw) * ox + math.cos(yaw) * oy
        message = Odometry()
        message.header.stamp, message.header.frame_id = fix.header.stamp, self.map_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x, message.pose.pose.position.y = x, y
        message.pose.pose.orientation.w = 1.0
        variance = max(fix.position_covariance[0], fix.position_covariance[4], 0.25)
        # The heading's uncertainty adds to a fix's the further it is from where the fit was made.
        lever = math.hypot(east - fit.tx, north - fit.ty)
        variance += (lever * fit.heading_sigma) ** 2
        covariance = [0.0] * 36
        covariance[0] = covariance[7] = variance
        for index in (14, 21, 28, 35):
            covariance[index] = 1e6  # not measured
        message.pose.covariance = covariance
        self.odometry.publish(message)

    def report(self) -> None:
        status = self.alignment.status()
        if self.last_fix_time is None:
            status = 'no GPS fix yet (gps_node: ros2 topic echo /diagnostics)' if not self.alignment.odom else status
        elif time.monotonic() - self.last_fix_time > GPS_LOST_S:
            status = (f'no GPS fix for {time.monotonic() - self.last_fix_time:.0f} s: driving on the camera\'s odometry; '
                      + status)
        self.status_pub.publish(String(data=status))
        if status.split(':')[0] != self.last_status.split(':')[0]:
            self.get_logger().info(status)
        self.last_status = status


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GpsAlignment()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
