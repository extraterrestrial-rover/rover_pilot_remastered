"""Rover-side mux: the sticks' /cmd_vel or Nav2's /cmd_vel_auto -> /cmd_vel_drive, which drive_node listens to.

The base station's AUTO heartbeat (/rover/auto) picks the input (mux_logic has the rules). drive_node is unchanged:
the autonomy launch file only remaps its /cmd_vel to /cmd_vel_drive, and it still takes /rover/estop straight from
the base station, so the e-stop never goes through this node. If this node stops, drive_node's command timeout
stops the wheels.

In AUTO it also watches the camera's odometry (zed_odometry's output) and the ESP32's state (drive_node's
/diagnostics), and reports on /rover/auto_fault when it stopped AUTO by itself, so that goal_manager cancels the goal.
"""
from __future__ import annotations

import signal
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from rover_autonomy.frames import yaw_of
from rover_autonomy.mux_logic import DriveMux, MuxConfig, checked_deadband
from rover_pilot_remastered.kinematics import DriveGeometry

LATEST_ONLY = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)


class DriveMuxNode(Node):
    def __init__(self) -> None:
        super().__init__('drive_mux')
        param = self.declare_parameter
        defaults = MuxConfig()
        geometry = DriveGeometry(param('wheel_diameter_m', 0.28).value, param('track_width_m', 0.84).value,
                                 param('max_wheel_rpm', 94.7).value, param('turn_track_factor', 1.0).value)
        # A wrong deadband (a percentage or a PWM value typed as rpm) would turn every AUTO command into nearly full
        # speed: such settings are refused here, and AUTO runs without the correction.
        straight, spin, problem = checked_deadband(param('min_wheel_rpm_straight', defaults.min_wheel_rpm_straight).value,
                                                   param('min_wheel_rpm_spin', defaults.min_wheel_rpm_spin).value,
                                                   geometry.max_wheel_rpm)
        if problem:
            self.get_logger().error(f'deadband settings ignored, AUTO runs without the correction: {problem}')
        elif straight == 0.0 and spin == 0.0:
            self.get_logger().warn("the motors' deadband is not set (0.0): in AUTO the rover may not move or turn at Nav2's "
                                   'slow commands. Measure it and set min_wheel_rpm_straight / min_wheel_rpm_spin '
                                   '(AUTONOMY.md, "Deadband")')
        config = MuxConfig(
            mode_timeout_s=param('mode_timeout_s', defaults.mode_timeout_s).value,
            command_timeout_s=param('command_timeout_s', defaults.command_timeout_s).value,
            neutral_hold_s=param('neutral_hold_s', defaults.neutral_hold_s).value,
            min_wheel_rpm_straight=straight,
            min_wheel_rpm_spin=spin,
            stop_below_rpm=param('stop_below_rpm', defaults.stop_below_rpm).value,
            odometry_timeout_s=param('odometry_timeout_s', defaults.odometry_timeout_s).value,
            no_motion_timeout_s=param('no_motion_timeout_s', defaults.no_motion_timeout_s).value,
            no_motion_distance_m=param('no_motion_distance_m', defaults.no_motion_distance_m).value,
            no_motion_angle_rad=param('no_motion_angle_rad', defaults.no_motion_angle_rad).value,
            rearm_interval_s=param('rearm_interval_s', defaults.rearm_interval_s).value,
        )
        self.mux = DriveMux(config, geometry)
        self.output = self.create_publisher(Twist, param('output_topic', '/cmd_vel_drive').value, LATEST_ONLY)
        self.create_subscription(Twist, param('manual_topic', '/cmd_vel').value, self.on_manual, LATEST_ONLY)
        self.create_subscription(Twist, param('auto_cmd_topic', '/cmd_vel_auto').value, self.on_auto_command, LATEST_ONLY)
        self.create_subscription(Bool, param('auto_topic', '/rover/auto').value, self.on_mode, LATEST_ONLY)
        self.create_subscription(Odometry, param('odometry_topic', '/zed/odom_base').value, self.on_odometry, 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, 10)
        self.fault = self.create_publisher(String, param('fault_topic', '/rover/auto_fault').value, LATEST_ONLY)
        rate_hz = param('rate_hz', 50.0).value
        self.create_timer(1.0 / rate_hz, self.tick)
        self.fault_every = max(1, round(rate_hz / 10.0))  # the fault heartbeat: 10 Hz
        self.ticks = 0
        self.last_state = None
        self.get_logger().info(
            f'drive mux: /cmd_vel (manual) or /cmd_vel_auto (Nav2) -> /cmd_vel_drive; deadband straight '
            f'{config.min_wheel_rpm_straight} rpm, spin {config.min_wheel_rpm_spin} rpm')

    def on_manual(self, msg: Twist) -> None:
        command = self.mux.on_manual(msg.linear.x, msg.angular.z, time.monotonic())
        if command is not None:
            self.send(command)  # manual: passed on as it arrives, so drive_node's own timeout works as on main

    def on_auto_command(self, msg: Twist) -> None:
        self.mux.on_auto_command(msg.linear.x, msg.angular.z, time.monotonic())

    def on_mode(self, msg: Bool) -> None:
        self.mux.on_mode(msg.data, time.monotonic())

    def on_odometry(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.mux.on_odometry(p.x, p.y, yaw_of((p.x, p.y, p.z, q.x, q.y, q.z, q.w)), time.monotonic())

    def on_diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name == 'rover drive':  # drive_node's entry
                for item in status.values:
                    if item.key == 'esp_state':
                        self.mux.on_drive_state(item.value, time.monotonic())

    def tick(self) -> None:
        command = self.mux.output(time.monotonic())
        if self.mux.state != self.last_state:
            self.get_logger().warn(self.mux.state) if self.mux.fault else self.get_logger().info(self.mux.state)
            self.last_state = self.mux.state
        if command is not None:
            self.send(command)
        self.ticks += 1
        if self.ticks % self.fault_every == 0:
            self.fault.publish(String(data=self.mux.fault))

    def send(self, command: tuple[float, float]) -> None:
        message = Twist()
        message.linear.x, message.angular.z = command
        self.output.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DriveMuxNode()
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
