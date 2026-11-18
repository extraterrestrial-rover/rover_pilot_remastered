"""Base-station arm teleop: PS5 controller (/joy) -> /arm/joint_cmd (20 Hz) and the e-stop heartbeat /arm/estop (10 Hz).

/arm/joint_cmd is a sensor_msgs/JointState with names and velocities (rad/s) only, so a planner can later publish the
same topic. Full input defaults to each joint's top speed, derived from the same gearing the arm driver uses.
"""
from __future__ import annotations

import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Bool

from arm_pilot_redemption.geometry_params import declare_geometry, declare_limited_geometry
from arm_pilot_redemption.kinematics import JOINTS
from arm_pilot_redemption.teleop_logic import JointInput, TeleopConfig, TeleopLogic

LATEST_ONLY = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)


class ArmTeleop(Node):
    def __init__(self) -> None:
        super().__init__('arm_teleop')
        param = self.declare_parameter
        defaults = TeleopConfig()
        geometry, pwm_limit = declare_limited_geometry(self, declare_geometry(self))  # full stick = the PWM limit
        inputs, max_velocity = [], []
        for joint, default, auto in zip(JOINTS, defaults.inputs, geometry.max_joint_velocity):
            inputs.append(JointInput(param(f'{joint}_axis', default.axis).value, param(f'{joint}_invert', default.invert).value,
                                     param(f'{joint}_positive_button', default.positive_button).value,
                                     param(f'{joint}_negative_button', default.negative_button).value))
            limit = param(f'{joint}_max_velocity', -1.0).value  # rad/s at full input; <= 0: the joint's top speed
            max_velocity.append(limit if limit > 0 else auto)
        config = TeleopConfig(
            inputs=tuple(inputs),
            deadzone=param('deadzone', defaults.deadzone).value,
            max_velocity=tuple(max_velocity),
            slow_button=param('slow_button', defaults.slow_button).value,
            slow_scale=param('slow_scale', defaults.slow_scale).value,
            estop_button=param('estop_button', defaults.estop_button).value,
            release_button=param('release_button', defaults.release_button).value,
            joy_timeout_s=param('joy_timeout_s', defaults.joy_timeout_s).value,
        )
        self.logic = TeleopLogic(config)
        self.joint_cmd = self.create_publisher(JointState, param('joint_cmd_topic', '/arm/joint_cmd').value, LATEST_ONLY)
        self.estop = self.create_publisher(Bool, param('estop_topic', '/arm/estop').value, LATEST_ONLY)
        self.create_subscription(Joy, param('joy_topic', '/joy').value, self.on_joy, LATEST_ONLY)
        self.create_timer(1.0 / param('publish_rate_hz', 20.0).value, self.publish_joint_cmd)
        self.create_timer(1.0 / param('estop_rate_hz', 10.0).value, self.publish_estop)
        speeds = ', '.join(f'{joint} {limit:.2f}' for joint, limit in zip(JOINTS, config.max_velocity))
        self.get_logger().info(f'arm teleop: full input = {speeds} rad/s (PWM limit {pwm_limit:.0%}); e-stop button {config.estop_button}, release '
                               f'button {config.release_button} (inputs centred). E-stop is latched until released.')

    def on_joy(self, msg: Joy) -> None:
        was_estopped = self.logic.estopped
        for event in self.logic.on_joy(list(msg.axes), list(msg.buttons), time.monotonic()):
            self.get_logger().warn(event) if 'E-STOP' in event or 'ignored' in event else self.get_logger().info(event)
        if self.logic.estopped != was_estopped:
            self.publish_estop()  # tell the arm now instead of at the next 10 Hz heartbeat

    def joint_state(self, velocities: tuple[float, ...]) -> JointState:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINTS)
        message.velocity = [float(value) for value in velocities]
        return message

    def publish_joint_cmd(self) -> None:
        was_estopped = self.logic.estopped
        velocities, events = self.logic.tick(time.monotonic())
        for event in events:
            self.get_logger().warn(event)
        if self.logic.estopped != was_estopped:
            self.publish_estop()
        self.joint_cmd.publish(self.joint_state(velocities))

    def publish_estop(self) -> None:
        self.estop.publish(Bool(data=self.logic.estopped))

    def stop_arm(self) -> None:
        """On shutdown: latch the e-stop on the arm now instead of leaving it to the 0.5 s heartbeat timeout."""
        if not rclpy.ok():
            return
        for _ in range(3):
            self.estop.publish(Bool(data=True))
            self.joint_cmd.publish(self.joint_state((0.0, 0.0, 0.0, 0.0)))
            time.sleep(0.02)


def main(args: list[str] | None = None) -> None:
    # rclpy's own SIGINT handler shuts the context down before `finally` runs, so a final publish would fail. Without
    # it, Ctrl-C (also from `ros2 launch`) raises KeyboardInterrupt and the context stays usable for stop_arm().
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ArmTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # One Ctrl-C on `ros2 launch` delivers SIGINT twice: from the terminal, then forwarded by launch. The second
        # one must not cut the final e-stop short (or end the node with a traceback).
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.stop_arm()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
