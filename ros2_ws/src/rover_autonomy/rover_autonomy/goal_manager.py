"""Rover-side goal manager: RViz's "2D Goal Pose" (/goal_pose) -> Nav2's navigate_to_pose, only in AUTO.

goal_logic has the rules (AUTO only; manual or e-stop cancels; base station lost: pause, then resume). bt_navigator's
own /goal_pose subscription is remapped away by the launch file, so this node is the only way a clicked goal reaches
Nav2. Events are logged and published on /rover/autonomy_events for the base station.
"""
from __future__ import annotations

import signal
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from rover_autonomy.goal_logic import CANCEL, SEND, GoalLogic

LATEST_ONLY = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)


class GoalManager(Node):
    def __init__(self) -> None:
        super().__init__('goal_manager')
        param = self.declare_parameter
        self.logic = GoalLogic(param('link_timeout_s', 0.5).value)
        self.nav = ActionClient(self, NavigateToPose, param('action_name', 'navigate_to_pose').value)
        self.events = self.create_publisher(String, param('events_topic', '/rover/autonomy_events').value, 10)
        self.create_subscription(PoseStamped, param('goal_topic', '/goal_pose').value, self.on_goal, 10)
        self.create_subscription(Bool, param('auto_topic', '/rover/auto').value, self.on_auto, LATEST_ONLY)
        self.create_subscription(Bool, param('estop_topic', '/rover/estop').value, self.on_estop, LATEST_ONLY)
        self.create_subscription(String, param('fault_topic', '/rover/auto_fault').value, self.on_fault, LATEST_ONLY)
        self.create_timer(1.0 / param('rate_hz', 10.0).value, self.update)
        self.sequence = 0  # every send and cancel; a response or result for an older one is stale
        self.wanted: int | None = None  # the sequence number of the goal that should be running
        self.goal_handle = None  # its goal handle, once Nav2 accepted it
        self.reported = 0
        self.get_logger().info('goal manager: /goal_pose -> navigate_to_pose, only in AUTO with the e-stop released')

    def on_goal(self, msg: PoseStamped) -> None:
        self.run(self.logic.on_goal(msg, time.monotonic()))

    # The heartbeats are only recorded here; the 10 Hz update looks at both together. base_teleop sends them as a
    # pair (Circle: e-stop on and AUTO off), but two topics arrive in either order, and reacting to the first alone
    # would report an e-stop as "manual driving". The motion itself never waits for this: drive_node and drive_mux
    # act on the heartbeats at once.
    def on_auto(self, msg: Bool) -> None:
        self.logic.on_auto(msg.data, time.monotonic())

    def on_estop(self, msg: Bool) -> None:
        self.logic.on_estop(msg.data, time.monotonic())

    def on_fault(self, msg: String) -> None:
        self.logic.on_fault(msg.data, time.monotonic())

    def update(self) -> None:
        self.run(self.logic.update(time.monotonic()))

    def run(self, actions) -> None:
        for action in actions:
            if action.kind == SEND:
                self.send(action.goal)
            elif action.kind == CANCEL:
                self.cancel()
        self.report()

    def report(self) -> None:
        for event in self.logic.events[self.reported:]:
            warn = any(word in event for word in ('ignored', 'lost', 'cancelled', 'dropped', 'gave up'))
            self.get_logger().warn(event) if warn else self.get_logger().info(event)
            self.events.publish(String(data=event))
        self.reported = len(self.logic.events)

    def send(self, pose: PoseStamped) -> None:
        if not self.nav.server_is_ready():
            self.logic.on_rejected('Nav2 is not running yet')
            return
        self.sequence += 1
        sequence = self.wanted = self.sequence
        self.goal_handle = None
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = pose.header.frame_id
        goal.pose.header.stamp = Time()  # zero: the latest transform, also when a paused goal is resent minutes later
        goal.pose.pose = pose.pose
        self.nav.send_goal_async(goal).add_done_callback(lambda future: self.on_response(future, sequence))
        p = pose.pose.position
        self.get_logger().info(f'goal sent to Nav2: x {p.x:.2f}, y {p.y:.2f} in {pose.header.frame_id or "?"}')

    def cancel(self) -> None:
        self.sequence += 1
        self.wanted = None
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        # A goal still waiting for Nav2's answer is cancelled in on_response.

    def on_response(self, future, sequence: int) -> None:
        handle = future.result()
        if sequence != self.wanted:
            # Superseded. After a cancel, stop it; after a newer goal, Nav2 has already replaced it (cancelling it
            # now could cancel the newer goal too, while bt_navigator switches).
            if handle is not None and handle.accepted and self.wanted is None:
                handle.cancel_goal_async()
            return
        if handle is None or not handle.accepted:
            self.wanted = None
            self.logic.on_rejected('Nav2 refused it')
            self.report()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda result: self.on_result(result, sequence))

    def on_result(self, future, sequence: int) -> None:
        if sequence != self.wanted:
            return
        status = future.result().status
        self.wanted, self.goal_handle = None, None
        if status == GoalStatus.STATUS_CANCELED:  # our own cancels clear self.wanted first: this came from elsewhere
            self.logic.on_rejected('cancelled outside goal_manager')
        else:
            self.logic.on_result(succeeded=status == GoalStatus.STATUS_SUCCEEDED, cancelled=False)
        self.report()

    def stop(self) -> None:
        """On shutdown: Nav2 should not keep a goal nobody manages."""
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GoalManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # One Ctrl-C on `ros2 launch` delivers SIGINT twice (the terminal's, then launch's): the second must not
        # interrupt the shutdown below.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
