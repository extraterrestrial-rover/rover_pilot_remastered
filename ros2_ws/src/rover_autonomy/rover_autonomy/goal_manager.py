"""Rover-side goal manager: RViz's "2D Goal Pose" (/goal_pose) -> Nav2's navigate_to_pose, only in AUTO.

goal_logic has the rules (AUTO only; manual or the e-stop cancels; with the base station lost the rover goes on to
its goal and stops there). bt_navigator's own /goal_pose subscription is remapped away by the launch file, so this
node is the only way a clicked goal reaches Nav2. Events are logged and published on /rover/autonomy_events for the
base station, and /rover/goal_active tells drive_mux at 10 Hz whether a goal is running.

GPS mode: lists of GPS waypoints come from the base station as JSON on /rover/gps_command (gps_goals), with one of
three modes (waypoint_logic). Each is placed in the map with gps_alignment's datum and TF map -> enu, and sent to Nav2
like a clicked goal, with the same rules; a far one in legs, each sent before the rover reaches the last. The
waypoints are shown in RViz (/rover/gps_waypoints).
"""
from __future__ import annotations

import math
import signal
import time

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from rclpy.time import Time as RosTime
from std_msgs.msg import Bool, String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from rover_autonomy import geo
from rover_autonomy.frames import compose
from rover_autonomy.goal_logic import CANCEL, SEND, GoalLogic
from rover_autonomy.waypoint_logic import Goal, Waypoints, decode

LATEST_ONLY = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)


class GoalManager(Node):
    def __init__(self) -> None:
        super().__init__('goal_manager')
        param = self.declare_parameter
        self.logic = GoalLogic(param('link_timeout_s', 0.5).value)
        action = param('action_name', 'navigate_to_pose').value
        self.nav = ActionClient(self, NavigateToPose, action)
        self.map_frame = param('map_frame', 'map').value
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        # Goals Nav2 may still hold from before this node started (it restarted mid-goal): cancelled once Nav2 is up.
        self.cancel_all = self.create_client(CancelGoal, f'{action}/_action/cancel_goal')
        self.cleared = False
        self.events = self.create_publisher(String, param('events_topic', '/rover/autonomy_events').value, 10)
        self.goal_active = self.create_publisher(Bool, param('goal_active_topic', '/rover/goal_active').value, LATEST_ONLY)
        self.create_subscription(PoseStamped, param('goal_topic', '/goal_pose').value, self.on_goal, 10)
        self.create_subscription(Bool, param('auto_topic', '/rover/auto').value, self.on_auto, LATEST_ONLY)
        self.create_subscription(Bool, param('estop_topic', '/rover/estop').value, self.on_estop, LATEST_ONLY)
        self.create_subscription(String, param('fault_topic', '/rover/auto_fault').value, self.on_fault, LATEST_ONLY)
        self.create_timer(1.0 / param('rate_hz', 10.0).value, self.update)
        # GPS waypoints (the GPS mode)
        self.base_frame = param('base_frame', 'base_footprint').value
        self.enu_frame = param('enu_frame', 'enu').value
        self.max_goal_distance_m = param('max_gps_goal_distance_m', 1000.0).value
        self.waypoints = Waypoints(param('leg_m', 40.0).value)
        self.datum: tuple[float, float] | None = None
        self.gps_status = ''
        self.sent_final: dict[int, bool] = {}  # per sequence number: was it a waypoint itself (not a leg)
        self.last_sent_final = True  # the goal sent last: a waypoint or a clicked goal (False: a leg)
        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(NavSatFix, param('datum_topic', '/rover/gps_datum').value,
                                 lambda m: setattr(self, 'datum', (m.latitude, m.longitude)), latched)
        self.create_subscription(String, param('gps_status_topic', '/rover/gps_status').value,
                                 lambda m: setattr(self, 'gps_status', m.data), latched)
        self.create_subscription(String, param('gps_command_topic', '/rover/gps_command').value, self.on_gps_command, 10)
        self.markers = self.create_publisher(MarkerArray, param('waypoint_markers_topic', '/rover/gps_waypoints').value, latched)
        self.create_timer(1.0, self.publish_markers)
        self.sequence = 0  # every send and cancel; a response or result for an older one is stale
        self.wanted: int | None = None  # the sequence number of the goal that should be running
        self.goal_handle = None  # its goal handle, once Nav2 accepted it
        self.reported = 0
        self.get_logger().info('goal manager: /goal_pose -> navigate_to_pose, only in AUTO with the e-stop released')

    def on_goal(self, msg: PoseStamped) -> None:
        goal = self.in_map(msg)
        if goal is not None:
            self.run(self.logic.on_goal(goal, time.monotonic()))

    def in_map(self, msg: PoseStamped) -> PoseStamped | None:
        """The goal in the map frame, fixed there as it was meant when it was clicked: RViz stamps it in its Fixed
        Frame, and if that is a frame that moves with the rover, sent as it is the goal would move with it."""
        if msg.header.frame_id in ('', self.map_frame):
            msg.header.frame_id = self.map_frame
            return msg
        try:
            transform = self.tf.lookup_transform(self.map_frame, msg.header.frame_id, RosTime())
        except TransformException as error:
            self.logic.events.append(f'goal ignored: it is in {msg.header.frame_id}, which has no transform to '
                                     f'{self.map_frame} ({error}); set RViz\'s Fixed Frame to {self.map_frame}')
            self.report()
            return None
        t, r = transform.transform.translation, transform.transform.rotation
        p, q = msg.pose.position, msg.pose.orientation
        x, y, z, qx, qy, qz, qw = compose((t.x, t.y, t.z, r.x, r.y, r.z, r.w), (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = x, y, z
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = qx, qy, qz, qw
        return goal

    # The heartbeats are only recorded here; the 10 Hz update looks at both together. base_teleop sends them as a
    # pair (the e-stop button: e-stop on and AUTO off), but two topics arrive in either order, and reacting to the
    # first alone would report an e-stop as "manual driving". The motion itself never waits for this: drive_mux
    # passes the e-stop on to drive_node the moment it arrives.
    def on_auto(self, msg: Bool) -> None:
        self.logic.on_auto(msg.data, time.monotonic())

    def on_estop(self, msg: Bool) -> None:
        self.logic.on_estop(msg.data, time.monotonic())

    def on_fault(self, msg: String) -> None:
        self.logic.on_fault(msg.data, time.monotonic())

    def update(self) -> None:
        if not self.cleared and self.cancel_all.service_is_ready():
            self.cancel_all.call_async(CancelGoal.Request())  # an empty request cancels every goal
            self.cleared = True
        now = time.monotonic()
        self.run(self.logic.update(now))
        if self.waypoints.active:
            self.drive_waypoints(now)

    # ------------------------------------------------------------------ GPS waypoints
    def on_gps_command(self, msg: String) -> None:
        now = time.monotonic()
        try:
            command = decode(msg.data)
        except ValueError as error:
            self.logic.events.append(f'GPS command ignored: {error}')
            self.report()
            return
        if command.kind == 'next':
            if not self.waypoints.on_next():
                self.logic.events.append('next ignored: no GPS waypoint is waiting')
        elif command.kind == 'cancel':
            self.waypoints.stop('cancelled from the base station')
            self.run(self.logic.cancel('cancelled from the base station'))
        else:
            refusal = self.gps_refusal(command.waypoints, now)
            if refusal:
                self.logic.events.append(f'GPS waypoints ignored: {refusal}')
            else:
                if self.logic.active:
                    self.run(self.logic.cancel('replaced by GPS waypoints'))
                self.waypoints.start(command)
        self.report()

    def gps_refusal(self, points, now: float) -> str:
        if self.datum is None:
            return 'no GPS datum: GPS goals need the GPS mode (localization:=gps) and a fix'
        if not self.gps_status.startswith('GPS ready'):
            return self.gps_status or 'the GPS is not lined up with the map yet'
        refusal = self.logic.refusal(now)
        if refusal:
            return refusal
        rover = self.rover_position()
        for index, point in enumerate(points):
            target = self.to_map(*point)
            if target is None:
                return 'the GPS frame is not available (TF map -> enu)'
            if rover is not None and math.hypot(target[0] - rover[0], target[1] - rover[1]) > self.max_goal_distance_m:
                return (f'waypoint {index + 1} ({geo.format_position(*point)}) is more than {self.max_goal_distance_m:g} m '
                        'away: latitude and longitude swapped, or a typing error?')
        return ''

    def lookup(self, target: str, source: str):
        try:
            return self.tf.lookup_transform(target, source, RosTime())
        except TransformException:
            return None

    def rover_position(self) -> tuple[float, float] | None:
        transform = self.lookup(self.map_frame, self.base_frame)
        return None if transform is None else (transform.transform.translation.x, transform.transform.translation.y)

    def to_map(self, latitude: float, longitude: float) -> tuple[float, float] | None:
        """A GPS position in the map frame: east/north of the datum, through TF map -> enu."""
        if self.datum is None:
            return None
        transform = self.lookup(self.map_frame, self.enu_frame)
        if transform is None:
            return None
        east, north, _ = geo.enu(latitude, longitude, self.datum)
        t, r = transform.transform.translation, transform.transform.rotation
        x, y, *_ = compose((t.x, t.y, t.z, r.x, r.y, r.z, r.w), (east, north, 0.0, 0.0, 0.0, 0.0, 1.0))
        return x, y

    def drive_waypoints(self, now: float) -> None:
        if (self.waypoints.waiting or self.waypoints.holding_until is not None) and self.logic.refusal(now):
            self.waypoints.stop(self.logic.refusal(now))  # e-stop, manual or no base station while waiting
            self.report()
            return
        rover = self.rover_position()
        if rover is None:
            return
        target = self.to_map(*self.waypoints.points[self.waypoints.index])
        goal = self.waypoints.next_goal(rover, target, now)
        if goal is None:
            return
        pose = self.pose_of(goal)
        if self.logic.active and not self.last_sent_final:
            actions = self.logic.continue_with(pose)  # the next leg of the waypoint under way
        else:
            actions = self.logic.on_goal(pose, now)
            if not actions:
                self.waypoints.stop(self.logic.events[-1].replace('goal ignored: ', ''))
        self.run(actions, goal)

    def pose_of(self, goal: Goal) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x, pose.pose.position.y = goal.x, goal.y
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(goal.yaw / 2), math.cos(goal.yaw / 2)
        return pose

    def publish_markers(self) -> None:
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for index, point in enumerate(self.waypoints.points):
            target = self.to_map(*point)
            if target is None:
                continue
            current = index == self.waypoints.index
            for kind, offset in ((Marker.CYLINDER, 0), (Marker.TEXT_VIEW_FACING, 1000)):
                marker = Marker(type=kind, action=Marker.ADD, id=index + offset, ns='gps_waypoints')
                marker.header.frame_id = self.map_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.pose.position.x, marker.pose.position.y = target
                marker.pose.position.z = 0.25 if kind == Marker.CYLINDER else 1.2
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = 0.6 if kind == Marker.CYLINDER else 0.0
                marker.scale.z = 0.5 if kind == Marker.CYLINDER else 0.5
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                    (1.0, 0.6, 0.0, 0.9) if current else (0.2, 0.6, 1.0, 0.7))
                marker.text = f'{index + 1}' if kind == Marker.TEXT_VIEW_FACING else ''
                markers.markers.append(marker)
        self.markers.publish(markers)

    def run(self, actions, waypoint_goal: Goal | None = None) -> None:
        for action in actions:
            if action.kind == SEND:
                self.send(action.goal)
                # A clicked goal is final; a GPS goal is final when it is the waypoint itself (not a leg).
                self.sent_final[self.sequence] = waypoint_goal.final if waypoint_goal is not None else True
                self.last_sent_final = self.sent_final[self.sequence]
            elif action.kind == CANCEL:
                self.cancel()
                self.waypoints.stop(self.logic.events[-1].replace('goal cancelled: ', '') if self.logic.events else 'cancelled')
        self.report()

    def report(self) -> None:
        self.logic.events.extend(self.waypoints.events)  # the waypoint list's own events, in the same stream
        self.waypoints.events.clear()
        self.goal_active.publish(Bool(data=self.logic.active))  # with every update (10 Hz) and every change
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
        goal.pose.header.frame_id = pose.header.frame_id  # the map frame (in_map)
        goal.pose.header.stamp = Time()  # zero: the latest transform
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
        final = self.sent_final.pop(sequence, True)
        if status == GoalStatus.STATUS_SUCCEEDED and not final and self.waypoints.active:
            self.waypoints.on_reached(time.monotonic())  # a leg: the goal goes on, the next leg follows
        elif status == GoalStatus.STATUS_CANCELED:  # our own cancels clear self.wanted first: this came from elsewhere
            self.logic.on_rejected('cancelled outside goal_manager')
            self.waypoints.stop('cancelled outside goal_manager')
        else:
            succeeded = status == GoalStatus.STATUS_SUCCEEDED
            self.logic.on_result(succeeded=succeeded)
            if self.waypoints.active:
                if succeeded:
                    rover = self.rover_position()
                    target = self.to_map(*self.waypoints.points[self.waypoints.index])
                    error = math.hypot(rover[0] - target[0], rover[1] - target[1]) if rover and target else None
                    self.waypoints.on_reached(time.monotonic(), error)
                else:
                    self.waypoints.stop('Nav2 gave up on it')
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
