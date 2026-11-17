#!/usr/bin/env python3
"""Dress rehearsal of AUTO mode (AUTONOMY.md) without hardware: the real autonomy launch files, Nav2 included.

Runs `ros2 launch rover_autonomy rover_autonomy.launch.py zed:=false slam:=false` (EKF, Nav2, drive_mux,
goal_manager, drive_node) and `base_station.launch.py rviz:=false` (base_teleop), presses the PS5's buttons, sends
RViz-style goals, and checks where the rover ends up. Stand-ins, all in this process:
  - the ESP32: fake_esp.Simulator on this branch's link;
  - the rover's motion: the ESP32's PWM integrated with a deadband on each side's motors, a little below the one
    the rehearsal's autonomy settings give drive_mux (as a measured value would be), published as the ZED's
    odometry (the camera's pose, as the ZED wrapper reports it);
  - the ZED's point cloud: flat ground and a post at (2.5, 0) in the map, ray-cast from the camera with the ZED 2i's
    field of view (110 x 70 degrees), so it goes through the real obstacle detector and costmaps; the rover can be
    tilted without the software knowing (a wheel on a stone);
  - RTAB-Map: an identity map -> odom and an empty 30 m x 30 m /map;
  - the radio: the base station runs in its own ROS domain, and this script relays its three topics (/cmd_vel,
    /rover/estop, /rover/auto) to the rover's domain; "radio lost" stops the relay;
  - both domains are this script's own (61 and 62, --rover-domain / --base-domain), and it refuses to start if
    another node is already in either domain: it drives a (simulated) rover;
  - the PS5: /joy from this script, in the base station's domain.

Needs ROS 2 Humble with Nav2 and robot_localization (AUTONOMY.md, "Install"), this workspace built and sourced:
    source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python3 tools/rehearsal_auto.py          # PASS/FAIL table, exit code 1 on any failure (about 5 minutes)
"""
from __future__ import annotations

import argparse
import array
import math
import os
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / 'ros2_ws' / 'src'
sys.path.insert(0, str(SRC / 'rover_pilot_remastered'))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rclpy.time import Time  # noqa: E402
from sensor_msgs.msg import Joy, PointCloud2, PointField  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402
from tf2_ros.buffer import Buffer  # noqa: E402
from tf2_ros.transform_listener import TransformListener  # noqa: E402

from rover_pilot_remastered import fake_esp, ros_sandbox  # noqa: E402
from rover_pilot_remastered.kinematics import DriveGeometry  # noqa: E402

PILOT = SRC / 'rover_pilot_remastered' / 'config'
AUTONOMY = yaml.safe_load((SRC / 'rover_autonomy' / 'config' / 'autonomy.yaml').read_text())
SETTINGS = AUTONOMY['/**']['ros__parameters']
TELEOP = yaml.safe_load((PILOT / 'drive.yaml').read_text())['base_teleop']['ros__parameters']
CIRCLE, OPTIONS, TRIANGLE = TELEOP['estop_button'], TELEOP['release_button'], TELEOP['auto_button']
STOPPED = (0, 0, 0, 0)
LATEST_ONLY = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)
POST, POST_RADIUS, POST_HEIGHT = (2.5, 0.0), 0.25, 0.6  # the obstacle (a square post), in the map frame
CLOUD_SIZE = (160, 90)  # points across and down: enough for the obstacle detector, light enough for this script
# The deadband (straight, on the spot) in wheel rpm: what the rehearsal's settings tell drive_mux, and what the
# simulated motors really have. autonomy.yaml itself ships 0.0 (not measured), which would leave the correction untested.
CONFIGURED_DEADBAND = (20.0, 60.0)
TRUE_DEADBAND = (18.0, 54.0)


def geometry_for(wheels: str) -> DriveGeometry:
    drive = yaml.safe_load((PILOT / 'drive.yaml').read_text())['/**']['ros__parameters']
    wheel = yaml.safe_load((PILOT / 'wheels' / f'{wheels}.yaml').read_text())['/**']['ros__parameters']
    return DriveGeometry(wheel['wheel_diameter_m'], drive['track_width_m'], drive['max_wheel_rpm'], drive.get('turn_track_factor', 1.0))


class Rover:
    """The rover's motion from the ESP32's PWM, with a deadband like the real motors, and what the ZED reports.

    The map frame is the camera's start pose (as the ZED's odometry and so the EKF's odom frame start there): the
    rover starts with its camera at the origin, facing +x.
    """

    def __init__(self, esp: fake_esp.Simulator, geometry: DriveGeometry) -> None:
        self.esp, self.geometry = esp, geometry
        self.mount = (SETTINGS['zed_x_m'], SETTINGS['zed_y_m'])
        self.x, self.y, self.yaw = -self.mount[0], -self.mount[1], 0.0  # base_footprint
        self.pitch = 0.0  # the rover's real tilt, nose down positive, which nothing in the software knows about
        self.pivots: list[tuple[float, float, float]] = []  # (x, y, angle) of every bit turned on the spot
        self.deadband = TRUE_DEADBAND  # what the motors need on the ground the rover stands on
        self.lock = threading.Lock()
        # The camera's rays, in its own frame (x forward, y left, z up): the ZED 2i's 110 x 70 degree view.
        across = (np.arange(CLOUD_SIZE[0]) + 0.5 - CLOUD_SIZE[0] / 2) / (CLOUD_SIZE[0] / 2) * math.tan(math.radians(55.0))
        down = (np.arange(CLOUD_SIZE[1]) + 0.5 - CLOUD_SIZE[1] / 2) / (CLOUD_SIZE[1] / 2) * math.tan(math.radians(35.0))
        grid_across, grid_down = np.meshgrid(across, down)
        rays = np.stack([np.ones_like(grid_across), -grid_across, -grid_down], axis=-1).reshape(-1, 3)
        self.rays = rays / np.linalg.norm(rays, axis=1, keepdims=True)

    def step(self, dt: float) -> None:
        state = self.esp.state()
        if state is None or state[0] != fake_esp.DRIVING:
            return
        pwm = state[1]
        top = self.geometry.max_wheel_rpm
        left, right = ((pwm[0] + pwm[2]) / 2 / 255 * top, (pwm[1] + pwm[3]) / 2 / 255 * top)
        if left == 0.0 and right == 0.0:
            return
        # Each side's motors turn only above their deadband, then about linearly up to full speed. Turning on the
        # spot scrubs the tyres, so the deadband is higher the more of the motion is turning.
        spin_share = abs(right - left) / (abs(left + right) + abs(right - left))
        deadband = self.deadband[0] + (self.deadband[1] - self.deadband[0]) * spin_share

        def turns(target: float) -> float:
            if abs(target) <= deadband:
                return 0.0  # the motor hums, the wheel stays put
            return math.copysign((abs(target) - deadband) / (top - deadband) * top, target)
        metres_per_rpm = math.pi * self.geometry.wheel_diameter_m / 60.0
        left, right = turns(left) * metres_per_rpm, turns(right) * metres_per_rpm
        speed = (left + right) / 2.0
        turn = (right - left) / (self.geometry.track_width_m * self.geometry.turn_track_factor)
        with self.lock:
            self.yaw += turn * dt
            self.x += speed * math.cos(self.yaw) * dt
            self.y += speed * math.sin(self.yaw) * dt
            if abs(speed) < 0.05 and abs(turn) > 0.1:  # turning on the spot, not rolling through a curve
                self.pivots.append((self.x, self.y, abs(turn) * dt))

    def pose(self) -> tuple[float, float, float]:
        with self.lock:
            return self.x, self.y, self.yaw

    def camera(self) -> tuple[float, float, float]:
        x, y, yaw = self.pose()
        mx, my = self.mount
        return x + mx * math.cos(yaw) - my * math.sin(yaw), y + mx * math.sin(yaw) + my * math.cos(yaw), yaw

    def sees(self) -> np.ndarray:
        """What the camera sees: the ground and the post, as points (x, y, z, rgb; float32) in the camera's frame, one
        per ray, NaN where a ray meets nothing between 0.3 and 15 m (the sky). The rover's tilt tilts the camera."""
        cx, cy, yaw = self.camera()
        pitch = SETTINGS['zed_pitch_rad'] + self.pitch
        cos_yaw, sin_yaw, cos_pitch, sin_pitch = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
        rotation = np.array([[cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch],
                             [sin_yaw * cos_pitch, cos_yaw, sin_yaw * sin_pitch],
                             [-sin_pitch, 0.0, cos_pitch]])
        world = self.rays @ rotation.T
        origin = (cx, cy, SETTINGS['zed_z_m'])
        box = ((POST[0] - POST_RADIUS, POST[0] + POST_RADIUS), (POST[1] - POST_RADIUS, POST[1] + POST_RADIUS), (0.0, POST_HEIGHT))
        with np.errstate(divide='ignore', invalid='ignore'):
            distance = np.where(world[:, 2] < -1e-9, -origin[2] / world[:, 2], np.inf)  # the ground
            near, far = np.full(len(world), -np.inf), np.full(len(world), np.inf)
            for axis in range(3):
                first, second = (box[axis][0] - origin[axis]) / world[:, axis], (box[axis][1] - origin[axis]) / world[:, axis]
                near, far = np.maximum(near, np.minimum(first, second)), np.minimum(far, np.maximum(first, second))
            hit = (far >= near) & (near > 0)
            distance = np.where(hit & (near < distance), near, distance)
        seen = (distance >= 0.3) & (distance <= 15.0)
        points = np.full((len(distance), 4), np.nan, dtype=np.float32)
        points[seen, :3] = (self.rays[seen] * distance[seen, None]).astype(np.float32)
        points[seen, 3] = 0.0
        return points

    def distance_to(self, x: float, y: float) -> float:
        rx, ry, _ = self.pose()
        return math.hypot(rx - x, ry - y)


def cloud(points: np.ndarray, frame: str, stamp) -> PointCloud2:
    """An organized cloud like the ZED's: x, y, z, rgb as float32, NaN where there is no point."""
    message = PointCloud2()
    message.header.frame_id, message.header.stamp = frame, stamp
    message.width, message.height = CLOUD_SIZE
    message.fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, name in enumerate(('x', 'y', 'z', 'rgb'))]
    message.is_bigendian, message.point_step, message.is_dense = False, 16, False
    message.row_step = 16 * message.width
    message.data = array.array('B', points.tobytes())  # an array: rclpy checks a bytes object one byte at a time
    return message


class Launch:
    """One `ros2 launch` in its own process group (Ctrl-C reaches every node, like in a terminal), in a ROS domain."""

    def __init__(self, name: str, logs: Path, domain: int, *arguments: str) -> None:
        self.name, self.log = name, logs / f'{name}.log'
        environment = dict(os.environ, PYTHONUNBUFFERED='1', RCUTILS_LOGGING_BUFFERED_STREAM='0', ROS_DOMAIN_ID=str(domain))
        self.process = subprocess.Popen(['ros2', 'launch', *arguments], stdout=self.log.open('a'), stderr=subprocess.STDOUT,
                                        start_new_session=True, env=environment)

    def ctrl_c(self) -> int | None:
        try:
            os.killpg(self.process.pid, signal.SIGINT)
            return self.process.wait(timeout=20)
        except ProcessLookupError:
            return self.process.poll()
        except subprocess.TimeoutExpired:
            self.kill()
            return None

    def kill(self) -> None:
        try:
            os.killpg(self.process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.process.wait(timeout=10)

    def text(self) -> str:
        return self.log.read_text(errors='replace')


class Rehearsal:
    def __init__(self, wheels: str, logs: Path, rover_domain: int, base_domain: int) -> None:
        self.wheels, self.logs = wheels, logs
        self.rover_domain, self.base_domain = rover_domain, base_domain
        self.results: list[tuple[str, bool, str]] = []
        self.esp = fake_esp.Simulator()
        self.rover = Rover(self.esp, geometry_for(wheels))
        self.events: list[str] = []
        self.radio = True
        self.cloud_on = True  # False: the camera's point cloud stops (its odometry goes on)
        self.costmap: OccupancyGrid | None = None  # Nav2's local costmap, as RViz gets it
        self.left_y = 0.0
        self.pressed: set[int] = set()
        self.rover_launch: Launch | None = None
        self.base_launch: Launch | None = None
        settings = yaml.safe_load((SRC / 'rover_autonomy' / 'config' / 'autonomy.yaml').read_text())
        mux = settings['drive_mux']['ros__parameters']
        mux['min_wheel_rpm_straight'], mux['min_wheel_rpm_spin'] = CONFIGURED_DEADBAND
        self.settings_file = logs / 'autonomy.yaml'
        self.settings_file.write_text(yaml.safe_dump(settings, sort_keys=False))

        self.rover_context, self.base_context = rclpy.Context(), rclpy.Context()
        rclpy.init(context=self.rover_context, domain_id=rover_domain)
        rclpy.init(context=self.base_context, domain_id=base_domain)
        self.node = rclpy.create_node('rehearsal_rover', context=self.rover_context)
        self.base = rclpy.create_node('rehearsal_base', context=self.base_context)

        node = self.node
        self.odom = node.create_publisher(Odometry, SETTINGS['zed_odom_topic'], 10)
        self.points = node.create_publisher(PointCloud2, SETTINGS['zed_cloud_topic'], 10)
        self.map = node.create_publisher(OccupancyGrid, '/map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                                            reliability=ReliabilityPolicy.RELIABLE))
        self.goals = node.create_publisher(PoseStamped, '/goal_pose', 10)
        node.create_subscription(String, '/rover/autonomy_events', lambda m: self.events.append(m.data), 50)
        self.fault: str | None = None  # drive_mux's /rover/auto_fault, as last heard
        node.create_subscription(String, '/rover/auto_fault', lambda m: setattr(self, 'fault', m.data), LATEST_ONLY)
        node.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: setattr(self, 'costmap', m),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.static_tf = StaticTransformBroadcaster(node)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, node)
        self.nav = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.ticks = 0
        node.create_timer(0.02, self.simulate)

        # The radio: the base station's heartbeats and sticks reach the rover only while self.radio is True.
        for topic, kind in (('/cmd_vel', Twist), ('/rover/estop', Bool), ('/rover/auto', Bool)):
            relay = node.create_publisher(kind, topic, LATEST_ONLY)
            self.base.create_subscription(kind, topic, lambda message, relay=relay: self.radio and relay.publish(message), LATEST_ONLY)
        self.base_auto: bool | None = None
        self.base.create_subscription(Bool, '/rover/auto', lambda m: setattr(self, 'base_auto', m.data), LATEST_ONLY)
        self.joy = self.base.create_publisher(Joy, '/joy', 10)
        self.base.create_timer(0.05, self.publish_joy)

        self.executors = []
        for context, owner in ((self.rover_context, self.node), (self.base_context, self.base)):
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(owner)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            self.executors.append((executor, thread))
        self.publish_world()

    # ------------------------------------------------------------------ the simulated world
    def publish_world(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id, transform.child_frame_id = 'map', 'odom'
        transform.transform.rotation.w = 1.0
        self.static_tf.sendTransform(transform)
        grid = OccupancyGrid()
        grid.header.frame_id, grid.header.stamp = 'map', transform.header.stamp
        grid.info.resolution, grid.info.width, grid.info.height = 0.1, 300, 300
        grid.info.origin.position.x = grid.info.origin.position.y = -15.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (300 * 300)
        self.map.publish(grid)

    def simulate(self) -> None:
        self.rover.step(0.02)
        self.ticks += 1
        stamp = self.node.get_clock().now().to_msg()
        if self.ticks % 2 == 0:  # the ZED's odometry, 25 Hz: the camera's pose from where it started
            x, y, yaw = self.rover.camera()
            message = Odometry()
            message.header.stamp, message.header.frame_id = stamp, 'odom'
            message.child_frame_id = f"{SETTINGS['zed_camera_name']}_camera_link"
            message.pose.pose.position.x, message.pose.pose.position.y = x, y
            message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            message.pose.covariance = [0.0] * 36
            for index in (0, 7, 14, 21, 28, 35):
                message.pose.covariance[index] = 1e-4
            self.odom.publish(message)
        if self.ticks % 10 == 0 and self.cloud_on:  # the point cloud, 5 Hz
            self.points.publish(cloud(self.rover.sees(), f"{SETTINGS['zed_camera_name']}_camera_link", stamp))

    def publish_joy(self) -> None:
        message = Joy()
        message.axes = [0.0] * 8
        message.axes[TELEOP['axis_linear']] = self.left_y
        message.buttons = [1 if index in self.pressed else 0 for index in range(15)]
        self.joy.publish(message)

    # ------------------------------------------------------------------ helpers
    def press(self, button: int) -> None:
        self.pressed = {button}
        time.sleep(0.25)
        self.pressed = set()
        time.sleep(0.15)

    def goal(self, x: float, y: float, frame: str = 'map', yaw: float = 0.0) -> None:
        message = PoseStamped()
        message.header.frame_id = frame
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.position.x, message.pose.position.y = x, y
        message.pose.orientation.z, message.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.goals.publish(message)

    def event(self, text: str, timeout: float, since: int = 0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if any(text in event for event in self.events[since:]):
                return True
            time.sleep(0.05)
        return False

    def wait(self, condition, timeout: float) -> float | None:
        started = time.monotonic()
        while time.monotonic() - started < timeout:
            state = self.esp.state()
            if state is not None and condition(state):
                return time.monotonic() - started
            time.sleep(0.005)
        return None

    def moves(self, seconds: float) -> bool:
        start = self.rover.pose()
        time.sleep(seconds)
        end = self.rover.pose()
        return math.hypot(end[0] - start[0], end[1] - start[1]) > 0.1 or abs(end[2] - start[2]) > 0.1

    def event_while_driving(self, text: str, timeout: float, since: int) -> tuple[bool, bool]:
        """(the event came, the ESP32 left DRIVING before it did)."""
        left_driving = False
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if any(text in event for event in self.events[since:]):
                return True, left_driving
            state = self.esp.state()
            left_driving = left_driving or state is None or state[0] != fake_esp.DRIVING
            time.sleep(0.02)
        return False, left_driving

    def holds_still(self, seconds: float) -> bool:
        start = self.rover.pose()
        time.sleep(seconds)
        end = self.rover.pose()
        return math.hypot(end[0] - start[0], end[1] - start[1]) < 0.02 and abs(end[2] - start[2]) < 0.02

    def lethal_cells(self, x0: float, x1: float, y0: float, y1: float) -> int:
        """How many cells of the local costmap in this rectangle of the map are obstacles themselves (not the
        inflation around one); -1 without a costmap."""
        grid = self.costmap
        if grid is None:
            return -1
        info, count = grid.info, 0
        for row in range(info.height):
            y = info.origin.position.y + (row + 0.5) * info.resolution
            if y0 <= y <= y1:
                for column in range(info.width):
                    x = info.origin.position.x + (column + 0.5) * info.resolution
                    count += x0 <= x <= x1 and grid.data[row * info.width + column] == 100
        return count

    def where(self) -> str:
        x, y, yaw = self.rover.pose()
        state = self.esp.state()
        esp = 'no ESP32' if state is None else f'{fake_esp.protocol.STATE_NAMES[state[0]]} PWM {state[1]}'
        return f'rover at ({x:.2f}, {y:.2f}), heading {math.degrees(yaw):.0f} deg; {esp}'

    def check(self, name: str, ok: bool, detail: str = '') -> bool:
        self.results.append((name, bool(ok), detail or self.where()))
        print(f"  {'PASS' if ok else 'FAIL'}  {name}: {detail or self.where()}", flush=True)
        return bool(ok)

    def drive_to(self, x: float, y: float, name: str, timeout: float = 150.0) -> bool:
        since = len(self.events)
        self.goal(x, y)
        reached = self.event('goal reached', timeout, since)
        distance = self.rover.distance_to(x, y)
        return self.check(name, reached and distance < 0.5, f'{"goal reached" if reached else "not reached"}, '
                          f'{distance:.2f} m from it; {self.where()}')

    def ekf_error(self) -> tuple[float, str] | None:
        try:
            transform = self.tf.lookup_transform('odom', 'base_footprint', Time())
        except Exception:  # noqa: BLE001  (any tf2 exception: not available yet)
            return None
        x, y, _ = self.rover.pose()
        ex, ey = transform.transform.translation.x, transform.transform.translation.y
        return math.hypot(ex - x, ey - y), f'EKF ({ex:.2f}, {ey:.2f}), rover ({x:.2f}, {y:.2f})'

    def start_rover(self) -> None:
        self.rover_launch = Launch('rover', self.logs, self.rover_domain, 'rover_autonomy', 'rover_autonomy.launch.py',
                                   'zed:=false', 'slam:=false', f'wheels:={self.wheels}', f'settings:={self.settings_file}',
                                   *self.esp.launch_arguments)

    def start_base(self) -> None:
        self.base_launch = Launch('base_station', self.logs, self.base_domain, 'rover_autonomy', 'base_station.launch.py',
                                  'rviz:=false', f'wheels:={self.wheels}')

    def ready_for_auto(self) -> bool:
        """Options (release) then Triangle (AUTO) with the sticks centred, as the operator does."""
        self.left_y = 0.0
        time.sleep(0.2)
        self.press(OPTIONS)
        self.press(TRIANGLE)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and self.base_auto is not True:
            time.sleep(0.05)
        return self.base_auto is True

    # ------------------------------------------------------------------ the run
    def run(self) -> bool:
        print(f'logs in {self.logs}; rover in ROS domain {self.rover_domain}, base station in {self.base_domain}', flush=True)
        for domain, node in ((self.rover_domain, self.node), (self.base_domain, self.base)):
            others = ros_sandbox.foreign_nodes(node)
            if not self.check(f'ROS domain {domain} is ours alone', not others, ros_sandbox.refusal(domain, others) if others else 'no other node'):
                return False
        print('\n1. Launch the rover (Nav2, EKF, mux, goal manager, drive node) and the base station', flush=True)
        self.start_rover()
        self.start_base()
        self.check('Nav2 comes up (navigate_to_pose available)', self.nav.wait_for_server(timeout_sec=90.0))
        self.check('the e-stop starts latched', self.wait(lambda s: s[0] == fake_esp.ESTOP, 30.0) is not None)
        time.sleep(3.0)  # let the costmaps settle
        cmd_vel = sorted(info.node_name for info in self.node.get_publishers_info_by_topic('/cmd_vel'))
        self.check('nothing on the rover publishes /cmd_vel but the base station (here: the relay)', cmd_vel == ['rehearsal_rover'], str(cmd_vel))
        drive = sorted(info.node_name for info in self.node.get_subscriptions_info_by_topic('/cmd_vel_drive'))
        self.check('drive_node listens to drive_mux (/cmd_vel_drive)', drive == ['drive_node'], str(drive))
        estop = sorted(info.node_name for info in self.node.get_subscriptions_info_by_topic('/rover/estop_drive'))
        direct = sorted(info.node_name for info in self.node.get_subscriptions_info_by_topic('/rover/estop'))
        self.check("drive_node takes its e-stop from drive_mux, which passes the base station's on",
                   estop == ['drive_node'] and 'drive_mux' in direct and 'drive_node' not in direct, f'{estop}; /rover/estop: {direct}')
        auto = sorted(info.node_name for info in self.node.get_publishers_info_by_topic('/cmd_vel_auto'))
        self.check("Nav2's output reaches the mux through the velocity smoother only", auto == ['velocity_smoother'], str(auto))

        print('\n1b. What the costmap makes of the camera\'s view', flush=True)
        post = (POST[0] - POST_RADIUS - 0.1, POST[0] + POST_RADIUS + 0.1, POST[1] - POST_RADIUS - 0.1, POST[1] + POST_RADIUS + 0.1)
        ground = (0.7, POST[0] - POST_RADIUS - 0.4, -1.5, 1.5)  # between the rover and the post

        def seen() -> str:
            return f'{self.lethal_cells(*post)} obstacle cells on the post, {self.lethal_cells(*ground)} on the open ground before it'
        self.check('the post is in the local costmap and the ground before it is free',
                   self.lethal_cells(*post) > 0 and self.lethal_cells(*ground) == 0, seen())
        self.rover.pitch = math.radians(4.0)
        time.sleep(3.0)
        self.check('rover tilted 4 degrees nose down (a wheel on a stone): the ground stays free, the post stays',
                   self.lethal_cells(*post) > 0 and self.lethal_cells(*ground) == 0, seen())
        # The same tilt, seen as a camera mounted 4 degrees further down than autonomy.yaml says.
        checked = subprocess.run(['ros2', 'run', 'rover_autonomy', 'mount_check', '--ros-args', '--params-file',
                                  str(self.settings_file), '-p', 'timeout_s:=20.0'], capture_output=True, text=True, timeout=60,
                                 env=dict(os.environ, ROS_DOMAIN_ID=str(self.rover_domain)))
        report = checked.stdout + checked.stderr
        self.check('a camera mounted 4 degrees off is reported, with the value to set',
                   '4.0 degrees further down' in report and 'set zed_pitch_rad: 0.070' in report,
                   ' | '.join(line.split('camera mount check: ')[-1] for line in report.splitlines() if 'mount check' in line)[:300])
        self.rover.pitch = 0.0
        time.sleep(1.0)

        print('\n2. Goals only in AUTO', flush=True)
        self.left_y = 0.0
        time.sleep(0.2)
        self.press(OPTIONS)
        self.check('Options: driving enabled (manual)', self.wait(lambda s: s == (fake_esp.DRIVING, STOPPED), 2.0) is not None)
        since = len(self.events)
        self.goal(3.0, 2.0)
        self.check('a goal in manual is ignored', self.event('not in AUTO', 3.0, since) and self.holds_still(2.0))
        self.press(TRIANGLE)
        self.check('Triangle: AUTO on, the rover holds still without a goal', self.base_auto is True and self.holds_still(2.0)
                   and self.esp.state() == (fake_esp.DRIVING, STOPPED))

        print('\n3. A goal behind a post (Nav2 plans around it; the mux lifts slow commands over the deadband)', flush=True)
        closest = [99.0]
        stop_watch = threading.Event()

        def watch_post() -> None:
            while not stop_watch.is_set():
                closest[0] = min(closest[0], self.rover.distance_to(*POST))
                time.sleep(0.02)
        threading.Thread(target=watch_post, daemon=True).start()
        self.drive_to(5.0, 0.0, 'goal (5, 0) reached')
        stop_watch.set()
        clearance = closest[0] - POST_RADIUS - SETTINGS['rover_width_m'] / 2
        self.check('kept clear of the post', clearance > 0.0, f'closest edge-to-post {clearance:.2f} m')
        error = self.ekf_error()
        self.check('EKF (odom -> base_footprint) matches the rover', error is not None and error[0] < 0.1,
                   f'{error[0]:.3f} m off: {error[1]}' if error is not None else 'no transform')

        print('\n3a. Arcs: a goal off to the side is reached rolling, then the rover turns to the arrow', flush=True)
        x, y, yaw = self.rover.pose()
        ahead, left = 3.5, 2.5  # in the rover's frame when the goal is given
        target = (x + ahead * math.cos(yaw) - left * math.sin(yaw), y + ahead * math.sin(yaw) + left * math.cos(yaw))
        arrow = yaw - math.pi / 2  # pointing to the rover's right: a quarter turn on the spot at the end
        before = len(self.rover.pivots)
        since = len(self.events)
        self.goal(*target, yaw=arrow)
        reached = self.event('goal reached', 90.0, since)
        time.sleep(1.0)
        on_the_way = sum(angle for px, py, angle in self.rover.pivots[before:] if math.hypot(px - target[0], py - target[1]) > 0.5)
        off = abs(math.remainder(self.rover.pose()[2] - arrow, 2 * math.pi))
        tolerance = math.radians(AUTONOMY['/**']['ros__parameters'].get('goal_heading_tolerance_deg', 15.0))
        self.check('on the way the rover rolls through the curve (no turning on the spot)',
                   reached and math.degrees(on_the_way) < 15.0, f'{math.degrees(on_the_way):.0f} degrees turned on the spot on the way')
        self.check('at the goal it turns to the arrow', reached and self.rover.distance_to(*target) < 0.5 and off < tolerance + math.radians(5),
                   f'{math.degrees(off):.0f} degrees from the arrow, {self.rover.distance_to(*target):.2f} m from the goal; {self.where()}')

        print('\n3b. Goals in a moving frame, and goals that bypass goal_manager', flush=True)
        since = len(self.events)
        x, y, yaw = self.rover.pose()
        target = (x + 2.0 * math.cos(yaw), y + 2.0 * math.sin(yaw))  # where "2 m ahead" is when the goal is given
        self.goal(2.0, 0.0, frame='base_footprint')  # RViz with its Fixed Frame on the rover
        reached = self.event('goal reached', 90.0, since)
        distance = self.rover.distance_to(*target)
        self.check("a goal in the rover's own frame is fixed in the map when it arrives", reached and distance < 0.5,
                   f'{distance:.2f} m from the point that was 2 m ahead; {self.where()}')
        bypass = NavigateToPose.Goal()
        bypass.pose.header.frame_id = 'map'
        bypass.pose.pose.position.x, bypass.pose.pose.position.y = target[0] - 2.0, target[1]
        bypass.pose.pose.orientation.w = 1.0
        sent = self.nav.send_goal_async(bypass)  # around goal_manager, like Nav2's RViz panel
        self.check('a goal sent to Nav2 around goal_manager does not move the rover', self.holds_still(5.0))
        handle = sent.result() if sent.done() else None
        if handle is not None and handle.accepted:
            handle.cancel_goal_async()
        time.sleep(1.0)

        print('\n3c. AUTO stops when the camera or Nav2 can no longer see', flush=True)
        since = len(self.events)
        self.goal(0.0, -3.0)
        time.sleep(4.0)
        self.cloud_on = False  # the camera's point cloud stops; its odometry goes on
        self.check("the point cloud stops: the goal is cancelled and the rover stands",
                   self.event('goal cancelled: no obstacle data from the camera', 5.0, since) and self.holds_still(2.0))
        self.cloud_on = True
        time.sleep(2.0)
        since = len(self.events)
        self.goal(0.0, -3.0)
        self.check('the point cloud is back: the next goal is driven', self.event('goal accepted', 3.0, since) and self.moves(4.0))
        ekf = subprocess.run(['pgrep', '-s', str(self.rover_launch.process.pid), '-f', 'robot_localization/ekf_node'],
                             capture_output=True, text=True).stdout.split()
        if self.check('found the EKF to freeze', len(ekf) == 1, f'pids {ekf}'):
            os.kill(int(ekf[0]), signal.SIGSTOP)  # Nav2's transforms stop arriving, as when its own copy of them froze
            try:
                self.check("Nav2's transforms stop: the goal is cancelled and the rover stands",
                           self.event('has lost track of the rover', 6.0, since) and self.holds_still(2.0))
            finally:
                os.kill(int(ekf[0]), signal.SIGCONT)
            # The costmaps catch up within seconds (in the rehearsal's VM up to ten); the next goal must wait for that.
            cleared = time.monotonic() + 30.0
            while time.monotonic() < cleared and self.fault != '':
                time.sleep(0.1)
            self.check("the transforms are back: AUTO is ready again", self.fault == '', f'fault: {self.fault!r}')

        print('\n3d. Ground that needs more than the measured deadband', flush=True)
        # The settings say 20 and 60 rpm. Here a turn on the spot needs 70: Nav2's turn commands come out at 68 at most,
        # so without the stall kick the rover would stand at the goal and never turn to its heading.
        self.rover.deadband = (26.0, 70.0)
        since = len(self.events)
        x, y, yaw = self.rover.pose()
        ahead, about_face = (x + 0.8 * math.cos(yaw), y + 0.8 * math.sin(yaw)), yaw + math.pi
        self.goal(*ahead, yaw=about_face)  # a short way straight ahead, then turn round on the spot
        reached = self.event('goal reached', 90.0, since)
        off = abs(math.remainder(self.rover.pose()[2] - about_face, 2 * math.pi))
        self.check('the rover turns round on the spot anyway (the stall kick)', reached and off < 0.6,
                   f'{"goal reached" if reached else self.events[since:]}, {math.degrees(off):.0f} degrees from the heading asked for; {self.where()}')
        self.rover.deadband = TRUE_DEADBAND

        print('\n4. Radio lost while a goal runs: the rover goes on to its goal, then the e-stop latches', flush=True)
        since = len(self.events)
        self.goal(0.0, 3.0)
        time.sleep(4.0)
        self.radio = False
        self.check('radio lost: the rover goes on', self.event('goes on to its goal', 3.0, since) and self.moves(3.0))
        reached, left_driving = self.event_while_driving('goal reached', 150.0, since)
        self.check('goal (0, 3) reached without the base station', reached and self.rover.distance_to(0.0, 3.0) < 0.5)
        self.check('the ESP32 was never stopped on the way', not left_driving)
        self.check('at the goal, still no base station: the e-stop is latched',
                   self.wait(lambda s: s == (fake_esp.ESTOP, STOPPED), 3.0) is not None and self.holds_still(3.0)
                   and self.event('e-stop is latched until it is back', 1.0, since))
        self.radio = True
        self.check('radio back, still in AUTO: enabled again and holding still',
                   self.wait(lambda s: s == (fake_esp.DRIVING, STOPPED), 3.0) is not None and self.holds_still(2.0))

        print('\n5. Base station restarted while a goal runs: the rover goes on until the base station is heard again', flush=True)
        since = len(self.events)
        self.goal(9.0, 3.0)  # far enough to be still under way when the base station is back
        time.sleep(4.0)
        self.base_launch.kill()  # the base station laptop crashes
        self.base_auto = None
        self.check('base station gone: the rover goes on', self.event('goes on to its goal', 3.0, since) and self.moves(3.0))
        self.start_base()  # it comes up e-stopped and in manual
        self.check('restarted base station heard (e-stopped): the goal is cancelled', self.event('goal cancelled: e-stop', 40.0, since))
        self.check('and the rover stops', self.wait(lambda s: s == (fake_esp.ESTOP, STOPPED), 2.0) is not None and self.holds_still(2.0)
                   and self.rover.distance_to(9.0, 3.0) > 1.0)
        self.check('Options + Triangle: AUTO again, and the old goal does not come back', self.ready_for_auto() and self.holds_still(3.0))

        print('\n6. E-stop in AUTO cancels the goal', flush=True)
        since = len(self.events)
        self.goal(0.0, 0.0)
        time.sleep(4.0)
        self.pressed = {CIRCLE}
        latency = self.wait(lambda s: s == (fake_esp.ESTOP, STOPPED), 2.0)
        self.pressed = set()
        self.check('Circle: e-stop at once', latency is not None and latency < 0.2, f'{latency:.3f} s' if latency is not None else self.where())
        self.check('the goal is cancelled', self.event('goal cancelled: e-stop', 3.0, since))
        self.check('released, back in AUTO: the cancelled goal does not come back', self.ready_for_auto() and self.holds_still(3.0))

        print('\n7. A stick push takes over from Nav2', flush=True)
        since = len(self.events)
        self.goal(0.0, 0.0)
        time.sleep(4.0)
        self.left_y = 0.4
        self.check('stick pushed: AUTO off, the goal is cancelled', self.event('goal cancelled: manual driving', 3.0, since)
                   and self.base_auto is False)
        self.check('the rover drives on the stick', self.wait(lambda s: s[0] == fake_esp.DRIVING and all(p > 0 for p in s[1]), 2.0) is not None)
        self.left_y = 0.0
        self.check('stick released: stops', self.wait(lambda s: s == (fake_esp.DRIVING, STOPPED), 2.0) is not None)

        print('\n8. End of the session', flush=True)
        self.press(CIRCLE)
        self.check('Circle: e-stop', self.wait(lambda s: s == (fake_esp.ESTOP, STOPPED), 1.0) is not None)
        codes = (self.base_launch.ctrl_c(), self.rover_launch.ctrl_c())
        logs = self.base_launch.text() + self.rover_launch.text()
        self.check('both launches stop on Ctrl-C', codes == (0, 0), f'exit codes {codes}')
        self.check('no tracebacks in any launch log', 'Traceback' not in logs)
        self.check('the mux said when it raised its output for a rover that did not start',
                   'the rover does not start: raising the output' in self.rover_launch.text())
        mount = [line.split('camera mount check: ')[-1] for line in self.rover_launch.text().splitlines() if 'camera mount check' in line]
        self.check("at the start the camera's mount was checked against the ground and agreed with the settings",
                   len(mount) == 1 and 'as the settings say' in mount[0], ' | '.join(mount)[:300] or 'no report')
        return all(ok for _, ok, _ in self.results)

    def close(self) -> None:
        for launch in (self.base_launch, self.rover_launch):
            if launch is not None and launch.process.poll() is None:
                launch.ctrl_c()
        self.esp.close()
        for executor, thread in self.executors:
            executor.shutdown()
            thread.join(5)
        self.node.destroy_node()
        self.base.destroy_node()
        rclpy.shutdown(context=self.rover_context)
        rclpy.shutdown(context=self.base_context)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--wheels', default='wheel_28cm')
    parser.add_argument('--rover-domain', type=int, default=61, help='ROS_DOMAIN_ID of the rover side (default 61)')
    parser.add_argument('--base-domain', type=int, default=62, help='ROS_DOMAIN_ID of the base station side (default 62)')
    args = parser.parse_args()
    try:
        ros_sandbox.enter(args.base_domain)  # checks both numbers; each side then gets its own domain
        ros_sandbox.enter(args.rover_domain, {})
    except ValueError as error:
        parser.error(str(error))
    if args.rover_domain == args.base_domain:
        parser.error('the rover and the base station need two different domains (the script is the radio between them)')
    logs = Path(tempfile.mkdtemp(prefix='rehearsal_auto_'))
    rehearsal = Rehearsal(args.wheels, logs, args.rover_domain, args.base_domain)
    try:
        ok = rehearsal.run()
    except KeyboardInterrupt:
        ok = False
    finally:
        rehearsal.close()
    failed = [name for name, passed, _ in rehearsal.results if not passed]
    print(f"\n{len(rehearsal.results) - len(failed)}/{len(rehearsal.results)} passed" + (f'; FAILED: {", ".join(failed)}' if failed else ''))
    print(f'logs: {logs}')
    return 0 if ok and not failed else 1


if __name__ == '__main__':
    sys.exit(main())
