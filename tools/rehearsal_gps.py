#!/usr/bin/env python3
"""Dress rehearsal of the GPS mode (AUTONOMY.md, "GPS") without hardware: the real launch with localization:=gps.

As tools/rehearsal_auto.py (the simulated ESP32, rover, camera and radio; Nav2, the EKFs, drive_mux, goal_manager),
plus a simulated Quectel L89HA on a pseudo-terminal, sending NMEA from where the simulated antenna really is:
  - the simulated world's x axis points 30 degrees north of east (the rehearsal knows; the rover must find it);
  - the GPS has a slowly wandering error of up to about 1.5 m and some noise on top, as a real receiver;
  - the camera's odometry overstates every distance by 4 %: drift, which the GPS has to correct.
It checks that no GPS goal is taken before the rover has found north by driving, that it then finds it, that the GPS
keeps the rover's position right although the odometry drifts, and that GPS waypoint lists are driven in all three
modes (continue, pause, wait), a far one in legs, on through a GPS outage, and stopped by the e-stop.

    python3 tools/rehearsal_gps.py      # PASS/FAIL table, exit code 1 on any failure (about 12 minutes)
"""
from __future__ import annotations

import argparse
import math
import os
import random
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rehearsal_auto as ra  # noqa: E402

sys.path.insert(0, str(ra.SRC / 'rover_autonomy'))
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.time import Time  # noqa: E402
from sensor_msgs.msg import Joy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from rover_autonomy import geo, nmea  # noqa: E402
from rover_autonomy.frames import yaw_of  # noqa: E402
from rover_autonomy.waypoint_logic import encode_command, encode_list  # noqa: E402

REFERENCE = (28.7500, 77.1100)  # latitude, longitude of the simulated world's east-north origin
WORLD_TO_ENU_DEG = 30.0  # the simulated world's x axis, counter-clockwise from east
START_ENU = (12.0, -5.0)  # where the simulated world's origin is, east and north of REFERENCE
ODOMETRY_SCALE = 1.04  # the camera's odometry overstates distances by 4 %


class GpsRehearsal(ra.Rehearsal):
    def __init__(self, *args) -> None:
        self.gps_master, _slave, self.gps_path = ra.fake_esp.open_pty()
        self.gps_on = True
        self.right_x = 0.0
        self.status = ''
        super().__init__(*args)
        settings = ra.yaml.safe_load(self.settings_file.read_text())
        settings['gps_node']['ros__parameters']['gps_port'] = self.gps_path
        self.settings_file.write_text(ra.yaml.safe_dump(settings, sort_keys=False))
        self.node.create_subscription(String, '/rover/gps_status', lambda m: setattr(self, 'status', m.data),
                                      ra.QoSProfile(depth=1, durability=ra.DurabilityPolicy.TRANSIENT_LOCAL,
                                                    reliability=ra.ReliabilityPolicy.RELIABLE))
        self.global_pose: tuple[float, float] | None = None
        self.node.create_subscription(Odometry, '/odometry/global', self.on_global, 10)
        self.gps_errors: list[float] = []
        threading.Thread(target=self.send_gps, daemon=True).start()

    # ------------------------------------------------------------------ the simulated world
    def publish_world(self) -> None:
        """No map -> odom here: in the GPS mode ekf_global publishes it."""

    def start_rover(self) -> None:
        self.rover_launch = ra.Launch('rover', self.logs, self.rover_domain, 'rover_autonomy', 'rover_autonomy.launch.py',
                                      'zed:=false', 'slam:=false', 'localization:=gps', f'wheels:={self.wheels}',
                                      f'settings:={self.settings_file}', *self.esp.launch_arguments)

    def publish_joy(self) -> None:
        message = Joy()
        message.axes = [0.0] * 8
        message.axes[ra.TELEOP['axis_linear']] = self.left_y
        message.axes[ra.TELEOP['axis_angular']] = self.right_x
        message.buttons = [1 if index in self.pressed else 0 for index in range(15)]
        self.joy.publish(message)

    def simulate(self) -> None:
        """As the rehearsal's, but the camera's odometry overstates distances (drift)."""
        self.rover.step(0.02)
        self.ticks += 1
        stamp = self.node.get_clock().now().to_msg()
        if self.ticks % 2 == 0:
            x, y, yaw = self.rover.camera()
            message = ra.Odometry()
            message.header.stamp, message.header.frame_id = stamp, 'odom'
            message.child_frame_id = f"{ra.SETTINGS['zed_camera_name']}_camera_link"
            message.pose.pose.position.x, message.pose.pose.position.y = x * ODOMETRY_SCALE, y * ODOMETRY_SCALE
            message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            message.pose.covariance = [0.0] * 36
            for index in (0, 7, 14, 21, 28, 35):
                message.pose.covariance[index] = 1e-4
            self.odom.publish(message)
        if self.ticks % 10 == 0 and self.cloud_on:
            self.points.publish(ra.cloud(self.rover.sees(), f"{ra.SETTINGS['zed_camera_name']}_camera_link", stamp))

    def world_to_latlon(self, x: float, y: float) -> tuple[float, float]:
        theta = math.radians(WORLD_TO_ENU_DEG)
        east = math.cos(theta) * x - math.sin(theta) * y + START_ENU[0]
        north = math.sin(theta) * x + math.cos(theta) * y + START_ENU[1]
        return geo.from_enu(east, north, REFERENCE)

    def send_gps(self) -> None:
        """The L89HA: GGA and RMC twice a second from the antenna's true position, with a wandering error."""
        rng = random.Random(7)
        bias = [0.0, 0.0]
        gx, gy = ra.SETTINGS.get('gps_x_m', 0.0), ra.SETTINGS.get('gps_y_m', 0.0)
        second = 43200.0
        while True:
            time.sleep(0.5)
            second += 0.5
            bias = [max(-1.5, min(1.5, b + rng.gauss(0.0, 0.05))) for b in bias]
            x, y, yaw = self.rover.pose()
            ax, ay = x + math.cos(yaw) * gx - math.sin(yaw) * gy, y + math.sin(yaw) * gx + math.cos(yaw) * gy
            theta = math.radians(WORLD_TO_ENU_DEG)
            east = math.cos(theta) * ax - math.sin(theta) * ay + START_ENU[0] + bias[0] + rng.gauss(0.0, 0.25)
            north = math.sin(theta) * ax + math.cos(theta) * ay + START_ENU[1] + bias[1] + rng.gauss(0.0, 0.25)
            latitude, longitude = geo.from_enu(east, north, REFERENCE)
            if not self.gps_on:
                lines = nmea.sentence('GNGGA,%s,,,,,0,0,,,M,,M,,' % f'{int(second // 3600):02d}{int(second % 3600 // 60):02d}{second % 60:06.3f}')
            else:
                lines = nmea.gga_sentence(second, latitude, longitude, quality=1, satellites=14, hdop=0.8)
                lines += nmea.rmc_sentence(second, latitude, longitude)
            try:
                os.write(self.gps_master, lines.encode('ascii'))
            except OSError:
                pass

    def on_global(self, message: Odometry) -> None:
        self.global_pose = (message.pose.pose.position.x, message.pose.pose.position.y)

    # ------------------------------------------------------------------ helpers
    def map_offset(self) -> tuple[float, float]:
        """ekf_global's map frame starts at the rover's first pose: the simulated world shifted by its start."""
        return 0.4, 0.0  # the rover starts with base_footprint at (-0.4, 0) in the world, yaw 0

    def position_error(self) -> float | None:
        """How far ekf_global has the rover from where it really is."""
        if self.global_pose is None:
            return None
        x, y, _ = self.rover.pose()
        ox, oy = self.map_offset()
        return math.hypot(self.global_pose[0] - (x + ox), self.global_pose[1] - (y + oy))

    def odom_error(self) -> float | None:
        try:
            transform = self.tf.lookup_transform('odom', 'base_footprint', Time())
        except Exception:  # noqa: BLE001
            return None
        x, y, _ = self.rover.pose()
        return math.hypot(transform.transform.translation.x - x, transform.transform.translation.y - y)

    def found_north(self) -> float | None:
        """The rover's estimate of the world's heading against east (degrees), from TF map -> enu."""
        try:
            transform = self.tf.lookup_transform('map', 'enu', Time())
        except Exception:  # noqa: BLE001
            return None
        r = transform.transform.rotation
        return -math.degrees(yaw_of((0.0, 0.0, 0.0, r.x, r.y, r.z, r.w)))

    def send_list(self, points, mode='continue', pause_s=10.0) -> None:
        latlon = [self.world_to_latlon(*point) for point in points]
        self.command(encode_list(latlon, mode, pause_s))

    def command(self, payload: str) -> None:
        publisher = getattr(self, '_gps_command', None)
        if publisher is None:
            publisher = self._gps_command = self.node.create_publisher(String, '/rover/gps_command', 10)
            time.sleep(1.0)
        publisher.publish(String(data=payload))

    def drive(self, forward: float, turn: float, seconds: float) -> None:
        self.left_y, self.right_x = forward, turn
        time.sleep(seconds)
        self.left_y = self.right_x = 0.0
        time.sleep(0.6)

    def true_distance(self, point) -> float:
        x, y, _ = self.rover.pose()
        return math.hypot(x - point[0], y - point[1])

    # ------------------------------------------------------------------ the run
    def run(self) -> bool:
        print(f'logs in {self.logs}; rover in ROS domain {self.rover_domain}, base station in {self.base_domain}; '
              f'GPS on {self.gps_path}', flush=True)
        for domain, node in ((self.rover_domain, self.node), (self.base_domain, self.base)):
            others = ra.ros_sandbox.foreign_nodes(node)
            if not self.check(f'ROS domain {domain} is ours alone', not others, ra.ros_sandbox.refusal(domain, others) if others else 'no other node'):
                return False
        print('\n1. Launch in the GPS mode', flush=True)
        self.start_rover()
        self.start_base()
        self.check('Nav2 comes up (navigate_to_pose available)', self.nav.wait_for_server(timeout_sec=90.0))
        self.check('the e-stop starts latched', self.wait(lambda s: s[0] == ra.fake_esp.ESTOP, 30.0) is not None)
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline and 'drive' not in self.status:
            time.sleep(0.2)
        self.check('the GPS has a fix; its heading is unknown until the rover drives', 'drive 15 m more' in self.status,
                   self.status)

        print('\n2. No GPS goal before north is known', flush=True)
        self.check('AUTO on', self.ready_for_auto())
        since = len(self.events)
        self.send_list([(5.0, 5.0)])
        self.check('a GPS waypoint is refused, with what to do', self.event('GPS waypoints ignored: GPS heading unknown', 5.0, since)
                   and self.holds_still(1.0), (self.events[since:] or ['no event'])[-1])

        print('\n3. Drive by hand: the rover finds north from its track', flush=True)
        self.left_y = 0.0
        self.press(ra.TRIANGLE)  # AUTO off: manual
        time.sleep(0.5)
        self.drive(0.6, 0.0, 1.0)
        self.drive(0.6, -0.25, 6.0)  # a curve round the post
        self.left_y = 0.7  # then straight on until the rover says it knows north (at most 40 s)
        deadline = time.monotonic() + 40.0
        while time.monotonic() < deadline and not self.status.startswith('GPS ready'):
            time.sleep(0.2)
        self.left_y = 0.0
        time.sleep(1.0)
        north = self.found_north()
        self.check('after the drive the GPS is ready', self.status.startswith('GPS ready'), f'{self.status}; {self.where()}')
        self.check('north found to within 5 degrees', north is not None and abs(north - WORLD_TO_ENU_DEG) < 5.0,
                   f'found {north:.1f}, really {WORLD_TO_ENU_DEG:g} degrees' if north is not None else 'no map -> enu')
        time.sleep(5.0)
        odom, error = self.odom_error(), self.position_error()
        self.check("the GPS corrects the odometry's drift", error is not None and odom is not None and error < 2.0,
                   f'odometry {odom:.2f} m off by now, the GPS-corrected position {error:.2f} m' if error is not None and odom is not None else 'no estimate')

        print('\n4. A list of GPS waypoints, driven in turn (continue)', flush=True)
        self.check('AUTO on again', self.ready_for_auto())
        x, y, yaw = self.rover.pose()
        ahead = lambda d, side: (x + d * math.cos(yaw) - side * math.sin(yaw), y + d * math.sin(yaw) + side * math.cos(yaw))
        first, second = ahead(10.0, 4.0), ahead(16.0, -4.0)
        since = len(self.events)
        self.send_list([first, second])
        self.check('waypoint 1 reached and on to 2 without stopping', self.event('waypoint 1/2 reached', 120.0, since),
                   f'{self.true_distance(first):.2f} m from it; {self.where()}')
        done = self.event('the last one, done', 120.0, since)
        self.check('the last waypoint reached, within 2.5 m of its coordinates', done and self.true_distance(second) < 2.5,
                   f'{self.true_distance(second):.2f} m from it; {self.where()}')

        print('\n5. Pause at each waypoint', flush=True)
        x, y, yaw = self.rover.pose()
        first, second = ahead(8.0, 3.0), ahead(14.0, 0.0)
        since = len(self.events)
        self.send_list([first, second], 'pause', 6.0)
        reached = self.event('pausing 6 s', 120.0, since)
        self.check('waypoint 1: stops and says it pauses', reached and self.holds_still(4.0), self.where())
        self.check('then goes on by itself to the last', self.event('the last one, done', 120.0, since)
                   and self.true_distance(second) < 2.5, f'{self.true_distance(second):.2f} m from it')

        print('\n6. Wait at each waypoint until next', flush=True)
        x, y, yaw = self.rover.pose()
        first, second = ahead(8.0, -3.0), ahead(14.0, 0.0)
        since = len(self.events)
        self.send_list([first, second], 'wait')
        self.check('waypoint 1: waits for next', self.event('waiting; send next', 120.0, since) and self.holds_still(8.0))
        subprocess.run(['ros2', 'run', 'rover_autonomy', 'gps_goals', 'next'], capture_output=True, text=True, timeout=30,
                       env=dict(os.environ, ROS_DOMAIN_ID=str(self.rover_domain)))
        self.check("'gps_goals next' sends it on to the last", self.event('the last one, done', 120.0, since)
                   and self.true_distance(second) < 2.5, f'{self.true_distance(second):.2f} m from it')

        print('\n7. A far waypoint in legs, on through a GPS outage, typed in degrees-minutes-seconds', flush=True)
        x, y, yaw = self.rover.pose()
        far = ahead(70.0, 0.0)
        latitude, longitude = self.world_to_latlon(*far)
        typed = geo.format_position(latitude, longitude).split('(')[1].rstrip(')')  # e.g. 28°45'01.2"N 77°07'03.0"E
        since = len(self.events)
        sent = subprocess.run(['ros2', 'run', 'rover_autonomy', 'gps_goals', typed, '--no-follow'], capture_output=True,
                              text=True, timeout=30, env=dict(os.environ, ROS_DOMAIN_ID=str(self.rover_domain)))
        self.check('gps_goals reads degrees-minutes-seconds and sends', 'sent' in sent.stdout, (sent.stdout + sent.stderr)[-300:])
        time.sleep(25.0)
        self.gps_on = False
        self.check('the GPS drops out on the way: the rover goes on', self.moves(5.0), self.status)
        time.sleep(10.0)
        self.gps_on = True
        done = self.event('the last one, done', 240.0, since)
        legs = self.rover_launch.text().count('goal sent to Nav2')
        self.check('the far waypoint is reached in legs, within 2.5 m', done and self.true_distance(far) < 2.5,
                   f'{self.true_distance(far):.2f} m from it; {legs} goals sent in all so far')
        time.sleep(3.0)
        odom, error = self.odom_error(), self.position_error()
        x, y, _ = self.rover.pose()
        self.check(f"after {math.hypot(x + 0.4, y):.0f} m from the start the GPS still holds the position, not the drift",
                   error is not None and odom is not None and error < 2.0 and odom > 2.0 * error,
                   f'odometry {odom:.2f} m off, the GPS-corrected position {error:.2f} m' if error is not None and odom is not None else 'no estimate')

        print('\n8. The e-stop stops a list', flush=True)
        x, y, yaw = self.rover.pose()
        since = len(self.events)
        self.send_list([ahead(15.0, 0.0), ahead(25.0, 0.0)])
        time.sleep(5.0)
        self.pressed = {ra.CIRCLE}
        time.sleep(0.5)
        self.pressed = set()
        self.check('Circle: the list is stopped', self.event('GPS waypoints stopped', 5.0, since) and self.holds_still(2.0))

        print('\n9. End', flush=True)
        codes = (self.base_launch.ctrl_c(), self.rover_launch.ctrl_c())
        logs = self.base_launch.text() + self.rover_launch.text()
        self.check('both launches stop on Ctrl-C', codes == (0, 0), f'exit codes {codes}')
        self.check('no tracebacks in any launch log', 'Traceback' not in logs)
        return all(ok for _, ok, _ in self.results)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--wheels', default='wheel_28cm')
    parser.add_argument('--rover-domain', type=int, default=65)
    parser.add_argument('--base-domain', type=int, default=66)
    args = parser.parse_args()
    try:
        ra.ros_sandbox.enter(args.base_domain)
        ra.ros_sandbox.enter(args.rover_domain, {})
    except ValueError as error:
        parser.error(str(error))
    logs = Path(tempfile.mkdtemp(prefix='rehearsal_gps_'))
    rehearsal = GpsRehearsal(args.wheels, logs, args.rover_domain, args.base_domain)
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
