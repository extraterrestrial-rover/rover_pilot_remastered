"""Rover-side drive node: /cmd_vel + the base station's e-stop heartbeat -> DRIVE_COMMAND to the drive ESP32 at 50 Hz.

Replaces the old rover_pilot drive_node. It talks to the ESP32 over the UDP link (udp_link.py, LINK.md), keeps sending
commands at a fixed rate (the firmware stops the motors 300 ms after the last one), and sends the e-stop flag whenever
the base station's e-stop is latched or its heartbeat is missing.
"""
from __future__ import annotations

import errno
import signal
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from rover_pilot_remastered import protocol
from rover_pilot_remastered.drive_logic import DriveGate
from rover_pilot_remastered.kinematics import DriveGeometry, twist_to_wheel_rpm
from rover_pilot_remastered.udp_link import UdpLink

LATEST_ONLY = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)
LOST_AFTER_S = 1.0  # no DRIVE_STATUS (10 Hz) for this long: the ESP32 counts as lost


class DriveNode(Node):
    def __init__(self) -> None:
        super().__init__('drive_node')
        param = self.declare_parameter
        self.link = UdpLink(param('master_host', '192.168.77.2').value, param('master_port', 47700).value,
                            param('local_port', 47701).value, param('interface', '').value)
        self.target = f'{self.link.master_host}:{self.link.master_port}'

        rate_hz = param('rate_hz', 50.0).value
        self.gate = DriveGate(param('cmd_timeout_s', 0.3).value, param('estop_timeout_s', 0.5).value,
                              param('require_estop_heartbeat', True).value)
        self.geometry = DriveGeometry(param('wheel_diameter_m', 0.28).value, param('track_width_m', 0.84).value,
                                      param('max_wheel_rpm', 94.7).value, param('turn_track_factor', 1.0).value)
        param('wheel_width_m', 0.14)  # informational: recorded with the wheel profile
        self.swap_sides = param('swap_sides', False).value

        self.create_subscription(Twist, param('cmd_vel_topic', '/cmd_vel').value, self.on_cmd_vel, LATEST_ONLY)
        self.create_subscription(Bool, param('estop_topic', '/rover/estop').value, self.on_estop, LATEST_ONLY)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.next_open_attempt = 0.0
        self.link_error: str | None = None
        self.started = time.monotonic()
        self.lost_reported = False
        self.warned_silent = False
        self.seq = 0
        self.status: protocol.DriveStatus | None = None
        self.status_time: float | None = None
        self.last_reason: str | None = None
        self.last_state: int | None = None
        self.warned_rpm_mismatch = False
        self.sent = 0

        self.create_timer(1.0 / rate_hz, self.tick)
        self.create_timer(1.0, self.publish_diagnostics)
        via = f', only through {self.link.interface}' if self.link.interface else ''
        self.get_logger().info(
            f'drive node: ESP32 at {self.target} over UDP (listening on {self.link.local_port}{via}), '
            f'wheel {self.geometry.wheel_diameter_m} m, track '
            f'{self.geometry.track_width_m} m, max {self.geometry.max_linear_speed:.2f} m/s '
            f'({self.geometry.max_wheel_rpm} wheel rpm at full PWM)')

    def on_cmd_vel(self, msg: Twist) -> None:
        self.gate.on_cmd_vel(msg.linear.x, msg.angular.z, time.monotonic())

    def on_estop(self, msg: Bool) -> None:
        self.gate.on_estop(msg.data, time.monotonic())

    def tick(self) -> None:
        now = time.monotonic()
        decision = self.gate.decide(now)
        if decision.reason != self.last_reason:
            if decision.reason:
                self.get_logger().warn(f'stopped: {decision.reason}')
            else:
                self.get_logger().info('driving enabled')
            self.last_reason = decision.reason
        targets = twist_to_wheel_rpm(decision.linear_x, decision.angular_z, self.geometry)
        if self.swap_sides:
            targets = (targets[1], targets[0], targets[3], targets[2])
        self.send(protocol.drive_command_payload(decision.estop, targets))
        self.receive()
        self.watch_link(now)

    def ensure_link(self) -> bool:
        if self.link.is_open:
            return True
        now = time.monotonic()
        if now < self.next_open_attempt:
            return False
        self.next_open_attempt = now + 1.0
        try:
            self.link.open()
        except OSError as error:
            text = (f'UDP port {self.link.local_port} is in use: the CAN bridge or another drive_node has it, and only one '
                    'program may drive') if error.errno == errno.EADDRINUSE else f'cannot open the UDP link: {error}'
            if text != self.link_error:
                self.get_logger().error(f'{text} (retrying every second)')
                self.link_error = text
            return False
        self.link_error = None
        self.get_logger().info(f'UDP link open: sending to {self.target}, receiving on port {self.link.address[1]}')
        return True

    def send(self, payload: bytes) -> None:
        if not self.ensure_link():
            return
        if self.link.send(protocol.encode_packet(protocol.TYPE_DRIVE_COMMAND, self.seq, payload)):
            self.seq = (self.seq + 1) & 0xFF
            self.sent += 1

    def watch_link(self, now: float) -> None:
        """Log when the ESP32 goes silent and when it is back; name the usual causes if it was never heard."""
        if self.status_time is not None:
            silent = now - self.status_time > LOST_AFTER_S
            if silent and not self.lost_reported:
                cable = ' (the PC port has no carrier: Ethernet cable out?)' if self.link.carrier() is False else ''
                self.get_logger().error(f'ESP32 lost: no DRIVE_STATUS from {self.target} for {LOST_AFTER_S:.0f} s{cable}')
                self.lost_reported = True
            elif not silent and self.lost_reported:
                self.get_logger().info('ESP32 back')
                self.lost_reported = False
        elif not self.warned_silent and now - self.started > 5.0 and self.link.is_open:
            self.get_logger().warn(f'nothing heard from the ESP32 at {self.target} yet: check the Ethernet cable, that the PC '
                                   'port has 192.168.77.1/24 (LINK.md), and that the ESP32 runs the Ethernet firmware (fw 3.x)')
            self.warned_silent = True

    def receive(self) -> None:
        for packet in self.link.receive():
            status = protocol.parse_drive_status(packet.payload) if packet.type == protocol.TYPE_DRIVE_STATUS else None
            if status is None:
                continue
            if status.state != self.last_state:
                self.get_logger().info(f'ESP32 state: {status.state_name}')
                self.last_state = status.state
            if not self.warned_rpm_mismatch and abs(status.max_wheel_rpm - self.geometry.max_wheel_rpm) > 0.05:
                self.get_logger().warn(f'firmware MAX_WHEEL_RPM is {status.max_wheel_rpm} but max_wheel_rpm parameter is '
                                       f'{self.geometry.max_wheel_rpm}: speeds will be scaled wrongly')
                self.warned_rpm_mismatch = True
            self.status, self.status_time = status, time.monotonic()

    def publish_diagnostics(self) -> None:
        now = time.monotonic()
        status, decision = self.status, self.gate.decide(now)
        entry = DiagnosticStatus(name='rover drive', hardware_id=self.target)
        if not self.link.is_open:
            entry.level, entry.message = DiagnosticStatus.ERROR, f'UDP link not open: {self.link_error or "opening"}'
        elif status is None or self.status_time is None or now - self.status_time > 0.5:
            cable = ' (Ethernet cable out?)' if self.link.carrier() is False else ''
            entry.level, entry.message = DiagnosticStatus.ERROR, f'no status from the ESP32{cable}'
        elif status.state_name == 'DRIVING' and not decision.reason:
            entry.level, entry.message = DiagnosticStatus.OK, 'driving'
        else:
            entry.level, entry.message = DiagnosticStatus.WARN, decision.reason or status.state_name.lower()
        values = {'gate': decision.reason or 'driving', 'commands_sent': self.sent, 'link': f'udp {self.target}',
                  'link_rx_good': self.link.decoder.good, 'link_rx_bad': self.link.bad,
                  'link_from_others': self.link.foreign, 'link_send_errors': self.link.send_errors}
        if self.link.interface:
            carrier = self.link.carrier()
            values['interface'] = (f'{self.link.interface}: ' + ('up' if carrier else 'no carrier') +
                                   ('' if self.link.pinned else f', NOT pinned ({self.link.pin_error or "missing"})'))
        if status is not None:
            values.update({'esp_state': status.state_name, 'command_age_ms': status.command_age_ms,
                           'pwm_fl_fr_rl_rr': ' '.join(str(p) for p in status.pwm), 'esp_rx_good': status.rx_good,
                           'esp_rx_bad': status.rx_bad, 'esp_uptime_s': round(status.uptime_ms / 1000, 1),
                           'esp_reset_reason': status.reset_reason, 'firmware': '.'.join(map(str, status.firmware_version))})
        entry.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        message = DiagnosticArray(status=[entry])
        message.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics.publish(message)

    def stop_motors(self) -> None:
        """Best effort on shutdown: e-stop right away instead of waiting for the firmware's 300 ms timeout."""
        for _ in range(3):
            self.send(protocol.drive_command_payload(True, (0.0, 0.0, 0.0, 0.0)))
            time.sleep(0.01)
        self.link.close()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DriveNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # One Ctrl-C on `ros2 launch` delivers SIGINT twice: from the terminal, then forwarded by launch. The second
        # one must not cut the final e-stop short (or end the node with a traceback).
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.stop_motors()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
