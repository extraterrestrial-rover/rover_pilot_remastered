"""Rover-side: the Quectel L89HA GPS (USB serial, /dev/rover-gps) -> /gps/fix for the GPS mode (AUTONOMY.md, "GPS").

Reads the receiver's NMEA, passes on only real fixes with enough satellites (gps_logic), with their uncertainty, and
reports the receiver's state on /diagnostics ('rover gps'): satellites per system, HDOP, why fixes are withheld. It
can send setup commands at start ($PAIR..., settings setup_commands) and logs the receiver's answer to each.
The port is opened without touching DTR and RTS, so that if it is ever the drive ESP32's by mistake, the ESP32 is not
reset; and it is said so when no NMEA arrives.
"""
from __future__ import annotations

import signal
import time

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus

from rover_autonomy import nmea
from rover_autonomy.gps_logic import GpsConfig, GpsState

NO_NMEA_WARNING_S = 5.0  # nothing that reads as NMEA this long after opening: probably not the GPS
SERVICES = {'GPS': NavSatStatus.SERVICE_GPS, 'GLONASS': NavSatStatus.SERVICE_GLONASS,
            'BeiDou': NavSatStatus.SERVICE_COMPASS, 'Galileo': NavSatStatus.SERVICE_GALILEO}


class GpsNode(Node):
    def __init__(self) -> None:
        super().__init__('gps_node')
        param = self.declare_parameter
        self.port = param('gps_port', '/dev/rover-gps').value
        self.baud = param('gps_baud', 9600).value
        self.frame_id = param('gps_frame', 'gps_link').value
        defaults = GpsConfig()
        self.state = GpsState(GpsConfig(uere_m=param('gps_uere_m', defaults.uere_m).value,
                                        min_satellites=param('gps_min_satellites', defaults.min_satellites).value,
                                        max_hdop=param('gps_max_hdop', defaults.max_hdop).value))
        # Commands sent at every start, separated by ';', e.g. 'PAIR410,1' (the $ and the checksum are added).
        self.setup_commands = [command.strip() for command in param('gps_setup', '').value.split(';') if command.strip()]
        self.fix = self.create_publisher(NavSatFix, param('gps_fix_topic', '/gps/fix').value, qos_profile_sensor_data)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.port_handle: serial.Serial | None = None
        self.buffer = b''
        self.opened_at = 0.0
        self.warned_no_nmea = False
        self.reported_acks = 0
        self.last_problem: str | None = None
        self.last_attempt = 0.0
        self.create_timer(0.05, self.poll)
        self.create_timer(1.0, self.report)
        self.get_logger().info(f'GPS: {self.port} at {self.baud} baud -> /gps/fix')

    # ------------------------------------------------------------------ serial
    def open(self) -> None:
        self.last_attempt = time.monotonic()
        handle = serial.Serial()
        handle.port, handle.baudrate, handle.timeout, handle.write_timeout = self.port, self.baud, 0, 0.5
        handle.dtr = handle.rts = False
        try:
            handle.exclusive = True
        except (AttributeError, ValueError):
            pass
        try:
            handle.open()
        except (serial.SerialException, OSError) as error:
            if self.last_problem != f'cannot open {self.port}':
                self.get_logger().warn(f'cannot open {self.port}: {error} (retrying every second)')
                self.last_problem = f'cannot open {self.port}'
            return
        self.port_handle, self.buffer, self.opened_at, self.warned_no_nmea = handle, b'', time.monotonic(), False
        self.get_logger().info(f'{self.port} open')
        for command in self.setup_commands:
            handle.write(nmea.sentence(command).encode('ascii'))
            self.get_logger().info(f'sent {nmea.sentence(command).strip()}')

    def close(self, why: str) -> None:
        if self.port_handle is not None:
            self.get_logger().warn(f'{self.port}: {why}; reopening')
            try:
                self.port_handle.close()
            except (serial.SerialException, OSError):
                pass
        self.port_handle = None

    def poll(self) -> None:
        now = time.monotonic()
        if self.port_handle is None:
            if now - self.last_attempt >= 1.0:
                self.open()
            return
        try:
            data = self.port_handle.read(4096)
        except (serial.SerialException, OSError) as error:
            self.close(str(error))
            return
        self.buffer += data
        *lines, self.buffer = self.buffer.split(b'\n')
        if len(self.buffer) > 1024:  # no line end in 1 kB: not NMEA (or the wrong baud rate)
            self.state.bad_lines += 1
            self.buffer = b''
        for line in lines:
            fix = self.state.feed(line, now)
            if fix is not None:
                self.publish(fix)
        if not self.warned_no_nmea and self.state.sentences == 0 and now - self.opened_at > NO_NMEA_WARNING_S:
            self.warned_no_nmea = True
            self.get_logger().error(f'no NMEA from {self.port} in {NO_NMEA_WARNING_S:g} s: is it the GPS, at '
                                    f'{self.baud} baud? (the drive ESP32 is /dev/rover-drive; AUTONOMY.md, "GPS")')
        for command, result in self.state.acks[self.reported_acks:]:
            text = nmea.ACK_RESULTS.get(result, f'result {result}')
            (self.get_logger().info if result == 0 else self.get_logger().warn)(f'the GPS answered PAIR{command:03d}: {text}')
        self.reported_acks = len(self.state.acks)

    def publish(self, fix) -> None:
        message = NavSatFix()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.status.status = NavSatStatus.STATUS_SBAS_FIX if fix.quality == 2 else NavSatStatus.STATUS_FIX
        service = 0
        for system, count in self.state.used.items():
            if count:
                service |= SERVICES.get(system, 0)
        message.status.service = service or NavSatStatus.SERVICE_GPS
        message.latitude, message.longitude, message.altitude = fix.latitude, fix.longitude, fix.altitude
        horizontal, vertical = fix.horizontal_sigma ** 2, fix.vertical_sigma ** 2
        message.position_covariance = [horizontal, 0.0, 0.0, 0.0, horizontal, 0.0, 0.0, 0.0, vertical]
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.fix.publish(message)

    # ------------------------------------------------------------------ status
    def report(self) -> None:
        now = time.monotonic()
        problem = (f'{self.port} not open' if self.port_handle is None else self.state.problem(now))
        if problem != self.last_problem:
            if problem:
                self.get_logger().warn(f'GPS fixes withheld: {problem}')
            else:
                fix = self.state.last_fix
                self.get_logger().info(f'GPS fix: {fix.satellites} satellites, HDOP {fix.hdop:g}, about '
                                       f'{fix.horizontal_sigma:.1f} m ({nmea.FIX_QUALITY.get(fix.quality, fix.quality)})')
            self.last_problem = problem
        status = DiagnosticStatus(name='rover gps', hardware_id=self.port)
        status.level = DiagnosticStatus.OK if not problem else (
            DiagnosticStatus.ERROR if self.port_handle is None or 'nothing' in problem else DiagnosticStatus.WARN)
        status.message = problem or 'fix'
        fix = self.state.last_fix
        values = {'satellites': self.state.summary(), 'antenna': self.state.antenna,
                  'sentences': str(self.state.sentences), 'bad_lines': str(self.state.bad_lines)}
        if fix is not None:
            values.update(hdop=f'{fix.hdop:g}', sigma_m=f'{fix.horizontal_sigma:.2f}',
                          quality=nmea.FIX_QUALITY.get(fix.quality, str(fix.quality)))
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        array = DiagnosticArray(status=[status])
        array.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics.publish(array)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GpsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node.port_handle is not None:
            node.port_handle.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
