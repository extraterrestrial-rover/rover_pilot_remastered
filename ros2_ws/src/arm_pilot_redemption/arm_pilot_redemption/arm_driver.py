"""Arm-side driver: /arm/joint_cmd + the base station's e-stop heartbeat -> ARM_COMMAND to the arm master at 50 Hz.

It owns the master ESP32's serial port (never scans other ports), keeps sending commands at a fixed rate (the master
stops every motor 300 ms after the last one), and sends the e-stop flag whenever the base station's e-stop is latched
or its heartbeat is missing. It publishes /arm/joint_states from the encoders and /diagnostics for the master and
each motor slave.
"""
from __future__ import annotations

import signal
import time

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from arm_pilot_redemption import protocol
from arm_pilot_redemption.arm_logic import ArmGate, joint_velocities
from arm_pilot_redemption.geometry_params import declare_geometry
from arm_pilot_redemption.kinematics import JOINTS, joint_velocity_to_motor_rpm, motor_revolutions_to_joint_rad, motor_rpm_to_joint_rad_s

LATEST_ONLY = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)
ZERO = (0.0, 0.0, 0.0, 0.0)


def open_port(port: str, baud: int) -> serial.Serial:
    handle = serial.Serial()
    handle.port, handle.baudrate, handle.timeout, handle.write_timeout = port, baud, 0, 0.1
    handle.dtr = handle.rts = False  # avoid pulsing the ESP32 DevKit's reset circuit when opening
    try:
        handle.exclusive = True  # POSIX: refuse a second opener such as a serial monitor
    except (AttributeError, ValueError):
        pass
    handle.open()
    return handle


def motor_problem(motor: protocol.MotorStatus) -> str:
    """'' when the motor's slave is healthy, else e.g. 'not answering on I2C' or 'stalled'."""
    reasons = []
    if motor.link_faults & ~protocol.LINK_SLAVE_FAULT:
        reasons.append(protocol.describe_faults(motor.link_faults & ~protocol.LINK_SLAVE_FAULT, protocol.LINK_FAULT_NAMES))
    if motor.slave_faults:
        reasons.append(protocol.describe_faults(motor.slave_faults, protocol.SLAVE_FAULT_NAMES))
    elif motor.link_faults & protocol.LINK_SLAVE_FAULT:
        reasons.append('in FAULT')
    return ', '.join(reasons)


def motor_problems(status: protocol.ArmStatus) -> str:
    """'' when every motor is healthy, else e.g. 'wrist_b: not answering on I2C; shoulder: stalled'."""
    return '; '.join(f'{name}: {problem}' for name, motor in zip(protocol.MOTORS, status.motors) if (problem := motor_problem(motor)))


class ArmDriver(Node):
    def __init__(self) -> None:
        super().__init__('arm_driver')
        param = self.declare_parameter
        self.port = param('port', '/dev/arm-master').value
        self.baud = param('baud', 115200).value
        rate_hz = param('rate_hz', 50.0).value
        self.gate = ArmGate(param('cmd_timeout_s', 0.3).value, param('estop_timeout_s', 0.5).value,
                            param('require_estop_heartbeat', True).value)
        self.geometry = declare_geometry(self)

        self.create_subscription(JointState, param('joint_cmd_topic', '/arm/joint_cmd').value, self.on_joint_cmd, LATEST_ONLY)
        self.create_subscription(Bool, param('estop_topic', '/arm/estop').value, self.on_estop, LATEST_ONLY)
        self.joint_states = self.create_publisher(JointState, param('joint_states_topic', '/arm/joint_states').value, 10)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.serial_port: serial.Serial | None = None
        self.next_open_attempt = 0.0
        self.port_error: str | None = None
        self.decoder = protocol.PacketDecoder()
        self.seq = 0
        self.sent = 0
        self.status: protocol.ArmStatus | None = None
        self.status_time: float | None = None
        self.info: tuple[protocol.MotorInfo, ...] | None = None
        self.last_reason: str | None = None
        self.last_state: int | None = None
        self.last_problems = ''
        self.warned: set[str] = set()
        self.rejected_cmd = False

        self.create_timer(1.0 / rate_hz, self.tick)
        self.create_timer(1.0, self.publish_diagnostics)
        limits = ', '.join(f'{joint} {limit:.2f}' for joint, limit in zip(JOINTS, self.geometry.max_joint_velocity))
        self.get_logger().info(f'arm driver: port {self.port} @ {self.baud}; joint top speeds {limits} rad/s '
                               f'(motors {", ".join(str(rpm) for rpm in self.geometry.motor_max_rpm)} rpm at full PWM)')

    def on_joint_cmd(self, msg: JointState) -> None:
        velocities = joint_velocities(list(msg.name), list(msg.velocity))
        if velocities is None:
            if not self.rejected_cmd:
                self.get_logger().warn(f'ignoring a joint command without usable velocities (expected names {JOINTS})')
                self.rejected_cmd = True
            return
        self.gate.on_joint_cmd(velocities, time.monotonic())

    def on_estop(self, msg: Bool) -> None:
        self.gate.on_estop(msg.data, time.monotonic())

    def tick(self) -> None:
        decision = self.gate.decide(time.monotonic())
        if decision.reason != self.last_reason:
            if decision.reason:
                self.get_logger().warn(f'stopped: {decision.reason}')
            else:
                self.get_logger().info('arm enabled')
            self.last_reason = decision.reason
        targets = joint_velocity_to_motor_rpm(decision.velocities, self.geometry)
        self.send(protocol.arm_command_payload(decision.estop, targets))
        self.receive()

    # ------------------------------------------------------------------ serial port
    def ensure_port(self) -> bool:
        if self.serial_port is not None:
            return True
        now = time.monotonic()
        if now < self.next_open_attempt:
            return False
        self.next_open_attempt = now + 1.0
        try:
            self.serial_port = open_port(self.port, self.baud)
        except (serial.SerialException, OSError) as error:
            if str(error) != self.port_error:
                self.get_logger().error(f'cannot open {self.port}: {error} (retrying every second)')
                self.port_error = str(error)
            return False
        self.port_error = None
        self.decoder = protocol.PacketDecoder()
        self.get_logger().info(f'serial port {self.port} open')
        return True

    def drop_port(self, error: Exception) -> None:
        self.get_logger().error(f'serial port {self.port} lost: {error}')
        try:
            if self.serial_port is not None:
                self.serial_port.close()
        except (serial.SerialException, OSError):
            pass
        self.serial_port = None
        self.status = None
        self.info = None

    def send(self, payload: bytes) -> None:
        if not self.ensure_port():
            return
        try:
            self.serial_port.write(protocol.encode_packet(protocol.TYPE_ARM_COMMAND, self.seq, payload))
            self.seq = (self.seq + 1) & 0xFF
            self.sent += 1
        except (serial.SerialException, OSError) as error:
            self.drop_port(error)

    def receive(self) -> None:
        if self.serial_port is None:
            return
        try:
            chunk = self.serial_port.read(self.serial_port.in_waiting or 0)
        except (serial.SerialException, OSError, TypeError) as error:
            self.drop_port(error)
            return
        for packet in self.decoder.feed(chunk):
            if packet.type == protocol.TYPE_ARM_STATUS:
                status = protocol.parse_arm_status(packet.payload)
                if status is not None:
                    self.on_status(status)
            elif packet.type == protocol.TYPE_ARM_INFO:
                info = protocol.parse_arm_info(packet.payload)
                if info is not None:
                    self.on_info(info)

    # ------------------------------------------------------------------ status
    def warn_once(self, key: str, text: str) -> None:
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warn(text)

    def on_status(self, status: protocol.ArmStatus) -> None:
        if status.state != self.last_state:
            text = f'arm master state: {status.state_name}'
            if status.state == protocol.FAULT:
                self.get_logger().error(f'{text}: all motors stopped. Press e-stop, fix the cause, then release')
            else:
                self.get_logger().info(text)
            self.last_state = status.state
        problems = motor_problems(status)
        if problems != self.last_problems:
            if problems:
                self.get_logger().error(f'arm motors: {problems}')
            else:
                self.get_logger().info('arm motors: all four slaves healthy')
            self.last_problems = problems
        self.status, self.status_time = status, time.monotonic()
        self.publish_joint_states(status)

    def on_info(self, info: tuple[protocol.MotorInfo, ...]) -> None:
        self.info = info
        for name, motor, expected in zip(protocol.MOTORS, info, self.geometry.motor_max_rpm):
            if motor.known and abs(motor.max_rpm - expected) > 0.05:
                self.warn_once(f'rpm-{name}', f'{name} slave max_rpm is {motor.max_rpm} but motor_max_rpm is {expected}: '
                                              'speeds will be scaled wrongly')
            if motor.known and self.status is not None and motor.firmware_version != self.status.firmware_version:
                self.warn_once(f'fw-{name}', f'{name} slave firmware {".".join(map(str, motor.firmware_version))} differs '
                                             f'from the master\'s {".".join(map(str, self.status.firmware_version))}')

    def publish_joint_states(self, status: protocol.ArmStatus) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINTS)
        message.velocity = [float(v) for v in motor_rpm_to_joint_rad_s([m.velocity_rpm for m in status.motors], self.geometry)]
        if self.info is not None and all(motor.known for motor in self.info):
            # Relative to where each slave powered up: there is no homing yet.
            revolutions = [m.position / i.counts_per_rev for m, i in zip(status.motors, self.info)]
            message.position = [float(p) for p in motor_revolutions_to_joint_rad(revolutions, self.geometry)]
        self.joint_states.publish(message)

    def publish_diagnostics(self) -> None:
        now = time.monotonic()
        status, decision = self.status, self.gate.decide(now)
        master = DiagnosticStatus(name='arm master', hardware_id=self.port)
        fresh = status is not None and self.status_time is not None and now - self.status_time <= 0.5
        if self.serial_port is None:
            master.level, master.message = DiagnosticStatus.ERROR, f'serial port not open: {self.port_error or "opening"}'
        elif not fresh:
            master.level, master.message = DiagnosticStatus.ERROR, 'no status from the arm master'
        elif status.state == protocol.FAULT:
            master.level, master.message = DiagnosticStatus.ERROR, f'FAULT: {motor_problems(status) or "cleared, e-stop to reset"}'
        elif status.state == protocol.ENABLED and not decision.reason:
            master.level, master.message = DiagnosticStatus.OK, 'enabled'
        else:
            master.level, master.message = DiagnosticStatus.WARN, decision.reason or status.state_name.lower()
        values = {'gate': decision.reason or 'enabled', 'commands_sent': self.sent}
        if status is not None:
            values.update({'master_state': status.state_name, 'slaves_ready': status.slaves_ready,
                           'command_age_ms': status.command_age_ms, 'rx_good': status.rx_good, 'rx_bad': status.rx_bad,
                           'uptime_s': round(status.uptime_ms / 1000, 1), 'reset_reason': status.reset_reason,
                           'firmware': '.'.join(map(str, status.firmware_version))})
        master.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        entries = [master]
        for index, name in enumerate(protocol.MOTORS):
            entry = DiagnosticStatus(name=f'arm motor {name}', hardware_id=f'{self.port} slave {index}')
            motor = status.motors[index] if fresh else None
            if motor is None:
                entry.level, entry.message = DiagnosticStatus.ERROR, 'no status from the arm master'
            elif motor.link_faults or motor.slave_faults:
                entry.level, entry.message = DiagnosticStatus.ERROR, motor_problem(motor)
            elif motor.slave_state == protocol.ENABLED:
                entry.level, entry.message = DiagnosticStatus.OK, 'enabled'
            else:
                entry.level, entry.message = DiagnosticStatus.WARN, motor.state_name.lower()
            values = {}
            if motor is not None:
                values = {'state': motor.state_name, 'pwm': motor.pwm, 'rpm': motor.velocity_rpm, 'position_counts': motor.position,
                          'link_faults': protocol.describe_faults(motor.link_faults, protocol.LINK_FAULT_NAMES),
                          'slave_faults': protocol.describe_faults(motor.slave_faults, protocol.SLAVE_FAULT_NAMES),
                          'i2c_errors': motor.i2c_errors}
            if self.info is not None and self.info[index].known:
                info = self.info[index]
                values.update({'max_rpm': info.max_rpm, 'counts_per_rev': info.counts_per_rev,
                               'firmware': '.'.join(map(str, info.firmware_version)), 'reset_reason': info.reset_reason})
            entry.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
            entries.append(entry)
        message = DiagnosticArray(status=entries)
        message.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics.publish(message)

    def stop_motors(self) -> None:
        """Best effort on shutdown: e-stop right away instead of waiting for the master's 300 ms timeout."""
        for _ in range(3):
            self.send(protocol.arm_command_payload(True, ZERO))
            time.sleep(0.01)
        if self.serial_port is not None:
            self.serial_port.close()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ArmDriver()
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
