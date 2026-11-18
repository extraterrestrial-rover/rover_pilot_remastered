"""Simulated arm (master ESP32 + four motor slaves) for bench tests without hardware.

It speaks the real serial protocol on a pseudo-terminal, and the master and slaves exchange real I2C frames (bytes,
CRC included) every bus cycle. FakeMaster mirrors the firmware's arm::ArmMaster (neutral interlock, 300 ms command
timeout, e-stop latch, slave link checks, latched FAULT); FakeSlave mirrors arm::JointController's safety rules with an
ideal motor (no ramp, no speed loop: it turns at exactly its target). Like an ESP32 I2C slave, each slave's answer is
the snapshot it prepared after the previous read.

    ros2 run arm_pilot_redemption fake_arm --link /tmp/arm-master
    ros2 launch arm_pilot_redemption arm.launch.py port:=/tmp/arm-master
"""
from __future__ import annotations

import argparse
import os
import select
import sys
import threading
import time
from pathlib import Path

from arm_pilot_redemption import protocol
from arm_pilot_redemption.protocol import BOOT, ENABLED, ESTOP, FAULT, NO_COMMAND, WAIT_NEUTRAL, WAIT_SLAVES

FIRMWARE_VERSION = (1, 1, 0)  # firmware_arm/include/config.h FIRMWARE_VERSION_*: the version this simulator mirrors
MAX_RPM, COUNTS_PER_REV = 117.0, 1425.1  # config.h placeholder motors
BUS_PERIOD_S, CONTROL_PERIOD_S, STATUS_PERIOD_S, INFO_PERIOD_S = 0.02, 0.01, 0.05, 1.0


def elapsed(now_ms: int, then_ms: int) -> int:
    return max(0, now_ms - then_ms)


class FakeMaster:
    """Python twin of arm::ArmMaster."""

    def __init__(self, command_timeout_ms: int = 300, slave_timeout_ms: int = 100, enable_grace_ms: int = 200) -> None:
        self.command_timeout_ms, self.slave_timeout_ms, self.enable_grace_ms = command_timeout_ms, slave_timeout_ms, enable_grace_ms
        self.state = BOOT
        self.targets = [0.0] * 4
        self.last_command_ms: int | None = None
        self.enabled_since_ms = 0
        self.status: list[protocol.SlaveStatus | None] = [None] * 4
        self.last_fresh_ms = [0] * 4
        self.link = [0] * 4
        self.latched_link = [0] * 4
        self.latched_slave = [0] * 4
        self.errors = [0] * 4

    def _stop(self, state: int) -> None:
        self.state, self.targets = state, [0.0] * 4

    def _latch(self) -> None:
        for motor in range(4):
            self.latched_link[motor] |= self.link[motor]
            if self.status[motor] is not None:
                self.latched_slave[motor] |= self.status[motor].faults

    def evaluate_links(self, now_ms: int) -> None:
        settled = self.state == ENABLED and elapsed(now_ms, self.enabled_since_ms) > self.enable_grace_ms
        for motor, status in enumerate(self.status):
            faults = 0
            if status is None or elapsed(now_ms, self.last_fresh_ms[motor]) > self.slave_timeout_ms:
                faults |= protocol.LINK_LOST
            if status is not None:
                if status.joint != motor:
                    faults |= protocol.LINK_WRONG_JOINT
                if status.faults or status.state == FAULT:
                    faults |= protocol.LINK_SLAVE_FAULT  # reported on its own: the slave's fault is why it does not follow
                elif settled and status.state != ENABLED:
                    faults |= protocol.LINK_NOT_ENABLED
            self.link[motor] = faults

    def slaves_ready(self) -> bool:
        return all(status is not None and not self.link[motor] and status.state == ENABLED for motor, status in enumerate(self.status))

    def link_faults(self, motor: int) -> int:
        return self.link[motor] | self.latched_link[motor]

    def slave_faults(self, motor: int) -> int:
        status = self.status[motor]
        return (status.faults if status is not None else 0) | self.latched_slave[motor]

    def on_command(self, estop: bool, targets: tuple[float, ...], now_ms: int) -> None:
        self.last_command_ms = now_ms
        if estop:
            self._stop(ESTOP)
            self.latched_link, self.latched_slave = [0] * 4, [0] * 4
            return
        if self.state == FAULT:
            return
        if self.state != ENABLED:
            if any(target != 0.0 for target in targets):
                self._stop(WAIT_NEUTRAL)
                return
            self.evaluate_links(now_ms)
            if any(self.link):
                self._stop(WAIT_SLAVES)
                return
            self.state, self.enabled_since_ms = ENABLED, now_ms
        self.targets = list(targets)

    def on_slave_status(self, motor: int, status: protocol.SlaveStatus, now_ms: int) -> None:
        if self.status[motor] is None or status.counter != self.status[motor].counter:
            self.last_fresh_ms[motor] = now_ms
        self.status[motor] = status

    def on_slave_error(self, motor: int) -> None:
        self.errors[motor] = min(0xFFFF, self.errors[motor] + 1)

    def update(self, now_ms: int) -> None:
        if self.state in (ENABLED, WAIT_NEUTRAL, WAIT_SLAVES) and self.last_command_ms is not None \
                and now_ms - self.last_command_ms > self.command_timeout_ms:
            self._stop(NO_COMMAND)
        self.evaluate_links(now_ms)
        if self.state == ENABLED and any(self.link):
            self._stop(FAULT)
        if self.state == FAULT:
            self._latch()

    def slave_command(self, motor: int) -> tuple[bool, float]:
        if self.state != ENABLED:
            return True, 0.0
        return False, self.targets[motor] if self.slaves_ready() else 0.0


class FakeSlave:
    """Python twin of arm::JointController's safety rules, with an ideal motor."""

    def __init__(self, joint: int, max_rpm: float = MAX_RPM, counts_per_rev: float = COUNTS_PER_REV,
                 command_timeout_ms: int = 100) -> None:
        self.joint, self.max_rpm, self.counts_per_rev, self.command_timeout_ms = joint, max_rpm, counts_per_rev, command_timeout_ms
        self.online = True  # False: does not answer on I2C (unplugged, unpowered, crashed)
        self.reported_joint = joint  # set to another joint to simulate a slave flashed with the wrong environment
        self.reboot(0)

    def reboot(self, now_ms: int) -> None:
        self.state, self.faults, self.target, self.position = BOOT, 0, 0.0, 0.0
        self.counter, self.rx_bad, self.last_command_ms = 0, 0, None
        self.booted_ms = now_ms
        self.reply = self.status_frame(now_ms)  # Wire.slaveWrite() at boot

    def on_command(self, estop: bool, target: float, now_ms: int) -> None:
        self.last_command_ms = now_ms
        if estop:
            self.state, self.faults, self.target = ESTOP, 0, 0.0
            return
        if self.state == FAULT:
            return
        if self.state != ENABLED:
            if target != 0.0:
                self.state, self.target = WAIT_NEUTRAL, 0.0
                return
            self.state = ENABLED
        self.target = target

    def inject_fault(self, bits: int = protocol.SLAVE_FAULT_STALL) -> None:
        self.state, self.faults, self.target = FAULT, self.faults | bits, 0.0

    def update(self, now_ms: int, dt_s: float) -> None:
        if self.state in (ENABLED, WAIT_NEUTRAL) and self.last_command_ms is not None \
                and now_ms - self.last_command_ms > self.command_timeout_ms:
            self.state, self.target = NO_COMMAND, 0.0
        self.position += self.velocity() / 60.0 * self.counts_per_rev * dt_s
        self.counter = (self.counter + 1) & 0xFF

    def velocity(self) -> float:
        return max(-self.max_rpm, min(self.max_rpm, self.target)) if self.state == ENABLED else 0.0

    def pwm(self) -> int:
        return max(-255, min(255, round(self.target / self.max_rpm * 255))) if self.state == ENABLED else 0

    def status_frame(self, now_ms: int) -> bytes:
        age = 0xFFFF if self.last_command_ms is None else min(0xFFFF, now_ms - self.last_command_ms)
        return protocol.slave_status_frame(protocol.SlaveStatus(
            self.counter, self.state, self.faults, self.reported_joint, self.pwm(), self.velocity(), round(self.position),
            self.rx_bad, age, self.max_rpm, self.counts_per_rev, FIRMWARE_VERSION, 1))

    def exchange(self, frame: bytes, now_ms: int) -> bytes | None:
        """One master write + read. Returns the bytes the master reads, or None if the slave does not answer."""
        if not self.online:
            return None
        command = protocol.parse_slave_command_frame(frame)
        if command is None:
            self.rx_bad += 1
        else:
            self.on_command(command.estop, command.target_rpm, now_ms)
        reply, self.reply = self.reply, self.status_frame(now_ms)  # the next read gets a snapshot taken now
        return reply


class FakeArm:
    """Runs a FakeMaster and four FakeSlaves against a file descriptor (the master side of a pseudo-terminal)."""

    def __init__(self, fd: int, verbose: bool = False) -> None:
        self.fd = fd
        self.master = FakeMaster()
        self.slaves = [FakeSlave(joint) for joint in range(4)]
        self.decoder = protocol.PacketDecoder()
        self.verbose = verbose
        self.stop_event = threading.Event()
        self.started = time.monotonic()
        self.seq = 0
        self.bus_seq = 0
        self.booting_until = 0.0
        self.lock = threading.Lock()

    def now_ms(self) -> int:
        return int((time.monotonic() - self.started) * 1000)

    def read(self) -> tuple[int, tuple[int, ...]]:
        """(master state, applied PWM per motor) for tests."""
        with self.lock:
            return self.master.state, tuple(slave.pwm() for slave in self.slaves)

    def reset_master(self, boot_s: float = 0.3) -> None:
        """The master's EN button: it is silent while it boots (no bus cycles, serial input lost), so the slaves stop
        on their own command timeout; then it starts again in BOOT."""
        with self.lock:
            self.master = FakeMaster()
            self.booting_until = time.monotonic() + boot_s

    def bus_cycle(self, now_ms: int) -> None:
        for motor, slave in enumerate(self.slaves):
            estop, target = self.master.slave_command(motor)
            frame = protocol.slave_command_frame(protocol.SlaveCommand(self.bus_seq, estop, target))
            reply = slave.exchange(frame, now_ms)
            status = protocol.parse_slave_status_frame(reply) if reply is not None else None
            if status is None:
                self.master.on_slave_error(motor)
            else:
                self.master.on_slave_status(motor, status, now_ms)
        self.bus_seq = (self.bus_seq + 1) & 0xFF

    def run(self) -> None:
        next_bus = next_control = next_status = next_info = time.monotonic()
        last_report = None
        while not self.stop_event.is_set():
            ready, _, _ = select.select([self.fd], [], [], 0.002)
            if ready:
                try:
                    chunk = os.read(self.fd, 4096)
                except OSError:
                    chunk = b''
                if time.monotonic() < self.booting_until:
                    chunk = b''  # a booting ESP32 drops what arrives on its UART
                for packet in self.decoder.feed(chunk):
                    command = protocol.parse_arm_command(packet.payload) if packet.type == protocol.TYPE_ARM_COMMAND else None
                    if command is not None:
                        with self.lock:
                            self.master.on_command(command[0], command[1], self.now_ms())
            now = time.monotonic()
            booting = now < self.booting_until
            with self.lock:
                now_ms = self.now_ms()
                if not booting:
                    self.master.update(now_ms)
                if now >= next_control:
                    next_control = now + CONTROL_PERIOD_S
                    for slave in self.slaves:
                        slave.update(now_ms, CONTROL_PERIOD_S)
                if now >= next_bus and not booting:
                    next_bus = now + BUS_PERIOD_S
                    self.bus_cycle(now_ms)
                report = (self.master.state, tuple(slave.pwm() for slave in self.slaves))
            if booting:
                continue
            if now >= next_status:
                next_status = now + STATUS_PERIOD_S
                self._send(protocol.TYPE_ARM_STATUS, self.status_payload())
            if now >= next_info:
                next_info = now + INFO_PERIOD_S
                self._send(protocol.TYPE_ARM_INFO, self.info_payload())
            if self.verbose and report != last_report:
                pwm = '  '.join(f'{name} {value:4}' for name, value in zip(protocol.MOTORS, report[1]))
                print(f'{protocol.state_name(report[0]):<12} PWM {pwm}', flush=True)
                last_report = report

    def status_payload(self) -> bytes:
        with self.lock:
            master, now_ms = self.master, self.now_ms()
            motors = []
            for motor in range(4):
                status = master.status[motor]
                motors.append(protocol.MotorStatus(
                    status.state if status else BOOT, master.slave_faults(motor), master.link_faults(motor),
                    status.pwm if status else 0, status.velocity_rpm if status else 0.0, status.position if status else 0,
                    master.errors[motor]))
            age = 0xFFFF if master.last_command_ms is None else min(0xFFFF, now_ms - master.last_command_ms)
            flags = protocol.STATUS_FLAG_SLAVES_READY if master.slaves_ready() else 0
            return protocol.arm_status_payload(protocol.ArmStatus(master.state, flags, age, self.decoder.good, self.decoder.bad,
                                                                  now_ms, 1, FIRMWARE_VERSION, tuple(motors)))

    def info_payload(self) -> bytes:
        with self.lock:
            motors = [protocol.MotorInfo(s.max_rpm, s.counts_per_rev, s.firmware_version, s.reset_reason) if s is not None
                      else protocol.MotorInfo(0.0, 0.0, (0, 0, 0), 0) for s in self.master.status]
        return protocol.arm_info_payload(motors)

    def _send(self, packet_type: int, payload: bytes) -> None:
        wire = protocol.encode_packet(packet_type, self.seq, payload)
        self.seq = (self.seq + 1) & 0xFF
        try:
            os.write(self.fd, wire)
        except OSError:
            pass  # nobody has the port open yet


def open_pty() -> tuple[int, str]:
    """Create a raw pseudo-terminal; returns (master fd for the fake, slave path for the arm driver)."""
    import pty  # noqa: PLC0415 - POSIX only
    import tty  # noqa: PLC0415

    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)
    os.set_blocking(master_fd, False)
    return master_fd, os.ttyname(slave_fd)


def start_in_background() -> tuple[FakeArm, str]:
    fd, path = open_pty()
    fake = FakeArm(fd)
    threading.Thread(target=fake.run, name='fake-arm', daemon=True).start()
    return fake, path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Simulated arm master + four motor slaves on a pseudo-terminal.')
    parser.add_argument('--link', default='/tmp/arm-master', help='symlink to create for the pseudo-terminal')
    args = parser.parse_args(argv)
    fd, path = open_pty()
    link = Path(args.link)
    if link.is_symlink() or link.exists():
        link.unlink()
    link.symlink_to(path)
    print(f'fake arm master on {args.link} -> {path} (Ctrl-C to stop)', flush=True)
    fake = FakeArm(fd, verbose=True)
    try:
        fake.run()
    except KeyboardInterrupt:
        pass
    finally:
        link.unlink(missing_ok=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
