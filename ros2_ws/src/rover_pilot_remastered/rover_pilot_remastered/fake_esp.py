"""Simulated drive ESP32 for bench tests without hardware.

It speaks the real serial protocol on a pseudo-terminal and mirrors the firmware's safety state machine
(firmware/lib/drive_core): neutral command required after boot / timeout / e-stop, 300 ms command timeout,
open-loop rpm -> PWM, DRIVE_STATUS at 10 Hz.

    ros2 run rover_pilot_remastered fake_esp --link /tmp/rover-drive
    ros2 launch rover_pilot_remastered rover.launch.py port:=/tmp/rover-drive

Simulator wraps it for tests and tools/rehearsal.py: the same calls on every branch, whatever the link is (LINK.md).
"""
from __future__ import annotations

import argparse
import os
import select
import sys
import threading
import time
from pathlib import Path

from rover_pilot_remastered import protocol

BOOT, NO_COMMAND, ESTOP, WAIT_NEUTRAL, DRIVING = 0, 1, 2, 3, 4
FIRMWARE_VERSION = (2, 0, 2)  # firmware/include/config.h FIRMWARE_VERSION_*: the version this simulator mirrors


class FakeController:
    """Python twin of drive::DriveController (no slew limit)."""

    def __init__(self, max_wheel_rpm: float = 94.7, command_timeout_ms: int = 300) -> None:
        self.max_wheel_rpm = max_wheel_rpm
        self.command_timeout_ms = command_timeout_ms
        self.state = BOOT
        self.targets = [0.0, 0.0, 0.0, 0.0]
        self.last_command_ms: int | None = None

    def on_command(self, estop: bool, targets: tuple[float, ...], now_ms: int) -> None:
        self.last_command_ms = now_ms
        if estop:
            self.state, self.targets = ESTOP, [0.0] * 4
            return
        neutral = all(round(target * 10) == 0 for target in targets)
        if self.state != DRIVING:
            if not neutral:
                self.state, self.targets = WAIT_NEUTRAL, [0.0] * 4
                return
            self.state = DRIVING
        self.targets = list(targets)

    def update(self, now_ms: int) -> None:
        if self.state in (DRIVING, WAIT_NEUTRAL) and self.last_command_ms is not None \
                and now_ms - self.last_command_ms > self.command_timeout_ms:
            self.state, self.targets = NO_COMMAND, [0.0] * 4

    def pwm(self) -> tuple[int, int, int, int]:
        if self.state != DRIVING:
            return (0, 0, 0, 0)
        return tuple(max(-255, min(255, round(target / self.max_wheel_rpm * 255))) for target in self.targets)  # type: ignore[return-value]


class FakeEsp:
    """Runs a FakeController against a file descriptor (the master side of a pseudo-terminal)."""

    def __init__(self, fd: int, max_wheel_rpm: float = 94.7, verbose: bool = False) -> None:
        self.fd = fd
        self.controller = FakeController(max_wheel_rpm)
        self.decoder = protocol.PacketDecoder()
        self.verbose = verbose
        self.stop_event = threading.Event()
        self.started = time.monotonic()
        self.seq = 0
        self.lock = threading.Lock()

    def now_ms(self) -> int:
        return int((time.monotonic() - self.started) * 1000)

    def run(self) -> None:
        next_status = 0.0
        last_report = None
        while not self.stop_event.is_set():
            ready, _, _ = select.select([self.fd], [], [], 0.005)
            if ready:
                try:
                    chunk = os.read(self.fd, 4096)
                except OSError:
                    chunk = b''
                for packet in self.decoder.feed(chunk):
                    command = protocol.parse_drive_command(packet.payload) if packet.type == protocol.TYPE_DRIVE_COMMAND else None
                    if command is not None:
                        with self.lock:
                            self.controller.on_command(command[0], command[1], self.now_ms())
            with self.lock:
                self.controller.update(self.now_ms())
                pwm = self.controller.pwm()
                state = self.controller.state
            now = time.monotonic()
            if now >= next_status:
                next_status = now + 0.1
                self._send_status()
            report = (state, pwm)
            if self.verbose and report != last_report:
                print(f'{protocol.STATE_NAMES[state]:<12} PWM FL {pwm[0]:4} FR {pwm[1]:4} RL {pwm[2]:4} RR {pwm[3]:4}', flush=True)
                last_report = report

    def _send_status(self) -> None:
        with self.lock:
            age = 0xFFFF if self.controller.last_command_ms is None else min(0xFFFF, self.now_ms() - self.controller.last_command_ms)
            status = protocol.DriveStatus(self.controller.state, age, self.controller.pwm(), self.decoder.good, self.decoder.bad,
                                          self.now_ms(), 1, self.controller.max_wheel_rpm, FIRMWARE_VERSION)
        wire = protocol.encode_packet(protocol.TYPE_DRIVE_STATUS, self.seq, protocol.drive_status_payload(status))
        self.seq = (self.seq + 1) & 0xFF
        try:
            os.write(self.fd, wire)
        except OSError:
            pass  # nobody has the port open yet


def open_pty() -> tuple[int, str]:
    """Create a raw pseudo-terminal; returns (master fd for the fake, slave path for the drive node)."""
    import pty  # noqa: PLC0415 - POSIX only
    import tty  # noqa: PLC0415

    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)
    os.set_blocking(master_fd, False)
    return master_fd, os.ttyname(slave_fd)


def start_in_background(max_wheel_rpm: float = 94.7) -> tuple[FakeEsp, str]:
    fd, path = open_pty()
    fake = FakeEsp(fd, max_wheel_rpm)
    threading.Thread(target=fake.run, name='fake-esp', daemon=True).start()
    return fake, path


class Simulator:
    """The simulated ESP32 on this branch's link (here a pseudo-terminal at `path`), for tests and tools/rehearsal.py.

    The other branch has a Simulator with the same methods, so the rehearsal does not depend on the link.
    """

    def __init__(self, path: str = '/tmp/rover-drive-sim') -> None:
        self.path = path
        self.fake: FakeEsp | None = None
        self._plug()

    @property
    def launch_arguments(self) -> list[str]:
        """What rover.launch.py needs to reach this ESP32."""
        return [f'port:={self.path}']

    @property
    def tool_arguments(self) -> list[str]:
        """What tools/wheel_test.py and tools/profile.py need to reach it."""
        return [self.path]

    def state(self) -> tuple[int, tuple[int, int, int, int]] | None:
        """(state, PWM per wheel) as the ESP32 has it now; None while it is unplugged (over USB: off)."""
        if self.fake is None:
            return None
        with self.fake.lock:
            return self.fake.controller.state, self.fake.controller.pwm()

    def reset(self) -> None:
        """The EN button: the ESP32 reboots, the link stays."""
        if self.fake is not None:
            with self.fake.lock:
                self.fake.controller = FakeController(self.fake.controller.max_wheel_rpm)

    def unplug(self) -> None:
        """The link cable is pulled. Over USB that is also the ESP32's power, so it stops."""
        if self.fake is None:
            return
        self.fake.stop_event.set()
        self._thread.join(2)
        os.close(self._fd)  # the drive node's port now fails, as when the cable is pulled
        Path(self.path).unlink(missing_ok=True)
        self.fake = None

    def plug(self) -> None:
        """The cable is back. Over USB the ESP32 boots again (motors off until a neutral command)."""
        if self.fake is None:
            self._plug()

    def close(self) -> None:
        self.unplug()

    def _plug(self) -> None:
        self._fd, real_path = open_pty()
        self.fake = FakeEsp(self._fd)
        self._thread = threading.Thread(target=self.fake.run, name='fake-esp', daemon=True)
        self._thread.start()
        link = Path(self.path)
        if link.is_symlink() or link.exists():
            link.unlink()
        link.symlink_to(real_path)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Simulated drive ESP32 on a pseudo-terminal.')
    parser.add_argument('--link', default='/tmp/rover-drive', help='symlink to create for the pseudo-terminal')
    parser.add_argument('--max-wheel-rpm', type=float, default=94.7)
    args = parser.parse_args(argv)
    fd, path = open_pty()
    link = Path(args.link)
    if link.is_symlink() or link.exists():
        link.unlink()
    link.symlink_to(path)
    print(f'fake drive ESP32 on {args.link} -> {path} (Ctrl-C to stop)', flush=True)
    fake = FakeEsp(fd, args.max_wheel_rpm, verbose=True)
    try:
        fake.run()
    except KeyboardInterrupt:
        pass
    finally:
        link.unlink(missing_ok=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
