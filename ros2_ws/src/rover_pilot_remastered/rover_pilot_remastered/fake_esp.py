"""Simulated drive ESP32 for bench tests without hardware.

It speaks the real protocol over UDP, like the Ethernet firmware (LINK.md), and mirrors the firmware's safety state
machine (firmware/lib/drive_core): neutral command required after boot / timeout / e-stop, 300 ms command timeout,
open-loop rpm -> PWM, DRIVE_STATUS at 10 Hz. It accepts datagrams only from the PC's address, as the firmware does.

    ros2 run rover_pilot_remastered fake_esp                        # 127.0.0.1:47700, status to 127.0.0.1:47701
    ros2 launch rover_pilot_remastered rover.launch.py master:=127.0.0.1

Simulator wraps it for tests and tools/rehearsal.py: the same calls on every branch, whatever the link is (LINK.md).
"""
from __future__ import annotations

import argparse
import select
import socket
import sys
import threading
import time

from rover_pilot_remastered import protocol

BOOT, NO_COMMAND, ESTOP, WAIT_NEUTRAL, DRIVING = 0, 1, 2, 3, 4
FIRMWARE_VERSION = (3, 0, 0)  # firmware/include/config.h FIRMWARE_VERSION_*: the version this simulator mirrors


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
    """Runs a FakeController on a UDP socket: commands from `pc` only, DRIVE_STATUS to `pc` at 10 Hz."""

    def __init__(self, sock: socket.socket, pc: tuple[str, int], max_wheel_rpm: float = 94.7, verbose: bool = False) -> None:
        self.sock, self.pc = sock, pc
        self.controller = FakeController(max_wheel_rpm)
        self.decoder = protocol.PacketDecoder()
        self.foreign = self.tails = 0
        self.connected = True  # False: the Ethernet cable is out (the ESP32 keeps running on its USB power)
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
            ready, _, _ = select.select([self.sock], [], [], 0.005)
            if ready:
                self._receive()
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

    def _receive(self) -> None:
        while True:
            try:
                data, (host, _port) = self.sock.recvfrom(2048)
            except (BlockingIOError, InterruptedError, OSError):
                return
            if not self.connected:
                continue  # cable out: nothing arrives
            if host != self.pc[0]:
                self.foreign += 1
                continue
            end = data.rfind(b'\x00') + 1  # complete packets end at the datagram's last 0x00
            if end < len(data):
                self.tails += 1
            for packet in self.decoder.feed(data[:end]):
                command = protocol.parse_drive_command(packet.payload) if packet.type == protocol.TYPE_DRIVE_COMMAND else None
                if command is not None:
                    with self.lock:
                        self.controller.on_command(command[0], command[1], self.now_ms())

    def _send_status(self) -> None:
        if not self.connected:
            return
        with self.lock:
            age = 0xFFFF if self.controller.last_command_ms is None else min(0xFFFF, self.now_ms() - self.controller.last_command_ms)
            bad = min(0xFFFF, self.decoder.bad + self.foreign + self.tails)
            status = protocol.DriveStatus(self.controller.state, age, self.controller.pwm(), self.decoder.good, bad,
                                          self.now_ms(), 1, self.controller.max_wheel_rpm, FIRMWARE_VERSION)
        wire = protocol.encode_packet(protocol.TYPE_DRIVE_STATUS, self.seq, protocol.drive_status_payload(status))
        self.seq = (self.seq + 1) & 0xFF
        try:
            self.sock.sendto(wire, self.pc)
        except OSError:
            pass  # nobody listening is fine for UDP; anything else is the PC's problem, as on the real link


def open_socket(host: str = '127.0.0.1', port: int = 0) -> socket.socket:
    """The ESP32's UDP socket; port 0 picks a free one (see getsockname())."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    sock.bind((host, port))
    return sock


def start_in_background(max_wheel_rpm: float = 94.7, pc: tuple[str, int] = ('127.0.0.1', 47701),
                        port: int = 0) -> tuple[FakeEsp, tuple[str, int]]:
    """A FakeEsp on 127.0.0.1 (a free port unless given), sending status to `pc`; returns it and its address."""
    sock = open_socket('127.0.0.1', port)
    fake = FakeEsp(sock, pc, max_wheel_rpm)
    threading.Thread(target=fake.run, name='fake-esp', daemon=True).start()
    return fake, sock.getsockname()


class Simulator:
    """The simulated ESP32 on this branch's link (here UDP on 127.0.0.1:47700, status to port 47701, the addresses of
    LINK.md moved to the loopback), for tests and tools/rehearsal.py.

    The other branch has a Simulator with the same methods, so the rehearsal does not depend on the link.
    """

    def __init__(self, host: str = '127.0.0.1', port: int = 47700, pc: tuple[str, int] = ('127.0.0.1', 47701)) -> None:
        self.host, self.port, self.pc = host, port, pc
        self.fake: FakeEsp | None = None
        self._start()

    @property
    def launch_arguments(self) -> list[str]:
        """What rover.launch.py needs to reach this ESP32."""
        return [f'master:={self.host}']

    @property
    def tool_arguments(self) -> list[str]:
        """What tools/wheel_test.py and tools/profile.py need to reach it."""
        return ['--master', self.host, '--port', str(self.port), '--local-port', str(self.pc[1])]

    def state(self) -> tuple[int, tuple[int, int, int, int]] | None:
        """(state, PWM per wheel) as the ESP32 has it now; it keeps running while the cable is out."""
        if self.fake is None:
            return None
        with self.fake.lock:
            return self.fake.controller.state, self.fake.controller.pwm()

    def reset(self) -> None:
        """The EN button: the ESP32 reboots, the cable stays."""
        if self.fake is not None:
            with self.fake.lock:
                self.fake.controller = FakeController(self.fake.controller.max_wheel_rpm)

    def unplug(self) -> None:
        """The Ethernet cable is pulled: nothing gets through; the ESP32 stays on (USB power) and times out."""
        if self.fake is not None:
            self.fake.connected = False

    def plug(self) -> None:
        """The cable is back; the ESP32 still waits for a neutral command after its timeout."""
        if self.fake is not None:
            self.fake.connected = True

    def close(self) -> None:
        if self.fake is None:
            return
        self.fake.stop_event.set()
        self._thread.join(2)
        self.fake.sock.close()
        self.fake = None

    def _start(self) -> None:
        self.fake = FakeEsp(open_socket(self.host, self.port), self.pc)
        self._thread = threading.Thread(target=self.fake.run, name='fake-esp', daemon=True)
        self._thread.start()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Simulated drive ESP32 on UDP.')
    parser.add_argument('--host', default='127.0.0.1', help="the simulated ESP32's address")
    parser.add_argument('--port', type=int, default=47700, help="the simulated ESP32's UDP port")
    parser.add_argument('--pc-host', default='127.0.0.1', help='the only address it takes commands from, and sends status to')
    parser.add_argument('--pc-port', type=int, default=47701)
    parser.add_argument('--max-wheel-rpm', type=float, default=94.7)
    args = parser.parse_args(argv)
    try:
        sock = open_socket(args.host, args.port)
    except OSError as error:
        print(f'cannot bind {args.host}:{args.port}: {error}', file=sys.stderr)
        return 1
    print(f'fake drive ESP32 on udp {args.host}:{args.port}, status to {args.pc_host}:{args.pc_port} (Ctrl-C to stop)', flush=True)
    fake = FakeEsp(sock, (args.pc_host, args.pc_port), args.max_wheel_rpm, verbose=True)
    try:
        fake.run()
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
