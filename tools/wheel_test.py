#!/usr/bin/env python3
"""Wheel identification test for the drive ESP32, without ROS. WHEELS OFF THE GROUND.

Spins each wheel forward on its own, then all forward, then a right turn, printing what the ESP32 reports. Use it after
flashing to check that FL/FR/RL/RR are the wheels you think they are and that "forward" really is forward.
Needs only the Python standard library. Stop the drive node first: only one program may use the PC's UDP port.

    python3 tools/wheel_test.py                             # the ESP32 at 192.168.77.2, 30 % of full speed
    python3 tools/wheel_test.py --percent 50 --interface enx001122334455
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws' / 'src' / 'rover_pilot_remastered'))
from rover_pilot_remastered import protocol  # noqa: E402
from rover_pilot_remastered.udp_link import UdpLink  # noqa: E402

ZERO = (0.0, 0.0, 0.0, 0.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--master', default='192.168.77.2', help="the ESP32's address (default: LINK.md's)")
    parser.add_argument('--port', type=int, default=47700, help="the ESP32's UDP port")
    parser.add_argument('--local-port', type=int, default=47701, help="this PC's UDP port (the ESP32 sends status to it)")
    parser.add_argument('--interface', default='', help='the PC port the ESP32 is plugged into: send only through it (Linux)')
    parser.add_argument('--percent', type=float, default=30.0, help='speed as a percentage of full PWM')
    parser.add_argument('--seconds', type=float, default=2.0, help='duration of each step')
    args = parser.parse_args()

    link = UdpLink(args.master, args.port, args.local_port, args.interface)
    try:
        link.open()
    except OSError as error:
        print(f'cannot open UDP port {args.local_port}: {error} (stop drive_node or the CAN bridge first: only one may use it)')
        return 1
    seq = 0
    status = None

    def run(label: str, estop: bool, targets: tuple, seconds: float, until=None) -> bool:
        """Send `targets` at 50 Hz for `seconds`, or until `until(status)` holds (then returns True)."""
        nonlocal seq, status
        print(f'\n>>> {label}')
        # First report 0.3 s in, so it shows this step's effect rather than the previous step's (status is 10 Hz).
        end, next_print = time.monotonic() + seconds, time.monotonic() + 0.3
        while time.monotonic() < end:
            link.send(protocol.encode_packet(protocol.TYPE_DRIVE_COMMAND, seq, protocol.drive_command_payload(estop, targets)))
            seq = (seq + 1) & 0xFF
            for packet in link.receive():
                if packet.type == protocol.TYPE_DRIVE_STATUS:
                    status = protocol.parse_drive_status(packet.payload) or status
            if status and time.monotonic() >= next_print:
                next_print = time.monotonic() + 0.5
                fl, fr, rl, rr = status.pwm
                print(f'    ESP32 {status.state_name:<12} PWM  FL {fl:4}  FR {fr:4}  RL {rl:4}  RR {rr:4}   '
                      f'(fw {".".join(map(str, status.firmware_version))}, bad packets {status.rx_bad})')
            if until is not None and status is not None and until(status):
                return True
            time.sleep(0.02)
        return until is None

    try:
        # Any non-neutral command before the ESP32 has seen neutral leaves it waiting, and its Ethernet link may still be
        # coming up after a reset (1-3 s). So send neutral until it is actually ready to drive.
        if not run('neutral until the ESP32 is ready', False, ZERO, 5.0, until=lambda s: s.state_name == 'DRIVING'):
            print(f'\nThe ESP32 did not reach DRIVING within 5 s: no answer from {args.master}:{args.port}. Check the '
                  'Ethernet cable and link LEDs, that this PC port has 192.168.77.1/24 (LINK.md), `ping 192.168.77.2`, '
                  'and that the ESP32 runs the Ethernet firmware (fw 3.x).')
            return 1
        rpm = status.max_wheel_rpm * max(0.0, min(100.0, args.percent)) / 100.0  # the firmware's own calibration
        steps = []
        for index, wheel in enumerate(protocol.WHEELS):
            targets = [0.0] * 4
            targets[index] = rpm
            steps += [(f'{wheel} forward only', False, tuple(targets), args.seconds), ('stop', False, ZERO, 0.7)]
        steps += [('all forward', False, (rpm,) * 4, args.seconds), ('stop', False, ZERO, 0.7),
                  ('turn right in place (left wheels forward, right wheels backward)', False, (rpm, -rpm, rpm, -rpm), args.seconds),
                  ('e-stop', True, ZERO, 1.0)]
        for step in steps:
            run(*step)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            link.send(protocol.encode_packet(protocol.TYPE_DRIVE_COMMAND, seq, protocol.drive_command_payload(True, ZERO)))
            seq = (seq + 1) & 0xFF
        link.close()
    print('\nDone. If a wheel was wrong, fix firmware/include/config.h (pins / forward_level); if only turning was '
          'mirrored, set swap_sides or swap the left/right pin entries.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
