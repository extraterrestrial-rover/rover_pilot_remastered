#!/usr/bin/env python3
"""Bench check of the Quectel L89HA GPS, without ROS (AUTONOMY.md, "GPS").

    python3 tools/gps_check.py [port] [--baud 9600] [--seconds 300] [--raw] [--send 'PAIR050,1000' ...]

Prints the receiver's state once a second: fix or why not, satellites used / in view per system with the strongest
signal, HDOP, and the position. Leave the rover standing under open sky for a few minutes: at the end it prints how far
the fixes wandered (the radius holding half of them, and 95 %), which is how well the GPS can place goals at that spot.
--send sends a command (the $ and the checksum are added) and prints the receiver's answer. Default port
/dev/rover-gps; stop the rover's launch first (it holds the port).
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws' / 'src' / 'rover_autonomy'))

import serial  # noqa: E402

from rover_autonomy import geo, nmea  # noqa: E402
from rover_autonomy.gps_logic import GpsState  # noqa: E402


def spread(fixes: list[tuple[float, float]]) -> str:
    if len(fixes) < 10:
        return f'{len(fixes)} fixes: too few to say how far they wander'
    latitude = sum(f[0] for f in fixes) / len(fixes)
    longitude = sum(f[1] for f in fixes) / len(fixes)
    distances = sorted(math.hypot(*geo.enu(lat, lon, (latitude, longitude))[:2]) for lat, lon in fixes)
    half, most = distances[len(distances) // 2], distances[min(len(distances) - 1, int(0.95 * len(distances)))]
    return (f'{len(fixes)} fixes around {geo.format_position(latitude, longitude)}: half within {half:.2f} m, '
            f'95 % within {most:.2f} m, the farthest {distances[-1]:.2f} m')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('port', nargs='?', default='/dev/rover-gps')
    parser.add_argument('--baud', type=int, default=9600)
    parser.add_argument('--seconds', type=float, default=0.0, help='stop after this long (0: Ctrl-C)')
    parser.add_argument('--raw', action='store_true', help='print every sentence as it comes')
    parser.add_argument('--send', action='append', default=[], help="a command such as 'PAIR050,1000' (repeatable)")
    args = parser.parse_args()
    handle = serial.Serial()
    handle.port, handle.baudrate, handle.timeout = args.port, args.baud, 0.1
    handle.dtr = handle.rts = False  # if this is the drive ESP32 by mistake, do not reset it
    try:
        handle.open()
    except (serial.SerialException, OSError) as error:
        print(f'cannot open {args.port}: {error}')
        return 1
    state, fixes, buffer = GpsState(), [], b''
    started = last = time.monotonic()
    for command in args.send:
        handle.write(nmea.sentence(command).encode('ascii'))
        print(f'sent {nmea.sentence(command).strip()}')
    reported = 0
    try:
        while not args.seconds or time.monotonic() - started < args.seconds:
            buffer += handle.read(4096)
            *lines, buffer = buffer.split(b'\n')
            now = time.monotonic()
            for line in lines:
                if args.raw:
                    print(line.decode('ascii', errors='replace').strip())
                fix = state.feed(line, now)
                if fix is not None:
                    fixes.append((fix.latitude, fix.longitude))
            for command, result in state.acks[reported:]:
                print(f'the GPS answered PAIR{command:03d}: {nmea.ACK_RESULTS.get(result, result)}')
            reported = len(state.acks)
            if now - last >= 1.0 and not args.raw:
                last = now
                problem = state.problem(now)
                fix = state.last_fix
                where = (f'{geo.format_position(fix.latitude, fix.longitude)}, HDOP {fix.hdop:g}, about '
                         f'{fix.horizontal_sigma:.1f} m' if fix and not problem else problem)
                print(f'{now - started:6.0f} s  {state.summary()}  |  {where}', flush=True)
            if not lines:
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        handle.close()
    print(spread(fixes))
    if not state.sentences:
        print(f'nothing readable from {args.port} at {args.baud} baud: the wrong port or baud rate?')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
