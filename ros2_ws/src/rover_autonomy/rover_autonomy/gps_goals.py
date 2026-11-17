"""Base station: send GPS waypoints to the rover (GPS mode), and follow what it does with them (AUTONOMY.md, "GPS").

    ros2 run rover_autonomy gps_goals "28.750123, 77.117456" "28°45'02.1\\"N 77°07'05.3\\"E"   # drive them in turn
    ros2 run rover_autonomy gps_goals --file waypoints.txt --mode pause --pause 15            # stop 15 s at each
    ros2 run rover_autonomy gps_goals --file waypoints.txt --mode wait                       # stop until 'next'
    ros2 run rover_autonomy gps_goals next          # wait mode: on to the next waypoint
    ros2 run rover_autonomy gps_goals cancel        # stop the list (and the goal under way)

Each position in any usual form (geo.py): decimal degrees, degrees minutes seconds, degrees decimal minutes, with
N/S/E/W or signs. A file has one position per line; '#' starts a comment. The positions are checked and printed before
anything is sent. Afterwards the command prints the rover's events (with a beep at each waypoint) until the list is
done, or Ctrl-C (which stops only this command, not the rover). The rover takes the list only in AUTO, and only once
its GPS is lined up with the map (after its first 15 m or so).
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from rover_autonomy import geo
from rover_autonomy.waypoint_logic import MODES, encode_command, encode_list

DONE_WORDS = ('the last one, done', 'GPS waypoints stopped', 'GPS waypoints ignored', 'GPS command ignored')


def read_positions(texts: list[str], file: str | None) -> list[tuple[float, float]]:
    """Positions from the command line and/or a file, in order; ValueError names the one that is wrong."""
    lines = list(texts)
    if file:
        for number, line in enumerate(Path(file).read_text().splitlines(), start=1):
            line = line.split('#')[0].strip()
            if line:
                lines.append(line)
    positions = []
    for index, text in enumerate(lines, start=1):
        try:
            positions.append(geo.parse_position(text))
        except ValueError as error:
            raise ValueError(f'waypoint {index}: {error}') from None
    if not positions:
        raise ValueError('no positions given (on the command line or with --file)')
    return positions


def main(args: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('positions', nargs='*', help="positions, or 'next' or 'cancel'")
    parser.add_argument('--file', help='a file with one position per line')
    parser.add_argument('--mode', choices=MODES, default='continue')
    parser.add_argument('--pause', type=float, default=10.0, help='pause mode: seconds at each waypoint')
    parser.add_argument('--no-follow', action='store_true', help='send, then exit at once')
    options = parser.parse_args(rclpy.utilities.remove_ros_args(args if args is not None else sys.argv)[1:])
    if options.positions in (['next'], ['cancel']) and not options.file:
        payload, positions = encode_command(options.positions[0]), []
    else:
        try:
            positions = read_positions(options.positions, options.file)
        except (ValueError, OSError) as error:
            print(f'not sent: {error}')
            return 1
        payload = encode_list(positions, options.mode, options.pause)

    rclpy.init(args=args)
    node = rclpy.create_node('gps_goals')
    publisher = node.create_publisher(String, '/rover/gps_command', 10)
    events: list[str] = []
    node.create_subscription(String, '/rover/autonomy_events', lambda m: events.append(m.data), 50)
    rover: list[NavSatFix] = []
    node.create_subscription(NavSatFix, '/gps/fix', lambda m: rover.append(m), qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 5.0  # wait for the rover's goal_manager to be heard
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            print('not sent: the rover is not heard (no goal_manager on /rover/gps_command)')
            return 1
        end = time.monotonic() + 1.0
        while time.monotonic() < end:  # a moment for the rover's GPS fix, for the distances below
            rclpy.spin_once(node, timeout_sec=0.1)
        for index, (latitude, longitude) in enumerate(positions, start=1):
            distance = ''
            if rover:
                distance = f', {geo.distance((rover[-1].latitude, rover[-1].longitude), (latitude, longitude)):.0f} m from the rover'
            print(f'  {index}: {geo.format_position(latitude, longitude)}{distance}')
        publisher.publish(String(data=payload))
        print('sent' + (f' ({len(positions)} waypoints, mode {options.mode})' if positions else f': {options.positions[0]}'))
        if options.no_follow or not positions:
            end = time.monotonic() + 3.0
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
                while events:
                    print(f'rover: {events.pop(0)}')
            return 0
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            while events:
                event = events.pop(0)
                print(('\a' if 'reached' in event else '') + f'rover: {event}', flush=True)
                if any(word in event for word in DONE_WORDS) or event.startswith('goal cancelled'):
                    return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
