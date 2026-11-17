"""GPS waypoint lists (waypoint_logic): legs, the three modes, and the commands from the base station."""
import math

import pytest

from rover_autonomy.waypoint_logic import Command, Waypoints, decode, encode_command, encode_list


def listed(mode='continue', points=((10.0, 0.0), (10.0, 10.0)), pause_s=10.0, leg_m=40.0):
    """A list of waypoints already placed in the map frame (the tests use map coordinates as the 'coordinates')."""
    waypoints = Waypoints(leg_m=leg_m)
    waypoints.start(Command('list', tuple(points), mode, pause_s))
    return waypoints


def test_commands_are_read_and_checked():
    command = decode(encode_list([(28.75, 77.11), (28.751, 77.112)], 'pause', 5.0))
    assert command == Command('list', ((28.75, 77.11), (28.751, 77.112)), 'pause', 5.0)
    assert decode(encode_command('next')).kind == 'next' and decode(encode_command('cancel')).kind == 'cancel'
    for text, why in (('nope', 'not JSON'), ('[1]', 'JSON object'), ('{"waypoints": []}', 'no waypoints'),
                      ('{"waypoints": [[95, 0]]}', 'longitude. in degrees'),
                      ('{"waypoints": [[1, 2]], "mode": "fast"}', 'mode'),
                      ('{"waypoints": [[1, 2]], "pause_s": -1}', 'pause_s'), ('{"command": "jump"}', 'unknown command'),
                      ('{"waypoints": [[1, true]]}', 'longitude. in degrees')):
        with pytest.raises(ValueError, match=why):
            decode(text)
    with pytest.raises(ValueError, match='at most 50'):
        decode(encode_list([(0.0, 0.0)] * 51))


def test_continue_drives_the_whole_list():
    waypoints = listed()
    goal = waypoints.next_goal((0.0, 0.0), (10.0, 0.0), 0.0)
    assert (goal.x, goal.y, goal.final) == (10.0, 0.0, True) and goal.yaw == 0.0  # heading: the way it drives
    assert waypoints.next_goal((2.0, 0.0), (10.0, 0.0), 1.0) is None  # nothing new while driving
    waypoints.on_reached(5.0, error_m=0.2)
    assert 'waypoint 1/2 reached (about 0.2 m from its coordinates): on to waypoint 2/2' in waypoints.events[-1]
    goal = waypoints.next_goal((10.0, 0.0), (10.0, 10.0), 5.1)
    assert (goal.x, goal.y) == (10.0, 10.0) and goal.yaw == pytest.approx(math.pi / 2)
    waypoints.on_reached(9.0)
    assert not waypoints.active and waypoints.events[-1] == 'waypoint 2/2 reached: the last one, done'


def test_pause_waits_then_goes_on_by_itself():
    waypoints = listed('pause', pause_s=10.0)
    waypoints.next_goal((0.0, 0.0), (10.0, 0.0), 0.0)
    waypoints.on_reached(5.0)
    assert 'pausing 10 s' in waypoints.events[-1]
    assert waypoints.next_goal((10.0, 0.0), (10.0, 10.0), 14.9) is None
    assert waypoints.next_goal((10.0, 0.0), (10.0, 10.0), 15.0).final and 'pause over' in waypoints.events[-1]


def test_wait_holds_until_next():
    waypoints = listed('wait')
    assert not waypoints.on_next()  # nothing to wait for yet
    waypoints.next_goal((0.0, 0.0), (10.0, 0.0), 0.0)
    waypoints.on_reached(5.0)
    assert 'waiting' in waypoints.events[-1] and 'gps_goals next' in waypoints.events[-1]
    assert waypoints.next_goal((10.0, 0.0), (10.0, 10.0), 500.0) is None
    assert waypoints.on_next() and waypoints.next_goal((10.0, 0.0), (10.0, 10.0), 501.0).final


def test_a_far_waypoint_is_reached_in_legs_without_stopping_between_them():
    waypoints = listed(points=((100.0, 0.0),), leg_m=40.0)
    leg = waypoints.next_goal((0.0, 0.0), (100.0, 0.0), 0.0)
    assert (leg.x, leg.y, leg.final) == (40.0, 0.0, False)
    assert waypoints.next_goal((29.0, 0.0), (100.0, 0.0), 1.0) is None  # 11 m before the leg's end: not yet
    leg = waypoints.next_goal((31.0, 0.0), (100.0, 0.0), 2.0)  # 9 m before: the next leg goes out (preempting)
    assert (leg.x, leg.final) == (71.0, False)
    last = waypoints.next_goal((62.0, 0.0), (100.0, 0.0), 3.0)
    assert (last.x, last.final) == (100.0, True)
    waypoints.on_reached(9.0)
    assert not waypoints.active


def test_a_leg_reached_early_sends_the_next_one():
    waypoints = listed(points=((100.0, 0.0),))
    waypoints.next_goal((0.0, 0.0), (100.0, 0.0), 0.0)
    waypoints.on_reached(1.0)  # Nav2 finished the leg before the early switch (an obstacle made it stop there)
    assert waypoints.active and waypoints.next_goal((40.0, 0.0), (100.0, 0.0), 1.1).x == 80.0


def test_a_waypoint_that_cannot_be_placed_sends_nothing_and_stop_says_why():
    waypoints = listed()
    assert waypoints.next_goal((0.0, 0.0), None, 0.0) is None  # the GPS's alignment is not ready
    waypoints.stop('e-stop')
    assert not waypoints.active and waypoints.events[-1] == 'GPS waypoints stopped at waypoint 1/2: e-stop'
    waypoints.stop('again')
    assert waypoints.events[-1].endswith('e-stop')  # said once
