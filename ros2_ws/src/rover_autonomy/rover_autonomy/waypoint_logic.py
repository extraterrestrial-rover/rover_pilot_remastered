"""GPS waypoint lists: which goal goes to Nav2 next, and what happens at each waypoint. No ROS, so it is unit-tested.

A list comes from the base station (gps_goals) as JSON on /rover/gps_command, with one of three modes:
  continue: drive the whole list, then hold still at the last waypoint;
  pause:    stop at each waypoint for pause_s, report it, then go on by itself;
  wait:     stop at each waypoint until the base station sends 'next'.
A waypoint further than leg_m from the rover is reached in legs: goals at most leg_m away on the straight line towards
it, each sent before the rover reaches the previous one (so it does not stop on the way). Nav2 plans each leg around
obstacles in its window around the rover. A leg's and a waypoint's heading is the direction of travel: no turn on the
spot when the rover gets there. goal_manager applies the usual rules (AUTO, e-stop, base station) to every goal sent.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass

MODES = ('continue', 'pause', 'wait')
MAX_WAYPOINTS = 50


def encode_list(waypoints: list[tuple[float, float]], mode: str = 'continue', pause_s: float = 10.0) -> str:
    return json.dumps({'waypoints': [[lat, lon] for lat, lon in waypoints], 'mode': mode, 'pause_s': pause_s})


def encode_command(command: str) -> str:
    return json.dumps({'command': command})


@dataclass(frozen=True)
class Command:
    kind: str  # 'list', 'next' or 'cancel'
    waypoints: tuple[tuple[float, float], ...] = ()
    mode: str = 'continue'
    pause_s: float = 10.0


def decode(text: str) -> Command:
    """A command from /rover/gps_command, checked: ValueError says what is wrong with it."""
    try:
        data = json.loads(text)
    except json.JSONDecodeError as error:
        raise ValueError(f'not JSON: {error}') from None
    if not isinstance(data, dict):
        raise ValueError('expected a JSON object')
    if 'command' in data:
        if data['command'] not in ('next', 'cancel'):
            raise ValueError(f"unknown command {data['command']!r} (next or cancel)")
        return Command(data['command'])
    points = data.get('waypoints')
    if not isinstance(points, list) or not points:
        raise ValueError('no waypoints')
    if len(points) > MAX_WAYPOINTS:
        raise ValueError(f'{len(points)} waypoints: at most {MAX_WAYPOINTS}')
    checked = []
    for point in points:
        if (not isinstance(point, list) or len(point) != 2
                or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in point)
                or not (-90 <= point[0] <= 90 and -180 <= point[1] <= 180)):
            raise ValueError(f'{point!r} is not [latitude, longitude] in degrees')
        checked.append((float(point[0]), float(point[1])))
    mode = data.get('mode', 'continue')
    if mode not in MODES:
        raise ValueError(f"mode {mode!r}: use {', '.join(MODES)}")
    pause = data.get('pause_s', 10.0)
    if isinstance(pause, bool) or not isinstance(pause, (int, float)) or not 0 <= pause <= 600:
        raise ValueError(f'pause_s {pause!r}: 0 to 600 s')
    return Command('list', tuple(checked), mode, float(pause))


@dataclass(frozen=True)
class Goal:
    x: float  # map frame
    y: float
    yaw: float
    final: bool  # the waypoint itself (False: a leg on the way to it)


class Waypoints:
    def __init__(self, leg_m: float = 40.0, early_m: float = 10.0) -> None:
        self.leg_m, self.early_m = leg_m, early_m  # legs of at most leg_m; the next one sent early_m before arriving
        self.points: tuple[tuple[float, float], ...] = ()
        self.mode, self.pause_s = 'continue', 10.0
        self.index = 0  # the waypoint being driven to (or waited at)
        self.goal: Goal | None = None  # the goal Nav2 has now
        self.holding_until: float | None = None  # pause mode: when to go on
        self.waiting = False  # wait mode: until 'next'
        self.events: list[str] = []

    @property
    def active(self) -> bool:
        return bool(self.points)

    def label(self, index: int | None = None) -> str:
        return f'waypoint {(self.index if index is None else index) + 1}/{len(self.points)}'

    def start(self, command: Command) -> None:
        self.points, self.mode, self.pause_s = command.waypoints, command.mode, command.pause_s
        self.index, self.goal, self.holding_until, self.waiting = 0, None, None, False
        self.events.append(f'{len(self.points)} GPS waypoint{"s" * (len(self.points) > 1)}, mode {self.mode}'
                           + (f' ({self.pause_s:g} s at each)' if self.mode == 'pause' else ''))

    def stop(self, why: str) -> None:
        if self.active:
            self.events.append(f'GPS waypoints stopped at {self.label()}: {why}')
        self.points, self.goal, self.holding_until, self.waiting = (), None, None, False

    def goal_towards(self, rover: tuple[float, float], target: tuple[float, float]) -> Goal:
        """The next goal to send towards `target` (map frame) from `rover`: the target itself, or a leg."""
        dx, dy = target[0] - rover[0], target[1] - rover[1]
        distance = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx) if distance > 1e-6 else (self.goal.yaw if self.goal else 0.0)
        if distance <= self.leg_m:
            return Goal(target[0], target[1], yaw, True)
        scale = self.leg_m / distance
        return Goal(rover[0] + dx * scale, rover[1] + dy * scale, yaw, False)

    def next_goal(self, rover: tuple[float, float], target: tuple[float, float] | None, now: float) -> Goal | None:
        """Call at 10 Hz with the rover's position and the current waypoint in the map frame (None: cannot be placed
        now). Returns a goal to send now, or None (nothing new)."""
        if not self.active or target is None:
            return None
        if self.waiting or (self.holding_until is not None and now < self.holding_until):
            return None
        if self.holding_until is not None:  # the pause is over
            self.holding_until = None
            self.events.append(f'pause over: on to {self.label()}')
        if self.goal is None:
            self.goal = self.goal_towards(rover, target)
            return self.goal
        if not self.goal.final and math.hypot(self.goal.x - rover[0], self.goal.y - rover[1]) < self.early_m:
            self.goal = self.goal_towards(rover, target)  # the next leg, before the rover stops at this one
            return self.goal
        return None

    def on_reached(self, now: float, error_m: float | None = None) -> None:
        """Nav2 reached the goal last sent."""
        if not self.active or self.goal is None:
            return
        if not self.goal.final:
            self.goal = None  # a leg: the next one goes out at the next call of next_goal
            return
        where = f' (about {error_m:.1f} m from its coordinates)' if error_m is not None else ''
        self.goal = None
        if self.index + 1 >= len(self.points):
            self.events.append(f'{self.label()} reached{where}: the last one, done')
            self.points = ()
            return
        reached = self.label()
        self.index += 1
        if self.mode == 'continue':
            self.events.append(f'{reached} reached{where}: on to {self.label()}')
        elif self.mode == 'pause':
            self.holding_until = now + self.pause_s
            self.events.append(f'{reached} reached{where}: pausing {self.pause_s:g} s, then on to {self.label()}')
        else:
            self.waiting = True
            self.events.append(f'{reached} reached{where}: waiting; send next for {self.label()} '
                               '(ros2 run rover_autonomy gps_goals next)')

    def on_next(self) -> bool:
        """'next' from the base station: go on when waiting (also skips a pause). False if nothing waited."""
        if not self.active or not (self.waiting or self.holding_until is not None):
            return False
        self.waiting, self.holding_until = False, None
        self.events.append(f'next: on to {self.label()}')
        return True
