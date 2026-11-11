"""When a navigation goal may run, without ROS so it is unit-tested.

Goals come from RViz at the base station (/goal_pose). Rules:
  - a goal is taken only in AUTO, with the e-stop released and the base station heard; a new goal replaces the old;
  - manual (the AUTO button or a stick) or the e-stop while the goal runs cancels it for good;
  - the base station going silent (radio, a crashed laptop) pauses it: Nav2's goal is cancelled, so it neither runs
    its recovery moves nor gives up while the rover stands still, but the goal is kept. It is sent again as soon as
    the base station is heard in AUTO with the e-stop released: at once after a radio drop, or once the operator
    releases the e-stop and presses AUTO after a base-station restart (which starts e-stopped, in manual). An e-stop
    press heard while the goal is paused, or a new goal, drops it;
  - reaching the goal, or Nav2 giving up on its own, ends it.
The heartbeats are /rover/auto and /rover/estop from base_teleop; both must be fresh.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

SEND, CANCEL = 'send', 'cancel'


@dataclass(frozen=True)
class Action:
    kind: str  # SEND or CANCEL
    goal: Any = None


class GoalLogic:
    def __init__(self, link_timeout_s: float = 0.5) -> None:
        self.link_timeout_s = link_timeout_s
        self.goal: Any = None  # the goal being pursued or paused
        self.running = False  # Nav2 is working on it
        self._auto, self._auto_time = False, None
        self._estop, self._estop_time = True, None
        self._estop_seen: bool | None = None  # the e-stop state at the last update with the base station heard
        self.events: list[str] = []

    def on_auto(self, auto: bool, now: float) -> None:
        self._auto, self._auto_time = auto, now

    def on_estop(self, estopped: bool, now: float) -> None:
        self._estop, self._estop_time = estopped, now

    def link_ok(self, now: float) -> bool:
        return all(stamp is not None and now - stamp <= self.link_timeout_s for stamp in (self._auto_time, self._estop_time))

    def on_goal(self, goal: Any, now: float) -> list[Action]:
        if not self.link_ok(now):
            self.events.append('goal ignored: the base station is not heard')
            return []
        if self._estop:
            self.events.append('goal ignored: the e-stop is latched')
            return []
        if not self._auto:
            self.events.append('goal ignored: not in AUTO (press the AUTO button first)')
            return []
        self.events.append('goal accepted' if self.goal is None else 'new goal replaces the current one')
        self.goal, self.running = goal, True
        return [Action(SEND, goal)]

    def update(self, now: float) -> list[Action]:
        """Call often (10 Hz) and after every heartbeat."""
        if not self.link_ok(now):
            self._estop_seen = None  # what is heard first afterwards is where the base station is, not a button press
            if self.goal is not None and self.running:
                self.running = False
                self.events.append('base station lost: goal paused (the rover stops)')
                return [Action(CANCEL)]
            return []
        pressed = self._estop and self._estop_seen is False
        first = self._estop_seen is None
        self._estop_seen = self._estop
        if self.goal is None:
            return []
        if self.running:
            if self._estop or not self._auto:
                self.events.append(f"goal cancelled: {'e-stop' if self._estop else 'manual driving'}")
                self.goal, self.running = None, False
                return [Action(CANCEL)]
            return []
        if self._auto and not self._estop:
            self.running = True
            self.events.append('base station in AUTO: resuming the goal')
            return [Action(SEND, self.goal)]
        if pressed:
            self.events.append('paused goal dropped: e-stop')
            self.goal = None
        elif first:
            self.events.append('base station back, not in AUTO: goal kept (release the e-stop and press AUTO to '
                               'resume it; an e-stop press drops it)')
        return []

    def on_result(self, succeeded: bool, cancelled: bool) -> None:
        """Nav2's result for the goal we sent last. Our own cancels (pause, manual) end with cancelled=True."""
        if cancelled or self.goal is None:
            return
        self.events.append('goal reached' if succeeded else 'Nav2 gave up on the goal (see its log); send a new one')
        self.goal, self.running = None, False

    def on_rejected(self, why: str) -> None:
        """The goal we sent last never ran (Nav2 not active, or it refused it)."""
        if self.goal is None:
            return
        self.events.append(f'goal dropped: {why}; send it again')
        self.goal, self.running = None, False
