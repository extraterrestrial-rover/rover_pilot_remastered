"""When a navigation goal may run, without ROS so it is unit-tested.

Goals come from RViz at the base station (/goal_pose). Rules:
  - a goal is taken only in AUTO, with the e-stop released and the base station heard; a new goal replaces the old;
  - manual (the AUTO button or a stick) or the e-stop, whenever the base station is heard, cancels the goal;
  - the base station going silent (radio, a crashed laptop) does not end a goal that is running: the rover goes on
    to it and stops there. drive_mux keeps the e-stop released for that stretch and latches it once the goal has
    ended, until the base station is heard again. Nothing at the base station can stop the rover on that stretch:
    the kill switch can, and so do the checks below;
  - drive_mux stopping AUTO by itself (no odometry from the camera, no motion although commanded), or not running,
    cancels it;
  - reaching the goal, or Nav2 giving up on its own, ends it.
The heartbeats are /rover/estop and /rover/auto from base_teleop; "heard" means the e-stop heartbeat, as for
drive_mux and drive_node.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

SEND, CANCEL = 'send', 'cancel'
MUX_TIMEOUT_S = 1.0  # drive_mux reports its fault state at 10 Hz; silent this long, it is not running


@dataclass(frozen=True)
class Action:
    kind: str  # SEND or CANCEL
    goal: Any = None


class GoalLogic:
    def __init__(self, link_timeout_s: float = 0.5) -> None:
        self.link_timeout_s = link_timeout_s
        self.goal: Any = None  # the goal Nav2 is working on
        self._auto, self._auto_time = False, None
        self._estop, self._estop_time = True, None
        self._fault, self._fault_time = '', None  # drive_mux's report: why AUTO stopped itself ('' = it has not)
        self._lost = False  # the base station fell silent while this goal ran
        self.events: list[str] = []

    @property
    def active(self) -> bool:
        """A goal is running: goal_manager publishes this for drive_mux (/rover/goal_active)."""
        return self.goal is not None

    def on_auto(self, auto: bool, now: float) -> None:
        self._auto, self._auto_time = auto, now

    def on_estop(self, estopped: bool, now: float) -> None:
        self._estop, self._estop_time = estopped, now

    def on_fault(self, fault: str, now: float) -> None:
        self._fault, self._fault_time = fault, now

    def fault(self, now: float) -> str:
        """Why AUTO cannot drive now: drive_mux's fault, or that drive_mux itself is not heard."""
        if self._fault_time is None or now - self._fault_time > MUX_TIMEOUT_S:
            return 'drive_mux is not running'
        return self._fault

    def _heard(self, stamp: float | None, now: float) -> bool:
        return stamp is not None and now - stamp <= self.link_timeout_s

    def on_goal(self, goal: Any, now: float) -> list[Action]:
        if not self._heard(self._estop_time, now):
            self.events.append('goal ignored: the base station is not heard')
            return []
        if self._estop:
            self.events.append('goal ignored: the e-stop is latched')
            return []
        if not (self._heard(self._auto_time, now) and self._auto):
            self.events.append('goal ignored: not in AUTO (press the AUTO button first)')
            return []
        self.events.append('goal accepted' if self.goal is None else 'new goal replaces the current one')
        self.goal, self._lost = goal, False
        return [Action(SEND, goal)]

    def update(self, now: float) -> list[Action]:
        """Call often (10 Hz)."""
        if self.goal is None:
            return []
        fault = self.fault(now)
        if fault:
            # AUTO stopped itself (no odometry, no motion): Nav2 must not go on trying, least of all with its spins.
            return self._cancel(fault)
        if self._heard(self._estop_time, now):
            if self._estop:
                return self._cancel('e-stop')
            if not (self._heard(self._auto_time, now) and self._auto):
                return self._cancel('manual driving')
            if self._lost:
                self._lost = False
                self.events.append('base station back: the goal goes on')
        elif not self._lost:
            self._lost = True
            self.events.append('base station lost: the rover goes on to its goal and stops there (only the kill '
                               'switch stops it now)')
        return []

    def _cancel(self, why: str) -> list[Action]:
        self.events.append(f'goal cancelled: {why}')
        self.goal, self._lost = None, False
        return [Action(CANCEL)]

    def on_result(self, succeeded: bool) -> None:
        """Nav2's result for the goal we sent last (not for one we cancelled ourselves)."""
        if self.goal is None:
            return
        event = 'goal reached' if succeeded else 'Nav2 gave up on the goal (see its log); send a new one'
        if self._lost:
            event += '; the base station is still not heard: the e-stop is latched until it is back'
        self.events.append(event)
        self.goal, self._lost = None, False

    def on_rejected(self, why: str) -> None:
        """The goal we sent last never ran (Nav2 not active, or it refused it), or was cancelled by someone else."""
        if self.goal is None:
            return
        self.events.append(f'goal dropped: {why}; send it again')
        self.goal, self._lost = None, False
