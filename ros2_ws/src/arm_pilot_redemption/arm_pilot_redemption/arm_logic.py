"""What the arm driver sends each cycle, without ROS so it can be unit-tested.

The arm moves only while the base station's e-stop heartbeat is fresh and says "released" and the joint command is
fresh. Anything else sends the e-stop flag (latched in the firmware until a neutral command) or zero targets.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from arm_pilot_redemption.kinematics import JOINTS, ZERO


@dataclass(frozen=True)
class Decision:
    estop: bool
    velocities: tuple[float, float, float, float]  # joint rad/s: shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll
    reason: str  # '' while enabled normally


def joint_velocities(names: list[str], velocities: list[float]) -> tuple[float, float, float, float] | None:
    """Velocities of a JointState in JOINTS order. Named joints are matched by name (unknown names ignored, missing
    joints 0); without names the message must carry exactly four velocities in JOINTS order. Non-finite values are 0.
    None if the message cannot be read that way."""
    if names:
        if len(names) != len(velocities):
            return None
        by_name = dict(zip(names, velocities))
        values = [by_name.get(joint, 0.0) for joint in JOINTS]
    elif len(velocities) == 4:
        values = list(velocities)
    else:
        return None
    return tuple(float(value) if math.isfinite(value) else 0.0 for value in values)  # type: ignore[return-value]


class ArmGate:
    def __init__(self, cmd_timeout_s: float = 0.3, estop_timeout_s: float = 0.5, require_estop_heartbeat: bool = True) -> None:
        self.cmd_timeout_s = cmd_timeout_s
        self.estop_timeout_s = estop_timeout_s
        self.require_estop_heartbeat = require_estop_heartbeat
        self._cmd: tuple[float, float, float, float] = ZERO
        self._cmd_time: float | None = None
        self._estop: bool = True
        self._estop_time: float | None = None

    def on_joint_cmd(self, velocities: tuple[float, float, float, float], now: float) -> None:
        self._cmd, self._cmd_time = velocities, now

    def on_estop(self, stopped: bool, now: float) -> None:
        self._estop, self._estop_time = stopped, now

    def decide(self, now: float) -> Decision:
        if self.require_estop_heartbeat:
            if self._estop_time is None:
                return Decision(True, ZERO, 'waiting for the base station e-stop heartbeat')
            if now - self._estop_time > self.estop_timeout_s:
                return Decision(True, ZERO, 'base station e-stop heartbeat lost')
        if self._estop_time is not None and self._estop:
            return Decision(True, ZERO, 'e-stop')
        if self._cmd_time is None or now - self._cmd_time > self.cmd_timeout_s:
            return Decision(False, ZERO, 'no fresh joint command')
        return Decision(False, self._cmd, '')
