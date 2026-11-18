"""Joystick -> joint velocity commands and the software e-stop latch, without ROS so it can be unit-tested.

Each joint reads one axis (a stick, or the D-pad, which joy_node reports as two axes) and/or a pair of buttons, so the
mapping can follow whatever layout this PS5 shows in `ros2 topic echo /joy`.

E-stop rules (the rover's):
  - starts latched: nothing moves until the operator releases it;
  - the e-stop button latches it at any time;
  - the release button clears it only while every joint input is centred (no lurch on release);
  - losing the joystick (no /joy for joy_timeout_s) latches it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class JointInput:
    axis: int = -1  # Joy axes index (-1: none)
    invert: bool = False
    positive_button: int = -1  # held = +1 (-1: none)
    negative_button: int = -1  # held = -1 (-1: none)


# PS5 DualSense through joy_node on Linux (kernel driver): axes 0/1 left stick, 3/4 right stick, 6/7 D-pad
# (left/up = +1); buttons 0 Cross, 1 Circle, 2 Triangle, 3 Square, 4 L1, 5 R1, 9 Options.
DEFAULT_INPUTS = (
    JointInput(axis=1),  # shoulder pitch: left stick up = up
    JointInput(axis=4),  # elbow pitch: right stick up = up
    JointInput(axis=7),  # wrist pitch: D-pad up = up
    JointInput(axis=6, invert=True),  # wrist roll (auger): D-pad right = positive (drilling direction)
)


@dataclass
class TeleopConfig:
    inputs: tuple[JointInput, ...] = DEFAULT_INPUTS  # shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll
    deadzone: float = 0.08
    max_velocity: tuple[float, ...] = field(default=(0.5, 0.5, 0.5, 3.0))  # rad/s at full input, per joint
    slow_button: int = 4  # L1 held = slow mode; -1 disables
    slow_scale: float = 0.35
    estop_button: int = 1  # Circle
    release_button: int = 9  # Options
    joy_timeout_s: float = 0.5


def apply_deadzone(value: float, deadzone: float) -> float:
    if not math.isfinite(value) or abs(value) <= deadzone:
        return 0.0
    magnitude = min(1.0, (abs(value) - deadzone) / (1.0 - deadzone))
    return math.copysign(magnitude, value)


class TeleopLogic:
    def __init__(self, config: TeleopConfig) -> None:
        if len(config.inputs) != 4 or len(config.max_velocity) != 4:
            raise ValueError('need an input and a max velocity for each of the four joints')
        self.config = config
        self.estopped = True
        self.reason = 'startup: press release with every joint input centred'
        self._axes: list[float] = []
        self._buttons: list[int] = []
        self._last_joy: float | None = None
        self._release_held = False

    def _axis(self, index: int) -> float:
        if index < 0 or index >= len(self._axes):
            return 0.0
        return apply_deadzone(float(self._axes[index]), self.config.deadzone)

    def _button(self, index: int) -> bool:
        return 0 <= index < len(self._buttons) and bool(self._buttons[index])

    def joint_input(self, joint: int) -> float:
        """-1..1 for one joint: its axis (inverted if configured) plus its buttons, clamped."""
        mapping = self.config.inputs[joint]
        value = self._axis(mapping.axis)
        if mapping.invert:
            value = -value
        value += float(self._button(mapping.positive_button)) - float(self._button(mapping.negative_button))
        return max(-1.0, min(1.0, value))

    def inputs_centred(self) -> bool:
        return all(self.joint_input(joint) == 0.0 for joint in range(4))

    def on_joy(self, axes: list[float], buttons: list[int], now: float) -> list[str]:
        """Feed one Joy message. Returns human-readable events for the log."""
        self._axes, self._buttons, self._last_joy = list(axes), list(buttons), now
        events = []
        if self._button(self.config.estop_button):
            if not self.estopped:
                events.append('E-STOP latched (button)')
            self.estopped, self.reason = True, 'e-stop button'
        release = self._button(self.config.release_button)
        if release and not self._release_held and self.estopped and not self._button(self.config.estop_button):
            if self.inputs_centred():
                self.estopped, self.reason = False, ''
                events.append('E-stop released: arm enabled')
            else:
                events.append('Release ignored: centre the sticks and the D-pad first')
        self._release_held = release
        return events

    def joystick_fresh(self, now: float) -> bool:
        return self._last_joy is not None and now - self._last_joy <= self.config.joy_timeout_s

    def tick(self, now: float) -> tuple[tuple[float, float, float, float], list[str]]:
        """Returns (joint velocities in rad/s, events). Call at the publish rate."""
        events = []
        if not self.joystick_fresh(now) and not self.estopped:
            events.append('E-STOP latched: joystick lost')
            self.estopped, self.reason = True, 'joystick lost'
        if self.estopped:
            return (0.0, 0.0, 0.0, 0.0), events
        scale = self.config.slow_scale if self._button(self.config.slow_button) else 1.0
        velocities = tuple(self.joint_input(joint) * self.config.max_velocity[joint] * scale for joint in range(4))
        return velocities, events  # type: ignore[return-value]
