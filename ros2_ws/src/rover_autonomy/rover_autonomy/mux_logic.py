"""Which velocity command reaches drive_node: the sticks or Nav2. No ROS, so it is unit-tested.

The base station's AUTO heartbeat (/rover/auto) decides:
  - MANUAL: the sticks' /cmd_vel is passed through unchanged, as when drive_node listens to /cmd_vel directly;
  - AUTO: Nav2's command, after a short run of zeros whenever AUTO (re)starts, because the ESP32 drives again after
    any stop only once it has seen a neutral command; then each command is corrected for the motors' deadband (an
    open-loop wheel turns only above some PWM), so Nav2's slow commands still move this skid-steer rover;
  - heartbeat stale (base station or radio lost): nothing, so drive_node's own e-stop heartbeat check stops the rover.
An input older than command_timeout_s is never passed on: drive_node then stops the wheels as usual.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from rover_pilot_remastered.kinematics import DriveGeometry, twist_to_wheel_rpm


@dataclass(frozen=True)
class MuxConfig:
    mode_timeout_s: float = 0.5   # /rover/auto older than this: the base station is lost
    command_timeout_s: float = 0.3  # a command older than this is not passed on (drive_node's own timeout)
    neutral_hold_s: float = 0.3   # zeros first whenever AUTO starts driving
    # The motors' deadband, as a wheel target in the firmware's rpm units (wheel_test --percent x MAX_WHEEL_RPM / 100):
    min_wheel_rpm_straight: float = 0.0  # where the rover just starts to roll straight ahead (0 = no correction)
    min_wheel_rpm_spin: float = 0.0  # where it just starts to turn in place (skid steer needs much more)
    stop_below_rpm: float = 0.5   # commands slower than this are noise: a stop


def deadband_corrected(linear_x: float, angular_z: float, geometry: DriveGeometry, config: MuxConfig) -> tuple[float, float]:
    """Map Nav2's command onto the speeds the motors really have: the fastest wheel's target f becomes
    minimum + (max - minimum) * f / max, where minimum is the deadband for this mix of rolling and turning. Full speed
    stays full speed, slow commands become slow but real motion, and direction and turning radius stay as Nav2 asked.
    """
    left, right, _, _ = twist_to_wheel_rpm(linear_x, angular_z, geometry)
    fastest = max(abs(left), abs(right))
    if fastest < config.stop_below_rpm:
        return 0.0, 0.0
    turning = abs(angular_z) * geometry.track_width_m * geometry.turn_track_factor / 2.0
    spin_share = turning / (abs(linear_x) + turning)  # 0 straight ahead, 1 in place
    minimum = config.min_wheel_rpm_straight + (config.min_wheel_rpm_spin - config.min_wheel_rpm_straight) * spin_share
    if minimum <= 0.0:
        return linear_x, angular_z
    top = geometry.max_wheel_rpm
    target = min(top, minimum + (top - minimum) * fastest / top)
    scale = target / fastest
    return linear_x * scale, angular_z * scale


class DriveMux:
    def __init__(self, config: MuxConfig, geometry: DriveGeometry) -> None:
        self.config, self.geometry = config, geometry
        self._manual: tuple[float, float] | None = None
        self._manual_time = 0.0
        self._auto_cmd: tuple[float, float] | None = None
        self._auto_cmd_time = 0.0
        self._auto: bool | None = None
        self._mode_time: float | None = None
        self._auto_since: float | None = None  # when the current AUTO stretch began (for the neutral hold)
        self.state = 'waiting for the base station'  # for the log

    def on_manual(self, linear_x: float, angular_z: float, now: float) -> None:
        self._manual, self._manual_time = (linear_x, angular_z), now

    def on_auto_command(self, linear_x: float, angular_z: float, now: float) -> None:
        self._auto_cmd, self._auto_cmd_time = (linear_x, angular_z), now

    def on_mode(self, auto: bool, now: float) -> None:
        self._auto, self._mode_time = auto, now

    def _fresh(self, command, stamp: float, now: float) -> bool:
        return command is not None and now - stamp <= self.config.command_timeout_s

    def output(self, now: float) -> tuple[float, float] | None:
        """The command to pass to drive_node now, or None to send nothing."""
        if self._mode_time is None or now - self._mode_time > self.config.mode_timeout_s:
            self._auto_since = None
            self.state = 'base station not heard: nothing passed on'
            return None
        if not self._auto:
            self._auto_since = None
            self.state = 'manual'
            return self._manual if self._fresh(self._manual, self._manual_time, now) else None
        if self._auto_since is None:
            self._auto_since = now
        if now - self._auto_since < self.config.neutral_hold_s:
            self.state = 'AUTO: neutral first'
            return 0.0, 0.0
        if not self._fresh(self._auto_cmd, self._auto_cmd_time, now):
            self.state = 'AUTO: no Nav2 command (no goal): holding still'
            return 0.0, 0.0
        self.state = 'AUTO: following Nav2'
        linear_x, angular_z = self._auto_cmd
        if not (math.isfinite(linear_x) and math.isfinite(angular_z)):
            return 0.0, 0.0
        return deadband_corrected(linear_x, angular_z, self.geometry, self.config)
