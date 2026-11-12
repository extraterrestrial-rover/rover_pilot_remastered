"""Which velocity command reaches drive_node: the sticks or Nav2. No ROS, so it is unit-tested.

The base station's AUTO heartbeat (/rover/auto) decides:
  - MANUAL: the sticks' /cmd_vel is passed through unchanged, as when drive_node listens to /cmd_vel directly;
  - AUTO: Nav2's command, after a short run of zeros whenever AUTO (re)starts, because the ESP32 drives again after
    any stop only once it has seen a neutral command; then each command is corrected for the motors' deadband (an
    open-loop wheel turns only above some PWM), per side, so Nav2's slow commands still move this skid-steer rover;
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


MAX_DEADBAND_SHARE = 0.8  # a deadband above this share of max_wheel_rpm is a unit mistake, not a measurement


def checked_deadband(straight_rpm: float, spin_rpm: float, max_wheel_rpm: float) -> tuple[float, float, str]:
    """The deadband settings if they are usable, else (0.0, 0.0) and why. A deadband is a wheel target in rpm between
    0 and MAX_DEADBAND_SHARE of max_wheel_rpm: above that it is a unit mistake (a percentage or a PWM value typed as
    rpm), and the correction would turn every command into nearly full speed."""
    for name, value in (('min_wheel_rpm_straight', straight_rpm), ('min_wheel_rpm_spin', spin_rpm)):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            return 0.0, 0.0, f'{name} is {value!r}, not a number'
        if not 0.0 <= value <= MAX_DEADBAND_SHARE * max_wheel_rpm:
            return 0.0, 0.0, (f'{name} is {value:g} rpm: it must be between 0 and {MAX_DEADBAND_SHARE * max_wheel_rpm:.1f} '
                              f'({MAX_DEADBAND_SHARE:.0%} of max_wheel_rpm {max_wheel_rpm:g}); it is a wheel target in rpm, '
                              'wheel_test percent x max_wheel_rpm / 100')
    return float(straight_rpm), float(spin_rpm), ''


def deadband_corrected(linear_x: float, angular_z: float, geometry: DriveGeometry, config: MuxConfig) -> tuple[float, float]:
    """Nav2's command with each side's wheel target lifted over the motors' deadband.

    An open-loop wheel turns only above some target d, and from there about linearly up to full speed. So each side's
    target t becomes d + (max - d) * |t| / max, with its sign: the side then really turns at about t. Full speed stays
    full speed, and a side that should stand still stays at zero. d depends on the manoeuvre: turning on the spot
    scrubs the tyres and needs more than rolling straight, so it is blended between the two measured values by how
    much of the command is turning. The result is given back as a twist, which drive_node turns into exactly these
    wheel targets.
    """
    left, right, _, _ = twist_to_wheel_rpm(linear_x, angular_z, geometry)
    threshold = max(config.stop_below_rpm, 0.0)
    if max(abs(left), abs(right)) <= threshold:
        return 0.0, 0.0  # noise, or exactly zero
    half_track = geometry.track_width_m * geometry.turn_track_factor / 2.0
    turning = abs(angular_z) * half_track
    spin_share = turning / (abs(linear_x) + turning)  # 0 straight ahead, 1 on the spot; the sum is above zero here
    deadband = config.min_wheel_rpm_straight + (config.min_wheel_rpm_spin - config.min_wheel_rpm_straight) * spin_share
    top = geometry.max_wheel_rpm
    if not 0.0 < deadband < top:
        return linear_x, angular_z  # no correction (0), or unusable settings: pass Nav2's command as it is

    def lifted(target: float) -> float:
        if abs(target) <= threshold:
            return 0.0
        return math.copysign(deadband + (top - deadband) * abs(target) / top, target)

    left, right = lifted(left), lifted(right)
    metres_per_rpm = math.pi * geometry.wheel_diameter_m / 60.0
    return (left + right) / 2.0 * metres_per_rpm, (right - left) / (2.0 * half_track) * metres_per_rpm


class DriveMux:
    def __init__(self, config: MuxConfig, geometry: DriveGeometry) -> None:
        problem = checked_deadband(config.min_wheel_rpm_straight, config.min_wheel_rpm_spin, geometry.max_wheel_rpm)[2]
        if problem:
            raise ValueError(problem)  # drive_mux checks first and runs without the correction instead
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
