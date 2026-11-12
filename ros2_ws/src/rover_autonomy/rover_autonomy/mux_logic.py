"""Which velocity command, and which e-stop, reach drive_node. No ROS, so it is unit-tested.

The base station's e-stop heartbeat (/rover/estop) is passed on to drive_node for as long as it arrives: a latched
e-stop stops the rover whatever else is going on. Its AUTO heartbeat (/rover/auto) decides who drives:
  - MANUAL (also when that heartbeat is missing, as from a base station on a branch without AUTO): the sticks'
    /cmd_vel is passed on as it arrives, unchanged, as when drive_node listens to /cmd_vel directly;
  - AUTO: Nav2's command, after a short run of zeros whenever AUTO (re)starts, because the ESP32 drives again after
    any stop only once it has seen a neutral command; then each command is corrected for the motors' deadband (an
    open-loop wheel turns only above some PWM), per side, so Nav2's slow commands still move this skid-steer rover.
    Nav2 is followed only while goal_manager reports a goal of its own (/rover/goal_active): a goal sent to Nav2 any
    other way (its RViz panel, a goal left over from a goal_manager that restarted) never moves the rover.

AUTO stops itself (zeros, and a fault that goal_manager cancels the goal on) when it cannot know where the rover is:
  - no odometry from the camera for odometry_timeout_s (unplugged, its node gone, tracking lost);
  - a motion command for no_motion_timeout_s without the odometry moving (odometry frozen, wheels stuck, or the
    deadband set too low for the motors to turn).
And when drive_node reports that the ESP32 is not driving (it was reset, or its link hiccuped), AUTO sends zeros
again first: nobody centres a stick for Nav2.

When the base station goes silent (radio, a crashed laptop):
  - in manual, or in AUTO without a goal, the e-stop is latched: the rover stops, as on main;
  - in AUTO with a goal running (goal_manager's /rover/goal_active), the rover goes on to its goal: the e-stop stays
    released for drive_node and Nav2 keeps driving. Nothing at the base station can stop the rover during that time;
    the kill switch, the odometry checks above and linkless_limit_s can. When the goal has ended, for any reason, the
    e-stop is latched until the base station is heard again. It is never released again without the base station.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from rover_pilot_remastered.kinematics import DriveGeometry, twist_to_wheel_rpm


@dataclass(frozen=True)
class MuxConfig:
    mode_timeout_s: float = 0.5   # /rover/auto older than this: the base station is lost
    command_timeout_s: float = 0.3  # Nav2's command older than this: zeros
    neutral_hold_s: float = 0.3   # zeros first whenever AUTO starts driving
    odometry_timeout_s: float = 0.5  # no odometry from the camera for this long: AUTO stops
    no_motion_timeout_s: float = 3.0  # a motion command this long without the odometry moving: AUTO stops (0 = off)
    no_motion_distance_m: float = 0.03  # what counts as moving: this far ...
    no_motion_angle_rad: float = 0.03  # ... or this much turned
    rearm_interval_s: float = 1.5  # the ESP32 not driving in AUTO: zeros again, at most this often
    goal_timeout_s: float = 0.5   # /rover/goal_active older than this: no goal is running
    linkless_limit_s: float = 0.0  # the longest AUTO may drive to its goal without the base station (0 = no limit)
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


ESP32_DRIVING = 'DRIVING'  # the ESP32 state, as drive_node reports it, in which it follows commands
DRIVE_STATE_MAX_AGE_S = 3.0  # drive_node reports once a second; an older report says nothing about now


class DriveMux:
    def __init__(self, config: MuxConfig, geometry: DriveGeometry) -> None:
        problem = checked_deadband(config.min_wheel_rpm_straight, config.min_wheel_rpm_spin, geometry.max_wheel_rpm)[2]
        if problem:
            raise ValueError(problem)  # drive_mux checks first and runs without the correction instead
        self.config, self.geometry = config, geometry
        self._auto_cmd: tuple[float, float] | None = None
        self._auto_cmd_time = 0.0
        self._auto: bool | None = None
        self._mode_time: float | None = None
        self._auto_since: float | None = None  # when the current neutral hold began
        self._last_rearm = 0.0
        self._odom: tuple[float, float, float] | None = None
        self._odom_time: float | None = None
        self._motion_since: float | None = None  # since when a motion command has gone out without the rover moving
        self._motion_ref: tuple[float, float, float] | None = None
        self._drive_state: str | None = None
        self._drive_state_time = 0.0
        self._estop: bool | None = None  # the base station's e-stop, as last heard
        self._estop_time: float | None = None
        self._goal_active = False
        self._goal_time: float | None = None
        self._linkless = False  # driving to the goal without the base station, right now
        self._linkless_since: float | None = None
        self._latched = True  # without the base station: stopped until it is heard (again)
        self.fault = ''  # why AUTO stopped itself; '' while it has not
        self.state = 'waiting for the base station'  # for the log

    # ------------------------------------------------------------------ inputs
    def on_manual(self, linear_x: float, angular_z: float, now: float) -> tuple[float, float] | None:
        """A command from the sticks: what to pass on to drive_node at once (None in AUTO). Passing it on as it
        arrives, instead of repeating the last one, leaves drive_node's own command timeout exactly as on main."""
        return None if self.in_auto(now) else (linear_x, angular_z)

    def on_auto_command(self, linear_x: float, angular_z: float, now: float) -> None:
        self._auto_cmd, self._auto_cmd_time = (linear_x, angular_z), now

    def on_mode(self, auto: bool, now: float) -> None:
        self._auto, self._mode_time = auto, now

    def on_estop(self, estopped: bool, now: float) -> bool:
        """The base station's e-stop heartbeat. Returns what to pass on to drive_node at once: always its value."""
        self._estop, self._estop_time = estopped, now
        return estopped

    def on_goal_active(self, active: bool, now: float) -> None:
        """goal_manager's heartbeat: a goal is running."""
        self._goal_active, self._goal_time = active, now

    def on_odometry(self, x: float, y: float, yaw: float, now: float) -> None:
        self._odom, self._odom_time = (x, y, yaw), now
        if self._motion_since is None:
            return
        if self._motion_ref is None or self._moved(self._motion_ref, self._odom):
            self._motion_ref, self._motion_since = self._odom, now

    def on_drive_state(self, esp32_state: str, now: float) -> None:
        """The ESP32's state from drive_node's /diagnostics (about once a second)."""
        self._drive_state, self._drive_state_time = esp32_state, now

    # ------------------------------------------------------------------ output
    def _heard(self, stamp: float | None, now: float) -> bool:
        return stamp is not None and now - stamp <= self.config.mode_timeout_s

    def in_auto(self, now: float) -> bool:
        if self._linkless:
            return True
        return (self._heard(self._estop_time, now) and self._estop is False
                and self._heard(self._mode_time, now) and self._auto is True)

    def output(self, now: float) -> tuple[float, float] | None:
        """Call at the node's rate, then estop_output(). In AUTO: the command to send now. In manual: None, because
        the sticks' commands are passed on as they arrive (on_manual)."""
        if self._heard(self._estop_time, now):  # the base station is there: what it says goes
            self._latched = self._linkless = False
            self._linkless_since = None
            if self._estop:
                self._leave_auto()
                self.state = 'e-stop'
                return None
            if self.in_auto(now):
                return self._drive_auto(now, 'AUTO')
            self._leave_auto()
            self.state = ('manual' if self._heard(self._mode_time, now)
                          else 'manual: no AUTO heartbeat from the base station (is it on a branch with AUTO?)')
            return None
        if not self._latched and self._may_finish_the_goal(now):
            if self._linkless_since is None:
                self._linkless_since = now
            limit = self.config.linkless_limit_s
            if limit > 0 and now - self._linkless_since > limit:
                self.fault = f'the base station has been silent for more than {limit:g} s'
            else:
                self._linkless = True
                command = self._drive_auto(now, 'AUTO without the base station')
                if not self.fault:
                    return command
        # Stopped, and it stays so until the base station is heard: no goal, no AUTO, or AUTO stopped itself.
        self._latched, self._linkless = True, False
        self._auto_since = self._motion_since = None  # the fault, if any, stays for goal_manager to see
        self.state = ('waiting for the base station' if self._estop_time is None
                      else 'base station not heard: e-stop latched until it is back')
        return None

    def estop_output(self, now: float) -> bool:
        """The e-stop for drive_node, every tick: the base station's while it is heard. Without it: released only
        while AUTO goes on to its goal (output() decides that), latched otherwise."""
        if self._heard(self._estop_time, now):
            return bool(self._estop)
        return not self._linkless

    def _goal_running(self, now: float) -> bool:
        return self._goal_active and self._goal_time is not None and now - self._goal_time <= self.config.goal_timeout_s

    def _may_finish_the_goal(self, now: float) -> bool:
        """The base station fell silent while AUTO was on, the e-stop released, and a goal is still running."""
        return self._auto is True and self._estop is False and self._goal_running(now)

    def _leave_auto(self) -> None:
        self._auto_since = self._motion_since = None
        self.fault = ''

    def _drive_auto(self, now: float, label: str) -> tuple[float, float]:
        commanded = self._auto_cmd is not None and now - self._auto_cmd_time <= self.config.command_timeout_s
        fault = self._find_fault(now)
        if fault:
            self.fault = fault
        elif self.fault and not self._goal_running(now):
            self.fault = ''  # the goal that ran into it is gone (goal_manager cancelled it): ready for the next one
        if self.fault:
            self._auto_since = self._motion_since = None
            self.state = f'{label} stopped: {self.fault}'
            return 0.0, 0.0
        if self._auto_since is None or self._needs_rearm(now):
            self._auto_since = self._last_rearm = now
        if now - self._auto_since < self.config.neutral_hold_s:
            self._motion_since = None
            self.state = f'{label}: neutral first'
            return 0.0, 0.0
        if not self._goal_running(now):
            self._motion_since = None
            self.state = f'{label}: no goal from goal_manager: holding still' + (' (ignoring Nav2)' if commanded else '')
            return 0.0, 0.0
        if not commanded:
            self._motion_since = None
            self.state = f'{label}: no Nav2 command: holding still'
            return 0.0, 0.0
        self.state = f'{label}: following Nav2'
        linear_x, angular_z = self._auto_cmd
        if not (math.isfinite(linear_x) and math.isfinite(angular_z)):
            self._motion_since = None
            return 0.0, 0.0
        command = deadband_corrected(linear_x, angular_z, self.geometry, self.config)
        if command == (0.0, 0.0):
            self._motion_since = None
        elif self._motion_since is None:
            self._motion_since, self._motion_ref = now, self._odom
        return command

    def _find_fault(self, now: float) -> str:
        if self._odom_time is None or now - self._odom_time > self.config.odometry_timeout_s:
            return 'no odometry from the camera'
        timeout = self.config.no_motion_timeout_s
        if timeout > 0 and self._motion_since is not None and now - self._motion_since > timeout:
            return (f'the rover has not moved for {timeout:g} s although commanded (odometry frozen, wheels stuck, or '
                    'the deadband setting too low)')
        return ''

    def _needs_rearm(self, now: float) -> bool:
        """drive_node says the ESP32 is not driving (reset, link hiccup, its command timeout): it waits for a neutral
        command, which in manual the operator gives by centring the sticks."""
        return (self._drive_state is not None and self._drive_state != ESP32_DRIVING
                and now - self._drive_state_time <= DRIVE_STATE_MAX_AGE_S
                and now - self._last_rearm >= self.config.rearm_interval_s)

    def _moved(self, start: tuple[float, float, float], end: tuple[float, float, float]) -> bool:
        turned = abs(math.atan2(math.sin(end[2] - start[2]), math.cos(end[2] - start[2])))
        return (math.hypot(end[0] - start[0], end[1] - start[1]) > self.config.no_motion_distance_m
                or turned > self.config.no_motion_angle_rad)
