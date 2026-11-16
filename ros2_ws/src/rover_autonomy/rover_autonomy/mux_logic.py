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
    deadband set too low for the motors to turn). Before that it tries: a rover that stands although commanded gets
    its deadband raised step by step (the stall kick), since ground differs from where the deadband was measured.
    The extra is kept while the rover needs it and taken back when it moves faster than commanded;
  - no obstacle data from the camera for obstacle_timeout_s (its point cloud, or the detector that splits it into
    ground and obstacles, has stopped): Nav2 would go on with the obstacles it last saw;
  - Nav2 itself no longer knows: one of its costmaps places the rover with a transform older than
    nav_pose_max_age_s, or is not heard at all. Each costmap has its own copy of the transforms, and a copy can
    stop (tf2 0.25.23 deadlocks there): Nav2 then steers from the last position it knew, and takes in no new
    obstacles, while the camera's odometry above is fine.
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
    obstacle_timeout_s: float = 1.0  # no obstacle cloud from the camera for this long: AUTO stops (0 = not checked)
    nav_pose_max_age_s: float = 1.0  # a Nav2 costmap placing the rover with an older transform: AUTO stops (0 = off)
    nav_pose_timeout_s: float = 3.0  # a Nav2 costmap not heard for this long: the same
    # The motors' deadband, as a wheel target in the firmware's rpm units (wheel_test --percent x MAX_WHEEL_RPM / 100):
    min_wheel_rpm_straight: float = 0.0  # where the rover just starts to roll straight ahead (0 = no correction)
    min_wheel_rpm_spin: float = 0.0  # where it just starts to turn in place (skid steer needs much more)
    stop_below_rpm: float = 0.5   # commands slower than this are noise: a stop
    # The stall kick: commanded but standing (this ground needs more than the measured deadband), the deadband is
    # raised until the rover moves, kept while it moves no faster than commanded, and taken back when it does.
    stall_kick_delay_s: float = 1.0  # standing this long first (the motors and the odometry take part of it)
    stall_kick_rpm_per_s: float = 12.0  # then raised this fast (0 = never)
    stall_kick_max_rpm: float = 20.0  # by at most this much; no command is lifted beyond the deadband plus this


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


def deadband_corrected(linear_x: float, angular_z: float, geometry: DriveGeometry, config: MuxConfig,
                       kick_rpm: float = 0.0) -> tuple[float, float]:
    """Nav2's command with each side's wheel target lifted over the motors' deadband.

    An open-loop wheel turns only above some target d, and from there about linearly up to full speed. So each side's
    target t becomes d + (max - d) * |t| / max, with its sign: the side then really turns at about t. Full speed stays
    full speed, and a side that should stand still stays at zero. d depends on the manoeuvre: turning on the spot
    scrubs the tyres and needs more than rolling straight, so it is blended between the two measured values by how
    much of the command is turning. The result is given back as a twist, which drive_node turns into exactly these
    wheel targets.

    kick_rpm raises d for a rover that does not start (DriveMux decides when). Only slow commands gain from it: no
    side is lifted beyond d + stall_kick_max_rpm, so a command that is already well above the deadband goes out as
    without the kick.
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
    measured = 0.0 < deadband < top
    kick = max(kick_rpm, 0.0)
    if not measured and kick == 0.0:
        return linear_x, angular_z  # no correction (0), or unusable settings: pass Nav2's command as it is
    base = deadband if measured else 0.0
    ceiling = min(base + max(config.stall_kick_max_rpm, 0.0), MAX_DEADBAND_SHARE * top)
    raised = min(base + kick, ceiling)

    def lifted(target: float) -> float:
        if abs(target) <= threshold:
            return 0.0
        plain = base + (top - base) * abs(target) / top if measured else abs(target)
        kicked = raised + (top - raised) * abs(target) / top
        return math.copysign(max(plain, min(kicked, ceiling)), target)

    sides = lifted(left), lifted(right)
    if not measured and sides == (left, right):
        return linear_x, angular_z  # nothing to lift: Nav2's command as it is, to the last digit
    left, right = sides
    metres_per_rpm = math.pi * geometry.wheel_diameter_m / 60.0
    return (left + right) / 2.0 * metres_per_rpm, (right - left) / (2.0 * half_track) * metres_per_rpm


ESP32_DRIVING = 'DRIVING'  # the ESP32 state, as drive_node reports it, in which it follows commands
RATE_WINDOW_S = 0.15  # the odometry's speed is taken over at least this long
# The stall kick compares how fast the rover moves with how fast it was asked to, both as the speed of the faster
# side's wheels (m/s): driving plus turning times half the track.
STANDING_SPEED = 0.03  # slower than this is standing still, whatever was asked (the odometry's own jitter is below it)
STANDING_SHARE = 0.5  # ... and so is less than this share of what was asked: the kick grows
OUTRUN_SHARE = 1.3  # more than this share of what was asked: the kick is not needed (any more) and is taken back
KICK_DECAY_RPM_PER_S = 60.0  # ... this fast
NAV_COSTMAPS = ('local', 'global')  # Nav2's costmaps: the controller's and the planner's, one process each
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
        self._nav_pose: dict[str, tuple[float, float]] = {}  # per costmap: (the age of its pose of the rover, when heard)
        self._rate_ref: tuple[float, float, float, float] | None = None  # an odometry pose and its time, for the speed
        self._speed = self._turn_rate = 0.0  # the rover's speed and turn rate by the odometry
        self._standing_since: float | None = None  # since when it has stood although commanded
        self._last_tick: float | None = None
        self.kick = 0.0  # rpm added to the deadband right now
        self._obstacle_time: float | None = None
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
        if self._rate_ref is None:
            self._rate_ref = (x, y, yaw, now)
        elif now - self._rate_ref[3] >= RATE_WINDOW_S:
            rx, ry, ryaw, rtime = self._rate_ref
            self._speed = math.hypot(x - rx, y - ry) / (now - rtime)
            self._turn_rate = abs(math.atan2(math.sin(yaw - ryaw), math.cos(yaw - ryaw))) / (now - rtime)
            self._rate_ref = (x, y, yaw, now)
        if self._motion_since is None:
            return
        if self._motion_ref is None or self._moved(self._motion_ref, self._odom):
            self._motion_ref, self._motion_since = self._odom, now

    def on_obstacles(self, now: float) -> None:
        """A cloud from the obstacle detector: it sends one for every point cloud of the camera, empty or not."""
        self._obstacle_time = now

    def on_nav_pose(self, costmap: str, age_s: float, now: float) -> None:
        """A Nav2 costmap's footprint. Nav2 publishes it with every costmap update, stamped with the time of the
        transform it placed the rover with: `age_s` is how old that transform was when the footprint arrived."""
        self._nav_pose[costmap] = (age_s, now)

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
        self._end_kick()
        self.fault = ''

    def _end_kick(self) -> None:
        self._standing_since = self._last_tick = None
        self.kick = 0.0

    def _kick(self, linear_x: float, angular_z: float, now: float) -> float:
        """The stall kick for this tick, in rpm on top of the deadband. It grows while the rover stands, or crawls at
        a fraction of what it is commanded; it stays while the rover moves about as commanded (ground that needs
        more all the time, as when turning on the spot, keeps it); it goes back quickly when the rover outruns the
        command (it only had to break loose)."""
        dt = 0.0 if self._last_tick is None else min(max(now - self._last_tick, 0.0), 0.1)
        self._last_tick = now
        told_to_stand = deadband_corrected(linear_x, angular_z, self.geometry, self.config) == (0.0, 0.0)
        if self.config.stall_kick_rpm_per_s <= 0 or told_to_stand:
            self._standing_since, self.kick = None, 0.0
            return 0.0
        half_track = self.geometry.track_width_m * self.geometry.turn_track_factor / 2.0
        asked = abs(linear_x) + abs(angular_z) * half_track
        made = self._speed + self._turn_rate * half_track
        if made >= max(STANDING_SHARE * asked, STANDING_SPEED):
            self._standing_since = None
            if made > OUTRUN_SHARE * asked:
                self.kick = max(self.kick - KICK_DECAY_RPM_PER_S * dt, 0.0)
        elif self._standing_since is None:
            self._standing_since = now
        elif now - self._standing_since >= self.config.stall_kick_delay_s:
            self.kick = min(self.kick + self.config.stall_kick_rpm_per_s * dt, max(self.config.stall_kick_max_rpm, 0.0))
        return self.kick

    def _drive_auto(self, now: float, label: str) -> tuple[float, float]:
        commanded = self._auto_cmd is not None and now - self._auto_cmd_time <= self.config.command_timeout_s
        fault = self._find_fault(now)
        if fault:
            self.fault = fault
        elif self.fault and not self._goal_running(now):
            self.fault = ''  # the goal that ran into it is gone (goal_manager cancelled it): ready for the next one
        if self.fault:
            self._auto_since = self._motion_since = None
            self._end_kick()
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
        linear_x, angular_z = self._auto_cmd
        if not (math.isfinite(linear_x) and math.isfinite(angular_z)):
            self._motion_since = None
            self.state = f'{label}: following Nav2'
            return 0.0, 0.0
        kick = self._kick(linear_x, angular_z, now)
        self.state = f'{label}: following Nav2' + (' (the rover does not start: raising the output)' if kick > 0 else '')
        command = deadband_corrected(linear_x, angular_z, self.geometry, self.config, kick)
        if command == (0.0, 0.0):
            self._motion_since = None
        elif self._motion_since is None:
            self._motion_since, self._motion_ref = now, self._odom
        return command

    def _find_fault(self, now: float) -> str:
        if self._odom_time is None or now - self._odom_time > self.config.odometry_timeout_s:
            return 'no odometry from the camera'
        limit = self.config.obstacle_timeout_s
        if limit > 0 and (self._obstacle_time is None or now - self._obstacle_time > limit):
            return 'no obstacle data from the camera (its point cloud, or the obstacle detector, has stopped)'
        if self.config.nav_pose_max_age_s > 0:
            for costmap in NAV_COSTMAPS:
                report = self._nav_pose.get(costmap)
                if report is None or now - report[1] > self.config.nav_pose_timeout_s:
                    return f"Nav2's {costmap} costmap is not heard (Nav2 not running yet, or stuck)"
                if report[0] > self.config.nav_pose_max_age_s:
                    # No age in the text: it grows, and the text is logged and published whenever it changes.
                    return (f"Nav2's {costmap} costmap has lost track of the rover: the transforms it works with have "
                            "stopped. Restart the rover's launch")
        timeout = self.config.no_motion_timeout_s
        # While the kick is still growing for a standing rover, it is too early to give up.
        kicking = (self.config.stall_kick_rpm_per_s > 0 and self._standing_since is not None
                   and self.kick < self.config.stall_kick_max_rpm)
        if timeout > 0 and self._motion_since is not None and now - self._motion_since > timeout and not kicking:
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
