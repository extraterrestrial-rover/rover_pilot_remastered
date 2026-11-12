"""drive_mux's choice between the sticks and Nav2 (mux_logic)."""
import math

import pytest

from rover_autonomy.mux_logic import DriveMux, MuxConfig, checked_deadband, deadband_corrected
from rover_pilot_remastered.kinematics import DriveGeometry, twist_to_wheel_rpm

ROVER = DriveGeometry(0.28, 0.84, 94.7)
MINIMUMS = MuxConfig(min_wheel_rpm_straight=20.0, min_wheel_rpm_spin=60.0)


def fastest(linear, angular):
    left, right, _, _ = twist_to_wheel_rpm(linear, angular, ROVER)
    return max(abs(left), abs(right))


class Run:
    """A mux fed like on the rover: the AUTO heartbeat, Nav2's command and the odometry, 50 times a second."""

    def __init__(self, config: MuxConfig = MuxConfig()) -> None:
        self.mux, self.now, self.x = DriveMux(config, ROVER), 0.0, 0.0

    def step(self, seconds, *, auto=True, command=(0.4, 0.0), odometry='moving'):
        """Returns the last output. odometry: 'moving', 'frozen' (the same pose again) or None (nothing arrives)."""
        out = None
        for _ in range(round(seconds / 0.02)):
            self.now += 0.02
            if auto is not None:
                self.mux.on_mode(auto, self.now)
            if command is not None:
                self.mux.on_auto_command(*command, self.now)
            if odometry == 'moving':
                self.x += 0.01
            if odometry is not None:
                self.mux.on_odometry(self.x, 0.0, 0.0, self.now)
            out = self.mux.output(self.now)
        return out


def test_manual_commands_are_passed_on_as_they_arrive_and_never_repeated():
    mux = DriveMux(MuxConfig(), ROVER)
    mux.on_mode(False, 0.0)
    mux.on_auto_command(0.3, 0.0, 0.0)  # Nav2 is ignored in manual
    assert mux.on_manual(0.7, -0.2, 0.1) == (0.7, -0.2)
    # The mux repeats nothing: drive_node sees each stick command once, so its 0.3 s command timeout counts from the
    # last one the base station really sent. (Repeating the last command for 0.3 s added that much to every stop.)
    assert mux.output(0.12) is None and mux.output(0.3) is None and mux.state == 'manual'


def test_a_base_station_without_the_auto_heartbeat_drives_manually():
    # A base station on a branch without AUTO never sends /rover/auto. The rover used to be undrivable then.
    mux = DriveMux(MINIMUMS, ROVER)
    assert mux.on_manual(0.05, 0.0, 0.0) == (0.05, 0.0)  # and manual is never deadband-corrected
    assert mux.output(0.0) is None and 'no AUTO heartbeat' in mux.state
    mux.on_mode(True, 1.0)
    assert mux.on_manual(0.5, 0.0, 1.0) is None  # in AUTO the sticks' zeros are not passed on
    assert mux.on_manual(0.5, 0.0, 1.6) == (0.5, 0.0)  # the AUTO heartbeat is 0.6 s old: manual again


def test_auto_starts_with_zeros_then_follows_nav2():
    run = Run(MuxConfig(neutral_hold_s=0.3))
    assert run.step(0.02) == (0.0, 0.0) and run.mux.state == 'AUTO: neutral first'
    assert run.step(0.26) == (0.0, 0.0)
    assert run.step(0.1, command=(0.4, 0.1)) == (0.4, 0.1)
    assert run.mux.state == 'AUTO: following Nav2' and run.mux.fault == ''


def test_auto_without_a_goal_holds_still():
    run = Run(MuxConfig(neutral_hold_s=0.0))
    assert run.step(0.1, command=None) == (0.0, 0.0)
    assert run.step(0.1) == (0.4, 0.0)
    assert run.step(0.36, command=None) == (0.0, 0.0)  # Nav2's last command is over 0.3 s old
    assert 'holding still' in run.mux.state


def test_losing_the_auto_heartbeat_ends_auto_and_it_restarts_with_zeros():
    run = Run(MuxConfig(neutral_hold_s=0.3))
    assert run.step(0.5) == (0.4, 0.0)
    assert run.step(0.6, auto=None) is None and 'base station not heard' in run.mux.state
    assert run.step(0.02) == (0.0, 0.0)  # back in AUTO: the ESP32 needs a neutral command after the e-stop
    assert run.step(0.4) == (0.4, 0.0)


def test_auto_does_not_drive_without_odometry():
    # Without odometry the EKF keeps predicting at the last speed and Nav2 follows its imagination: in the rehearsal
    # the rover drove 3.5 m blind in 12 s after the camera's odometry stopped.
    run = Run(MuxConfig(neutral_hold_s=0.0))
    assert run.step(0.1, odometry=None) == (0.0, 0.0)  # none yet: AUTO does not even start
    assert run.mux.fault == 'no odometry from the camera' and 'AUTO stopped' in run.mux.state
    assert run.step(0.2) == (0.0, 0.0) and run.mux.fault  # odometry is back, but Nav2 still runs the goal that hit it
    assert run.step(0.4, command=None) == (0.0, 0.0) and run.mux.fault == ''  # that goal is gone: ready again
    assert run.step(0.1) == (0.4, 0.0)
    assert run.step(0.48, odometry=None) == (0.4, 0.0)  # a short gap is not a loss
    assert run.step(0.06, odometry=None) == (0.0, 0.0) and run.mux.fault == 'no odometry from the camera'


def test_a_motion_command_without_motion_stops_auto():
    # The ZED wrapper repeats its last pose when it loses tracking: the rover drove 9 m in 20 s with frozen odometry.
    run = Run(MuxConfig(neutral_hold_s=0.0, no_motion_timeout_s=3.0))
    assert run.step(2.9, odometry='frozen') == (0.4, 0.0)
    assert run.step(0.2, odometry='frozen') == (0.0, 0.0)
    assert 'has not moved for 3 s' in run.mux.fault
    assert run.step(0.4, odometry='frozen', command=None) == (0.0, 0.0) and run.mux.fault == ''
    assert run.step(10.0) == (0.4, 0.0) and run.mux.fault == ''  # moving: never a fault
    assert run.step(10.0, odometry='frozen', command=(0.0, 0.0)) == (0.0, 0.0) and run.mux.fault == ''  # told to stand
    off = Run(MuxConfig(neutral_hold_s=0.0, no_motion_timeout_s=0.0))
    assert off.step(10.0, odometry='frozen') == (0.4, 0.0)  # 0 turns the check off


def test_turning_on_the_spot_counts_as_motion():
    run = Run(MuxConfig(neutral_hold_s=0.0))
    for step in range(300):  # 6 s of turning: only the heading changes
        run.now += 0.02
        run.mux.on_mode(True, run.now)
        run.mux.on_auto_command(0.0, 0.5, run.now)
        run.mux.on_odometry(0.0, 0.0, math.remainder(0.01 * step, 2 * math.pi), run.now)
        assert run.mux.output(run.now) == (0.0, 0.5)


def test_an_esp32_that_is_not_driving_gets_zeros_again():
    # After an ESP32 reset or a link hiccup it waits for a neutral command. Nav2 never sends one while it drives: in
    # the rehearsal the rover stood for 30 s after a reset, until Nav2's own recovery paused its commands.
    run = Run(MuxConfig(neutral_hold_s=0.3, rearm_interval_s=1.5))
    assert run.step(2.0) == (0.4, 0.0)
    run.mux.on_drive_state('WAIT_NEUTRAL', run.now)
    assert run.step(0.02) == (0.0, 0.0) and run.mux.state == 'AUTO: neutral first'
    assert run.step(0.4) == (0.4, 0.0)  # zeros for 0.3 s, then Nav2 again
    assert run.step(1.0) == (0.4, 0.0)  # the same report is still the latest: not again before 1.5 s
    assert run.step(0.2) == (0.0, 0.0)  # still not driving after 1.5 s: once more
    run.mux.on_drive_state('DRIVING', run.now)
    assert run.step(2.0) == (0.4, 0.0)
    run.mux.on_drive_state('NO_COMMAND', run.now)
    run.step(0.4)
    assert run.step(4.0) == (0.4, 0.0)  # a report older than 3 s says nothing: drive_node itself is gone


def sides(linear, angular):
    left, right, _, _ = twist_to_wheel_rpm(linear, angular, ROVER)
    return left, right


def motor(target_rpm, deadband_rpm, top=94.7):
    """A side's real speed for a wheel target: nothing below the deadband, then linear up to full speed."""
    if abs(target_rpm) <= deadband_rpm:
        return 0.0
    return math.copysign((abs(target_rpm) - deadband_rpm) / (top - deadband_rpm) * top, target_rpm)


def deadband_for(linear, angular):
    turning = abs(angular) * 0.42  # half the track width
    return 20.0 + 40.0 * turning / (abs(linear) + turning)


@pytest.mark.parametrize('linear, angular', [(0.1, 0.0), (0.5, 0.0), (0.0, 0.3), (0.0, -0.8), (0.4, 0.05), (0.3, 0.2),
                                             (0.3, 0.5), (0.2, -0.6)])
def test_each_side_really_turns_at_what_nav2_asked(linear, angular):
    asked = sides(linear, angular)
    sent = sides(*deadband_corrected(linear, angular, ROVER, MINIMUMS))
    deadband = deadband_for(linear, angular)
    for side in (0, 1):
        assert motor(sent[side], deadband) == pytest.approx(asked[side], abs=1e-6)
    # Scaling both sides by the faster one (the first version) left the slower side inside its deadband in arcs:
    # asked for 0.3 m/s with 0.2 rad/s, the rover turned at more than twice that rate.
    assert all(abs(target) > deadband for target, wanted in zip(sent, asked) if abs(wanted) > 0.5)


def test_full_speed_stays_full_speed_and_noise_is_a_stop():
    top = ROVER.max_linear_speed
    assert deadband_corrected(top, 0.0, ROVER, MINIMUMS) == pytest.approx((top, 0.0))
    assert sides(*deadband_corrected(3 * top, 0.0, ROVER, MINIMUMS)) == pytest.approx((94.7, 94.7))  # clamped as always
    assert deadband_corrected(0.005, 0.0, ROVER, MINIMUMS) == (0.0, 0.0)
    assert deadband_corrected(0.05, 0.0, ROVER, MuxConfig()) == (0.05, 0.0)  # no deadband set: untouched
    left, right = sides(*deadband_corrected(0.25, 0.595, ROVER, MINIMUMS))  # a pivot: the left side asked for 0.01 rpm
    assert left == pytest.approx(0.0, abs=1e-9) and right > 40.0


@pytest.mark.parametrize('stop_below_rpm', [0.5, 0.0, -1.0])
def test_a_zero_command_is_zero_whatever_the_stop_threshold(stop_below_rpm):
    # With stop_below_rpm 0.0, Nav2's first zero command raised ZeroDivisionError and drive_mux restarted in a loop.
    config = MuxConfig(min_wheel_rpm_straight=20.0, min_wheel_rpm_spin=60.0, stop_below_rpm=stop_below_rpm)
    assert deadband_corrected(0.0, 0.0, ROVER, config) == (0.0, 0.0)


def test_deadband_settings_are_checked():
    assert checked_deadband(20.0, 60.0, 94.7) == (20.0, 60.0, '')
    assert checked_deadband(0, 0, 94.7) == (0.0, 0.0, '')  # not measured yet: no correction
    for straight, spin in ((150.0, 60.0), (20.0, 80.0), (-1.0, 60.0), (float('nan'), 0.0), ('20', 60.0), (True, 0.0)):
        checked = checked_deadband(straight, spin, 94.7)
        assert checked[:2] == (0.0, 0.0) and checked[2], (straight, spin)
    assert 'min_wheel_rpm_straight is 150 rpm' in checked_deadband(150.0, 60.0, 94.7)[2]
    with pytest.raises(ValueError, match='min_wheel_rpm_spin'):
        DriveMux(MuxConfig(min_wheel_rpm_spin=150.0), ROVER)


def test_a_wrong_deadband_can_never_mean_full_speed():
    # 150 typed as rpm (it was a PWM value): a 0.05 m/s crawl used to come out as 1.39 m/s, full speed.
    wrong = MuxConfig(min_wheel_rpm_straight=150.0, min_wheel_rpm_spin=150.0)
    assert deadband_corrected(0.05, 0.0, ROVER, wrong) == (0.05, 0.0)
    assert deadband_corrected(0.0, 0.3, ROVER, wrong) == (0.0, 0.3)


def test_non_finite_nav2_commands_stop():
    run = Run(MuxConfig(neutral_hold_s=0.0))
    assert run.step(0.1, command=(float('nan'), 0.0)) == (0.0, 0.0)
    assert run.step(0.1, command=(0.2, float('inf'))) == (0.0, 0.0)
