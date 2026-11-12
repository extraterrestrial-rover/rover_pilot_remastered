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


def test_manual_passes_the_sticks_through_and_stale_input_is_dropped():
    mux = DriveMux(MuxConfig(), ROVER)
    assert mux.output(0.0) is None  # nothing heard yet
    mux.on_mode(False, 0.0)
    mux.on_manual(0.7, -0.2, 0.0)
    mux.on_auto_command(0.3, 0.0, 0.0)  # Nav2 is ignored in manual
    assert mux.output(0.1) == (0.7, -0.2)
    mux.on_mode(False, 0.4)
    assert mux.output(0.4) is None  # the sticks' command is 0.4 s old: drive_node times out as usual


def test_auto_starts_with_zeros_then_follows_nav2():
    mux = DriveMux(MuxConfig(neutral_hold_s=0.3), ROVER)
    mux.on_mode(True, 0.0)
    mux.on_manual(0.0, 0.0, 0.0)
    mux.on_auto_command(0.4, 0.1, 0.0)
    assert mux.output(0.0) == (0.0, 0.0)
    assert mux.output(0.29) == (0.0, 0.0)
    mux.on_mode(True, 0.3)
    mux.on_auto_command(0.4, 0.1, 0.3)
    assert mux.output(0.31) == (0.4, 0.1)
    assert mux.state == 'AUTO: following Nav2'


def test_auto_without_a_goal_holds_still():
    mux = DriveMux(MuxConfig(neutral_hold_s=0.0), ROVER)
    mux.on_mode(True, 0.0)
    assert mux.output(0.1) == (0.0, 0.0)
    mux.on_auto_command(0.4, 0.0, 0.1)
    mux.on_mode(True, 0.45)
    assert mux.output(0.45) == (0.0, 0.0)  # Nav2's last command is 0.35 s old


def test_losing_the_base_station_passes_nothing_and_auto_restarts_with_zeros():
    mux = DriveMux(MuxConfig(neutral_hold_s=0.3), ROVER)
    mux.on_mode(True, 0.0)
    assert mux.output(0.0) == (0.0, 0.0)  # the mux runs at 50 Hz: the AUTO stretch starts here
    mux.on_auto_command(0.4, 0.0, 0.5)
    mux.on_mode(True, 0.5)
    assert mux.output(0.5) == (0.4, 0.0)
    assert mux.output(1.1) is None  # heartbeat 0.6 s old: drive_node's heartbeat check e-stops
    mux.on_mode(True, 3.0)  # the radio is back, still AUTO
    mux.on_auto_command(0.4, 0.0, 3.0)
    assert mux.output(3.0) == (0.0, 0.0)  # the ESP32 needs a neutral command after the e-stop
    mux.on_mode(True, 3.3)
    mux.on_auto_command(0.4, 0.0, 3.3)
    assert mux.output(3.31) == (0.4, 0.0)


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


def test_manual_is_never_raised():
    mux = DriveMux(MINIMUMS, ROVER)
    mux.on_mode(False, 0.0)
    mux.on_manual(0.05, 0.0, 0.0)
    assert mux.output(0.0) == (0.05, 0.0)


def test_non_finite_nav2_commands_stop():
    mux = DriveMux(MuxConfig(neutral_hold_s=0.0), ROVER)
    mux.on_mode(True, 0.0)
    mux.on_auto_command(float('nan'), 0.0, 0.0)
    assert mux.output(0.0) == (0.0, 0.0)
