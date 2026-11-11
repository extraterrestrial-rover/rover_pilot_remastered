"""drive_mux's choice between the sticks and Nav2 (mux_logic)."""
import pytest

from rover_autonomy.mux_logic import DriveMux, MuxConfig, deadband_corrected
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


def test_auto_commands_are_corrected_for_the_motor_deadband():
    # Straight: 0.1 m/s is 6.82 rpm; 20 + (94.7 - 20) x 6.82 / 94.7 = 25.4 rpm, with no turning added.
    linear, angular = deadband_corrected(0.1, 0.0, ROVER, MINIMUMS)
    assert angular == 0.0 and fastest(linear, angular) == pytest.approx(20 + 74.7 * fastest(0.1, 0.0) / 94.7)
    # In place: 0.2 rad/s is 5.7 rpm per side; it starts from the spin deadband (60 rpm).
    linear, angular = deadband_corrected(0.0, 0.2, ROVER, MINIMUMS)
    assert linear == 0.0 and fastest(linear, angular) == pytest.approx(60 + 34.7 * fastest(0.0, 0.2) / 94.7)
    # An arc keeps its radius: both parts scale together, between the two deadbands.
    linear, angular = deadband_corrected(0.1, 0.2, ROVER, MINIMUMS)
    assert linear / angular == pytest.approx(0.1 / 0.2)
    assert 20.0 < fastest(linear, angular) < 60.0 + 34.7
    # Faster commands stay faster (monotonic), full speed stays full speed.
    assert fastest(*deadband_corrected(0.3, 0.0, ROVER, MINIMUMS)) > fastest(*deadband_corrected(0.2, 0.0, ROVER, MINIMUMS))
    top = ROVER.max_linear_speed
    assert deadband_corrected(top, 0.0, ROVER, MINIMUMS)[0] == pytest.approx(top)
    # Noise is a stop; no deadband configured (0): untouched.
    assert deadband_corrected(0.005, 0.0, ROVER, MINIMUMS) == (0.0, 0.0)
    assert deadband_corrected(0.0, 0.0, ROVER, MINIMUMS) == (0.0, 0.0)
    assert deadband_corrected(0.05, 0.0, ROVER, MuxConfig()) == (0.05, 0.0)


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
