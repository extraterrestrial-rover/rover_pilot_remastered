"""AUTO mode in the base-station teleop (teleop_logic): when the rover may follow Nav2 instead of the sticks."""
from rover_pilot_remastered.teleop_logic import TeleopConfig, TeleopLogic

CIRCLE, OPTIONS, TRIANGLE = 1, 6, 2


def joy(left_y=0.0, right_x=0.0, pressed=()):
    axes = [0.0, left_y, right_x, 0.0, 1.0, 1.0]
    buttons = [1 if i in pressed else 0 for i in range(15)]
    return axes, buttons


def press(logic, button, now, **sticks):
    events = logic.on_joy(*joy(pressed=(button,), **sticks), now)
    return events + logic.on_joy(*joy(**sticks), now + 0.01)


def ready(now=0.0):
    logic = TeleopLogic(TeleopConfig(auto_button=TRIANGLE))
    press(logic, OPTIONS, now)
    assert not logic.estopped
    return logic


def test_auto_needs_the_estop_released_and_centred_sticks():
    logic = TeleopLogic(TeleopConfig(auto_button=TRIANGLE))
    events = press(logic, TRIANGLE, 0.0)
    assert not logic.auto and 'release the e-stop' in events[0]
    press(logic, OPTIONS, 0.1)
    events = logic.on_joy(*joy(left_y=0.5, pressed=(TRIANGLE,)), 0.2)
    assert not logic.auto and 'centre' in events[0]
    logic.on_joy(*joy(), 0.3)
    events = press(logic, TRIANGLE, 0.4)
    assert logic.auto and 'AUTO on' in events[0]


def test_in_auto_the_teleop_sends_no_motion_and_the_button_toggles_back():
    logic = ready()
    press(logic, TRIANGLE, 0.1)
    assert logic.auto
    assert logic.tick(0.2)[:2] == (0.0, 0.0)
    holding = logic.on_joy(*joy(pressed=(TRIANGLE,)), 0.3)  # held down: one press, one toggle
    assert holding and not logic.auto
    assert logic.on_joy(*joy(pressed=(TRIANGLE,)), 0.35) == [] and not logic.auto


def test_a_stick_push_takes_manual_control_at_once():
    logic = ready()
    press(logic, TRIANGLE, 0.1)
    events = logic.on_joy(*joy(left_y=0.6), 0.2)
    assert not logic.auto and 'stick moved' in events[0]
    linear, _, _ = logic.tick(0.2)
    assert linear > 0  # this very tick drives manually
    logic.on_joy(*joy(right_x=0.05), 0.3)  # inside the deadzone is not a push
    press(logic, TRIANGLE, 0.4)
    logic.on_joy(*joy(right_x=0.05), 0.5)
    assert logic.auto


def test_estop_and_a_lost_joystick_end_auto():
    logic = ready()
    press(logic, TRIANGLE, 0.1)
    events = logic.on_joy(*joy(pressed=(CIRCLE,)), 0.2)
    assert logic.estopped and not logic.auto and any('AUTO off (e-stop)' in e for e in events)
    logic = ready()
    press(logic, TRIANGLE, 0.1)
    _, _, events = logic.tick(1.0)  # no /joy for 0.89 s
    assert logic.estopped and not logic.auto and any('joystick lost' in e for e in events)


def test_auto_is_off_without_an_auto_button():
    logic = TeleopLogic(TeleopConfig())  # auto_button -1, as on main
    press(logic, OPTIONS, 0.0)
    for button in range(15):
        if button not in (CIRCLE, OPTIONS):
            press(logic, button, 0.1 + button)
    assert not logic.auto
