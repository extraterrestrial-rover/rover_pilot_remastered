import pytest

from arm_pilot_redemption.teleop_logic import JointInput, TeleopConfig, TeleopLogic, apply_deadzone

CIRCLE, L1, OPTIONS = 1, 4, 9  # TeleopConfig defaults (config/arm.yaml)
LEFT_Y, RIGHT_Y, DPAD_X, DPAD_Y = 1, 4, 6, 7


def joy(left_y=0.0, right_y=0.0, dpad_x=0.0, dpad_y=0.0, pressed=()):
    axes = [0.0] * 8
    axes[LEFT_Y], axes[RIGHT_Y], axes[DPAD_X], axes[DPAD_Y] = left_y, right_y, dpad_x, dpad_y
    axes[2] = axes[5] = 1.0  # triggers at rest
    return axes, [1 if i in pressed else 0 for i in range(13)]


def released(logic, now=0.0):
    logic.on_joy(*joy(pressed=(OPTIONS,)), now)
    logic.on_joy(*joy(), now)
    assert not logic.estopped


def test_starts_latched_and_release_needs_every_input_centred():
    logic = TeleopLogic(TeleopConfig())
    logic.on_joy(*joy(dpad_y=1.0), 0.0)
    assert logic.tick(0.0)[0] == (0.0, 0.0, 0.0, 0.0) and logic.estopped
    events = logic.on_joy(*joy(dpad_y=1.0, pressed=(OPTIONS,)), 0.1)  # the D-pad counts too
    assert logic.estopped and 'centre' in events[0]
    logic.on_joy(*joy(), 0.2)
    logic.on_joy(*joy(pressed=(OPTIONS,)), 0.3)
    assert not logic.estopped


def test_each_input_drives_its_joint_proportionally():
    logic = TeleopLogic(TeleopConfig(max_velocity=(0.4, 0.6, 0.8, 3.0)))
    released(logic)
    logic.on_joy(*joy(left_y=1.0, right_y=-0.5), 0.1)
    velocities, _ = logic.tick(0.1)
    assert velocities[0] == pytest.approx(0.4)
    assert velocities[1] == pytest.approx(-apply_deadzone(0.5, 0.08) * 0.6)
    assert velocities[2:] == (0.0, 0.0)
    logic.on_joy(*joy(dpad_y=-1.0, dpad_x=-1.0), 0.2)  # D-pad down + right
    velocities, _ = logic.tick(0.2)
    assert velocities == pytest.approx((0.0, 0.0, -0.8, 3.0))  # right = positive roll (inverted axis)


def test_slow_mode_and_deadzone():
    logic = TeleopLogic(TeleopConfig(max_velocity=(1.0, 1.0, 1.0, 1.0)))
    released(logic)
    logic.on_joy(*joy(left_y=1.0, pressed=(L1,)), 0.1)
    assert logic.tick(0.1)[0][0] == pytest.approx(0.35)
    logic.on_joy(*joy(left_y=0.05), 0.2)
    assert logic.tick(0.2)[0][0] == 0.0


def test_buttons_can_drive_a_joint():
    inputs = (JointInput(axis=1), JointInput(axis=4), JointInput(axis=7), JointInput(positive_button=5, negative_button=2))
    logic = TeleopLogic(TeleopConfig(inputs=inputs, max_velocity=(1.0, 1.0, 1.0, 2.0)))
    released(logic)
    logic.on_joy(*joy(pressed=(5,)), 0.1)
    assert logic.tick(0.1)[0][3] == 2.0
    logic.on_joy(*joy(pressed=(2,)), 0.2)
    assert logic.tick(0.2)[0][3] == -2.0
    logic.on_joy(*joy(pressed=(2, 5)), 0.3)
    assert logic.tick(0.3)[0][3] == 0.0


def test_estop_button_latches_and_release_needs_a_new_press():
    logic = TeleopLogic(TeleopConfig())
    released(logic)
    logic.on_joy(*joy(left_y=0.8, pressed=(CIRCLE,)), 1.0)
    assert logic.estopped and logic.tick(1.0)[0] == (0.0, 0.0, 0.0, 0.0)
    logic.on_joy(*joy(pressed=(CIRCLE, OPTIONS)), 1.1)  # release ignored while the e-stop button is held
    assert logic.estopped
    logic.on_joy(*joy(pressed=(OPTIONS,)), 1.2)  # still held from before: not a new press
    assert logic.estopped
    logic.on_joy(*joy(), 1.3)
    logic.on_joy(*joy(pressed=(OPTIONS,)), 1.4)
    assert not logic.estopped


def test_joystick_loss_latches():
    logic = TeleopLogic(TeleopConfig(joy_timeout_s=0.5))
    released(logic)
    logic.on_joy(*joy(left_y=1.0), 1.0)
    assert logic.tick(1.4)[0][0] > 0
    velocities, events = logic.tick(1.6)
    assert velocities == (0.0, 0.0, 0.0, 0.0) and logic.estopped and 'joystick lost' in events[0]
    logic.on_joy(*joy(left_y=1.0), 2.0)
    assert logic.tick(2.0)[0][0] == 0.0 and logic.estopped


def test_short_or_missing_axes_are_safe():
    logic = TeleopLogic(TeleopConfig(inputs=(JointInput(axis=20),) * 4))
    released(logic)
    assert logic.tick(0.1)[0] == (0.0, 0.0, 0.0, 0.0)
