from arm_pilot_redemption.arm_logic import ArmGate, joint_velocities

COMMAND = (0.1, -0.2, 0.3, 1.0)


def test_waits_for_the_heartbeat_then_follows_fresh_commands():
    gate = ArmGate()
    assert gate.decide(0.0).estop and 'heartbeat' in gate.decide(0.0).reason
    gate.on_estop(False, 0.0)
    assert gate.decide(0.1) == gate.decide(0.1).__class__(False, (0.0,) * 4, 'no fresh joint command')
    gate.on_joint_cmd(COMMAND, 0.1)
    decision = gate.decide(0.2)
    assert not decision.estop and decision.velocities == COMMAND and decision.reason == ''


def test_estop_and_stale_heartbeat_send_the_estop_flag():
    gate = ArmGate()
    gate.on_estop(True, 0.0)
    gate.on_joint_cmd(COMMAND, 0.0)
    assert gate.decide(0.1).estop and gate.decide(0.1).velocities == (0.0,) * 4
    gate.on_estop(False, 0.2)
    gate.on_joint_cmd(COMMAND, 0.2)
    assert not gate.decide(0.3).estop
    decision = gate.decide(0.75)
    assert decision.estop and decision.reason == 'base station e-stop heartbeat lost'


def test_stale_command_gives_zero_without_estop():
    gate = ArmGate()
    gate.on_joint_cmd(COMMAND, 0.0)
    gate.on_estop(False, 0.35)
    decision = gate.decide(0.35)
    assert not decision.estop and decision.velocities == (0.0,) * 4


def test_heartbeat_can_be_optional():
    gate = ArmGate(require_estop_heartbeat=False)
    gate.on_joint_cmd(COMMAND, 0.0)
    assert gate.decide(0.1).velocities == COMMAND


def test_joint_state_reading():
    assert joint_velocities(['wrist_roll', 'shoulder_pitch'], [2.0, 0.5]) == (0.5, 0.0, 0.0, 2.0)
    assert joint_velocities(['shoulder_pitch', 'gripper'], [0.5, 9.0]) == (0.5, 0.0, 0.0, 0.0)
    assert joint_velocities([], [1.0, 2.0, 3.0, 4.0]) == (1.0, 2.0, 3.0, 4.0)
    assert joint_velocities([], [1.0, 2.0]) is None
    assert joint_velocities(['shoulder_pitch'], []) is None
    assert joint_velocities([], [float('nan'), float('inf'), 1.0, 0.0]) == (0.0, 0.0, 1.0, 0.0)
