"""goal_manager's rules (goal_logic): goals only in AUTO, cancelled by manual or the e-stop, finished without the
base station."""
from rover_autonomy.goal_logic import CANCEL, SEND, Action, GoalLogic


def heard(logic, now, auto=True, estop=False):
    """The base station's two heartbeats arrive, and the 10 Hz update runs."""
    logic.on_auto(auto, now)
    logic.on_estop(estop, now)
    return later(logic, now)


def later(logic, now, fault=''):
    """The 10 Hz update, with drive_mux reporting as it does ('': AUTO has not stopped itself)."""
    logic.on_fault(fault, now)
    return logic.update(now)


def running(now=0.0):
    logic = GoalLogic()
    heard(logic, now)
    assert logic.on_goal('A', now) == [Action(SEND, 'A')] and logic.active
    return logic


def test_goals_are_taken_only_in_auto_with_the_estop_released():
    logic = GoalLogic()
    assert logic.on_goal('A', 0.0) == [] and 'not heard' in logic.events[-1]
    heard(logic, 0.0, auto=False)
    assert logic.on_goal('A', 0.1) == [] and 'not in AUTO' in logic.events[-1]
    heard(logic, 0.2, auto=True, estop=True)
    assert logic.on_goal('A', 0.2) == [] and 'e-stop' in logic.events[-1]
    assert not logic.active
    heard(logic, 0.3)
    assert logic.on_goal('A', 0.3) == [Action(SEND, 'A')] and logic.active
    assert logic.on_goal('B', 0.4) == [Action(SEND, 'B')] and 'replaces' in logic.events[-1]


def test_manual_or_the_estop_cancels_for_good():
    for change, reason in (({'auto': False}, 'manual driving'), ({'estop': True}, 'e-stop'),
                           ({'auto': False, 'estop': True}, 'e-stop')):
        logic = running()
        assert heard(logic, 0.1, **change) == [Action(CANCEL)]
        assert not logic.active and logic.events[-1] == f'goal cancelled: {reason}'
        assert heard(logic, 0.2) == []  # back in AUTO later: nothing comes back


def test_the_goal_goes_on_when_the_base_station_falls_silent():
    logic = running()
    assert later(logic, 0.4) == []
    assert later(logic, 0.6) == [] and logic.active  # heartbeats 0.6 s old: lost, and the goal is not touched
    assert 'goes on to its goal' in logic.events[-1] and 'kill switch' in logic.events[-1]
    count = len(logic.events)
    assert later(logic, 5.0) == [] and len(logic.events) == count  # said once
    assert heard(logic, 9.0) == [] and logic.active and logic.events[-1] == 'base station back: the goal goes on'


def test_reaching_the_goal_without_the_base_station_says_that_the_estop_is_latched():
    logic = running()
    later(logic, 1.0)  # lost
    logic.on_result(succeeded=True)
    assert not logic.active  # drive_mux latches the e-stop on this
    assert logic.events[-1].startswith('goal reached') and 'e-stop is latched until it is back' in logic.events[-1]
    assert later(logic, 2.0) == [] and heard(logic, 3.0) == []


def test_what_the_base_station_says_when_it_is_back_goes():
    # It restarted (e-stopped, manual), or the operator pressed the e-stop or took the sticks during the outage.
    for change, reason in (({'auto': False, 'estop': True}, 'e-stop'), ({'auto': False}, 'manual driving')):
        logic = running()
        later(logic, 1.0)
        assert heard(logic, 5.0, **change) == [Action(CANCEL)] and logic.events[-1] == f'goal cancelled: {reason}'
        assert heard(logic, 6.0) == []  # AUTO again later: the old goal does not come back


def test_an_estop_heard_alone_is_enough():
    logic = running()
    logic.on_estop(True, 1.0)  # the AUTO heartbeat has not arrived (two topics, either order, or one of them lost)
    assert later(logic, 1.0) == [Action(CANCEL)] and logic.events[-1] == 'goal cancelled: e-stop'
    logic = running()
    logic.on_estop(False, 1.0)  # the base station is heard, but says nothing about AUTO any more
    assert later(logic, 1.0) == [Action(CANCEL)] and logic.events[-1] == 'goal cancelled: manual driving'


def test_reaching_the_goal_or_nav2_giving_up_ends_it():
    logic = running()
    logic.on_result(succeeded=True)
    assert not logic.active and logic.events[-1] == 'goal reached'
    logic.on_goal('B', 0.1)
    logic.on_result(succeeded=False)
    assert not logic.active and 'gave up' in logic.events[-1]
    assert heard(logic, 0.2) == []  # no automatic retry
    logic.on_goal('C', 0.3)
    logic.on_rejected('Nav2 refused it')
    assert not logic.active and 'refused' in logic.events[-1]
    logic.on_result(succeeded=True)  # a late result for a goal that is gone: nothing
    assert logic.events[-1] != 'goal reached'


def test_a_fault_from_drive_mux_cancels_the_goal():
    # AUTO stopped itself (no odometry, no motion): Nav2 must not go on trying, least of all with its spins.
    logic = running()
    assert later(logic, 0.1) == []
    assert later(logic, 0.2, fault='no odometry from the camera') == [Action(CANCEL)]
    assert not logic.active and logic.events[-1] == 'goal cancelled: no odometry from the camera'
    assert later(logic, 0.3, fault='no odometry from the camera') == []  # once
    logic = running(1.0)
    later(logic, 2.0)  # the base station is lost; then the camera fails on the way
    assert later(logic, 3.0, fault='no odometry from the camera') == [Action(CANCEL)]


def test_a_silent_drive_mux_cancels_the_goal():
    logic = GoalLogic()
    logic.on_auto(True, 0.0)
    logic.on_estop(False, 0.0)
    assert logic.on_goal('A', 0.0) == [Action(SEND, 'A')]
    assert logic.update(0.1) == [Action(CANCEL)]  # drive_mux was never heard
    assert 'drive_mux is not running' in logic.events[-1]
    logic = running(1.0)
    logic.on_auto(True, 2.2)
    logic.on_estop(False, 2.2)
    assert logic.update(2.2) == [Action(CANCEL)]  # its last report is 1.2 s old
