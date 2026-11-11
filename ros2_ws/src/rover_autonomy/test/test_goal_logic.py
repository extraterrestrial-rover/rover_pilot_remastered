"""goal_manager's rules (goal_logic): goals only in AUTO, cancel on manual or e-stop, pause and resume on link loss."""
from rover_autonomy.goal_logic import CANCEL, SEND, Action, GoalLogic


def heard(logic, now, auto=True, estop=False):
    logic.on_auto(auto, now)
    logic.on_estop(estop, now)
    return logic.update(now)


def test_goals_are_taken_only_in_auto_with_the_estop_released():
    logic = GoalLogic()
    assert logic.on_goal('A', 0.0) == [] and 'not heard' in logic.events[-1]
    heard(logic, 0.0, auto=False)
    assert logic.on_goal('A', 0.1) == [] and 'not in AUTO' in logic.events[-1]
    heard(logic, 0.2, auto=True, estop=True)
    assert logic.on_goal('A', 0.2) == [] and 'e-stop' in logic.events[-1]
    heard(logic, 0.3)
    assert logic.on_goal('A', 0.3) == [Action(SEND, 'A')]
    assert logic.on_goal('B', 0.4) == [Action(SEND, 'B')] and 'replaces' in logic.events[-1]


def test_manual_or_estop_cancels_for_good():
    for change in ({'auto': False}, {'estop': True}):
        logic = GoalLogic()
        heard(logic, 0.0)
        logic.on_goal('A', 0.0)
        assert heard(logic, 0.1, **change) == [Action(CANCEL)]
        assert logic.goal is None
        assert heard(logic, 0.2) == []  # back in AUTO later: nothing resumes


def test_link_loss_pauses_and_the_return_resumes():
    logic = GoalLogic(link_timeout_s=0.5)
    heard(logic, 0.0)
    logic.on_goal('A', 0.0)
    assert logic.update(0.4) == []
    assert logic.update(0.6) == [Action(CANCEL)] and 'paused' in logic.events[-1]
    assert logic.update(5.0) == []  # still lost: nothing more
    logic.on_result(succeeded=False, cancelled=True)  # Nav2 confirms our cancel: the goal is kept
    assert heard(logic, 9.0) == [Action(SEND, 'A')] and 'resuming' in logic.events[-1]
    assert heard(logic, 9.1) == []


def test_after_a_base_station_restart_the_goal_waits_for_auto():
    logic = GoalLogic()
    heard(logic, 0.0)
    logic.on_goal('A', 0.0)
    logic.update(1.0)  # paused
    assert heard(logic, 5.0, auto=False, estop=True) == [] and 'goal kept' in logic.events[-1]  # restarted: e-stopped
    assert heard(logic, 5.1, auto=False, estop=False) == []  # released, driving manually: still kept
    assert logic.goal == 'A'
    assert heard(logic, 6.0, auto=True) == [Action(SEND, 'A')]  # AUTO pressed: on its way again


def test_an_estop_press_or_a_new_goal_drops_a_paused_goal():
    logic = GoalLogic()
    heard(logic, 0.0)
    logic.on_goal('A', 0.0)
    logic.update(1.0)  # paused
    heard(logic, 5.0, auto=False)  # back in manual
    assert heard(logic, 5.1, auto=False, estop=True) == [] and logic.goal is None
    assert 'dropped' in logic.events[-1]
    assert heard(logic, 6.0) == []  # AUTO later: nothing resumes
    logic.on_goal('B', 6.0)
    logic.update(7.0)  # paused
    assert heard(logic, 8.0) == [Action(SEND, 'B')]
    assert logic.on_goal('C', 8.1) == [Action(SEND, 'C')] and logic.goal == 'C'


def test_reaching_the_goal_or_nav2_giving_up_ends_it():
    logic = GoalLogic()
    heard(logic, 0.0)
    logic.on_goal('A', 0.0)
    logic.on_result(succeeded=True, cancelled=False)
    assert logic.goal is None and logic.events[-1] == 'goal reached'
    logic.on_goal('B', 0.1)
    logic.on_result(succeeded=False, cancelled=False)
    assert logic.goal is None and 'gave up' in logic.events[-1]
    assert heard(logic, 0.2) == []  # no automatic retry
    logic.on_goal('C', 0.3)
    logic.on_rejected('Nav2 refused it')
    assert logic.goal is None and 'refused' in logic.events[-1]
