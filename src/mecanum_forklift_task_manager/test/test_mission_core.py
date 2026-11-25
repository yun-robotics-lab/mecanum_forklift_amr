import pytest

from mecanum_forklift_task_manager.mission_core import (
    MissionGoal, MissionRunner, Policy, StepResult)

OK = StepResult(True)


class Mock:
    """Scripted executor: results[name] is a list consumed per attempt; records every call."""

    def __init__(self, script):
        self.script = {k: list(v) for k, v in script.items()}
        self.calls = []
        self.stops = 0
        self.on_call = None                    # optional hook(call_name)

    def next(self, key):
        self.calls.append(key)
        if self.on_call:
            self.on_call(key)
        queue = self.script.get(key, [OK])
        return queue.pop(0) if len(queue) > 1 else queue[0]

    def dock(self, dock_id, timeout, cancelled):
        self.last_timeout = timeout
        return self.next('PICK' if dock_id == 'pick' else 'DROP')

    def navigate(self, pose, timeout, cancelled):
        self.last_timeout = timeout
        return self.next('RETURN')

    def stop(self):
        self.stops += 1


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def make(script, policy=None, ret=True, clock=None):
    ex = Mock(script)
    clock = clock or Clock()
    runner = MissionRunner(ex, policy or Policy(), lambda g: '' if g.pick_dock_id else 'no pick',
                           clock)
    goal = MissionGoal('m1', pick_dock_id='pick', drop_dock_id='drop',
                       return_pose=(0, 0, 0) if ret else None)
    return runner, ex, goal, clock


def run(runner, goal, cancelled=lambda: False):
    return runner.run(goal, cancelled)


def test_success_runs_all_steps_in_order():
    runner, ex, goal, _ = make({})
    res = run(runner, goal)
    assert res.success and ex.calls == ['PICK', 'DROP', 'RETURN'] and ex.stops == 0


def test_no_return_pose_skips_return():
    runner, ex, goal, _ = make({}, ret=False)
    assert run(runner, goal).success and ex.calls == ['PICK', 'DROP']


def test_invalid_mission_runs_nothing():
    runner, ex, goal, _ = make({})
    goal.pick_dock_id = ''
    res = run(runner, goal)
    assert (res.success, res.error_code, res.failed_step) == (False, 'INVALID_MISSION', 'VALIDATE')
    assert ex.calls == []


@pytest.mark.parametrize('step', ['PICK', 'DROP', 'RETURN'])
@pytest.mark.parametrize('code', ['STAGING_FAILED', 'MARKER_NOT_FOUND', 'NAV_FAILED'])
def test_retryable_error_succeeds_on_second_attempt(step, code):
    runner, ex, goal, _ = make({step: [StepResult(False, code), OK]})
    res = run(runner, goal)
    assert res.success and ex.calls.count(step) == 2


@pytest.mark.parametrize('step', ['PICK', 'DROP', 'RETURN'])
def test_retries_are_bounded(step):
    runner, ex, goal, _ = make({step: [StepResult(False, 'STAGING_FAILED')]})
    res = run(runner, goal)
    assert not res.success and res.error_code == 'STAGING_FAILED' and res.failed_step == step
    assert ex.calls.count(step) == 2 and ex.stops == 1            # 1 try + 1 retry, then safe stop


@pytest.mark.parametrize('step', ['PICK', 'DROP'])
@pytest.mark.parametrize('code', ['LIFT_FAILED', 'ALIGN_FAILED', 'NOT_CARRYING', 'TIMEOUT',
                                  'MAP_UPDATE_FAILED', 'UNDOCK_FAILED'])
def test_other_errors_are_not_retried(step, code):
    runner, ex, goal, _ = make({step: [StepResult(False, code, 'boom')]})
    res = run(runner, goal)
    assert (res.error_code, res.failed_step, res.message) == (code, step, 'boom')
    assert ex.calls.count(step) == 1 and ex.stops == 1


def test_failure_stops_later_steps():
    runner, ex, goal, _ = make({'PICK': [StepResult(False, 'LIFT_FAILED')]})
    run(runner, goal)
    assert ex.calls == ['PICK']


@pytest.mark.parametrize('step', ['PICK', 'DROP', 'RETURN'])
def test_cancel_during_a_step(step):
    runner, ex, goal, _ = make({step: [StepResult(False, 'CANCELED')]})
    res = run(runner, goal)
    assert (res.error_code, res.failed_step) == ('CANCELED', step)
    assert ex.stops == 1 and ex.calls[-1] == step


@pytest.mark.parametrize('step', ['PICK', 'DROP', 'RETURN'])
def test_cancel_requested_between_steps(step):
    runner, ex, goal, _ = make({})
    order = ['PICK', 'DROP', 'RETURN']
    flag = {'c': False}
    ex.on_call = lambda call: flag.__setitem__(
        'c', call == order[order.index(step) - 1] if step != 'PICK' else False)
    res = run(runner, goal, cancelled=(lambda: True) if step == 'PICK' else (lambda: flag['c']))
    assert res.error_code == 'CANCELED' and res.failed_step == step
    assert step not in ex.calls and ex.stops == 1


def test_mission_timeout_binds_the_step_timeout():
    runner, ex, goal, clock = make({'PICK': [StepResult(False, 'TIMEOUT')]},
                                   policy=Policy(dock_timeout=180, mission_timeout=100))
    res = run(runner, goal)
    assert ex.last_timeout == 100                                 # min(180, remaining)
    assert res.error_code == 'MISSION_TIMEOUT' and res.failed_step == 'PICK'


def test_step_timeout_without_mission_limit_is_a_plain_timeout():
    runner, ex, goal, _ = make({'DROP': [StepResult(False, 'TIMEOUT')]})
    res = run(runner, goal)
    assert res.error_code == 'TIMEOUT' and res.failed_step == 'DROP'


def test_mission_time_exhausted_between_steps():
    clock = Clock()
    runner, ex, goal, _ = make({}, policy=Policy(mission_timeout=100), clock=clock)
    ex.on_call = lambda call: setattr(clock, 't', 150.0) if call == 'PICK' else None
    res = run(runner, goal)
    assert res.error_code == 'MISSION_TIMEOUT' and res.failed_step == 'DROP'
    assert ex.calls == ['PICK'] and ex.stops == 1


def test_navigation_timeout_uses_the_nav_limit():
    runner, ex, goal, _ = make({}, policy=Policy(nav_timeout=77, dock_timeout=180))
    run(runner, goal)
    assert ex.last_timeout == 77                                  # last call was RETURN


def test_reports_progress_states():
    seen = []
    ex = Mock({})
    runner = MissionRunner(ex, Policy(), lambda g: '', Clock(),
                           report=lambda state, i, n, attempt, msg: seen.append((state, i, n)))
    runner.run(MissionGoal('m', pick_dock_id='pick', drop_dock_id='drop', return_pose=(0, 0, 0)),
               lambda: False)
    assert [s[0] for s in seen] == ['VALIDATE', 'PICK', 'DROP', 'RETURN', 'DONE']
