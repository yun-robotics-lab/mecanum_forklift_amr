"""
Mission state machine (no ROS): PICK -> DROP -> (RETURN) with timeouts, bounded retries and cancel.

The runner talks to the robot only through an executor with `dock`, `navigate` and `stop`, so every
transition can be tested with a scripted mock. Error codes: the DockPallet codes pass through;
the mission adds INVALID_MISSION, MISSION_TIMEOUT, CANCELED, NAV_FAILED and TIMEOUT.
"""
from dataclasses import dataclass


@dataclass
class Policy:
    """Timeouts (seconds, node clock) and retry rules."""

    nav_timeout: float = 150.0
    dock_timeout: float = 180.0
    mission_timeout: float = 600.0
    max_retries: int = 1                      # extra attempts per step for a retryable error
    retryable: frozenset = frozenset({'STAGING_FAILED', 'MARKER_NOT_FOUND', 'NAV_FAILED'})


@dataclass
class StepResult:
    """Outcome of one child action."""

    success: bool
    error_code: str = ''
    message: str = ''


@dataclass
class MissionGoal:
    """A PALLET_TRANSPORT mission."""

    mission_id: str = ''
    mission_type: str = 'PALLET_TRANSPORT'
    pick_dock_id: str = ''
    drop_dock_id: str = ''
    return_pose: object = None                # None = do not drive back; else passed to navigate


@dataclass
class MissionResult:
    """Terminal state of a mission."""

    success: bool
    error_code: str = ''
    failed_step: str = ''
    message: str = ''


class MissionRunner:
    """Run one mission against an executor."""

    def __init__(self, executor, policy, validate, clock, report=None):
        """Store the collaborators: executor, policy, validate(goal) -> str, clock() -> s."""
        self.executor = executor
        self.policy = policy
        self.validate = validate
        self.clock = clock
        self.report = report or (lambda *args, **kwargs: None)

    def steps(self, goal):
        """Ordered step names for a goal."""
        names = ['PICK', 'DROP']
        if goal.return_pose is not None:
            names.append('RETURN')
        return names

    def run(self, goal, cancelled):
        """Run the mission; `cancelled()` is polled before every attempt."""
        start = self.clock()
        self.report('VALIDATE', 0, 0, 1, '')
        problem = self.validate(goal)
        if problem:
            return MissionResult(False, 'INVALID_MISSION', 'VALIDATE', problem)
        names = self.steps(goal)
        for index, name in enumerate(names, start=1):
            result = self.run_step(goal, name, index, len(names), start, cancelled)
            if not result.success:
                self.executor.stop()
                self.report('CANCELED' if result.error_code == 'CANCELED' else 'FAILED',
                            index, len(names), 1, result.message)
                return MissionResult(False, result.error_code, name, result.message)
        self.report('DONE', len(names), len(names), 1, '')
        return MissionResult(True)

    def run_step(self, goal, name, index, total, start, cancelled):
        """One step with bounded retries; returns a StepResult."""
        limit = self.policy.nav_timeout if name == 'RETURN' else self.policy.dock_timeout
        attempt = 0
        while True:
            attempt += 1
            if cancelled():
                return StepResult(False, 'CANCELED', f'canceled before {name}')
            remaining = self.policy.mission_timeout - (self.clock() - start)
            if remaining <= 0:
                return StepResult(False, 'MISSION_TIMEOUT', 'mission time limit reached')
            self.report(name, index, total, attempt, '')
            timeout = min(limit, remaining)
            if name == 'PICK':
                result = self.executor.dock(goal.pick_dock_id, timeout, cancelled)
            elif name == 'DROP':
                result = self.executor.dock(goal.drop_dock_id, timeout, cancelled)
            else:
                result = self.executor.navigate(goal.return_pose, timeout, cancelled)
            if result.success:
                return result
            if result.error_code == 'TIMEOUT' and remaining <= limit:
                return StepResult(False, 'MISSION_TIMEOUT', f'{name}: mission time limit reached')
            if (result.error_code in self.policy.retryable
                    and attempt <= self.policy.max_retries):
                continue
            return result
