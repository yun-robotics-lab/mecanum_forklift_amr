"""
Lift state machine without ROS dependencies (unit-testable).

Codes mirror mecanum_forklift_interfaces (LiftState, SetLiftHeight); they are duplicated here
so the core has no ROS import. The node maps them; no test compares them with the .msg files.
"""
import math

UNINITIALIZED, HOMING, IDLE, MOVING, FAULT = 0, 1, 2, 3, 4

FAULT_NONE, FAULT_TIMEOUT, FAULT_OVER_LIMIT = 0, 1, 2
FAULT_NO_FEEDBACK, FAULT_STALL, FAULT_SAFETY_STOP = 3, 4, 5

OK = 0
REJECTED_NOT_IDLE, REJECTED_OUT_OF_RANGE = 1, 2


class LiftCore:
    """Ramps a position command toward a target and supervises the measured height."""

    def __init__(self, min_height=0.0, max_height=0.12, default_speed=0.05, max_speed=0.1,
                 tolerance=0.002, settle_time=0.3, timeout_margin=3.0, feedback_timeout=1.0,
                 stall_error=0.02, stall_time=1.5, limit_margin=0.01):
        self.min_height = min_height
        self.max_height = max_height
        self.default_speed = default_speed
        self.max_speed = max_speed
        self.tolerance = tolerance
        self.settle_time = settle_time
        self.timeout_margin = timeout_margin
        self.feedback_timeout = feedback_timeout
        self.stall_error = stall_error
        self.stall_time = stall_time
        self.limit_margin = limit_margin

        self.state = UNINITIALIZED
        self.fault = FAULT_NONE
        self.height = None          # last measured height
        self.last_feedback = None   # time of last measurement
        self.cmd = None             # ramped position command
        self.target = 0.0
        self.speed = default_speed
        self._t_start = 0.0
        self._deadline = 0.0
        self._settled_since = None
        self._stall_since = None
        self.last_step = None

    # ---- inputs ----
    def feedback(self, height, now):
        self.height = height
        self.last_feedback = now

    def _speed(self, speed):
        """Return the effective speed, or None if the request is invalid."""
        if not math.isfinite(speed) or speed < 0.0:
            return None
        if speed == 0.0:
            return self.default_speed
        return min(speed, self.max_speed)

    def _in_range(self, target):
        return math.isfinite(target) and self.min_height <= target <= self.max_height

    def _begin_move(self, target, speed, now):
        self.target = target
        self.speed = speed
        self._t_start = now
        self._deadline = now + abs(target - self.cmd) / speed + self.timeout_margin
        self._settled_since = None
        self._stall_since = None
        self.state = MOVING

    def set_goal(self, target, speed, now):
        """Start a move from an action goal. Returns OK or a REJECTED_* code."""
        if self.state != IDLE:
            return REJECTED_NOT_IDLE
        spd = self._speed(speed)
        if spd is None or not self._in_range(target):
            return REJECTED_OUT_OF_RANGE
        self._begin_move(target, spd, now)
        return OK

    def set_teleop_target(self, target, now):
        """Teleop target: clamped to limits, ignored in FAULT/startup, no-op if already there."""
        if self.state not in (IDLE, MOVING) or not math.isfinite(target):
            return False
        target = min(max(target, self.min_height), self.max_height)
        if self.state == IDLE and abs(target - self.cmd) <= 1e-6:
            return True
        self._begin_move(target, self.default_speed, now)
        return True

    def cancel(self):
        """Stop and hold the current position."""
        if self.state == MOVING:
            self.cmd = self.height
            self.target = self.height
            self.state = IDLE

    def safety_stop(self):
        self._fault(FAULT_SAFETY_STOP)

    def reset_fault(self, now):
        """FAULT -> IDLE holding the measured height; needs fresh feedback."""
        if self.state != FAULT or self.height is None or self.last_feedback is None:
            return False
        if now - self.last_feedback > self.feedback_timeout:
            return False
        self.fault = FAULT_NONE
        self.cmd = self.height
        self.target = self.height
        self.state = IDLE
        return True

    def _fault(self, code):
        if self.height is not None:
            self.cmd = self.height  # hold where it is
        self.fault = code
        self.state = FAULT

    # ---- periodic ----
    def step(self, now):
        """Advance the state machine; returns the position command to send (or None)."""
        dt = 0.0 if self.last_step is None else max(now - self.last_step, 0.0)
        self.last_step = now

        if self.state == FAULT:
            return self.cmd

        have_fb = self.last_feedback is not None
        if have_fb and now - self.last_feedback > self.feedback_timeout:
            self._fault(FAULT_NO_FEEDBACK)
            return self.cmd

        if (have_fb and self.state != UNINITIALIZED
                and not self.min_height - self.limit_margin <= self.height
                <= self.max_height + self.limit_margin):
            self._fault(FAULT_OVER_LIMIT)
            return self.cmd

        if self.state == UNINITIALIZED:
            if have_fb:
                self.cmd = self.height
                self.target = self.min_height
                self.speed = self.default_speed
                self._begin_move(self.min_height, self.default_speed, now)
                self.state = HOMING
            return self.cmd

        if self.state in (HOMING, MOVING):
            err_cmd = self.target - self.cmd
            max_dx = self.speed * dt
            self.cmd += math.copysign(min(abs(err_cmd), max_dx), err_cmd)

            if abs(self.cmd - self.height) > self.stall_error:
                if self._stall_since is None:
                    self._stall_since = now
                elif now - self._stall_since > self.stall_time:
                    self._fault(FAULT_STALL)
                    return self.cmd
            else:
                self._stall_since = None

            if now > self._deadline:
                self._fault(FAULT_TIMEOUT)
                return self.cmd

            reached = (abs(self.cmd - self.target) < 1e-9
                       and abs(self.height - self.target) <= self.tolerance)
            if reached:
                if self._settled_since is None:
                    self._settled_since = now
                elif now - self._settled_since >= self.settle_time:
                    self.state = IDLE
            else:
                self._settled_since = None
        return self.cmd
