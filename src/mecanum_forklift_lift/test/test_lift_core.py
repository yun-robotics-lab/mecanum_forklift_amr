from mecanum_forklift_lift import lift_core as lc


class Sim:
    """Perfect follower of the command with optional frozen height (stall) and no feedback."""

    def __init__(self, h0=0.0, **kw):
        self.core = lc.LiftCore(**kw)
        self.t = 0.0
        self.h = h0
        self.follow = True
        self.feedback_on = True

    def run(self, seconds, dt=0.02):
        n = int(round(seconds / dt))
        for _ in range(n):
            self.t += dt
            if self.feedback_on:
                self.core.feedback(self.h, self.t)
            cmd = self.core.step(self.t)
            if self.follow and cmd is not None:
                self.h = cmd


def idle_sim(**kw):
    s = Sim(**kw)
    s.run(1.0)
    assert s.core.state == lc.IDLE
    return s


def test_homing_reaches_idle():
    s = Sim(h0=-0.001)
    s.run(1.0)
    assert s.core.state == lc.IDLE


def test_startup_waits_for_feedback():
    s = Sim()
    s.feedback_on = False
    s.run(3.0)
    assert s.core.state == lc.UNINITIALIZED


def test_move_succeeds_within_tolerance():
    s = idle_sim()
    assert s.core.set_goal(0.1, 0.0, s.t) == lc.OK
    s.run(5.0)
    assert s.core.state == lc.IDLE
    assert abs(s.h - 0.1) <= 0.002


def test_rejects_out_of_range_and_bad_speed():
    s = idle_sim()
    assert s.core.set_goal(0.2, 0.0, s.t) == lc.REJECTED_OUT_OF_RANGE
    assert s.core.set_goal(-0.01, 0.0, s.t) == lc.REJECTED_OUT_OF_RANGE
    assert s.core.set_goal(float('nan'), 0.0, s.t) == lc.REJECTED_OUT_OF_RANGE
    assert s.core.set_goal(0.05, -1.0, s.t) == lc.REJECTED_OUT_OF_RANGE
    assert s.core.state == lc.IDLE


def test_rejects_when_not_idle():
    s = idle_sim()
    assert s.core.set_goal(0.1, 0.0, s.t) == lc.OK
    s.run(0.2)
    assert s.core.set_goal(0.05, 0.0, s.t) == lc.REJECTED_NOT_IDLE


def test_speed_is_clamped():
    s = idle_sim()
    s.core.set_goal(0.12, 5.0, s.t)
    assert s.core.speed == s.core.max_speed


def test_command_never_leaves_limits():
    s = idle_sim()
    s.core.set_teleop_target(5.0, s.t)
    s.run(10.0)
    assert s.core.cmd <= 0.12 + 1e-9
    s.core.set_teleop_target(-5.0, s.t)
    s.run(10.0)
    assert s.core.cmd >= -1e-9


def test_cancel_holds_position():
    s = idle_sim()
    s.core.set_goal(0.12, 0.0, s.t)
    s.run(0.5)
    s.core.cancel()
    h = s.h
    s.run(2.0)
    assert s.core.state == lc.IDLE
    assert abs(s.h - h) <= 0.0015  # one control step of lag


def test_feedback_loss_faults_and_reset_needs_feedback():
    s = idle_sim()
    s.core.set_goal(0.1, 0.0, s.t)
    s.feedback_on = False
    s.run(1.5)
    assert s.core.state == lc.FAULT and s.core.fault == lc.FAULT_NO_FEEDBACK
    assert not s.core.reset_fault(s.t)
    s.feedback_on = True
    s.run(0.1)
    assert s.core.reset_fault(s.t)
    assert s.core.state == lc.IDLE


def test_stall_fault():
    s = idle_sim()
    s.core.set_goal(0.12, 0.0, s.t)
    s.follow = False
    s.run(4.0)
    assert s.core.state == lc.FAULT and s.core.fault == lc.FAULT_STALL


def test_timeout_fault_when_height_stays_just_short():
    s = idle_sim(stall_error=1.0)  # disable stall so the timeout is what triggers
    s.core.set_goal(0.06, 0.0, s.t)
    s.follow = False
    s.run(8.0)
    assert s.core.state == lc.FAULT and s.core.fault == lc.FAULT_TIMEOUT


def test_over_limit_fault():
    s = idle_sim()
    s.h = 0.2
    s.run(0.1)
    assert s.core.state == lc.FAULT and s.core.fault == lc.FAULT_OVER_LIMIT


def test_fault_ignores_goals_and_teleop():
    s = idle_sim()
    s.core.safety_stop()
    assert s.core.set_goal(0.05, 0.0, s.t) == lc.REJECTED_NOT_IDLE
    assert not s.core.set_teleop_target(0.05, s.t)
