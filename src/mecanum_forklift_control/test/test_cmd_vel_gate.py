import math

from mecanum_forklift_control.cmd_vel_gate import GateCore


def make():
    return GateCore(timeout=0.5, max_vx=0.6, max_vy=0.6, max_wz=2.0)


def test_zero_before_first_command():
    assert make().output(100.0) == (0.0, 0.0, 0.0)


def test_forward_then_timeout():
    g = make()
    assert g.command(0.3, 0.1, 0.5, now=10.0)
    assert g.output(10.4) == (0.3, 0.1, 0.5)
    assert g.output(10.6) == (0.0, 0.0, 0.0)       # older than 0.5 s
    assert g.command(0.2, 0.0, 0.0, now=11.0)       # recovers on a fresh command
    assert g.output(11.1) == (0.2, 0.0, 0.0)


def test_clamp_and_reject_nonfinite():
    g = make()
    g.command(5.0, -5.0, 9.0, now=0.0)
    assert g.output(0.1) == (0.6, -0.6, 2.0)
    assert not g.command(math.nan, 0.0, 0.0, now=0.2)
    assert g.last_rx == 0.0                         # rejected command does not refresh the timer
    assert not g.command(0.0, math.inf, 0.0, now=0.2)
