import math

import pytest

from mecanum_forklift_docking.align_core import (
    AlignParams, align_step, robot_frame_errors, target_from_marker, within_tolerance)


def simulate(start, target, p, dt=0.05, t_max=40.0):
    """Integrate a perfect holonomic robot; return (final errors, time, overshoot past target)."""
    x, y, yaw = start
    t = 0.0
    dwell = 0.0
    peak = 0.0
    while t < t_max:
        ex, ey, eyaw = robot_frame_errors((x, y, yaw), target)
        peak = max(peak, -ex)            # how far the robot went past the target
        if within_tolerance(ex, ey, eyaw, p):
            dwell += dt
            if dwell >= 0.5:
                return (ex, ey, eyaw), t, peak
        else:
            dwell = 0.0
        vx, vy, wz = align_step(ex, ey, eyaw, p)
        c, s = math.cos(yaw), math.sin(yaw)
        x += (c * vx - s * vy) * dt
        y += (s * vx + c * vy) * dt
        yaw += wz * dt
        t += dt
    return robot_frame_errors((x, y, yaw), target), t, peak


@pytest.mark.parametrize('dy', [0.0, 0.05, -0.05, 0.08])
@pytest.mark.parametrize('dyaw_deg', [0.0, 10.0, -10.0, 15.0])
@pytest.mark.parametrize('back', [0.15, 0.05])
def test_converges_within_tolerance(dy, dyaw_deg, back):
    p = AlignParams()
    target = (0.0, 0.0, 0.0)
    start = (-back, dy, math.radians(dyaw_deg))
    (ex, ey, eyaw), t, _ = simulate(start, target, p)
    assert abs(ex) <= p.tol_long and abs(ey) <= p.tol_lat and abs(eyaw) <= p.tol_yaw
    assert t < 30.0


def test_overshoot_is_small():
    p = AlignParams()
    _, _, peak = simulate((-0.15, 0.05, math.radians(10)), (0.0, 0.0, 0.0), p)
    assert peak < 0.02


def test_limits_and_zero_inside_tolerance():
    p = AlignParams()
    vx, vy, wz = align_step(5.0, 5.0, 3.0, p)
    assert abs(vx) <= p.max_vx and abs(vy) <= p.max_vy and abs(wz) <= p.max_wz
    assert align_step(0.002, 0.002, 0.005, p) == (0.0, 0.0, 0.0)


def test_forward_waits_for_alignment():
    p = AlignParams()
    vx_aligned, _, _ = align_step(0.2, 0.0, 0.0, p)
    vx_off, _, _ = align_step(0.2, 0.05, 0.0, p)      # lateral error above the gate
    assert vx_aligned > 0.0 and vx_off == 0.0


def test_target_from_marker():
    tx, ty, tyaw = target_from_marker(0.81, 0.0, 0.0, 0.278)
    assert (round(tx, 3), ty, tyaw) == (0.532, 0.0, 0.0)
    tx, ty, _ = target_from_marker(0.0, 1.0, math.pi / 2, 0.3)
    assert abs(tx) < 1e-9 and abs(ty - 0.7) < 1e-9


def test_robot_frame_errors_rotated():
    ex, ey, eyaw = robot_frame_errors((0, 0, math.pi / 2), (0.0, 1.0, math.pi / 2))
    assert abs(ex - 1.0) < 1e-9 and abs(ey) < 1e-9 and abs(eyaw) < 1e-9
