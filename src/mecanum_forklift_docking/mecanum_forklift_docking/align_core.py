"""Holonomic final alignment (pure, no ROS): errors in the robot frame -> body velocity."""
import math
from dataclasses import dataclass


@dataclass
class AlignParams:
    """Tolerances, gains and limits of the final-alignment controller."""

    tol_long: float = 0.010      # m
    tol_lat: float = 0.010       # m
    tol_yaw: float = 0.035       # rad (2 deg)
    k_lin: float = 1.5           # 1/s
    k_yaw: float = 1.5           # 1/s
    max_vx: float = 0.08         # m/s
    max_vy: float = 0.08         # m/s
    max_wz: float = 0.25         # rad/s
    min_lin: float = 0.01        # m/s creep speed while outside tolerance
    min_yaw: float = 0.02        # rad/s creep speed while outside tolerance
    gate_lat: float = 0.03       # m: forward speed fades to 0 as lateral error nears this
    gate_yaw: float = 0.10       # rad: ... and as heading error nears this


def clamp(v, lim):
    """Clamp v to [-lim, lim]."""
    return max(-lim, min(lim, v))


def within_tolerance(ex, ey, eyaw, p):
    """Return True if the (target - robot) robot-frame errors are inside the tolerances."""
    return abs(ex) <= p.tol_long and abs(ey) <= p.tol_lat and abs(eyaw) <= p.tol_yaw


def _with_creep(v, err, tol, v_min, v_max):
    """Keep a small minimum speed toward the target until the error is inside the tolerance."""
    if abs(err) <= tol:
        return 0.0
    if abs(v) < v_min:
        v = math.copysign(v_min, err)
    return clamp(v, v_max)


def align_step(ex, ey, eyaw, p):
    """Return (vx, vy, wz) for robot-frame errors; sideways and yaw first, forward when aligned."""
    vy = _with_creep(p.k_lin * ey, ey, p.tol_lat * 0.5, p.min_lin, p.max_vy)
    wz = _with_creep(p.k_yaw * eyaw, eyaw, p.tol_yaw * 0.5, p.min_yaw, p.max_wz)
    fade = max(0.0, 1.0 - max(abs(ey) / p.gate_lat, abs(eyaw) / p.gate_yaw))
    vx = p.k_lin * ex * fade
    vx = _with_creep(vx, ex, p.tol_long * 0.5, p.min_lin * fade, p.max_vx) if fade > 0 else 0.0
    return vx, vy, wz


def robot_frame_errors(robot, target):
    """(ex, ey, eyaw) of target relative to robot; both are (x, y, yaw) in the same frame."""
    dx, dy = target[0] - robot[0], target[1] - robot[1]
    c, s = math.cos(robot[2]), math.sin(robot[2])
    eyaw = math.atan2(math.sin(target[2] - robot[2]), math.cos(target[2] - robot[2]))
    return c * dx + s * dy, -s * dx + c * dy, eyaw


def target_from_marker(mx, my, psi, stop_distance):
    """Return the final base_link pose: stop_distance before the marker along heading psi."""
    return mx - stop_distance * math.cos(psi), my - stop_distance * math.sin(psi), psi
