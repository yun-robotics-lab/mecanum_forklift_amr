"""
Mecanum (X roller pattern) kinematics, body frame x forward, y left, z up.

Wheel order everywhere: (front_left, front_right, rear_left, rear_right).
A positive wheel rate means that wheel drives the robot forward (+x).
"""
import math


def inverse_kinematics(vx, vy, wz, radius, half_length, half_width):
    """Body twist (m/s, m/s, rad/s) -> wheel rates (rad/s)."""
    k = half_length + half_width
    return (
        (vx - vy - k * wz) / radius,
        (vx + vy + k * wz) / radius,
        (vx + vy - k * wz) / radius,
        (vx - vy + k * wz) / radius,
    )


def forward_kinematics(wheels, radius, half_length, half_width):
    """Wheel values (rates or increments) -> body (x, y, yaw) values."""
    fl, fr, rl, rr = wheels
    k = half_length + half_width
    return (
        radius / 4.0 * (fl + fr + rl + rr),
        radius / 4.0 * (-fl + fr + rl - rr),
        radius / (4.0 * k) * (-fl + fr - rl + rr),
    )


def integrate_pose(x, y, yaw, dx, dy, dyaw):
    """Apply a body-frame displacement (dx, dy, dyaw) to a pose, midpoint heading."""
    mid = yaw + dyaw / 2.0
    c, s = math.cos(mid), math.sin(mid)
    return (x + c * dx - s * dy,
            y + s * dx + c * dy,
            math.atan2(math.sin(yaw + dyaw), math.cos(yaw + dyaw)))
