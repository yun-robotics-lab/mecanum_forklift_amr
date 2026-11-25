import math

from mecanum_forklift_control.kinematics import (
    forward_kinematics, integrate_pose, inverse_kinematics)

R, LX, LY = 0.039, 0.11, 0.1305


def test_round_trip():
    for twist in [(0.3, 0.0, 0.0), (0.0, 0.2, 0.0), (0.0, 0.0, 1.0), (0.2, -0.1, 0.5)]:
        wheels = inverse_kinematics(*twist, R, LX, LY)
        out = forward_kinematics(wheels, R, LX, LY)
        assert all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(twist, out))


def test_wheel_patterns():
    fl, fr, rl, rr = inverse_kinematics(0.0, 0.1, 0.0, R, LX, LY)   # strafe left
    assert fl < 0 < fr and rl > 0 > rr and math.isclose(fl, -fr) and math.isclose(rl, -rr)
    fl, fr, rl, rr = inverse_kinematics(0.0, 0.0, 0.5, R, LX, LY)   # rotate CCW
    assert fl < 0 < fr and rl < 0 < rr


def test_integrate_pose_turning():
    x, y, yaw = integrate_pose(0.0, 0.0, math.pi / 2, 1.0, 0.0, 0.0)
    assert math.isclose(x, 0.0, abs_tol=1e-9) and math.isclose(y, 1.0)
    assert math.isclose(yaw, math.pi / 2)
    assert math.isclose(integrate_pose(0, 0, math.pi, 0, 0, 0.1)[2], -math.pi + 0.1)
