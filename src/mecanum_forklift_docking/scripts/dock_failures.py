#!/usr/bin/env python3
"""
DockPallet failure-injection tests in Gazebo (development tool, not installed).

Needs the simulation, vision and docking launches running (and the test-only dock entry
`test_lift_out_of_range` of dock_database.yaml). Each case ends with a check that the robot is
stopped (ground-truth displacement over 2 s).
"""
import math
import subprocess
import time

from dock_trials import place_pallet, set_pose, Trials
from mecanum_forklift_interfaces.action import DockPallet
import rclpy


def send(node, dock_id, undock=False):
    """Send a goal; return the goal handle future result (handle)."""
    node.dock.wait_for_server()
    goal = DockPallet.Goal()
    goal.dock_id = dock_id
    goal.navigate_to_staging = False
    goal.undock_after = undock
    sent = node.dock.send_goal_async(goal)
    while not sent.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    return sent.result()


def finish(node, handle, timeout=200):
    """Wait for the result of a goal handle."""
    res = handle.get_result_async()
    t0 = time.time()
    while not res.done() and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    return res.result().result if res.done() else None


def moved(node, seconds=2.0):
    """Ground-truth displacement (m) over a wall-clock interval."""
    node.spin(0.2)
    x0, y0, _ = node.pose()
    node.spin(seconds)
    x1, y1, _ = node.pose()
    return math.hypot(x1 - x0, y1 - y0)


def set_param(name, value):
    """Set a workflow parameter through the CLI."""
    subprocess.run(['ros2', 'param', 'set', '/pallet_dock_workflow', name, str(value)],
                   check=True, capture_output=True)


def reset(node):
    """Fork down, robot at the start pose, pallet and marker back in place."""
    node.fork(0.0)
    node.goto(0.0, 0.0, 0.0)
    place_pallet(0.0, 0.0)
    node.spin(2.0)


def report(name, res, expect, node, extra=''):
    """Print one line: expected vs actual error code and whether the robot is stopped."""
    code = 'none (rejected/aborted)' if res is None else (res.error_code or 'SUCCESS')
    stopped = moved(node) < 0.002
    ok = code == expect and stopped
    print(f'{name:34s} expect {expect:12s} got {code:22s} stopped {stopped!s:5s} '
          f'{"PASS" if ok else "FAIL"} {extra}', flush=True)
    return ok


def main():
    """Run the failure matrix."""
    rclpy.init()
    node = Trials()
    while node.gt is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    passed = []

    reset(node)
    x0 = node.pose()[0]
    handle = send(node, 'does_not_exist')
    passed.append(report('unknown dock id', finish(node, handle), 'DOCK_NOT_IN_DB', node,
                         f'(moved {abs(node.pose()[0] - x0) * 1000:.1f} mm)'))

    reset(node)
    set_pose('box_1', 5.0, 5.0, 0.056, 0.0)        # marker not visible
    node.spin(1.0)
    handle = send(node, 'pallet_pick_1')
    passed.append(report('marker not visible', finish(node, handle), 'MARKER_NOT_FOUND', node))

    reset(node)
    handle = send(node, 'pallet_pick_1')
    node.spin(7.0)
    handle.cancel_goal_async()
    res = finish(node, handle)
    passed.append(report('cancel mid-dock', res, 'CANCELED', node))

    reset(node)
    set_param('total_timeout', 6.0)
    handle = send(node, 'pallet_pick_1')
    passed.append(report('total timeout (6 s)', finish(node, handle), 'TIMEOUT', node))
    set_param('total_timeout', 90.0)

    reset(node)
    handle = send(node, 'test_lift_out_of_range')
    passed.append(report('lift above limit', finish(node, handle), 'LIFT_FAILED', node))

    reset(node)
    handle = send(node, 'pallet_pick_1')
    second = send(node, 'pallet_pick_1')
    rejected = not second.accepted
    res = finish(node, handle)
    print(f'{"second goal while docking":34s} rejected {rejected} first result '
          f'{"SUCCESS" if res is not None and res.success else "other"} '
          f'{"PASS" if rejected and res is not None and res.success else "FAIL"}', flush=True)
    passed.append(rejected and res is not None and res.success)

    reset(node)
    handle = send(node, 'pallet_pick_1', undock=True)
    res = finish(node, handle)
    x, _, _ = node.pose()
    print(f'{"dock + lift + undock":34s} result '
          f'{"SUCCESS" if res is not None and res.success else (res.error_code if res else None)} '
          f'final x {x:.3f} m (undock backs up ~0.20-0.25 m from the docked pose)', flush=True)
    passed.append(res is not None and res.success)

    print(f'passed {sum(bool(p) for p in passed)}/{len(passed)}')
    reset(node)


if __name__ == '__main__':
    main()
