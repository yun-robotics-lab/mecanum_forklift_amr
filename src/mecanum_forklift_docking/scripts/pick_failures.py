#!/usr/bin/env python3
"""
Failure-injection tests for the integrated pick (DockPallet with staging navigation).

Needs the simulation, navigation, vision and docking launches running. Cases:
  1 cancel during the staging navigation
  2 staging pose blocked by a tall box (Nav2 does not give up by itself: the workflow's
    total_timeout ends the goal)
  3 marker not visible at the staging pose
Each case checks the error code and that the robot is stopped afterwards.
"""
import math
import subprocess
import time

from dock_trials import place_pallet, remove_box, set_pose, spawn_box
from mecanum_forklift_interfaces.action import DockPallet
from pick_trials import Pick
import rclpy

START = (0.0, -1.0, 0)


def send(node, dock_id='pallet_pick_1'):
    """Send a DockPallet goal with staging navigation; return the goal handle."""
    node.dock.wait_for_server()
    goal = DockPallet.Goal()
    goal.dock_id = dock_id
    goal.navigate_to_staging = True
    goal.undock_after = True
    sent = node.dock.send_goal_async(goal)
    while not sent.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    return sent.result()


def finish(node, handle, timeout=400):
    """Wait for the goal result."""
    res = handle.get_result_async()
    t0 = time.time()
    while not res.done() and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    return res.result().result if res.done() else None


def moved(node, seconds=2.0):
    """Ground-truth displacement over an interval, after 1 s for the robot to decelerate."""
    node.spin(1.0)
    x0, y0, _ = node.pose()
    node.spin(seconds)
    x1, y1, _ = node.pose()
    return math.hypot(x1 - x0, y1 - y0)


def set_param(name, value):
    """Set a workflow parameter."""
    subprocess.run(['ros2', 'param', 'set', '/pallet_dock_workflow', name, str(value)],
                   check=True, capture_output=True)


def report(name, res, expected, node):
    """Print one result line."""
    code = 'none' if res is None else (res.error_code or 'SUCCESS')
    stopped = moved(node) < 0.002
    ok = code in expected and stopped
    print(f'{name:34s} expect {"/".join(expected):24s} got {code:16s} stopped {stopped!s:5s} '
          f'{"PASS" if ok else "FAIL"}', flush=True)
    return ok


def reset(node):
    """Fork down, unloaded footprint, full map, pallet in place, robot at the start pose."""
    node.fork(0.0)
    node.set_unloaded_footprint()
    place_pallet(0.0, 0.0)
    node.restore_full_map()
    node.navigate(*START)


def main():
    """Run the matrix."""
    rclpy.init()
    node = Pick()
    while node.gt is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    results = []

    reset(node)
    handle = send(node)
    node.spin(6.0)                                  # robot is on its way to the staging pose
    handle.cancel_goal_async()
    results.append(report('cancel during staging', finish(node, handle), ['CANCELED'], node))

    reset(node)
    spawn_box('test_staging_box', 0.165, 0.0, 0.5, 0.5)          # on the staging pose
    node.spin(3.0)
    set_param('total_timeout', 40.0)
    handle = send(node)
    results.append(report('staging pose blocked', finish(node, handle),
                          ['STAGING_FAILED', 'TIMEOUT'], node))
    set_param('total_timeout', 180.0)
    remove_box('test_staging_box')
    node.spin(2.0)

    reset(node)
    set_pose('box_1', 5.0, 5.0, 0.056, 0.0)          # marker not visible
    node.spin(1.0)
    handle = send(node)
    results.append(report('marker not visible at staging', finish(node, handle),
                          ['MARKER_NOT_FOUND'], node))

    print(f'passed {sum(bool(r) for r in results)}/{len(results)}')
    reset(node)


if __name__ == '__main__':
    main()
