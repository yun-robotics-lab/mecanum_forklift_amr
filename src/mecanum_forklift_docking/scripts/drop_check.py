#!/usr/bin/env python3
"""
Pick-then-drop check in Gazebo (development tool, not installed).

Usage: drop_check.py [trials]. Needs the simulation, navigation, vision and docking launches.
Each trial: place the pallet, DockPallet pick at pallet_pick_1, DockPallet drop at pallet_drop_1
(both with staging navigation and back-out), then print where pallet_1 stands relative to the pad
centre (2.7, -1.6), its tilt and which other objects moved.
"""
import math
import sys
import time

from dock_trials import pallet_state, place_pallet, world_objects
from mecanum_forklift_interfaces.action import DockPallet
from pick_trials import OURS, Pick
import rclpy

PAD = (2.7, -1.6)
START = (0.0, -1.0, 0)


def dock(node, dock_id):
    """Run one DockPallet goal (staging + back-out); return (success, error_code, seconds)."""
    node.dock.wait_for_server()
    goal = DockPallet.Goal()
    goal.dock_id = dock_id
    goal.navigate_to_staging = True
    goal.undock_after = True
    sent = node.dock.send_goal_async(goal)
    while not sent.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    res = sent.result().get_result_async()
    t0 = time.time()
    while not res.done() and time.time() - t0 < 400:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not res.done():
        return False, 'HANG', time.time() - t0
    result = res.result().result
    return result.success, result.error_code, time.time() - t0


def main():
    """Run the trials."""
    trials = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    rclpy.init()
    node = Pick()
    while node.gt is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    for k in range(trials):
        node.fork(0.0)
        node.set_unloaded_footprint()
        place_pallet(0.0, 0.0)
        node.restore_full_map()
        print(f'trial {k + 1}: reach start', node.navigate(*START), flush=True)
        before = world_objects()
        ok, code, secs = dock(node, 'pallet_pick_1')
        print(f'  pick: {ok} {code} {secs:.0f} s', flush=True)
        if not ok:
            continue
        ok, code, secs = dock(node, 'pallet_drop_1')
        print(f'  drop: {ok} {code} {secs:.0f} s', flush=True)
        node.spin(2.0)
        pal = pallet_state()
        after = world_objects()
        pushed = [n for n, a in after.items() if n in before and n not in OURS
                  and math.dist(a, before[n]) > 0.005]
        if pal:
            print(f'  pallet at ({pal[0]:.3f}, {pal[1]:.3f}) z {pal[2] * 1000:.1f} mm, '
                  f'{math.hypot(pal[0] - PAD[0], pal[1] - PAD[1]) * 100:.1f} cm from the pad '
                  f'centre, tilt {pal[3]:.2f} deg, pushed {pushed or "-"}', flush=True)
    node.fork(0.0)
    node.set_unloaded_footprint()
    place_pallet(0.0, 0.0)
    node.restore_full_map()


if __name__ == '__main__':
    main()
