#!/usr/bin/env python3
"""
Full PALLET_TRANSPORT missions in Gazebo (development tool, not installed).

Usage: mission_trials.py [missions] [first]. Needs the simulation, navigation, vision, docking and
mission launches. Each mission: reset the world, drive to the start pose, send
pick pallet_pick_1 -> drop pallet_drop_1 -> return to (0, 0, 0), then measure.
Success = mission success, pallet fully on the pad (centre within 8 cm, tilt <= 2 deg), robot
within 7 cm / 5 deg of the return pose, no other object pushed, no child goal left active.
"""
import math
import sys
import time

from mission_common import OURS, pallet_state, PAD, Rig, world_objects
import rclpy


def main():
    """Run the missions."""
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    first = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    rclpy.init()
    rig = Rig()
    while rig.gt is None:
        rclpy.spin_once(rig, timeout_sec=0.1)
    print(f"{'#':>2s} {'result':24s} {'pad cm':>7s} {'tilt':>5s} {'ret cm':>7s} {'ret deg':>8s} "
          f"{'pushed':10s} {'orphans':8s} {'states':32s} {'s':>5s}", flush=True)
    ok = 0
    for k in range(first, first + count):
        start = rig.reset_world()
        if start != 'SUCCEEDED':
            print(f'{k + 1:2d} cannot reach the start pose: {start}', flush=True)
            continue
        before = world_objects()
        t0 = time.time()
        handle, log = rig.send_mission(mission_id=f'm{k + 1}')
        res = rig.finish(handle) if handle else None
        secs = time.time() - t0
        rig.spin(2.0)
        pal = pallet_state() or (0, 0, 0, float('nan'))
        pad_cm = math.hypot(pal[0] - PAD[0], pal[1] - PAD[1]) * 100
        x, y, yaw = rig.pose()
        ret_cm = math.hypot(x, y) * 100
        ret_deg = math.degrees(yaw)
        after = world_objects()
        pushed = [n for n, a in after.items() if n in before and n not in OURS
                  and math.dist(a, before[n]) > 0.005]
        orphans = rig.orphan_goals()
        good = (res is not None and res.success and pad_cm <= 8.0 and pal[3] <= 2.0
                and ret_cm <= 7.0 and abs(ret_deg) <= 5.0 and not pushed and not orphans)
        ok += good
        label = 'OK' if good else ('FAIL ' + (res.error_code if res else 'HANG'))
        states = '>'.join(dict.fromkeys(s for s, _, _ in log))
        print(f'{k + 1:2d} {label:24s} {pad_cm:7.1f} {pal[3]:5.2f} {ret_cm:7.1f} {ret_deg:8.1f} '
              f'{",".join(pushed) or "-":10s} {",".join(orphans) or "-":8s} {states:32s} '
              f'{secs:5.0f}', flush=True)
    print(f'success {ok}/{count} (missions {first + 1}-{first + count})')
    rig.reset_world(navigate=False)


if __name__ == '__main__':
    main()
