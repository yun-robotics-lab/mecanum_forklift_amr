#!/usr/bin/env python3
"""
Mission failure-injection tests in Gazebo (development tool, not installed).

Usage: mission_failures.py [case numbers, comma separated]. Needs the simulation, navigation,
vision, docking and mission launches. Every case checks the mission result code and failed step,
that the robot is stopped afterwards and that no child goal (dock_pallet, navigate_to_pose) is
still active.
"""
import math
import subprocess
import sys
import time

from mission_common import NS, pallet_state, remove_box, Rig, set_pose, spawn_box
import rclpy


def set_param(name, value):
    """Set a mission_manager parameter."""
    subprocess.run(['ros2', 'param', 'set', f'{NS}/mission_manager', name, str(value)], check=True,
                   capture_output=True)


def wait_state(rig, log, state, timeout=300):
    """Spin until the feedback log contains `state`."""
    t0 = time.time()
    while not any(s == state for s, _, _ in log) and time.time() - t0 < timeout:
        rig.spin(0.2)
    return any(s == state for s, _, _ in log)


def check(rig, name, res, code, step, extra_ok=True, note=''):
    """Print one result line and return True on PASS."""
    got = 'HANG' if res is None else (res.error_code or 'SUCCESS')
    got_step = '' if res is None else res.failed_step
    stopped = rig.stopped()
    orphans = rig.orphan_goals()
    codes = (code,) if isinstance(code, str) else tuple(code)
    steps = (step,) if isinstance(step, str) else tuple(step)
    ok = got in codes and got_step in steps and stopped and not orphans and extra_ok
    want = f'{"|".join(codes)}/{"|".join(steps)}'
    print(f'{name:30s} expect {want:30s} got {got:16s}/{got_step:8s} '
          f'stopped {stopped!s:5s} orphans {",".join(orphans) or "-":8s} '
          f'{"PASS" if ok else "FAIL"} {note}', flush=True)
    return ok


def case_invalid(rig):
    """Unknown dock; a drop dock used as the pick dock."""
    rig.reset_world()
    x0, y0, _ = rig.pose()
    results = []
    for name, pick, drop in (('invalid: unknown dock', 'no_such_dock', 'pallet_drop_1'),
                             ('invalid: drop dock as pick', 'pallet_drop_1', 'pallet_drop_1')):
        handle, _ = rig.send_mission(pick=pick, drop=drop)
        res = rig.finish(handle, 60) if handle else None
        x1, y1, _ = rig.pose()
        results.append(check(rig, name, res, 'INVALID_MISSION', 'VALIDATE',
                             math.hypot(x1 - x0, y1 - y0) < 0.005))
    return all(results)


def case_hidden_marker(rig):
    """Marker not visible: one retry, then MARKER_NOT_FOUND."""
    rig.reset_world()
    set_pose('box_1', 5.0, 5.0, 0.056, 0.0)   # marker (joined to the box) not visible
    rig.spin(1.0)
    handle, log = rig.send_mission()
    res = rig.finish(handle, 600)
    tries = sum(1 for s, _, _ in log if s == 'PICK')
    return check(rig, 'hidden marker (1 retry)', res, 'MARKER_NOT_FOUND', 'PICK', tries == 2,
                 f'PICK attempts {tries}')


def case_blocked_staging(rig):
    """Box on the pick staging pose: Nav2 aborts (retry, STAGING_FAILED) or the dock times out."""
    rig.reset_world()
    spawn_box('mission_test_box', 0.165, 0.0, 0.5, 0.5)
    rig.spin(3.0)
    set_param('dock_timeout', 60.0)
    handle, _ = rig.send_mission()
    res = rig.finish(handle, 600)
    set_param('dock_timeout', 180.0)
    remove_box('mission_test_box')
    return check(rig, 'blocked staging pose', res, ('STAGING_FAILED', 'TIMEOUT'), 'PICK')


def case_lift_fault(rig):
    """Pick dock whose lift target is above the limit."""
    rig.reset_world()
    handle, _ = rig.send_mission(pick='test_lift_out_of_range')
    res = rig.finish(handle, 600)
    return check(rig, 'lift fault (no retry)', res, 'LIFT_FAILED', 'PICK')


def case_cancel(rig, state, delay):
    """Cancel `delay` seconds after the mission reaches `state`."""
    rig.reset_world()
    handle, log = rig.send_mission()
    if not wait_state(rig, log, state):
        print(f'cancel at {state}: state never reached', flush=True)
        return False
    rig.spin(delay)
    handle.cancel_goal_async()
    res = rig.finish(handle, 120)
    extra = True
    note = ''
    if state in ('DROP', 'RETURN'):
        pal = pallet_state()
        extra = pal is not None and (state == 'RETURN' or pal[2] > 0.02)
        note = f'pallet z {pal[2] * 1000:.0f} mm' if pal else ''
    return check(rig, f'cancel during {state}', res, 'CANCELED', state, extra, note)


def case_busy(rig):
    """Reject a second mission while one is active (BUSY); the first can then be cancelled."""
    rig.reset_world()
    first, log = rig.send_mission(mission_id='first')
    wait_state(rig, log, 'PICK')
    rig.spin(3.0)
    second, _ = rig.send_mission(mission_id='second')
    res2 = rig.finish(second, 30) if second else None
    first.cancel_goal_async()
    res1 = rig.finish(first, 120)
    ok = (res2 is not None and res2.error_code == 'BUSY' and not res2.success
          and res1 is not None and res1.error_code == 'CANCELED')
    stopped = rig.stopped()
    orphans = rig.orphan_goals()
    print(f'{"double submit":30s} second -> {res2.error_code if res2 else None}, first -> '
          f'{res1.error_code if res1 else None} stopped {stopped} orphans {orphans or "-"} '
          f'{"PASS" if ok and stopped and not orphans else "FAIL"}', flush=True)
    return ok and stopped and not orphans


def case_mission_timeout(rig):
    """End the mission with a 45 s limit of sim time (a mission takes ~50-65 s)."""
    rig.reset_world()
    set_param('mission_timeout', 45.0)
    handle, _ = rig.send_mission()
    res = rig.finish(handle, 600)
    set_param('mission_timeout', 600.0)
    return check(rig, 'mission timeout (45 s)', res, 'MISSION_TIMEOUT',
                 ('PICK', 'DROP', 'RETURN'))


def case_restart(rig):
    """Kill the mission node (SIGINT) mid-pick: its child goal is cancelled, no orphan."""
    rig.reset_world()
    handle, log = rig.send_mission()
    wait_state(rig, log, 'PICK')
    rig.spin(12.0)
    subprocess.run(['pkill', '-INT', '-f', 'lib/mecanum_forklift_task_manager/mission_manager'],
                   check=False)
    rig.spin(8.0)
    stopped = rig.stopped()
    orphans = rig.orphan_goals()
    ok = stopped and not orphans
    print(f'{"node restart mid-mission":30s} stopped {stopped} orphans {orphans or "-"} '
          f'{"PASS" if ok else "FAIL"}', flush=True)
    subprocess.run(['pkill', '-f', 'mission.launch.py'], check=False)
    return ok


CASES = {1: case_invalid, 2: case_hidden_marker, 3: case_blocked_staging, 4: case_lift_fault,
         5: lambda r: case_cancel(r, 'PICK', 8.0), 6: lambda r: case_cancel(r, 'DROP', 10.0),
         7: lambda r: case_cancel(r, 'RETURN', 6.0), 8: case_busy, 9: case_mission_timeout,
         10: case_restart}


def main():
    """Run the selected cases."""
    wanted = [int(c) for c in sys.argv[1].split(',')] if len(sys.argv) > 1 else list(CASES)
    rclpy.init()
    rig = Rig()
    while rig.gt is None:
        rclpy.spin_once(rig, timeout_sec=0.1)
    results = {}
    for number in wanted:
        results[number] = CASES[number](rig)
    print(f'passed {sum(results.values())}/{len(results)}: '
          f'{ {k: ("PASS" if v else "FAIL") for k, v in results.items()} }')
    rig.reset_world(navigate=False)


if __name__ == '__main__':
    main()
