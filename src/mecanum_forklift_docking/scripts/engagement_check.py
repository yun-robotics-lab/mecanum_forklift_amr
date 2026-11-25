#!/usr/bin/env python3
"""
Pallet engagement / undock check in Gazebo (development tool, not installed).

Runs one DockPallet (dock, lift, undock) from the start pose and prints, every 2 s, the robot
pose, fork height and the pallet pose (position, tilt from vertical). Needs the simulation,
vision and docking launches running.
"""
import math
import time

from dock_trials import pallet_state, place_pallet, Trials
from mecanum_forklift_interfaces.action import DockPallet
import rclpy


def main():
    """Dock with undock and log the sequence."""
    rclpy.init()
    node = Trials()
    while node.gt is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.fork(0.0)
    node.goto(0.0, 0.0, 0.0)
    place_pallet(0.0, 0.0)
    node.spin(2.0)
    node.dock.wait_for_server()
    goal = DockPallet.Goal()
    goal.dock_id = 'pallet_pick_1'
    goal.navigate_to_staging = False
    goal.undock_after = True
    states = []

    def feedback(msg):
        states.append(msg.feedback.state)

    sent = node.dock.send_goal_async(goal, feedback_callback=feedback)
    while not sent.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    result = sent.result().get_result_async()
    t0 = time.time()
    last = 0.0
    while not result.done() and time.time() - t0 < 200:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - last > 2.0:
            last = time.time()
            x, y, yaw = node.pose()
            pal = pallet_state()
            print(f't={time.time() - t0:4.0f} state={states[-1] if states else "-":9s} '
                  f'robot x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):5.1f} | pallet '
                  f'{"x=%.3f y=%.3f z=%.3f tilt=%.1f" % pal if pal else "?"}', flush=True)
    res = result.result().result if result.done() else None
    node.spin(3.0)
    x, y, yaw = node.pose()
    pal = pallet_state()
    print('result', None if res is None else (res.success, res.error_code, res.message))
    print(f'final robot x={x:.3f} y={y:.3f}; pallet {pal}')
    node.fork(0.0)
    node.goto(0.0, 0.0, 0.0)
    place_pallet(0.0, 0.0)


if __name__ == '__main__':
    main()
