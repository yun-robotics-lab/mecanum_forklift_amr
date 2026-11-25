#!/usr/bin/env python3
"""
Pick-and-carry integration trials in Gazebo (development tool, not installed).

Usage: pick_trials.py [trials] [first]
(default 20 trials from index 0; `first` resumes an interrupted run). Needs the simulation,
navigation, vision and docking launches running. Each trial: navigate (unloaded) to one of 5
start poses, place pallet_1 with an offset (lateral / yaw), send DockPallet with
navigate_to_staging and undock_after, then navigate to a carry goal with the pallet on the
forks and check that the pallet stays put and nothing else in the world was pushed.
Success = DockPallet success, dock error (pose when the lift starts) within 10 mm / 10 mm / 2 deg,
pallet tilt <= 2 deg, pallet drift on the forks <= 2 cm after the drive, carry goal reached,
no other object moved.
"""
import math
import os
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
from dock_trials import NS, pallet_state, place_pallet, STOP, Trials, world_objects
from geometry_msgs.msg import Point32, Polygon
from mecanum_forklift_interfaces.action import DockPallet
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient

STARTS = [(0.0, -1.0, 0), (1.2, 1.0, 0), (-1.8, 0.0, 180), (-1.8, -1.6, 0), (0.0, 1.0, 180)]
OFFSETS = [(0.0, 0), (0.05, 0), (-0.05, 0), (0.0, 10), (0.0, -10), (0.05, 10), (-0.05, -10)]
CARRY_GOAL = (1.0, -1.2, 0)
UNLOADED = [(0.368, 0.154), (0.368, -0.154), (-0.205, -0.154), (-0.205, 0.154)]
OURS = {'pallet_1', 'box_1'}
FULL_MAP = os.path.join(get_package_share_directory('mecanum_forklift_navigation'), 'maps',
                        'simple_warehouse_nav.yaml')


def rel_pose(robot, pallet_xy):
    """Pallet position in the robot frame."""
    dx, dy = pallet_xy[0] - robot[0], pallet_xy[1] - robot[1]
    c, s = math.cos(robot[2]), math.sin(robot[2])
    return c * dx + s * dy, -s * dx + c * dy


class Pick(Trials):
    """Trials plus a Nav2 client and a footprint publisher."""

    def __init__(self):
        """Create the NavigateToPose client and the footprint publishers."""
        super().__init__()
        self.navc = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.fp = [self.create_publisher(Polygon, t, 10)
                   for t in ('local_costmap/footprint', 'global_costmap/footprint')]

    def restore_full_map(self):
        """Load the full curated map (pallet back in its slot)."""
        subprocess.run(['ros2', 'service', 'call', f'{NS}/nav_map_server/load_map',
                        'nav2_msgs/srv/LoadMap', f'{{map_url: {FULL_MAP}}}'],
                       capture_output=True, timeout=30, check=False)
        self.spin(1.0)

    def set_unloaded_footprint(self):
        """Publish the bare-fork footprint (a trial resets the carrying state)."""
        msg = Polygon()
        msg.points = [Point32(x=x, y=y, z=0.0) for x, y in UNLOADED]
        for _ in range(3):
            for pub in self.fp:
                pub.publish(msg)
            self.spin(0.2)

    def navigate(self, x, y, yaw_deg, timeout=240):
        """Navigate and wait; return the status name."""
        self.navc.wait_for_server()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
        goal.pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
        sent = self.navc.send_goal_async(goal)
        while not sent.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        handle = sent.result()
        if not handle.accepted:
            return 'REJECTED'
        res = handle.get_result_async()
        t0 = time.time()
        while not res.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not res.done():
            handle.cancel_goal_async()
            self.spin(3.0)
            return 'TIMEOUT'
        self.spin(1.5)
        return {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(res.result().status, '?')

    def run_pick(self):
        """Run the dock action; return (result, pose when the lift started)."""
        self.dock.wait_for_server()
        goal = DockPallet.Goal()
        goal.dock_id = 'pallet_pick_1'
        goal.navigate_to_staging = True
        goal.undock_after = True
        lift_pose = []

        def feedback(msg):
            if msg.feedback.state == 'LIFTING' and not lift_pose:
                lift_pose.append(self.pose())

        sent = self.dock.send_goal_async(goal, feedback_callback=feedback)
        while not sent.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        res = sent.result().get_result_async()
        t0 = time.time()
        while not res.done() and time.time() - t0 < 400:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not res.done():
            return None, None
        return res.result().result, (lift_pose[0] if lift_pose else None)


def main():
    """Run the trials."""
    trials = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    first = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    rclpy.init()
    node = Pick()
    while node.gt is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"{'#':>2s} {'start':14s} {'pallet':9s} {'result':26s} {'long':>6s} {'lat':>6s} "
          f"{'yaw':>5s} {'tilt':>5s} {'drift':>6s} {'carry':10s} {'pushed':12s} {'s':>5s}",
          flush=True)
    ok = 0
    for k in range(first, first + trials):
        start = STARTS[k % len(STARTS)]
        dy, dyaw = OFFSETS[k % len(OFFSETS)]
        node.fork(0.0)
        node.set_unloaded_footprint()
        place_pallet(0.0, 0.0)
        node.restore_full_map()
        st = node.navigate(*start)
        if st != 'SUCCEEDED':
            print(f'{k + 1:2d} cannot reach start {start}: {st}', flush=True)
            continue
        mx, my = place_pallet(dy, math.radians(dyaw))
        node.spin(1.5)
        before = world_objects()
        t0 = time.time()
        res, lift_pose = node.run_pick()
        label = 'TIMEOUT' if res is None else ('OK' if res.success else res.error_code)
        lon = lat = eyaw = tilt = drift = float('nan')
        carry = '-'
        pushed = []
        if res is not None and res.success and lift_pose is not None:
            psi = math.radians(dyaw)
            ex = lift_pose[0] - (mx - STOP * math.cos(psi))
            ey = lift_pose[1] - (my - STOP * math.sin(psi))
            lon = (ex * math.cos(psi) + ey * math.sin(psi)) * 1000
            lat = (-ex * math.sin(psi) + ey * math.cos(psi)) * 1000
            eyaw = math.degrees(math.atan2(math.sin(lift_pose[2] - psi),
                                           math.cos(lift_pose[2] - psi)))
            pal = pallet_state()
            r0 = rel_pose(node.pose(), pal[:2])
            tilt = pal[3]
            carry = node.navigate(*CARRY_GOAL)
            pal2 = pallet_state()
            r1 = rel_pose(node.pose(), pal2[:2])
            drift = math.hypot(r1[0] - r0[0], r1[1] - r0[1]) * 100
            tilt = max(tilt, pal2[3])
            after = world_objects()
            pushed = [n for n, a in after.items() if n in before and n not in OURS
                      and math.dist(a, before[n]) > 0.005]
        good = (res is not None and res.success and lift_pose is not None
                and abs(lon) <= 10 and abs(lat) <= 10 and abs(eyaw) <= 2.0 and tilt <= 2.0
                and drift <= 2.0 and carry == 'SUCCEEDED' and not pushed)
        ok += good
        print(f'{k + 1:2d} {str(start):14s} {dy:+.2f}/{dyaw:+3d} '
              f'{("OK" if good else "FAIL " + label):26s} {lon:6.1f} {lat:6.1f} {eyaw:5.2f} '
              f'{tilt:5.2f} {drift:6.1f} {carry:10s} {",".join(pushed) or "-":12s} '
              f'{time.time() - t0:5.0f}', flush=True)
    print(f'success {ok}/{trials} (trials {first + 1}-{first + trials})')
    node.fork(0.0)
    node.set_unloaded_footprint()
    place_pallet(0.0, 0.0)
    node.restore_full_map()


if __name__ == '__main__':
    main()
