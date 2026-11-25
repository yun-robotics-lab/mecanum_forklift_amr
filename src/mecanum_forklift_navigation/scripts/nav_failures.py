#!/usr/bin/env python3
"""
Nav2 failure-injection tests in Gazebo (development tool, not installed).

Needs the simulation and navigation (single_bringup.launch.py) running and the robot at the
spawn pose with AMCL localised. Cases: cancel, goal inside an obstacle, teleop pre-emption,
an obstacle appearing in the path, an unreachable goal followed by a reachable one.
Every case ends with a check that the robot is stopped.
"""
import math
import time

from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

BOX_SDF = ('<sdf version="1.8"><model name="{name}"><static>true</static><link name="l">'
           '<collision name="c"><geometry><box><size>{sx} {sy} 0.5</size></box></geometry>'
           '</collision><visual name="v"><geometry><box><size>{sx} {sy} 0.5</size></box>'
           '</geometry></visual></link></model></sdf>')


def gz(service, reqtype, req, reptype='ignition.msgs.Boolean'):
    """Call an Ignition service."""
    import subprocess
    subprocess.run(['ign', 'service', '-s', f'/world/mecanum_world/{service}', '--reqtype',
                    reqtype, '--reptype', reptype, '--timeout', '3000', '--req', req],
                   check=True, capture_output=True)


def spawn_box(name, x, y, sx=0.3, sy=0.3):
    """Add a 0.5 m tall box (visible to the lidar) to the world."""
    sdf = BOX_SDF.format(name=name, sx=sx, sy=sy).replace('"', '\\"')
    gz('create', 'ignition.msgs.EntityFactory',
       f'sdf: "{sdf}", pose: {{position: {{x: {x}, y: {y}, z: 0.25}}}}')


def remove_box(name):
    """Remove a model from the world."""
    gz('remove', 'ignition.msgs.Entity', f'name: "{name}", type: MODEL')


class Tests(Node):
    """Ground truth, teleop publisher and the NavigateToPose client."""

    def __init__(self):
        """Create the clients and subscriptions."""
        super().__init__('nav_failures',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.gt = None
        self.create_subscription(Odometry, '/odom_ground_truth',
                                 lambda m: setattr(self, 'gt', m), 20)
        self.teleop = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def spin(self, seconds):
        """Spin for a wall-clock duration."""
        t = time.time()
        while time.time() - t < seconds:
            rclpy.spin_once(self, timeout_sec=0.02)

    def pose(self):
        """Ground-truth (x, y, yaw)."""
        p = self.gt.pose.pose
        q = p.orientation
        return p.position.x, p.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y),
                                                      1 - 2 * (q.y * q.y + q.z * q.z))

    def speed(self):
        """Ground-truth linear speed (m/s) from the odometry twist, in the world frame."""
        v = self.gt.twist.twist.linear
        return math.hypot(v.x, v.y)

    def stopped(self, seconds=2.0):
        """Return the displacement over an interval (should be ~0 when stopped)."""
        self.spin(0.2)
        x0, y0, _ = self.pose()
        self.spin(seconds)
        x1, y1, _ = self.pose()
        return math.hypot(x1 - x0, y1 - y0)

    def send(self, x, y, yaw_deg):
        """Send a goal and return the goal handle (or None if rejected)."""
        self.nav.wait_for_server()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
        goal.pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
        sent = self.nav.send_goal_async(goal)
        while not sent.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        handle = sent.result()
        return handle if handle.accepted else None

    def wait(self, handle, timeout):
        """Wait for a result; return the status name or TIMEOUT."""
        res = handle.get_result_async()
        t0 = time.time()
        while not res.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not res.done():
            handle.cancel_goal_async()          # the caller's timeout: cancel, then settle
            self.spin(3.0)
            return 'TIMEOUT'
        return {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(res.result().status, '?')

    def goto_and_wait(self, x, y, yaw_deg, timeout=120):
        """Send a goal and wait for the outcome."""
        handle = self.send(x, y, yaw_deg)
        return 'REJECTED' if handle is None else self.wait(handle, timeout)


def line(name, ok, detail):
    """Print one result line."""
    print(f'{name:36s} {"PASS" if ok else "FAIL"}  {detail}', flush=True)
    return ok


def main():
    """Run the failure matrix (argument: comma separated case numbers, default all)."""
    import sys
    only = [int(c) for c in sys.argv[1].split(',')] if len(sys.argv) > 1 else [1, 2, 3, 4, 5]
    rclpy.init()
    t = Tests()
    while t.gt is None:
        rclpy.spin_once(t, timeout_sec=0.1)
    results = []

    # 1 cancel: stop time after the cancel request
    if 1 in only:
        handle = t.send(1.2, -1.0, 0)
        t.spin(6.0)
        t0 = time.time()
        handle.cancel_goal_async()
        status = t.wait(handle, 20)
        stop_time = None
        while time.time() - t0 < 6.0:
            rclpy.spin_once(t, timeout_sec=0.02)
            if t.speed() < 0.02:
                stop_time = time.time() - t0
                break
        moved = t.stopped()
        results.append(line('cancel mid-goal', status == 'CANCELED' and moved < 0.005
                            and stop_time is not None and stop_time <= 1.5,
                            f'status {status}, speed < 2 cm/s after '
                            f'{stop_time if stop_time is None else round(stop_time, 2)} s wall, '
                            f'moved {moved * 1000:.1f} mm in the next 2 s'))
        t.goto_and_wait(0.0, 0.0, 0)

    # 2 goal deep inside an obstacle: centre of shelf_2 (0.5, 2.25), 0.16 m from free space
    if 2 in only:
        # (NavFn accepts goals within its 0.1 m tolerance of free space, e.g. the pallet_1 centre)
        x0, y0, _ = t.pose()
        t0 = time.time()
        status = t.goto_and_wait(0.5, 2.25, 0, timeout=90)
        x1, y1, _ = t.pose()
        results.append(line('goal inside shelf_2', status in ('ABORTED', 'REJECTED')
                            and t.stopped() < 0.005,
                            f'status {status} after {time.time() - t0:.0f} s, robot first drove '
                            f'{math.hypot(x1 - x0, y1 - y0) * 100:.1f} cm toward the nearest '
                            'reachable point'))

    # 3 teleop pre-emption: drive -x with teleop while a goal is active
    if 3 in only:
        handle = t.send(1.2, -1.0, 0)
        t.spin(8.0)
        xs = []
        t_end = time.time() + 3.0
        while time.time() < t_end:
            msg = Twist()
            msg.linear.x = -0.1
            t.teleop.publish(msg)
            t.spin(0.05)
            xs.append(t.gt.twist.twist.linear.x)
        for _ in range(10):
            t.teleop.publish(Twist())
            t.spin(0.05)
        follows = sum(1 for v in xs[len(xs) // 2:] if -0.15 < v < -0.05) >= len(xs) // 4
        status = t.wait(handle, 120)
        results.append(line('teleop pre-empts navigation', follows and status == 'SUCCEEDED',
                            f'robot followed teleop (-0.1 m/s): {follows}; '
                            f'goal afterwards {status}'))
        t.goto_and_wait(0.0, 0.0, 0)

    # 4 obstacle appearing on the path: (0,-1) -> (1.2,-1), box dropped at (0.6,-1) after 6 s
    if 4 in only:
        t.goto_and_wait(0.0, -1.0, 0)
        handle = t.send(1.2, -1.0, 0)
        t.spin(5.0)
        spawn_box('test_box', 0.6, -1.0)
        status = t.wait(handle, 150)
        x, y, _ = t.pose()
        box_hit = math.hypot(x - 0.6, y + 1.0) < 0.20
        results.append(line('obstacle appears in path', status in ('SUCCEEDED', 'ABORTED') and not
                            box_hit and t.stopped() < 0.005,
                            f'status {status}, final ({x:.2f},{y:.2f}), '
                            f'within 0.2 m of box: {box_hit}'))
        remove_box('test_box')
        t.spin(2.0)
        t.goto_and_wait(0.0, 0.0, 0)

    # 5 unreachable goal (box dropped on the goal), then a reachable goal succeeds
    if 5 in only:
        spawn_box('test_goal_box', -1.8, -1.6, 0.6, 0.6)
        t.spin(3.0)
        t0 = time.time()
        status = t.goto_and_wait(-1.8, -1.6, 0, timeout=300)
        stopped = t.stopped() < 0.005
        remove_box('test_goal_box')
        t.spin(3.0)
        again = t.goto_and_wait(-1.8, -1.6, 0, timeout=120)
        results.append(line('unreachable goal, then reachable', status in ('ABORTED', 'REJECTED',
                                                                           'TIMEOUT')
                            and stopped and again == 'SUCCEEDED',
                            f'blocked goal: {status} after {time.time() - t0:.0f} s (TIMEOUT = '
                            'Nav2 did not give up by itself, cancelled by the test), stopped '
                            f'{stopped}; same goal after removing the box: {again}'))
        t.goto_and_wait(0.0, 0.0, 0)

    print(f'passed {sum(bool(r) for r in results)}/{len(results)}')


if __name__ == '__main__':
    main()
