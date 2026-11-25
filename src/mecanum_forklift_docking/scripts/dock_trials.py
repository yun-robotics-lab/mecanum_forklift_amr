#!/usr/bin/env python3
"""
DockPallet trials in Gazebo with pallet offsets (development tool, not installed).

Usage: dock_trials.py [repeats]. Needs the simulation, vision and docking launches running.
Each case moves pallet_1 and box_1 (the marker is joined to box_1) rigidly (lateral dy, yaw about
the pallet centre), returns the robot to the start pose, sends a DockPallet goal and compares the
final base_link pose with the marker-derived target (stop distance before the marker).
Success = action success and |long| <= 10 mm, |lat| <= 10 mm, |yaw| <= 2 deg.
"""
import math
import os
import re
import subprocess
import sys
import time

from geometry_msgs.msg import Twist
from mecanum_forklift_interfaces.action import DockPallet, SetLiftHeight
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

NS = os.environ.get('ROBOT_NS', '')    # e.g. /robot1; empty = the single-robot launch
STOP = 0.245            # base_link stops this far before the marker (workflow.yaml)
PAL = (0.9, 0.0)        # pallet_1 centre in the world
MARK = (0.81, 0.0, 0.117)
CASES = [(0.0, 0), (0.05, 0), (-0.05, 0), (0.0, 10), (0.0, -10), (0.05, 10), (-0.05, -10)]


def set_pose(name, x, y, z, yaw):
    """Teleport a Gazebo model."""
    q = f'x: 0, y: 0, z: {math.sin(yaw / 2):.7f}, w: {math.cos(yaw / 2):.7f}'
    req = f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{{q}}}'
    subprocess.run(['ign', 'service', '-s', '/world/mecanum_world/set_pose', '--reqtype',
                    'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean', '--timeout',
                    '3000', '--req', req], check=True, capture_output=True)


def pallet_state(name='pallet_1'):
    """Return (x, y, z, tilt_deg) of a Gazebo model from the pose info topic."""
    out = subprocess.run(['ign', 'topic', '-e', '-n', '1', '-t',
                          '/world/mecanum_world/pose/info'],
                         capture_output=True, text=True, timeout=20).stdout
    block = re.search(r'name: "%s"\n\s+id: \d+\n\s+position \{(.*?)\}\n\s+orientation \{(.*?)\}'
                      % name, out, re.S)
    if not block:
        return None
    pos = {k: float(v) for k, v in re.findall(r'(\w): (\S+)', block.group(1))}
    q = {k: float(v) for k, v in re.findall(r'(\w): (\S+)', block.group(2))}
    # angle between the model z axis and the world z axis
    uz = 1 - 2 * (q.get('x', 0) ** 2 + q.get('y', 0) ** 2)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, uz))))
    return pos.get('x', 0.0), pos.get('y', 0.0), pos.get('z', 0.0), tilt


def spawn_box(name, x, y, sx=0.3, sy=0.3):
    """Add a 0.5 m tall static box (visible to the lidar) to the world."""
    sdf = ('<sdf version="1.8"><model name="%s"><static>true</static><link name="l"><collision '
           'name="c"><geometry><box><size>%s %s 0.5</size></box></geometry></collision><visual '
           'name="v"><geometry><box><size>%s %s 0.5</size></box></geometry></visual></link>'
           '</model></sdf>' % (name, sx, sy, sx, sy)).replace('"', '\\"')
    subprocess.run(['ign', 'service', '-s', '/world/mecanum_world/create', '--reqtype',
                    'ignition.msgs.EntityFactory', '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '3000', '--req',
                    f'sdf: "{sdf}", pose: {{position: {{x: {x}, y: {y}, z: 0.25}}}}'],
                   check=True, capture_output=True)


def remove_box(name):
    """Remove a model from the world."""
    subprocess.run(['ign', 'service', '-s', '/world/mecanum_world/remove', '--reqtype',
                    'ignition.msgs.Entity', '--reptype', 'ignition.msgs.Boolean', '--timeout',
                    '3000', '--req', f'name: "{name}", type: MODEL'],
                   check=True, capture_output=True)


def world_objects():
    """Return {name: (x, y, z)} of the movable world objects from Gazebo."""
    out = subprocess.run(['ign', 'topic', '-e', '-n', '1', '-t', '/world/mecanum_world/pose/info'],
                         capture_output=True, text=True, timeout=30, check=False).stdout
    found = {}
    for m in re.finditer(r'name: "(\w+)"\n\s+id: \d+\n\s+position \{(.*?)\}', out, re.S):
        if re.match(r'(pallet|box|cone|shelf|charging|pickup|dropoff)', m.group(1)):
            v = {k: float(x) for k, x in re.findall(r'(\w): (\S+)', m.group(2))}
            found[m.group(1)] = (v.get('x', 0.0), v.get('y', 0.0), v.get('z', 0.0))
    return found


def place_pallet(dy, dyaw):
    """Move pallet and box (with its joined marker) rigidly; return the marker position."""
    c, s = math.cos(dyaw), math.sin(dyaw)

    def tf(x, y):
        rx, ry = x - PAL[0], y - PAL[1]
        return PAL[0] + c * rx - s * ry, PAL[1] + s * rx + c * ry + dy

    px, py = tf(*PAL)
    set_pose('pallet_1', px, py, 0.0, math.pi / 2 + dyaw)
    set_pose('box_1', px, py, 0.056, dyaw)
    mx, my = tf(MARK[0], MARK[1])
    return mx, my


class Trials(Node):
    """Helpers around ground truth, teleop and the two action clients."""

    def __init__(self):
        """Create subscriptions, publisher and action clients."""
        super().__init__('dock_trials', namespace=NS,
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.gt = None
        self.create_subscription(Odometry, 'odom_ground_truth',
                                 lambda m: setattr(self, 'gt', m), 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        self.dock = ActionClient(self, DockPallet, 'dock_pallet')
        self.lift = ActionClient(self, SetLiftHeight, 'set_lift_height')

    def spin(self, seconds):
        """Spin for a wall-clock duration."""
        t = time.time()
        while time.time() - t < seconds:
            rclpy.spin_once(self, timeout_sec=0.02)

    def pose(self):
        """Ground-truth (x, y, yaw)."""
        p = self.gt.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return p.position.x, p.position.y, yaw

    def goto(self, x, y, yaw, timeout=120):
        """Drive to a world pose with teleop commands (proportional, ground truth)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            px, py, pyaw = self.pose()
            ex, ey = x - px, y - py
            eyaw = math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw))
            if math.hypot(ex, ey) < 0.005 and abs(eyaw) < 0.01:
                break
            c, s = math.cos(pyaw), math.sin(pyaw)
            m = Twist()
            m.linear.x = max(-0.2, min(0.2, c * ex + s * ey))
            m.linear.y = max(-0.2, min(0.2, -s * ex + c * ey))
            m.angular.z = max(-0.5, min(0.5, 1.5 * eyaw))
            self.pub.publish(m)
        for _ in range(10):
            self.pub.publish(Twist())
            self.spin(0.05)
        self.spin(1.0)

    def fork(self, height):
        """Move the fork and wait."""
        self.lift.wait_for_server()
        goal = SetLiftHeight.Goal()
        goal.target_height = height
        sent = self.lift.send_goal_async(goal)
        while not sent.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        res = sent.result().get_result_async()
        while not res.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        self.spin(1.0)

    def run_dock(self, undock):
        """Send a DockPallet goal; return (result, seconds) or None on a 300 s hang."""
        self.dock.wait_for_server()
        goal = DockPallet.Goal()
        goal.dock_id = 'pallet_pick_1'
        goal.navigate_to_staging = False
        goal.undock_after = undock
        sent = self.dock.send_goal_async(goal)
        while not sent.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        res = sent.result().get_result_async()
        t0 = time.time()
        while not res.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > 300:
                return None
        return res.result().result, time.time() - t0


def main():
    """Run the cases and print one line per trial."""
    reps = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    rclpy.init()
    node = Trials()
    while node.gt is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"{'pallet dy/yaw':14s} {'result':22s} {'long mm':>8s} {'lat mm':>7s} {'yaw deg':>8s} "
          f"{'time s':>7s} {'pallet z mm':>11s} {'tilt deg':>8s}", flush=True)
    ok = total = 0
    for _ in range(reps):
        for dy, dyaw in CASES:
            node.fork(0.0)
            node.goto(0.0, 0.0, 0.0)
            mx, my = place_pallet(dy, math.radians(dyaw))
            node.spin(2.0)
            out = node.run_dock(False)
            node.spin(1.0)
            total += 1
            if out is None:
                print(f'{dy:+.2f}/{dyaw:+3d}      TIMEOUT', flush=True)
                continue
            res, seconds = out
            x, y, yaw = node.pose()
            pal = pallet_state() or (0.0, 0.0, 0.0, float('nan'))
            psi = math.radians(dyaw)
            ex, ey = x - (mx - STOP * math.cos(psi)), y - (my - STOP * math.sin(psi))
            lon = ex * math.cos(psi) + ey * math.sin(psi)
            lat = -ex * math.sin(psi) + ey * math.cos(psi)
            eyaw = math.degrees(math.atan2(math.sin(yaw - psi), math.cos(yaw - psi)))
            good = res.success and abs(lon) <= 0.010 and abs(lat) <= 0.010 and abs(eyaw) <= 2.0
            ok += good
            label = 'OK' if good else 'FAIL ' + (res.error_code or 'out of tolerance')
            print(f'{dy:+.2f}/{dyaw:+3d}      {label:22s} {lon * 1000:8.1f} {lat * 1000:7.1f} '
                  f'{eyaw:8.2f} {seconds:7.1f} {pal[2] * 1000:11.1f} {pal[3]:8.2f}', flush=True)
    print(f'success {ok}/{total}')
    node.fork(0.0)
    node.goto(0.0, 0.0, 0.0)
    place_pallet(0.0, 0.0)


if __name__ == '__main__':
    main()
