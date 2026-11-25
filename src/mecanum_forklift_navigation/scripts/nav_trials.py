#!/usr/bin/env python3
"""
Nav2 goal trials in Gazebo (development tool, not installed).

Usage: nav_trials.py [--label NAME] [--reps N] [--goals 1,2,3] [--csv FILE] [--check-goals]
Needs the simulation and navigation (single_bringup.launch.py) running, AMCL localised.

The goals are chained: each starts where the previous one ended (first start: spawn pose).
Per goal it records the action result, time, final pose error against ground truth, path
length, maximum and mean |vy| of the controller output (strafing), the minimum clearance of the
footprint to the uninflated curated map, whether the footprint ever touched an occupied cell
(collision) and the AMCL error at the end.
"""
import argparse
import csv
import math
import os
import re
import subprocess
import time

import cv2
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

MAP = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'maps',
                   'simple_warehouse_nav')
NS = os.environ.get('ROBOT_NS', '')    # e.g. /robot1; empty = the single-robot launch
FOOTPRINT = (-0.205, 0.368, -0.154, 0.154)      # x_min, x_max, y_min, y_max (incl. fork tines)

# id, x, y, yaw_deg, description
GOALS = [
    (1, 0.0, -1.0, 0, 'strafe -1.0 m'),
    (2, 1.2, -1.0, 0, 'forward 1.2 m'),
    (3, 1.2, 1.0, 0, 'strafe +2.0 m'),
    (4, 0.0, 1.0, 180, 'move and turn 180 deg'),
    (5, -1.8, 0.0, 180, 'between the west cones'),
    (6, -1.8, -1.6, 0, 'facing pallet_2'),
    (7, 0.165, 0.0, 0, 'staging pose pallet_1'),
    (8, 1.9, 1.6, 0, 'staging pose pickup pallet'),
    (9, -2.25, -1.6, 180, 'staging pose charger 1'),
    (10, 0.0, 0.0, 0, 'back to start'),
]


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


def moved_objects(before, after, tol=0.005):
    """Names of objects that moved more than tol metres between two snapshots."""
    return [n for n, a in after.items() if n in before
            and math.dist(a, before[n]) > tol]


class Grid:
    """Uninflated curated map: occupancy and distance to the nearest occupied cell."""

    def __init__(self):
        """Load the pgm/yaml of the curated navigation map."""
        img = cv2.imread(MAP + '.pgm', cv2.IMREAD_GRAYSCALE)
        self.h, self.w = img.shape
        self.res = 0.05
        self.ox, self.oy = -3.04, -2.54                  # origin from the yaml
        self.occ = (255 - img) / 255.0 > 0.65
        free = (~self.occ).astype(np.uint8)
        self.dist = cv2.distanceTransform(free, cv2.DIST_L2, 5) * self.res
        gx = np.arange(FOOTPRINT[0], FOOTPRINT[1] + 1e-9, 0.02)
        gy = np.arange(FOOTPRINT[2], FOOTPRINT[3] + 1e-9, 0.02)
        self.pts = np.array([(a, b) for a in gx for b in gy])

    def clearance(self, x, y, yaw):
        """Return (min distance footprint points -> occupied cells, collision flag)."""
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x + c * self.pts[:, 0] - s * self.pts[:, 1]
        wy = y + s * self.pts[:, 0] + c * self.pts[:, 1]
        col = ((wx - self.ox) / self.res).astype(int)
        row = self.h - 1 - ((wy - self.oy) / self.res).astype(int)
        if (col < 0).any() or (row < 0).any() or (col >= self.w).any() or (row >= self.h).any():
            return 0.0, True
        d = self.dist[row, col]
        return float(d.min()), bool(self.occ[row, col].any())


class Trials(Node):
    """Action client plus ground-truth, controller-output and AMCL monitors."""

    def __init__(self):
        """Create the clients and subscriptions."""
        super().__init__('nav_trials', namespace=NS,
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.grid = Grid()
        self.gt = None
        self.amcl = None
        self.run = None
        self.create_subscription(Odometry, 'odom_ground_truth', self.on_gt, 20)
        self.create_subscription(Twist, 'cmd_vel_nav', self.on_cmd, 20)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl, 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    @staticmethod
    def yaw_of(q):
        """Yaw from a quaternion message."""
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_gt(self, msg):
        """Track ground truth: path length, clearance, collision."""
        self.gt = msg
        if self.run is None:
            return
        p = msg.pose.pose.position
        yaw = self.yaw_of(msg.pose.pose.orientation)
        if self.run['last'] is not None:
            self.run['path'] += math.hypot(p.x - self.run['last'][0], p.y - self.run['last'][1])
        self.run['last'] = (p.x, p.y)
        clear, hit = self.grid.clearance(p.x, p.y, yaw)
        self.run['clear'] = min(self.run['clear'], clear)
        if hit and not self.run['collision']:
            self.run['hit_at'] = f'({p.x:.2f},{p.y:.2f},{math.degrees(yaw):.0f}deg)'
        self.run['collision'] = self.run['collision'] or hit

    def on_amcl(self, msg):
        """Keep the latest AMCL pose and the largest error against ground truth in the run."""
        self.amcl = msg
        if self.run is not None and self.gt is not None:
            a, g = msg.pose.pose.position, self.gt.pose.pose.position
            self.run['amcl_max'] = max(self.run['amcl_max'], math.hypot(a.x - g.x, a.y - g.y))

    def on_cmd(self, msg):
        """Collect the controller's sideways speed."""
        if self.run is not None:
            self.run['vy'].append(abs(msg.linear.y))

    def pose(self):
        """Ground-truth (x, y, yaw)."""
        p = self.gt.pose.pose
        return p.position.x, p.position.y, self.yaw_of(p.orientation)

    def spin(self, seconds):
        """Spin for a wall-clock duration."""
        t = time.time()
        while time.time() - t < seconds:
            rclpy.spin_once(self, timeout_sec=0.02)

    def goto(self, goal, timeout=240.0):
        """Send one goal; return a result dict."""
        _, gx, gy, gyaw, _ = goal
        self.nav.wait_for_server()
        req = NavigateToPose.Goal()
        req.pose.header.frame_id = 'map'
        req.pose.pose.position.x, req.pose.pose.position.y = gx, gy
        req.pose.pose.orientation.z = math.sin(math.radians(gyaw) / 2)
        req.pose.pose.orientation.w = math.cos(math.radians(gyaw) / 2)
        before = world_objects()
        self.run = {'path': 0.0, 'last': None, 'clear': 9.9, 'collision': False, 'vy': [],
                    'amcl_max': 0.0, 'hit_at': ''}
        t0 = time.time()
        sent = self.nav.send_goal_async(req)
        while not sent.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        handle = sent.result()
        if not handle.accepted:
            self.run = None
            return {'status': 'REJECTED'}
        res = handle.get_result_async()
        while not res.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not res.done():
            handle.cancel_goal_async()
            self.spin(2.0)
            status = 'TIMEOUT'
        else:
            status = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(
                res.result().status, str(res.result().status))
        elapsed = time.time() - t0
        self.spin(1.5)
        run, self.run = self.run, None
        pushed = moved_objects(before, world_objects())
        x, y, yaw = self.pose()
        eyaw = math.degrees(math.atan2(math.sin(yaw - math.radians(gyaw)),
                                       math.cos(yaw - math.radians(gyaw))))
        amcl_err = float('nan')
        if self.amcl is not None:
            a = self.amcl.pose.pose
            amcl_err = math.hypot(a.position.x - x, a.position.y - y) * 1000
        vy = run['vy'] or [0.0]
        return {'status': status, 'time': elapsed, 'err_xy_cm': math.hypot(x - gx, y - gy) * 100,
                'err_yaw_deg': eyaw, 'path_m': run['path'], 'vy_max': max(vy),
                'vy_frac': sum(v > 0.05 for v in vy) / len(vy), 'clear_cm': run['clear'] * 100,
                'collision': run['collision'], 'amcl_mm': amcl_err,
                'amcl_max_mm': run['amcl_max'] * 1000, 'hit_at': run['hit_at'],
                'pushed': ','.join(pushed)}


def main():
    """Run the goal chain and print one line per goal."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', default='dwb')
    ap.add_argument('--reps', type=int, default=1)
    ap.add_argument('--goals', default='')
    ap.add_argument('--csv', default='')
    ap.add_argument('--check-goals', action='store_true')
    args = ap.parse_args()
    ids = [int(i) for i in args.goals.split(',')] if args.goals else [g[0] for g in GOALS]
    goals = [g for g in GOALS if g[0] in ids]
    if args.check_goals:
        grid = Grid()
        for g in goals:
            clear, hit = grid.clearance(g[1], g[2], math.radians(g[3]))
            print(f'goal {g[0]:2d} {g[4]:28s} clearance {clear * 100:5.1f} cm '
                  f'{"IN OBSTACLE" if hit else "ok"}')
        return
    rclpy.init()
    node = Trials()
    while node.gt is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    subprocess.run(['ros2', 'service', 'call', f'{NS}/request_nomotion_update',
                    'std_srvs/srv/Empty'],
                   capture_output=True, timeout=20, check=False)      # makes AMCL publish a pose
    node.spin(2.0)
    print(f'controller {args.label}')
    head = ('goal', 'description', 'status', 'time s', 'xy cm', 'yaw deg', 'path m', 'vy max',
            'vy>0.05 %', 'clear cm', 'hit', 'amcl mm')
    print(f'{head[0]:>4s} {head[1]:28s} {head[2]:10s} {head[3]:>6s} {head[4]:>6s} {head[5]:>7s} '
          f'{head[6]:>6s} {head[7]:>6s} {head[8]:>9s} {head[9]:>8s} {head[10]:>4s} '
          f'{head[11]:>7s}', flush=True)
    rows = []
    for rep in range(args.reps):
        for g in goals:
            r = node.goto(g)
            rows.append((rep, g, r))
            if r['status'] in ('REJECTED',):
                print(f'{g[0]:4d} {g[4]:28s} REJECTED', flush=True)
                continue
            print(f'{g[0]:4d} {g[4]:28s} {r["status"]:10s} {r["time"]:6.1f} {r["err_xy_cm"]:6.1f} '
                  f'{r["err_yaw_deg"]:7.1f} {r["path_m"]:6.2f} {r["vy_max"]:6.2f} '
                  f'{r["vy_frac"] * 100:9.0f} {r["clear_cm"]:8.1f} '
                  f'{"YES" if r["collision"] else "no":>4s} {r["amcl_mm"]:7.0f} '
                  f'max {r["amcl_max_mm"]:5.0f} '
                  f'{r["hit_at"]} {"PUSHED " + r["pushed"] if r["pushed"] else ""}', flush=True)
    ok = [r for _, _, r in rows if r['status'] == 'SUCCEEDED']
    good = [r for r in ok if r['err_xy_cm'] <= 5.0 and abs(r['err_yaw_deg']) <= 5.0]
    print(f'objects pushed in {sum(bool(r["pushed"]) for _, _, r in rows)} goals')
    print(f'summary {args.label}: {len(ok)}/{len(rows)} succeeded, '
          f'{len(good)} within 5 cm / 5 deg, '
          f'{sum(r["collision"] for _, _, r in rows)} with collision')
    if args.csv:
        with open(args.csv, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['label', 'rep', 'goal', 'description'] + list(rows[0][2].keys()))
            for rep, g, r in rows:
                w.writerow([args.label, rep, g[0], g[4]] + list(r.values()))


if __name__ == '__main__':
    main()
