#!/usr/bin/env python3
"""
Isolation tests for a two-robot simulation (development tool, not installed).

Usage: multi_tests.py [case numbers]. Needs `robot_bringup` for robot1 and robot2 (the default
fleet), with the web layer on (REST 8000 / 8001). Navigation is not needed.
Cases: 1 TF isolation, 2 motion cross-talk, 3 lift cross-talk, 4 ports + REST isolation,
5 removing robot2 leaves robot1 working (destructive: run last).
"""
import math
import os
import subprocess
import sys
import time

from geometry_msgs.msg import Twist
import httpx
from mecanum_forklift_interfaces.action import SetLiftHeight
from mecanum_forklift_interfaces.msg import LiftState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

ROBOTS = ('robot1', 'robot2')
TOKEN = open(os.path.expanduser('~/.config/mecanum_forklift/web_token'), encoding='utf-8').read(
).strip()
AUTH = {'Authorization': f'Bearer {TOKEN}'}


def line(name, ok, detail=''):
    """Print one result line."""
    print(f'{name:46s} {"PASS" if ok else "FAIL"}  {detail}', flush=True)
    return ok


class Rig(Node):
    """Ground truth, lift state and teleop publishers of every robot."""

    def __init__(self):
        """Create the per-robot subscriptions, publishers and lift clients."""
        super().__init__('multi_tests')
        self.gt = {}
        self.lift = {}
        self.teleop = {}
        self.lift_client = {}
        for name in ROBOTS:
            self.create_subscription(Odometry, f'/{name}/odom_ground_truth',
                                     lambda m, n=name: self.gt.__setitem__(n, m), 10)
            self.create_subscription(LiftState, f'/{name}/lift_state',
                                     lambda m, n=name: self.lift.__setitem__(n, m), 10)
            self.teleop[name] = self.create_publisher(Twist, f'/{name}/cmd_vel_teleop', 10)
            self.lift_client[name] = ActionClient(self, SetLiftHeight,
                                                  f'/{name}/set_lift_height')

    def spin(self, seconds):
        """Spin for a wall-clock duration."""
        t = time.time()
        while time.time() - t < seconds:
            rclpy.spin_once(self, timeout_sec=0.02)

    def pose(self, name):
        """Ground-truth (x, y) of a robot."""
        p = self.gt[name].pose.pose.position
        return p.x, p.y

    def wait_ready(self, names, timeout=30):
        """Wait until ground truth of the robots has arrived."""
        t0 = time.time()
        while time.time() - t0 < timeout and not all(n in self.gt for n in names):
            self.spin(0.2)
        return all(n in self.gt for n in names)

    def drive(self, name, vx, vy, seconds):
        """Publish a teleop velocity to one robot for `seconds`."""
        t = time.time()
        while time.time() - t < seconds:
            msg = Twist()
            msg.linear.x, msg.linear.y = vx, vy
            self.teleop[name].publish(msg)
            self.spin(0.05)
        for _ in range(8):
            self.teleop[name].publish(Twist())
            self.spin(0.05)


def publishers_of(topic):
    """Return the node namespaces publishing on a topic."""
    out = subprocess.run(['ros2', 'topic', 'info', '-v', topic], capture_output=True,
                         text=True).stdout
    nodes = []
    name = None
    kind = None
    for ln in out.splitlines():
        if ln.startswith('Publisher count'):
            kind = 'pub'
        elif ln.startswith('Subscription count'):
            kind = 'sub'
        elif ln.startswith('Node name:'):
            name = ln.split(':', 1)[1].strip()
        elif ln.startswith('Node namespace:') and kind == 'pub':
            nodes.append(f'{ln.split(":", 1)[1].strip()}/{name}')
    return nodes


def case_tf(rig):
    """No global /tf; each robot's TF topic is published only by that robot's own nodes."""
    topics = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True).stdout
    global_tf = [t for t in topics.split() if t in ('/tf', '/tf_static')]
    results = [line('no global /tf or /tf_static topic', not global_tf, str(global_tf))]
    for name in ROBOTS:
        other = [r for r in ROBOTS if r != name][0]
        pubs = publishers_of(f'/{name}/tf') + publishers_of(f'/{name}/tf_static')
        clean = bool(pubs) and all(p.startswith(f'/{name}/') for p in pubs)
        results.append(line(f'/{name}/tf published only by /{name}/*', clean,
                            f'{len(pubs)} publishers, foreign: '
                            f'{[p for p in pubs if p.startswith("/" + other)] or "none"}'))
    frames = subprocess.run(['ros2', 'topic', 'echo', '/robot2/tf_static', '--once'],
                            capture_output=True, text=True, timeout=20).stdout
    results.append(line('robot2 has its own static frames', 'base_link' in frames
                        and 'front_camera_link' in frames or 'lidar_link' in frames, ''))
    return all(results)


def case_motion(rig):
    """Check that a teleop command to one robot moves only that robot (and the reverse)."""
    rig.wait_ready(ROBOTS)
    results = []
    for mover, other in (('robot1', 'robot2'), ('robot2', 'robot1')):
        a0, b0 = rig.pose(mover), rig.pose(other)
        rig.drive(mover, 0.0, 0.2, 2.5)                 # strafe: also exercises the mecanum drive
        rig.spin(0.5)
        a1, b1 = rig.pose(mover), rig.pose(other)
        moved = math.hypot(a1[0] - a0[0], a1[1] - a0[1])
        stayed = math.hypot(b1[0] - b0[0], b1[1] - b0[1])
        results.append(line(f'teleop {mover} moves only {mover}', moved > 0.1 and stayed < 0.005,
                            f'{mover} moved {moved * 100:.1f} cm, {other} {stayed * 1000:.1f} mm'))
    return all(results)


def case_lift(rig):
    """Check that a lift command to one robot raises only that robot's fork."""
    rig.spin(1.0)
    results = []
    for mover, other in (('robot1', 'robot2'), ('robot2', 'robot1')):
        client = rig.lift_client[mover]
        client.wait_for_server(timeout_sec=10)
        h_other0 = rig.lift[other].height
        goal = SetLiftHeight.Goal()
        goal.target_height = 0.06
        sent = client.send_goal_async(goal)
        while not sent.done():
            rclpy.spin_once(rig, timeout_sec=0.05)
        result = sent.result().get_result_async()
        while not result.done():
            rclpy.spin_once(rig, timeout_sec=0.05)
        rig.spin(1.0)
        up = rig.lift[mover].height
        h_other1 = rig.lift[other].height
        results.append(line(f'lift {mover} raises only {mover}', up > 0.05
                            and abs(h_other1 - h_other0) < 0.002,
                            f'{mover} {up * 1000:.0f} mm, {other} moved '
                            f'{abs(h_other1 - h_other0) * 1000:.1f} mm'))
        goal.target_height = 0.0                       # lower it again
        sent = client.send_goal_async(goal)
        while not sent.done():
            rclpy.spin_once(rig, timeout_sec=0.05)
        result = sent.result().get_result_async()
        while not result.done():
            rclpy.spin_once(rig, timeout_sec=0.05)
    return all(results)


def listeners():
    """Return the local TCP listeners on the REST / rosbridge / video ports."""
    out = subprocess.run(['ss', '-ltn'], capture_output=True, text=True).stdout
    return sorted(c[3] for c in (ln.split() for ln in out.splitlines()[1:])
                  if c[3].rsplit(':', 1)[-1] in ('8000', '8001', '9090', '9091', '8080', '8081'))


def case_ports(rig):
    """Per-robot ports are unique and each REST API sees and controls only its own robot."""
    got = listeners()
    want = ['127.0.0.1:8000', '127.0.0.1:8001', '127.0.0.1:9090', '127.0.0.1:9091']
    results = [line('listeners = REST 8000/8001, rosbridge 9090/9091', got == want, str(got))]
    base = {'robot1': 'http://127.0.0.1:8000', 'robot2': 'http://127.0.0.1:8001'}
    for name, url in base.items():
        status = httpx.get(url + '/status', headers=AUTH, timeout=10).json()
        results.append(line(f'REST {url[-4:]} reports lift of {name}', status['lift'] is not None
                            and abs(status['lift']['height'] - rig.lift[name].height) < 0.01,
                            f'lift {status["lift"]}'))
    engage = httpx.post(base['robot1'] + '/estop', headers=AUTH, timeout=10)
    rig.spin(1.5)
    states = {n: httpx.get(u + '/status', headers=AUTH, timeout=10).json()['estop']
              for n, u in base.items()}
    results.append(line('e-stop on :8000 affects only robot1', engage.status_code == 200
                        and states == {'robot1': True, 'robot2': False}, str(states)))
    x0 = rig.pose('robot2')
    rig.drive('robot2', 0.2, 0.0, 1.5)
    x1 = rig.pose('robot2')
    results.append(line('robot2 still drivable while robot1 is e-stopped',
                        math.hypot(x1[0] - x0[0], x1[1] - x0[1]) > 0.05, ''))
    release = subprocess.run(['ros2', 'service', 'call', '/robot1/estop/release',
                              'std_srvs/srv/Trigger'], capture_output=True, text=True).stdout
    results.append(line('local release of robot1', 'success=True' in release, ''))
    return all(results)


def case_removal(rig):
    """Remove robot2 (processes and model): robot1 keeps driving and its topics keep flowing."""
    rig.wait_ready(ROBOTS)
    nodes_before = subprocess.run(['ros2', 'node', 'list'], capture_output=True,
                                  text=True).stdout.split()
    r1_before = [n for n in nodes_before if n.startswith('/robot1/')]
    subprocess.run(['pkill', '-f', '__ns:=/robot2'], check=False)
    subprocess.run(['ign', 'service', '-s', '/world/mecanum_world/remove', '--reqtype',
                    'ignition.msgs.Entity', '--reptype', 'ignition.msgs.Boolean', '--timeout',
                    '3000', '--req', 'name: "robot2", type: MODEL'], capture_output=True)
    rig.spin(8.0)
    nodes_after = subprocess.run(['ros2', 'node', 'list'], capture_output=True,
                                 text=True).stdout.split()
    gone = not any(n.startswith('/robot2/') for n in nodes_after)
    r1_after = [n for n in nodes_after if n.startswith('/robot1/')]
    x0 = rig.pose('robot1')
    rig.drive('robot1', 0.2, 0.0, 2.0)
    x1 = rig.pose('robot1')
    models = subprocess.run(['ign', 'model', '--list'], capture_output=True, text=True).stdout
    results = [line('robot2 nodes are gone', gone, f'{len(nodes_after)} nodes left'),
               line('robot1 nodes unchanged', sorted(r1_before) == sorted(r1_after),
                    f'{len(r1_before)} -> {len(r1_after)}'),
               line('robot1 still drivable', math.hypot(x1[0] - x0[0], x1[1] - x0[1]) > 0.1, ''),
               line('robot2 model removed from Gazebo', 'robot2' not in models
                    and 'robot1' in models, models.replace('\n', ' ')[:80])]
    return all(results)


CASES = {1: case_tf, 2: case_motion, 3: case_lift, 4: case_ports, 5: case_removal}


def main():
    """Run the selected cases."""
    wanted = [int(c) for c in sys.argv[1].split(',')] if len(sys.argv) > 1 else [1, 2, 3, 4]
    rclpy.init()
    rig = Rig()
    results = {}
    for number in wanted:
        results[number] = CASES[number](rig)
    print(f'passed {sum(results.values())}/{len(results)}: '
          f'{ {k: ("PASS" if v else "FAIL") for k, v in results.items()} }')


if __name__ == '__main__':
    main()
