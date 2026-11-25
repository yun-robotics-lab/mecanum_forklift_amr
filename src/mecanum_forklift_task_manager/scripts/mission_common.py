"""Shared helpers of the mission test scripts (development tools, not installed)."""
import math
import os
import sys
import time

from action_msgs.msg import GoalStatusArray
from mecanum_forklift_interfaces.action import ExecuteMission
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                                'mecanum_forklift_docking', 'scripts'))
from dock_trials import (NS, pallet_state, place_pallet, remove_box,  # noqa: E402
                         set_pose, spawn_box, world_objects)
from pick_trials import OURS, Pick  # noqa: E402

PAD = (2.7, -1.6)
START = (0.0, -1.0, 0)
HOME = (0.0, 0.0, 0)
__all__ = ['NS', 'PAD', 'START', 'HOME', 'OURS', 'Rig', 'pallet_state', 'place_pallet',
           'remove_box',
           'set_pose', 'spawn_box', 'world_objects']


class Rig(Pick):
    """Pick (ground truth, Nav2, footprint, map helpers) plus the mission client."""

    def __init__(self):
        """Create the mission client and the child-goal status monitors."""
        super().__init__()
        self.mission = ActionClient(self, ExecuteMission, 'execute_mission')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.child_status = {}
        for name in ('dock_pallet', 'navigate_to_pose'):
            self.create_subscription(
                GoalStatusArray, f'{name}/_action/status',
                lambda m, n=name: self.child_status.__setitem__(
                    n, [s.status for s in m.status_list]), qos)

    def orphan_goals(self):
        """Names of child actions that still have an accepted / executing / canceling goal."""
        self.spin(0.5)
        return [n for n, states in self.child_status.items()
                if any(s in (1, 2, 3) for s in states)]

    def reset_world(self, navigate=True):
        """
        Reset the world: fork down, bare footprint, full map, pallet in its slot, robot at start.

        The robot is driven back with teleop (it may be left inside painted obstacle cells, e.g.
        after a cancelled pick, where Nav2 cannot plan out), then checked against the start pose.
        """
        self.fork(0.0)
        self.set_unloaded_footprint()
        place_pallet(0.0, 0.0)
        self.restore_full_map()
        if not navigate:
            return 'SKIPPED'
        self.goto(START[0], START[1], math.radians(START[2]))
        x, y, _ = self.pose()
        if math.hypot(x - START[0], y - START[1]) > 0.05:
            raise RuntimeError(f'reset failed: robot at ({x:.2f}, {y:.2f})')
        return 'SUCCEEDED'

    def send_mission(self, pick='pallet_pick_1', drop='pallet_drop_1', ret=HOME, mission_id='m'):
        """Send a mission; return (goal handle or None, feedback log list)."""
        self.mission.wait_for_server()
        goal = ExecuteMission.Goal()
        goal.mission_id = mission_id
        goal.mission_type = 'PALLET_TRANSPORT'
        goal.pick_dock_id, goal.drop_dock_id = pick, drop
        if ret is not None:
            goal.return_pose.header.frame_id = 'map'
            goal.return_pose.pose.position.x, goal.return_pose.pose.position.y = ret[0], ret[1]
            goal.return_pose.pose.orientation.z = math.sin(math.radians(ret[2]) / 2)
            goal.return_pose.pose.orientation.w = math.cos(math.radians(ret[2]) / 2)
        log = []
        sent = self.mission.send_goal_async(
            goal, feedback_callback=lambda m: log.append((m.feedback.state, m.feedback.step_index,
                                                          m.feedback.message)))
        while not sent.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        handle = sent.result()
        return (handle if handle.accepted else None), log

    def finish(self, handle, timeout=900):
        """Wait for the mission result; None on hang."""
        res = handle.get_result_async()
        t0 = time.time()
        while not res.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        return res.result().result if res.done() else None

    def stopped(self, seconds=2.0):
        """Displacement over an interval after a 1 s settle (should be ~0)."""
        self.spin(1.0)
        x0, y0, _ = self.pose()
        self.spin(seconds)
        x1, y1, _ = self.pose()
        return math.hypot(x1 - x0, y1 - y0) < 0.002
