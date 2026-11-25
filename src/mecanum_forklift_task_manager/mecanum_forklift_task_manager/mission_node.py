#!/usr/bin/env python3
"""
Mission manager: ExecuteMission action server over Nav2 (NavigateToPose) and DockPallet.

Structure: a ROS-free state machine (mission_core.MissionRunner) and this thin wrapper. The
wrapper owns the action clients, turns child results into StepResults, propagates cancel and
timeouts to the active child goal, confirms that the robot is stopped, publishes MissionStatus
and rejects a second mission with error_code BUSY. It never publishes velocity itself.
"""
import math
import time

from mecanum_forklift_interfaces.action import DockPallet, ExecuteMission
from mecanum_forklift_interfaces.msg import MissionStatus
from mecanum_forklift_task_manager.mission_core import (
    MissionGoal, MissionRunner, Policy, StepResult)
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import yaml


class RosExecutor:
    """Child-action access for the MissionRunner."""

    def __init__(self, node):
        """Create the clients; `node` supplies the clock, logger and callback group."""
        self.node = node
        group = node.group
        self.dock_client = ActionClient(node, DockPallet, 'dock_pallet', callback_group=group)
        self.nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose',
                                       callback_group=group)
        self.active = None                       # goal handle of the running child goal

    def now(self):
        """Node clock in seconds (sim time when use_sim_time is set)."""
        return self.node.get_clock().now().nanoseconds * 1e-9

    def call(self, client, goal, timeout, cancelled, name):
        """Send a child goal and wait; return (state, response), state done|canceled|timeout."""
        if not client.wait_for_server(timeout_sec=3.0):
            return 'unavailable', None
        sent = client.send_goal_async(goal)
        t0 = time.monotonic()
        while not sent.done():
            if time.monotonic() - t0 > 10.0:
                return 'unavailable', None
            time.sleep(0.02)
        handle = sent.result()
        if not handle.accepted:
            return 'rejected', None
        self.active = handle
        start = self.now()
        result = handle.get_result_async()
        state = 'done'
        try:
            while not result.done():
                if cancelled():
                    state = 'canceled'
                elif self.now() - start > timeout:
                    state = 'timeout'
                if state != 'done':
                    self.node.get_logger().warn(f'{name}: {state}, cancelling the child goal')
                    handle.cancel_goal_async()
                    t1 = time.monotonic()
                    while not result.done() and time.monotonic() - t1 < 20.0:
                        time.sleep(0.02)
                    break
                time.sleep(0.02)
        finally:
            self.active = None
        return state, (result.result() if result.done() else None)

    def dock(self, dock_id, timeout, cancelled):
        """Run DockPallet (staging navigation and back-out included)."""
        goal = DockPallet.Goal()
        goal.dock_id = dock_id
        goal.navigate_to_staging = True
        goal.undock_after = True
        state, response = self.call(self.dock_client, goal, timeout, cancelled,
                                    f'dock {dock_id}')
        if state == 'unavailable':
            return StepResult(False, 'SERVER_UNAVAILABLE', 'dock_pallet server not available')
        if state == 'rejected':
            return StepResult(False, 'SERVER_BUSY', 'dock_pallet rejected the goal')
        if state == 'canceled':
            return StepResult(False, 'CANCELED', f'canceled during {dock_id}')
        if state == 'timeout':
            return StepResult(False, 'TIMEOUT', f'{dock_id} exceeded {timeout:.0f} s')
        result = response.result
        if result.success:
            return StepResult(True)
        return StepResult(False, result.error_code or 'DOCK_FAILED', result.message)

    def navigate(self, pose, timeout, cancelled):
        """Run NavigateToPose to pose = (frame, x, y, yaw)."""
        frame, x, y, yaw = pose
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        state, response = self.call(self.nav_client, goal, timeout, cancelled, 'navigate')
        if state == 'unavailable':
            return StepResult(False, 'SERVER_UNAVAILABLE', 'navigate_to_pose not available')
        if state == 'rejected':
            return StepResult(False, 'NAV_FAILED', 'navigation goal rejected')
        if state == 'canceled':
            return StepResult(False, 'CANCELED', 'canceled during navigation')
        if state == 'timeout':
            return StepResult(False, 'TIMEOUT', f'navigation exceeded {timeout:.0f} s')
        if response.status == 4:
            return StepResult(True)
        return StepResult(False, 'NAV_FAILED', f'navigation ended with status {response.status}')

    def stop(self):
        """Wait (up to 3 s) until the robot is stopped (the child cancel already zeroes it)."""
        t0 = time.monotonic()
        quiet_since = None
        while time.monotonic() - t0 < 3.0:
            if self.node.speed() < 0.01:
                quiet_since = quiet_since or time.monotonic()
                if time.monotonic() - quiet_since > 0.5:
                    return
            else:
                quiet_since = None
            time.sleep(0.05)
        self.node.get_logger().warn('robot did not report standstill within 3 s')


class MissionNode(Node):
    """ExecuteMission action server."""

    def __init__(self):
        """Declare parameters, load the dock database, create the executor and the server."""
        super().__init__('mission_manager')
        for name, default in (('dock_database', ''), ('nav_timeout', 150.0),
                              ('dock_timeout', 180.0), ('mission_timeout', 600.0),
                              ('max_retries', 1), ('odom_topic', 'odom'),
                              ('status_rate', 2.0)):
            self.declare_parameter(name, default)
        p = self.get_parameter
        self.docks = {}
        if p('dock_database').value:
            with open(p('dock_database').value, encoding='utf-8') as f:
                self.docks = (yaml.safe_load(f) or {}).get('docks', {}) or {}
        self.policy = Policy(nav_timeout=p('nav_timeout').value,
                             dock_timeout=p('dock_timeout').value,
                             mission_timeout=p('mission_timeout').value,
                             max_retries=int(p('max_retries').value))
        self.group = ReentrantCallbackGroup()
        self.executor_ros = RosExecutor(self)
        self.busy = False
        self.add_on_set_parameters_callback(self.on_params)
        self.status = MissionStatus(state='IDLE')
        self.odom_speed = 0.0
        self.create_subscription(Odometry, p('odom_topic').value, self.on_odom, 10,
                                 callback_group=self.group)
        self.status_pub = self.create_publisher(MissionStatus, 'mission_status', 10)
        self.create_timer(1.0 / p('status_rate').value, self.publish_status,
                          callback_group=self.group)
        self.server = ActionServer(
            self, ExecuteMission, 'execute_mission', execute_callback=self.execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal: CancelResponse.ACCEPT, callback_group=self.group)
        self.get_logger().info(f'mission_manager ready, docks: {list(self.docks)}')

    def on_params(self, params):
        """Timeouts and retries can be changed while the node runs (used by the tests)."""
        for prm in params:
            if prm.name in ('nav_timeout', 'dock_timeout', 'mission_timeout'):
                setattr(self.policy, prm.name, float(prm.value))
            elif prm.name == 'max_retries':
                self.policy.max_retries = int(prm.value)
        return SetParametersResult(successful=True)

    def on_odom(self, msg):
        """Remember the measured planar speed."""
        v = msg.twist.twist.linear
        self.odom_speed = math.hypot(v.x, v.y) + abs(msg.twist.twist.angular.z) * 0.2

    def speed(self):
        """Return the latest planar speed in m/s (rotation weighted by a 0.2 m radius)."""
        return self.odom_speed

    def publish_status(self):
        """Publish MissionStatus."""
        self.status.header.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(self.status)

    def validate(self, goal):
        """Return '' if the mission is valid, else the reason."""
        if goal.mission_type != 'PALLET_TRANSPORT':
            return f'unknown mission_type {goal.mission_type!r}'
        for dock_id, operation in ((goal.pick_dock_id, 'pick'), (goal.drop_dock_id, 'drop')):
            entry = self.docks.get(dock_id)
            if entry is None:
                return f'unknown dock {dock_id!r}'
            if entry.get('operation', 'pick') != operation:
                return f'dock {dock_id!r} is not a {operation} dock'
        return ''

    def execute(self, goal_handle):
        """Run one mission (a second one is aborted at once with BUSY)."""
        req = goal_handle.request
        result = ExecuteMission.Result()
        if self.busy:
            result.success, result.error_code = False, 'BUSY'
            result.message = 'another mission is active'
            goal_handle.abort()
            return result
        self.busy = True
        ret = None
        if req.return_pose.header.frame_id:
            q = req.return_pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            ret = (req.return_pose.header.frame_id, req.return_pose.pose.position.x,
                   req.return_pose.pose.position.y, yaw)
        goal = MissionGoal(req.mission_id, req.mission_type, req.pick_dock_id,
                           req.drop_dock_id, ret)

        def report(state, index, total, attempt, message):
            self.status = MissionStatus(active=state not in ('DONE', 'FAILED', 'CANCELED'),
                                        mission_id=req.mission_id, state=state, step_index=index,
                                        total_steps=total, attempt=attempt, message=message)
            fb = ExecuteMission.Feedback(mission_id=req.mission_id, state=state, step_index=index,
                                         total_steps=total, message=message)
            fb.progress = 100.0 * max(index - 1, 0) / total if total else 0.0
            if state == 'DONE':
                fb.progress = 100.0
            goal_handle.publish_feedback(fb)
            self.get_logger().info(f'mission {req.mission_id}: {state} {index}/{total} '
                                   f'attempt {attempt} {message}')

        runner = MissionRunner(self.executor_ros, self.policy, self.validate,
                               self.executor_ros.now, report)
        try:
            outcome = runner.run(goal, lambda: goal_handle.is_cancel_requested)
        except Exception as exc:                     # a bug must not leave the robot driving
            self.get_logger().error(f'mission crashed: {exc!r}')
            self.executor_ros.stop()
            outcome = type('R', (), {'success': False, 'error_code': 'INTERNAL_ERROR',
                                     'failed_step': '', 'message': repr(exc)})()
        finally:
            self.busy = False
        result.success, result.error_code = outcome.success, outcome.error_code
        result.failed_step, result.message = outcome.failed_step, outcome.message
        self.status.error_code = outcome.error_code
        self.status.active = False
        if outcome.success:
            goal_handle.succeed()
        elif outcome.error_code == 'CANCELED':
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def cancel_active(self):
        """Cancel the running child goal (called on shutdown, so no orphan goal is left)."""
        handle = self.executor_ros.active
        if handle is not None:
            handle.cancel_goal_async()
            time.sleep(0.5)


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = MissionNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_active()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
