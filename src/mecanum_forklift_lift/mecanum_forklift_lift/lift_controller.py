#!/usr/bin/env python3
"""
Fork lift controller node.

Serves the `set_lift_height` action (mecanum_forklift_interfaces/SetLiftHeight), accepts clamped
teleop targets on `fork/cmd_teleop`, drives the Gazebo position controller on `fork/cmd_pos`,
publishes `lift_state` (LiftState) and offers `lift_reset_fault` (std_srvs/Trigger).
Simulation only: the real actuator and its feedback are unknown.
"""
import threading
import time

import rclpy
from mecanum_forklift_interfaces.action import SetLiftHeight
from mecanum_forklift_interfaces.msg import LiftState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from mecanum_forklift_lift import lift_core as lc

_FAULT_TO_RESULT = {
    lc.FAULT_TIMEOUT: SetLiftHeight.Result.RESULT_TIMEOUT,
    lc.FAULT_OVER_LIMIT: SetLiftHeight.Result.RESULT_REJECTED_OUT_OF_RANGE,
    lc.FAULT_NO_FEEDBACK: SetLiftHeight.Result.RESULT_NO_FEEDBACK,
    lc.FAULT_STALL: SetLiftHeight.Result.RESULT_STALL,
    lc.FAULT_SAFETY_STOP: SetLiftHeight.Result.RESULT_SAFETY_STOP,
}


class LiftController(Node):
    def __init__(self):
        super().__init__('lift_controller')
        d = {
            'joint_name': 'fork_joint', 'cmd_topic': 'fork/cmd_pos',
            'teleop_topic': 'fork/cmd_teleop', 'min_height': 0.0, 'max_height': 0.12,
            'default_speed': 0.05, 'max_speed': 0.1, 'tolerance': 0.002, 'settle_time': 0.3,
            'timeout_margin': 3.0, 'feedback_timeout': 1.0, 'stall_error': 0.02,
            'stall_time': 1.5, 'control_rate': 50.0, 'state_rate': 10.0,
        }
        for k, v in d.items():
            self.declare_parameter(k, v)
        p = {k: self.get_parameter(k).value for k in d}
        self.joint_name = p['joint_name']
        self.core = lc.LiftCore(
            p['min_height'], p['max_height'], p['default_speed'], p['max_speed'],
            p['tolerance'], p['settle_time'], p['timeout_margin'], p['feedback_timeout'],
            p['stall_error'], p['stall_time'])
        self.lock = threading.Lock()
        self.action_active = False
        self.velocity = 0.0
        self._prev = None
        self._last_state = None

        cb = ReentrantCallbackGroup()
        self.cmd_pub = self.create_publisher(Float64, p['cmd_topic'], 10)
        self.state_pub = self.create_publisher(LiftState, 'lift_state', 10)
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10,
                                 callback_group=cb)
        self.create_subscription(Float64, p['teleop_topic'], self.on_teleop, 10,
                                 callback_group=cb)
        self.create_service(Trigger, 'lift_reset_fault', self.on_reset, callback_group=cb)
        self.create_timer(1.0 / p['control_rate'], self.control_step, callback_group=cb)
        self.create_timer(1.0 / p['state_rate'], self.publish_state, callback_group=cb)
        self.server = ActionServer(
            self, SetLiftHeight, 'set_lift_height', execute_callback=self.execute,
            goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=cb)
        self.get_logger().info('lift_controller ready (waiting for /joint_states to home).')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_joint_states(self, msg):
        if self.joint_name not in msg.name:
            return
        h = float(msg.position[msg.name.index(self.joint_name)])
        t = self.now()
        with self.lock:
            if self._prev is not None and t > self._prev[0]:
                self.velocity = (h - self._prev[1]) / (t - self._prev[0])
            self._prev = (t, h)
            self.core.feedback(h, t)

    def on_teleop(self, msg):
        with self.lock:
            if not self.action_active:
                self.core.set_teleop_target(msg.data, self.now())

    def on_reset(self, _req, resp):
        with self.lock:
            resp.success = self.core.reset_fault(self.now())
        resp.message = 'fault cleared, holding height' if resp.success else \
            'not in FAULT or no fresh feedback'
        return resp

    def control_step(self):
        with self.lock:
            prev_state = self.core.state
            cmd = self.core.step(self.now())
            state = self.core.state
        if cmd is not None:
            self.cmd_pub.publish(Float64(data=float(cmd)))
        if state != prev_state:
            self.get_logger().info(f'lift state {prev_state} -> {state} '
                                   f'(fault {self.core.fault})')

    def publish_state(self):
        with self.lock:
            c = self.core
            msg = LiftState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.state = c.state
            msg.fault_code = c.fault
            msg.height = float(c.height if c.height is not None else 0.0)
            msg.velocity = float(self.velocity)
            msg.target_height = float(c.target if c.state in (lc.MOVING, lc.HOMING)
                                      else msg.height)
            fresh = (c.last_feedback is not None
                     and self.now() - c.last_feedback <= c.feedback_timeout)
            msg.feedback_available = bool(fresh)
        self.state_pub.publish(msg)

    def on_goal(self, goal):
        return GoalResponse.ACCEPT

    def _result(self, code, message):
        r = SetLiftHeight.Result()
        r.success = code == SetLiftHeight.Result.RESULT_SUCCESS
        r.error_code = code
        with self.lock:
            r.final_height = float(self.core.height if self.core.height is not None else 0.0)
        r.message = message
        return r

    def execute(self, goal_handle):
        g = goal_handle.request
        with self.lock:
            if self.action_active:
                rc = lc.REJECTED_NOT_IDLE
            else:
                rc = self.core.set_goal(g.target_height, g.speed, self.now())
                if rc == lc.OK:
                    self.action_active = True
        if rc != lc.OK:
            goal_handle.abort()
            if rc == lc.REJECTED_NOT_IDLE:
                return self._result(SetLiftHeight.Result.RESULT_REJECTED_NOT_IDLE,
                                    'lift is not IDLE')
            return self._result(SetLiftHeight.Result.RESULT_REJECTED_OUT_OF_RANGE,
                                'target or speed out of range')
        try:
            return self._run(goal_handle)
        finally:
            with self.lock:
                self.action_active = False

    def _run(self, goal_handle):
        fb = SetLiftHeight.Feedback()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                with self.lock:
                    self.core.cancel()
                goal_handle.canceled()
                return self._result(SetLiftHeight.Result.RESULT_CANCELED, 'canceled, holding')
            with self.lock:
                st, fault = self.core.state, self.core.fault
                h = self.core.height or 0.0
                target = self.core.target
            fb.current_height = float(h)
            fb.remaining = float(target - h)
            goal_handle.publish_feedback(fb)
            if st == lc.FAULT:
                goal_handle.abort()
                return self._result(_FAULT_TO_RESULT.get(fault, 5), f'lift fault {fault}')
            if st == lc.IDLE:
                goal_handle.succeed()
                return self._result(SetLiftHeight.Result.RESULT_SUCCESS, 'reached')
            time.sleep(0.1)
        goal_handle.abort()
        return self._result(SetLiftHeight.Result.RESULT_SAFETY_STOP, 'shutdown')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = LiftController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
