#!/usr/bin/env python3
"""PS4 (DualShock 4) teleop for the mecanum forklift, using sensor_msgs/Joy from `joy_node`.

Default mapping (Linux hid-playstation/hid-sony driver, all indices are parameters):
  Hold L1 (enable)        -> required to move (deadman switch)
  Left stick up/down      -> vx   (forward / back)
  Left stick left/right   -> vy   (strafe left / right)
  Right stick left/right  -> wz   (rotate)
  R1 (hold)               -> turbo
  D-pad up / down         -> speed scale +/-
  Triangle / Cross        -> raise / lower fork (while held)
  Square                  -> fork to bottom
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


def _get(seq, index, default=0.0):
    return seq[index] if 0 <= index < len(seq) else default


class PS4Node(Node):
    def __init__(self):
        super().__init__('ps4_node')
        # axes
        self.declare_parameter('axis_vx', 1)
        self.declare_parameter('axis_vy', 0)
        self.declare_parameter('axis_wz', 3)
        self.declare_parameter('axis_dpad_vertical', 7)
        # buttons
        self.declare_parameter('button_enable', 4)       # L1
        self.declare_parameter('button_turbo', 5)        # R1
        self.declare_parameter('button_fork_up', 2)      # Triangle
        self.declare_parameter('button_fork_down', 0)    # Cross
        self.declare_parameter('button_fork_reset', 3)   # Square
        # limits
        self.declare_parameter('linear_speed', 0.3)      # m/s at full stick
        self.declare_parameter('angular_speed', 1.0)     # rad/s at full stick
        self.declare_parameter('turbo_scale', 2.0)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('fork_rate', 0.04)        # m/s while held
        self.declare_parameter('fork_min', 0.0)
        self.declare_parameter('fork_max', 0.12)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('joy_timeout', 0.5)       # s; no /joy for this long -> stop

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.p = p
        self.scale = 1.0
        self.fork_target = 0.0
        self.joy = None
        self.joy_time = None
        self.was_moving = False
        self.dpad_prev = 0.0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        self.fork_pub = self.create_publisher(Float64, 'fork/cmd_teleop', 10)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)

        self.dt = 1.0 / p('publish_rate')
        self.create_timer(self.dt, self.update)
        self.get_logger().info('PS4 teleop ready: hold L1 to drive, Triangle/Cross for the fork.')

    def on_joy(self, msg):
        self.joy = msg
        self.joy_time = self.get_clock().now()

    def _stick(self, axes, name):
        value = _get(axes, self.p(name))
        return 0.0 if abs(value) < self.p('deadzone') else value

    def update(self):
        if self.joy is None:
            return
        age = (self.get_clock().now() - self.joy_time).nanoseconds * 1e-9
        if age > self.p('joy_timeout'):
            # joystick feed lost: treat as released (deadman) instead of replaying the last Joy
            if self.was_moving:
                self.cmd_pub.publish(Twist())
                self.was_moving = False
                self.get_logger().warn('no /joy for %.1f s: stopping' % age)
            return
        axes, buttons = self.joy.axes, self.joy.buttons

        # speed scale from D-pad (edge triggered)
        dpad = _get(axes, self.p('axis_dpad_vertical'))
        if dpad != self.dpad_prev and dpad != 0.0:
            self.scale = min(max(self.scale + 0.1 * dpad, 0.2), 2.0)
            self.get_logger().info(f'speed scale: {self.scale:.1f}')
        self.dpad_prev = dpad

        # fork
        up = _get(buttons, self.p('button_fork_up'), 0)
        down = _get(buttons, self.p('button_fork_down'), 0)
        if _get(buttons, self.p('button_fork_reset'), 0):
            self.fork_target = self.p('fork_min')
        self.fork_target += (up - down) * self.p('fork_rate') * self.dt
        self.fork_target = min(max(self.fork_target, self.p('fork_min')), self.p('fork_max'))
        if up or down or _get(buttons, self.p('button_fork_reset'), 0):
            self.fork_pub.publish(Float64(data=self.fork_target))

        # drive (deadman)
        enabled = bool(_get(buttons, self.p('button_enable'), 0))
        if enabled:
            gain = self.scale * (self.p('turbo_scale') if _get(buttons, self.p('button_turbo'), 0) else 1.0)
            cmd = Twist()
            cmd.linear.x = self._stick(axes, 'axis_vx') * self.p('linear_speed') * gain
            cmd.linear.y = self._stick(axes, 'axis_vy') * self.p('linear_speed') * gain
            cmd.angular.z = self._stick(axes, 'axis_wz') * self.p('angular_speed') * gain
            self.cmd_pub.publish(cmd)
            self.was_moving = True
        elif self.was_moving:
            self.cmd_pub.publish(Twist())  # one zero command on release
            self.was_moving = False


def main(args=None):
    # keep rclpy's own SIGINT handler off: it would invalidate the context before the final zero
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PS4Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        time.sleep(0.1)  # let the zero command leave before the node is destroyed
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
