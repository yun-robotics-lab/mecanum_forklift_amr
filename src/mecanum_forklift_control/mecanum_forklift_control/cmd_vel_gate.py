#!/usr/bin/env python3
"""
Velocity safety gate: cmd_vel_teleop -> cmd_vel with watchdog and limits.

The Gazebo MecanumDrive plugin keeps the last Twist forever, so a dead or silent publisher
leaves the robot driving. This node is the only publisher of `cmd_vel`: it forwards the latest
valid command (clamped) and publishes zero when no command arrived within `timeout` seconds.
The timeout uses a wall (monotonic) clock on purpose, so it also works at a low
real-time factor and does not depend on /clock.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class GateCore:
    """Pure watchdog/limit logic (no ROS), so it can be unit tested."""

    def __init__(self, timeout, max_vx, max_vy, max_wz):
        """Store the timeout (s) and the symmetric limits."""
        self.timeout = timeout
        self.limits = (max_vx, max_vy, max_wz)
        self.last = (0.0, 0.0, 0.0)
        self.last_rx = None

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def command(self, vx, vy, wz, now):
        """Register a command received at time `now`; reject non-finite values."""
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            return False
        self.last = tuple(self._clamp(v, lim) for v, lim in zip((vx, vy, wz), self.limits))
        self.last_rx = now
        return True

    def timed_out(self, now):
        """Return True if no valid command was received within the timeout."""
        return self.last_rx is None or (now - self.last_rx) > self.timeout

    def output(self, now):
        """Twist tuple to publish at time `now` (zero if timed out)."""
        return (0.0, 0.0, 0.0) if self.timed_out(now) else self.last


class CmdVelGate(Node):
    """ROS wrapper around GateCore."""

    def __init__(self):
        """Declare parameters, create the subscription, publisher and timer."""
        super().__init__('cmd_vel_gate')
        self.declare_parameter('input_topic', 'cmd_vel_teleop')
        self.declare_parameter('output_topic', 'cmd_vel')
        self.declare_parameter('timeout', 0.5)        # s without input -> zero
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('max_vx', 0.6)         # m/s (sim placeholder, confirm for hardware)
        self.declare_parameter('max_vy', 0.6)         # m/s
        self.declare_parameter('max_wz', 2.0)         # rad/s

        p = self.get_parameter
        self.core = GateCore(p('timeout').value, p('max_vx').value, p('max_vy').value,
                             p('max_wz').value)
        self.was_timed_out = True
        self.pub = self.create_publisher(Twist, p('output_topic').value, 10)
        self.create_subscription(Twist, p('input_topic').value, self.on_cmd, 10)
        self.create_timer(1.0 / p('publish_rate').value, self.tick)
        self.get_logger().info(
            f'gate {p("input_topic").value} -> {p("output_topic").value}, '
            f'timeout {self.core.timeout} s, limits {self.core.limits}')

    def on_cmd(self, msg):
        """Store the latest command."""
        if not self.core.command(msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic()):
            self.get_logger().warn('rejected non-finite cmd', throttle_duration_sec=2.0)

    def tick(self):
        """Publish the gated command at a fixed rate."""
        now = time.monotonic()
        timed_out = self.core.timed_out(now)
        if timed_out and not self.was_timed_out:
            self.get_logger().warn('command timeout: publishing zero velocity')
        self.was_timed_out = timed_out
        vx, vy, wz = self.core.output(now)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self.pub.publish(msg)


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = CmdVelGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
