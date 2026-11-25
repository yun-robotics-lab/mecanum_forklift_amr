#!/usr/bin/env python3
"""Keyboard teleop for the mecanum forklift.

Publishes geometry_msgs/Twist on `cmd_vel_teleop` (vx, vy, wz; goes through cmd_vel_gate) and the fork target
height (std_msgs/Float64, metres) on `fork/cmd_teleop` (the lift controller clamps it and drives `fork/cmd_pos`).
"""
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

HELP = """
Mecanum forklift keyboard teleop
--------------------------------
Move (latched until changed):      Rotate:
      w                              q : turn left
   a  s  d   (a/d = strafe)          e : turn right
      x : back

   space / k : stop

Fork:  r : raise    f : lower    v : fork to bottom
Speed: t / g : linear speed +/- 10%     y / h : angular speed +/- 10%

CTRL-C to quit
"""

# key -> (vx, vy, wz) direction
MOVE_KEYS = {
    'w': (1.0, 0.0, 0.0),
    'x': (-1.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0),
    'd': (0.0, -1.0, 0.0),
    'q': (0.0, 0.0, 1.0),
    'e': (0.0, 0.0, -1.0),
}
STOP_KEYS = (' ', 'k', 's')


class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.declare_parameter('linear_speed', 0.2)    # m/s
        self.declare_parameter('angular_speed', 0.8)   # rad/s
        self.declare_parameter('fork_step', 0.01)      # m per key press
        self.declare_parameter('fork_min', 0.0)
        self.declare_parameter('fork_max', 0.12)
        self.declare_parameter('publish_rate', 20.0)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.fork_step = self.get_parameter('fork_step').value
        self.fork_min = self.get_parameter('fork_min').value
        self.fork_max = self.get_parameter('fork_max').value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        self.fork_pub = self.create_publisher(Float64, 'fork/cmd_teleop', 10)

        self.direction = (0.0, 0.0, 0.0)
        self.fork_target = 0.0

        self.create_timer(1.0 / self.get_parameter('publish_rate').value, self.publish)

    def publish(self):
        vx, vy, wz = self.direction
        msg = Twist()
        msg.linear.x = vx * self.linear_speed
        msg.linear.y = vy * self.linear_speed
        msg.angular.z = wz * self.angular_speed
        self.cmd_pub.publish(msg)

    def publish_fork(self):
        self.fork_pub.publish(Float64(data=self.fork_target))

    def handle_key(self, key):
        if key in MOVE_KEYS:
            self.direction = MOVE_KEYS[key]
        elif key in STOP_KEYS:
            self.direction = (0.0, 0.0, 0.0)
        elif key in ('r', 'f', 'v'):
            if key == 'r':
                self.fork_target += self.fork_step
            elif key == 'f':
                self.fork_target -= self.fork_step
            else:
                self.fork_target = self.fork_min
            self.fork_target = min(max(self.fork_target, self.fork_min), self.fork_max)
            self.publish_fork()
            print(f'\rfork target: {self.fork_target:.3f} m      ', end='', flush=True)
        elif key in ('t', 'g'):
            self.linear_speed *= 1.1 if key == 't' else 0.9
            print(f'\rlinear speed: {self.linear_speed:.2f} m/s      ', end='', flush=True)
        elif key in ('y', 'h'):
            self.angular_speed *= 1.1 if key == 'y' else 0.9
            print(f'\rangular speed: {self.angular_speed:.2f} rad/s      ', end='', flush=True)

    def stop(self):
        self.direction = (0.0, 0.0, 0.0)
        self.cmd_pub.publish(Twist())


def read_key(timeout):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if ready else ''


def main(args=None):
    if not sys.stdin.isatty():
        print('keyboard_node needs an interactive terminal (use `ros2 run`, not ros2 launch).')
        return

    settings = termios.tcgetattr(sys.stdin)
    # keep rclpy's own SIGINT handler off: it would invalidate the context before the final zero
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardNode()
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key(0.02)
            if key == '\x03':  # CTRL-C
                break
            if key:
                node.handle_key(key.lower())
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # always restore the terminal
        try:
            node.stop()
            time.sleep(0.1)  # let the zero command leave before the node is destroyed
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
