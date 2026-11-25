#!/usr/bin/env python3
"""Move the cylinder obstacles back and forth (triangle wave over simulation time)."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64


def triangle(t, amplitude, speed):
    """Offset at time t: starts at 0, moves at `speed` between -amplitude and +amplitude."""
    if amplitude <= 0.0 or speed <= 0.0:
        return 0.0
    s = (speed * t + amplitude) % (4.0 * amplitude)
    return s - amplitude if s <= 2.0 * amplitude else 3.0 * amplitude - s


class ObstacleMover(Node):
    """Publish a position setpoint for every configured obstacle."""

    def __init__(self):
        """Read the obstacle list and start the timer."""
        super().__init__('obstacle_mover')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('obstacles', ['obstacle_x', 'obstacle_y'])
        self.moving = []
        for name in self.get_parameter('obstacles').value:
            self.declare_parameter(f'{name}.amplitude', 0.0)
            self.declare_parameter(f'{name}.speed', 0.0)
            self.moving.append((self.create_publisher(Float64, f'{name}/cmd_pos', 10),
                                float(self.get_parameter(f'{name}.amplitude').value),
                                float(self.get_parameter(f'{name}.speed').value)))
        self.start = None
        self.create_timer(1.0 / self.get_parameter('rate').value, self.step)

    def step(self):
        """Publish the setpoints for the current simulation time."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:                          # no /clock yet
            return
        if self.start is None or now < self.start:
            self.start = now                    # first tick, or the simulation was reset
        for publisher, amplitude, speed in self.moving:
            publisher.publish(Float64(data=triangle(now - self.start, amplitude, speed)))


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = ObstacleMover()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()        # also releases the node


if __name__ == '__main__':
    main()
