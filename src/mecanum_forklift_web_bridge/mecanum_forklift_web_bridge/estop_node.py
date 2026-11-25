#!/usr/bin/env python3
"""
Software e-stop: a latched state that holds the robot at zero velocity and cancels all goals.

Independent of the mission layer and of the REST API (a separate process). While engaged it
publishes zero Twists at 20 Hz on `cmd_vel_estop`, the highest-priority input of twist_mux (200,
above teleop), and every second it asks the action servers of the stack to cancel all their goals
(so goals accepted afterwards are cancelled too). Release is a ROS service (`/estop/release`, call
it on the robot); the REST API has no release. This is a simulation-level software stop; a real
robot needs a hardware e-stop.
"""
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class EstopNode(Node):
    """Latched software e-stop."""

    def __init__(self):
        """Create publishers, services and the cancel clients."""
        super().__init__('estop_node')
        self.declare_parameter('velocity_topic', 'cmd_vel_estop')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('recancel_period', 1.0)
        self.declare_parameter('cancel_actions', ['execute_mission', 'dock_pallet',
                                                  'navigate_to_pose', 'set_lift_height'])
        p = self.get_parameter
        self.engaged = False
        self.vel_pub = self.create_publisher(Twist, p('velocity_topic').value, 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(Bool, 'estop/state', latched)
        self.create_service(Trigger, 'estop/engage', self.on_engage)
        self.create_service(Trigger, 'estop/release', self.on_release)
        self.cancel_clients = [self.create_client(CancelGoal, f'{name}/_action/cancel_goal')
                               for name in p('cancel_actions').value]
        self.create_timer(1.0 / p('rate').value, self.tick)
        self.create_timer(p('recancel_period').value, self.cancel_all_if_engaged)
        self.create_timer(1.0, self.publish_state)
        self.publish_state()
        self.get_logger().info('estop_node ready (released)')

    def publish_state(self):
        """Publish the latched state."""
        self.state_pub.publish(Bool(data=self.engaged))

    def tick(self):
        """Hold zero velocity while engaged."""
        if self.engaged:
            self.vel_pub.publish(Twist())

    def cancel_all(self):
        """Ask every configured action server to cancel all goals."""
        for client in self.cancel_clients:
            if client.service_is_ready():
                client.call_async(CancelGoal.Request())      # zero id and stamp: cancel all

    def cancel_all_if_engaged(self):
        """Repeat the cancel so goals accepted after the e-stop are cancelled as well."""
        if self.engaged:
            self.cancel_all()

    def on_engage(self, request, response):
        """Engage (idempotent)."""
        self.engaged = True
        self.vel_pub.publish(Twist())
        self.cancel_all()
        self.publish_state()
        self.get_logger().warn('E-STOP ENGAGED')
        response.success, response.message = True, 'engaged'
        return response

    def on_release(self, request, response):
        """Release (local operator only: this service is not exposed to the web)."""
        self.engaged = False
        self.publish_state()
        self.get_logger().warn('e-stop released')
        response.success, response.message = True, 'released'
        return response


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = EstopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
