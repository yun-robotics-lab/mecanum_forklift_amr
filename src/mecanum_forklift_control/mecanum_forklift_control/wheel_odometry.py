#!/usr/bin/env python3
"""
Wheel odometry for the 4-wheel mecanum base.

Pose is integrated from wheel joint position increments (robust to message timing); the twist
comes from the joint velocities. Publishes nav_msgs/Odometry on `odom` and, if `publish_tf`
is true, the odom -> base_footprint transform. This node is the single owner of that
transform: do not run it together with another publisher of it (EKF, simulator odometry).
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from mecanum_forklift_control.kinematics import forward_kinematics, integrate_pose

WHEELS = ('front_left', 'front_right', 'rear_left', 'rear_right')


class WheelOdometry(Node):
    """Integrates wheel joint states into an odometry estimate."""

    def __init__(self):
        """Declare parameters and create the subscription/publishers."""
        super().__init__('wheel_odometry')
        self.declare_parameter('wheel_radius', 0.039)
        self.declare_parameter('wheel_separation', 0.261)   # left-right distance (m)
        self.declare_parameter('wheelbase', 0.22)           # front-rear distance (m)
        self.declare_parameter('joint_names', ['fl_wheel_joint', 'fr_wheel_joint',
                                               'rl_wheel_joint', 'rr_wheel_joint'])
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 50.0)        # Hz (stamp time)
        self.declare_parameter('pose_variance', 0.01)
        self.declare_parameter('twist_variance', 0.01)

        p = self.get_parameter
        self.r = p('wheel_radius').value
        self.lx = p('wheelbase').value / 2.0
        self.ly = p('wheel_separation').value / 2.0
        self.joints = list(p('joint_names').value)
        self.odom_frame = p('odom_frame').value
        self.base_frame = p('base_frame').value
        self.publish_tf = p('publish_tf').value
        self.period = 1.0 / p('publish_rate').value
        self.pose_var = p('pose_variance').value
        self.twist_var = p('twist_variance').value

        self.x = self.y = self.yaw = 0.0
        self.prev_pos = None
        self.last_pub = None
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(JointState, 'joint_states', self.on_joints, 50)
        self.get_logger().info(f'wheel odometry on {self.joints}, publish_tf={self.publish_tf}')

    def _lookup(self, msg, values):
        """Return the 4 wheel values in wheel order, or None if a joint is missing."""
        try:
            return tuple(values[msg.name.index(j)] for j in self.joints)
        except (ValueError, IndexError):
            return None

    def on_joints(self, msg):
        """Update the pose from a joint state message and publish at the configured rate."""
        pos = self._lookup(msg, msg.position)
        vel = self._lookup(msg, msg.velocity)
        if pos is None or vel is None:
            self.get_logger().warn('wheel joints missing in joint_states',
                                   throttle_duration_sec=5.0)
            return
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        if self.prev_pos is not None:
            delta = tuple(a - b for a, b in zip(pos, self.prev_pos))
            dx, dy, dyaw = forward_kinematics(delta, self.r, self.lx, self.ly)
            self.x, self.y, self.yaw = integrate_pose(self.x, self.y, self.yaw, dx, dy, dyaw)
        self.prev_pos = pos
        if self.last_pub is not None and (t - self.last_pub) < self.period:
            if t < self.last_pub:      # clock went backwards (sim reset)
                self.last_pub = t
            return
        self.last_pub = t
        self.publish(stamp, forward_kinematics(vel, self.r, self.lx, self.ly))

    def publish(self, stamp, twist):
        """Publish the Odometry message and optionally the TF."""
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = twist
        big = 1e6
        odom.pose.covariance[0] = odom.pose.covariance[7] = self.pose_var
        odom.pose.covariance[14] = odom.pose.covariance[21] = odom.pose.covariance[28] = big
        odom.pose.covariance[35] = self.pose_var
        odom.twist.covariance[0] = odom.twist.covariance[7] = self.twist_var
        odom.twist.covariance[14] = odom.twist.covariance[21] = odom.twist.covariance[28] = big
        odom.twist.covariance[35] = self.twist_var
        self.pub.publish(odom)
        if self.tf is not None:
            tfm = TransformStamped()
            tfm.header = odom.header
            tfm.child_frame_id = self.base_frame
            tfm.transform.translation.x = self.x
            tfm.transform.translation.y = self.y
            tfm.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(tfm)


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
