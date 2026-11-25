#!/usr/bin/env python3
"""
ArUco detector: camera image -> marker poses in a stable frame (default `odom`).

Adapted in structure from the reference `forklift_aruco` node (target_marker_id parameter,
widened detector parameters, debug image). Unlike the reference, the camera->world transform is
NOT a constant matrix: the camera sits on the fork carriage, so each pose is transformed with
tf2 at the image timestamp using the live `fork_joint` state.

Topics: `aruco_marker_pose` = raw target marker pose; `aruco_dock_pose` = same position with a
level, approach-direction heading (what opennav_docking consumes).

Published marker frame: x right, y up, z out of the marker face (OpenCV convention). A robot
docking onto a marker therefore faces -z of that frame.
"""
import os
import time
from collections import deque

import cv2
from geometry_msgs.msg import PoseArray, PoseStamped
import numpy as np
import rclpy
import rclpy.clock
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CameraInfo, Image
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped transforms)
from tf2_ros import Buffer, TransformException, TransformListener

from mecanum_forklift_vision.pose_core import (
    bgr_to_image, estimate_marker_pose, expand_corners, image_to_bgr, make_detector,
    planar_approach_yaw, rvec_to_quaternion, yaw_to_quaternion)


class ArucoNode(Node):
    """Detect ArUco markers and publish their poses."""

    def __init__(self):
        """Declare parameters, build the detector, create I/O."""
        super().__init__('aruco_node')
        for name, default in (
                ('image_topic', 'camera/image_raw'),
                ('camera_info_topic', 'camera/camera_info'),
                ('dictionary', 'DICT_4X4_50'),
                ('marker_size', 0.05),
                ('target_marker_id', -1),
                ('target_frame', 'odom'),
                ('corner_expand_px', 0.0),
                ('max_tf_age', 0.5),
                ('publish_debug_image', True)):
            self.declare_parameter(name, default)
        p = self.get_parameter
        self.size = p('marker_size').value
        self.target_id = p('target_marker_id').value
        self.target_frame = p('target_frame').value
        self.max_tf_age = p('max_tf_age').value
        self.expand = p('corner_expand_px').value
        self.debug = p('publish_debug_image').value

        self.detect = make_detector(p('dictionary').value)

        self.K = None
        self.D = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.create_subscription(CameraInfo, p('camera_info_topic').value, self.on_info,
                                 qos_profile_sensor_data)
        self.image_topic = p('image_topic').value
        self.image_sub = None
        self.subscribe_image()
        self.pose_pub = self.create_publisher(PoseStamped, 'aruco_marker_pose', 10)
        self.dock_pub = self.create_publisher(PoseStamped, 'aruco_dock_pose', 10)
        self.all_pub = self.create_publisher(PoseArray, 'aruco/poses', 10)
        self.img_pub = self.create_publisher(Image, 'aruco/image', 1)
        self.pending = deque(maxlen=10)
        self.last_image = time.monotonic()
        self.last_recovery = self.last_image
        self.stalls = 0
        self.create_timer(1.0, self.check_images, clock=rclpy.clock.Clock())
        self.stats = {'images': 0, 'dets': 0, 'published': 0, 'slowest_ms': 0.0}
        self.create_timer(5.0, self.log_health, clock=rclpy.clock.Clock())
        self.create_timer(0.02, self.process_pending)
        self.add_on_set_parameters_callback(self.on_param)
        self.get_logger().info(
            f'aruco_node: {p("dictionary").value}, size {self.size} m, '
            f'target id {self.target_id}, frame {self.target_frame}')

    def on_param(self, params):
        """Allow target_marker_id and publish_debug_image to change at runtime."""
        for prm in params:
            if prm.name == 'target_marker_id':
                self.target_id = int(prm.value)
            elif prm.name == 'publish_debug_image':
                self.debug = bool(prm.value)
        return SetParametersResult(successful=True)

    def on_info(self, msg):
        """Cache intrinsics."""
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64) if msg.d else np.zeros(5)

    def subscribe_image(self):
        """(Re)create the image subscription."""
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
        self.image_sub = self.create_subscription(Image, self.image_topic, self.on_image,
                                                  qos_profile_sensor_data)

    def check_images(self):
        """
        Recover from a silent image stall: re-subscribe once, then exit so launch respawns us.

        Workaround: the image subscription sometimes stops receiving (a few minutes into a run,
        also seen when the node starts together with other nodes); a fresh process receives the
        same topic fine and re-creating the subscription does not help. Root cause not found.
        """
        now = time.monotonic()
        if self.last_image > self.last_recovery:      # an image arrived since the last attempt
            self.stalls = 0
        if now - max(self.last_image, self.last_recovery) <= 5.0:
            return
        self.stalls += 1
        self.last_recovery = now
        if self.stalls >= 2:
            self.get_logger().error('no image for 10 s: exiting so that launch respawns the node')
            os._exit(3)
        self.get_logger().warn('no image for 5 s: re-creating the image subscription')
        self.subscribe_image()

    def log_health(self):
        """Log counters every 5 s (wall clock) so a silent stall is visible."""
        self.get_logger().info(f'health {self.stats} pending {len(self.pending)}')
        self.stats = {'images': 0, 'dets': 0, 'published': 0, 'slowest_ms': 0.0}

    def on_image(self, msg):
        """Detect markers and queue their camera-frame poses (never blocks on TF)."""
        if self.K is None:
            return
        t_start = time.monotonic()
        self.last_image = t_start
        self.stats['images'] += 1
        frame = image_to_bgr(msg)
        if frame is None:
            self.get_logger().warn(f'unsupported encoding {msg.encoding}',
                                   throttle_duration_sec=5.0)
            return
        corners, ids = self.detect(frame)
        dets = []
        if ids is not None:
            for c, marker_id in zip(corners, ids.flatten()):
                est = estimate_marker_pose(expand_corners(c, self.expand), self.size,
                                           self.K, self.D)
                if est is None:
                    continue
                dets.append((int(marker_id), est[0], est[1]))
                if self.debug:
                    cv2.drawFrameAxes(frame, self.K, self.D, est[0], est[1], self.size / 2.0)
            if self.debug:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        self.get_logger().debug(f'{len(dets)} marker(s) detected',
                                throttle_duration_sec=1.0)
        if dets:
            self.pending.append((msg.header, dets))
        else:
            self.all_pub.publish(PoseArray(header=msg.header))
        if self.debug:
            self.img_pub.publish(bgr_to_image(frame, msg.header))
        self.stats['dets'] += len(dets)
        self.stats['slowest_ms'] = max(self.stats['slowest_ms'],
                                       (time.monotonic() - t_start) * 1000)

    def process_pending(self):
        """Transform queued detections once TF reaches their stamp; drop stale ones."""
        now = self.get_clock().now()
        while self.pending:
            header, dets = self.pending[0]
            stamp = rclpy.time.Time.from_msg(header.stamp)
            if (now - stamp).nanoseconds * 1e-9 > self.max_tf_age:
                self.pending.popleft()
                self.get_logger().warn('dropped detection: TF too late',
                                       throttle_duration_sec=2.0)
                continue
            if not self.tf_buffer.can_transform(self.target_frame, header.frame_id, stamp):
                return                       # TF not there yet; try again next tick
            self.pending.popleft()
            poses = PoseArray()
            poses.header.stamp = header.stamp
            poses.header.frame_id = self.target_frame
            for marker_id, rvec, tvec in dets:
                out = self.to_target(header, tvec, rvec)
                if out is None:
                    continue
                poses.poses.append(out.pose)
                if marker_id == self.target_id:
                    self.pose_pub.publish(out)
                    self.dock_pub.publish(self.planar_dock_pose(out))
            self.get_logger().debug(f'transformed {len(poses.poses)} pose(s)',
                                    throttle_duration_sec=1.0)
            self.all_pub.publish(poses)
            self.stats['published'] += 1

    @staticmethod
    def planar_dock_pose(marker):
        """Dock pose for opennav_docking: marker position, x axis = approach direction, level."""
        out = PoseStamped()
        out.header = marker.header
        out.pose.position = marker.pose.position
        o = marker.pose.orientation
        q = yaw_to_quaternion(planar_approach_yaw(o.x, o.y, o.z, o.w))
        out.pose.orientation.x, out.pose.orientation.y = q[0], q[1]
        out.pose.orientation.z, out.pose.orientation.w = q[2], q[3]
        return out

    def to_target(self, header, tvec, rvec):
        """Marker pose in the camera frame -> PoseStamped in target_frame (None on TF failure)."""
        ps = PoseStamped()
        ps.header = header
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in tvec)
        q = rvec_to_quaternion(rvec)
        ps.pose.orientation.x, ps.pose.orientation.y = q[0], q[1]
        ps.pose.orientation.z, ps.pose.orientation.w = q[2], q[3]
        try:
            return self.tf_buffer.transform(ps, self.target_frame)
        except TransformException as exc:
            self.get_logger().warn(f'TF failed: {exc}', throttle_duration_sec=2.0)
            return None


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
