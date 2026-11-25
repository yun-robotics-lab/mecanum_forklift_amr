"""Pure marker geometry (no ROS): square-marker PnP and pose helpers, unit testable."""
import array
import math

import cv2
import numpy as np


def marker_object_points(size):
    """Corner points of a square marker in its own frame (x right, y up, z out of the face)."""
    h = size / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float64)


def expand_corners(corners, pixels):
    """Move each corner `pixels` away from the marker centre (compensates edge blur)."""
    q = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    if pixels == 0.0:
        return q
    v = q - q.mean(axis=0)
    return q + pixels * v / np.linalg.norm(v, axis=1, keepdims=True)


def estimate_marker_pose(corners, size, camera_matrix, dist_coeffs):
    """Return (rvec, tvec, reproj_error_px) of a marker in the camera optical frame, or None."""
    obj = marker_object_points(size)
    img = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    ok, rvec, tvec = cv2.solvePnP(obj, img, camera_matrix, dist_coeffs,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok or not np.all(np.isfinite(tvec)) or tvec[2, 0] <= 0.0:
        return None
    proj, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist_coeffs)
    err = float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - img) ** 2, axis=1))))
    return rvec.reshape(3), tvec.reshape(3), err


def rvec_to_quaternion(rvec):
    """Rodrigues vector -> quaternion (x, y, z, w)."""
    angle = float(np.linalg.norm(rvec))
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    axis = np.asarray(rvec, dtype=np.float64) / angle
    s = math.sin(angle / 2.0)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2.0))


def planar_approach_yaw(x, y, z, w):
    """
    Heading (rad about +z) a robot must have to drive straight into a marker.

    The marker z axis points out of its face toward the viewer, so the approach direction is
    -z. Only its horizontal projection is used; marker roll/pitch noise is ignored (vertical
    marker, level floor).
    """
    nx = 2.0 * (x * z + y * w)           # marker z axis expressed in the target frame
    ny = 2.0 * (y * z - x * w)
    return math.atan2(-ny, -nx)


def yaw_to_quaternion(yaw):
    """Quaternion (x, y, z, w) of a pure yaw."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def yaw_from_quaternion(x, y, z, w):
    """Yaw (rad) of a quaternion about +z."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def make_detector(dictionary_name):
    """Return detect(frame) -> (corners, ids) working on OpenCV 4.5 (legacy) and >= 4.7 APIs."""
    dict_id = getattr(cv2.aruco, dictionary_name, None)
    if dict_id is None:
        raise ValueError(f'unknown ArUco dictionary {dictionary_name}')
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        params = cv2.aruco.DetectorParameters()
    else:
        params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.minMarkerPerimeterRate = 0.01   # accept small / distant markers
    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector = cv2.aruco.ArucoDetector(dictionary, params)

        def detect(frame):
            corners, ids, _ = detector.detectMarkers(frame)
            return corners, ids
    else:
        def detect(frame):
            corners, ids, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=params)
            return corners, ids
    return detect


def draw_marker_image(dictionary_name, marker_id, pixels):
    """Render a marker bitmap on OpenCV 4.5 and >= 4.7."""
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, pixels)
    return cv2.aruco.drawMarker(dictionary, marker_id, pixels)


def image_to_bgr(msg):
    """sensor_msgs/Image (rgb8, bgr8, mono8) -> contiguous BGR uint8 array, None if unsupported."""
    channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(msg.encoding)
    if channels is None:
        return None
    data = np.frombuffer(msg.data, dtype=np.uint8)
    data = data.reshape(msg.height, msg.step)[:, :msg.width * channels]
    if channels == 1:
        return cv2.cvtColor(data.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    frame = data.reshape(msg.height, msg.width, 3)
    return np.ascontiguousarray(frame[:, :, ::-1] if msg.encoding == 'rgb8' else frame)


def bgr_to_image(frame, header):
    """BGR uint8 array -> sensor_msgs/Image (bgr8) with the given header."""
    from sensor_msgs.msg import Image
    msg = Image()
    msg.header = header
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = 'bgr8'
    msg.step = msg.width * 3
    # array.array is ~25000x faster to assign than bytes in rclpy (5 us vs 130 ms per frame)
    msg.data = array.array('B', np.ascontiguousarray(frame).tobytes())
    return msg
