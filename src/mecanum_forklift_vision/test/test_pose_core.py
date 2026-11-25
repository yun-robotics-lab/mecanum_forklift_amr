import cv2
import numpy as np

from mecanum_forklift_vision.pose_core import (
    estimate_marker_pose, marker_object_points, rvec_to_quaternion, yaw_from_quaternion)

K = np.array([[500.0, 0, 320.0], [0, 500.0, 240.0], [0, 0, 1.0]])
D = np.zeros(5)


def project(size, rvec, tvec):
    pts, _ = cv2.projectPoints(marker_object_points(size), np.array(rvec, float),
                               np.array(tvec, float), K, D)
    return pts.reshape(4, 2)


def test_round_trip_pose():
    rvec, tvec = [0.1, -0.3, 0.05], [0.04, -0.02, 0.8]
    est = estimate_marker_pose(project(0.05, rvec, tvec), 0.05, K, D)
    assert est is not None
    assert np.allclose(est[1], tvec, atol=1e-4)
    assert np.allclose(est[0], rvec, atol=1e-3)
    assert est[2] < 1e-3


def test_quaternion_yaw():
    q = rvec_to_quaternion([0.0, 0.0, 0.5])
    assert abs(yaw_from_quaternion(*q) - 0.5) < 1e-9
    assert rvec_to_quaternion([0, 0, 0]) == (0.0, 0.0, 0.0, 1.0)


def test_detector_finds_rendered_marker():
    from mecanum_forklift_vision.pose_core import draw_marker_image, make_detector
    img = draw_marker_image('DICT_4X4_50', 3, 200)
    img = cv2.copyMakeBorder(img, 60, 60, 60, 60, cv2.BORDER_CONSTANT, value=255)
    corners, ids = make_detector('DICT_4X4_50')(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
    assert ids is not None and list(ids.flatten()) == [3]


def test_image_round_trip():
    from sensor_msgs.msg import Image
    from std_msgs.msg import Header

    from mecanum_forklift_vision.pose_core import bgr_to_image, image_to_bgr
    frame = np.random.randint(0, 255, (6, 8, 3), dtype=np.uint8)
    msg = bgr_to_image(frame, Header())
    assert np.array_equal(image_to_bgr(msg), frame)
    rgb = Image()
    rgb.height, rgb.width, rgb.encoding, rgb.step = 6, 8, 'rgb8', 24
    rgb.data = frame[:, :, ::-1].tobytes()
    assert np.array_equal(image_to_bgr(rgb), frame)


def test_expand_corners():
    from mecanum_forklift_vision.pose_core import expand_corners
    q = project(0.05, [0, 0, 0], [0, 0, 0.5])
    grown = expand_corners(q, 1.0)
    assert np.allclose(grown.mean(axis=0), q.mean(axis=0))
    d0 = np.linalg.norm(q - q.mean(axis=0), axis=1)
    d1 = np.linalg.norm(grown - grown.mean(axis=0), axis=1)
    assert np.allclose(d1 - d0, 1.0)
    assert np.array_equal(expand_corners(q, 0.0), q)


def test_planar_approach_yaw():
    import math

    from mecanum_forklift_vision.pose_core import planar_approach_yaw, yaw_to_quaternion
    # marker (z = face normal) facing -x in the world, as pallet_1: rotation columns
    # x=(0,-1,0) y=(0,0,1) z=(-1,0,0) -> quaternion of that matrix
    r = np.array([[0, 0, -1.0], [-1.0, 0, 0], [0, 1.0, 0]])
    rvec, _ = cv2.Rodrigues(r)
    q = rvec_to_quaternion(rvec.reshape(3))
    assert abs(planar_approach_yaw(*q)) < 1e-9          # approach +x
    # same marker turned 30 deg about the vertical: approach direction follows
    c, s = math.cos(math.radians(30)), math.sin(math.radians(30))
    rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    rvec2, _ = cv2.Rodrigues(rz @ r)
    q2 = rvec_to_quaternion(rvec2.reshape(3))
    assert abs(planar_approach_yaw(*q2) - math.radians(30)) < 1e-9
    assert abs(yaw_from_quaternion(*yaw_to_quaternion(0.4)) - 0.4) < 1e-9
