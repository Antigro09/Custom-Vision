import cv2
import numpy as np

from custom_vision.localization import estimate_tag_pose
from custom_vision.overlay import annotate


def fixture():
    calibration = {"width": 640, "height": 480,
                   "camera_matrix": [[600., 0., 320.], [0., 600., 240.], [0., 0., 1.]],
                   "dist_coeffs": [0.] * 5}
    h = .1651 / 2
    points = np.array([[-h, h, 0.], [h, h, 0.], [h, -h, 0.], [-h, -h, 0.]])
    corners = cv2.projectPoints(points, np.array([.1, -.2, 2.9]), np.array([.08, .02, 1.]),
                                np.array(calibration["camera_matrix"]), np.zeros(5))[0].reshape(4, 2)
    detection = {"id": 7, "corners": corners.tolist(), "center": corners.mean(axis=0).tolist()}
    detection.update(estimate_tag_pose(corners, .1651, calibration["camera_matrix"], calibration["dist_coeffs"]))
    return calibration, detection


def test_overlay_projects_3d_box_in_raw_pixels_without_mutating_frame():
    calibration, detection = fixture()
    frame = np.zeros((480, 640, 3), np.uint8)
    annotated = annotate(frame, [detection], calibration)
    two_d = annotate(frame, [detection])
    assert not frame.any()
    assert np.count_nonzero(annotated) > np.count_nonzero(two_d)
    assert np.count_nonzero(annotated != two_d) > 100
    # Top box corners extend from tag toward camera (negative raw +Z).
    h = .1651 / 2
    top = np.array([[-h, h, -h], [h, h, -h], [h, -h, -h], [-h, -h, -h]])
    projected = cv2.projectPoints(top, np.array(detection["rvec_rad"]), np.array(detection["tvec_m"]),
                                  np.array(calibration["camera_matrix"]), np.zeros(5))[0].reshape(4, 2)
    for point in projected:
        x, y = np.rint(point).astype(int)
        assert annotated[y-2:y+3, x-2:x+3].any()


def test_mismatched_calibration_draws_only_2d():
    calibration, detection = fixture()
    calibration["width"] = 800
    frame = np.zeros((480, 640, 3), np.uint8)
    assert np.array_equal(annotate(frame, [detection], calibration), annotate(frame, [detection]))


def test_grayscale_preview_converts_without_changing_source():
    _, detection = fixture()
    frame = np.zeros((480, 640), np.uint8)
    result = annotate(frame, [detection])
    assert result.shape == (480, 640, 3)
    assert not frame.any()
