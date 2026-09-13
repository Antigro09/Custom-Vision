"""Image-level checks use real tag36h11 pixels and the native detector."""

import json
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from custom_vision.apriltags import AprilTagPipeline, tag_object_points


@pytest.fixture
def calibration():
    return {"width": 640, "height": 480,
            "camera_matrix": [[600, 0, 320], [0, 600, 240], [0, 0, 1]],
            "dist_coeffs": [0, 0, 0, 0, 0]}


def tag_image(rotation=0):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    tag = cv2.aruco.generateImageMarker(dictionary, 7, 240)
    tag = np.rot90(tag, rotation)
    image = np.full((480, 640), 255, dtype=np.uint8)
    image[120:360, 200:440] = tag
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def test_real_tag_without_calibration_retains_pixels():
    pipeline = AprilTagPipeline({"threads": 1})
    detections = pipeline.process(tag_image())
    assert len(detections) == 1
    tag = detections[0]
    assert tag["id"] == 7
    assert tag["hamming"] == 0
    assert tag["center"] == pytest.approx([320, 240], abs=1)
    assert not tag["pose_valid"]
    assert tag["pose_invalid_reason"] == "no_calibration"
    assert "tvec_m" not in tag
    # OpenCV's rendered dictionary orientation decodes with this native pupil
    # corner order. It is not OpenCV ArUco's top-left-first marker order.
    np.testing.assert_allclose(tag["corners"], [[440, 120], [200, 120], [200, 360], [440, 360]], atol=1)
    json.dumps(detections, allow_nan=False)


@pytest.mark.parametrize("rotation", [0, 1, 2, 3])
def test_real_tag_pose_respects_decoded_orientation(calibration, rotation):
    pipeline = AprilTagPipeline({"threads": 1}, calibration)
    tag = pipeline.process(tag_image(rotation))[0]
    assert tag["id"] == 7
    assert tag["pose_valid"]
    assert tag["tvec_m"] == pytest.approx([0, 0, 600 * 0.1651 / 240], abs=0.003)
    assert tag["reprojection_error_px"] < 0.5
    projected, _ = cv2.projectPoints(tag_object_points(0.1651), np.array(tag["rvec_rad"]),
                                      np.array(tag["tvec_m"]),
                                      np.array(calibration["camera_matrix"], dtype=float), np.zeros(5))
    np.testing.assert_allclose(projected.reshape(4, 2), tag["corners"], atol=0.5)
    # A 90-degree rotation must rotate decoded corners rather than re-sort them.
    unrotated = pipeline.process(tag_image())[0]
    expected = np.array(unrotated["corners"])
    for _ in range(rotation):
        expected = np.column_stack((expected[:, 1] - 240 + 320, -(expected[:, 0] - 320) + 240))
    np.testing.assert_allclose(tag["corners"], expected, atol=1)


def test_mismatched_capture_resolution_withholds_metric_pose(calibration):
    pipeline = AprilTagPipeline({"threads": 1}, calibration)
    resized = cv2.resize(tag_image(), (800, 600), interpolation=cv2.INTER_NEAREST)
    tag = pipeline.process(resized)[0]
    assert tag["id"] == 7
    assert not tag["pose_valid"]
    assert tag["pose_invalid_reason"] == "calibration_resolution_mismatch"
    assert "distance_m" not in tag


def test_blank_frames_return_empty_detections(calibration):
    pipeline = AprilTagPipeline({"threads": 1}, calibration)
    for value in (0, 127, 255):
        assert pipeline.process(np.full((480, 640, 3), value, dtype=np.uint8)) == []


def test_distorted_pose_recovers_translation_and_orientation(calibration):
    calibration["dist_coeffs"] = [-0.22, 0.08, 0.002, -0.003, 0.01]
    pipeline = AprilTagPipeline({"threads": 1}, calibration)
    rvec = np.array([0.22, -0.35, 0.17], dtype=np.float64)
    tvec = np.array([0.12, -0.06, 0.8], dtype=np.float64)
    projected, _ = cv2.projectPoints(tag_object_points(0.1651), rvec, tvec,
                                      np.array(calibration["camera_matrix"], dtype=float),
                                      np.array(calibration["dist_coeffs"]))
    pose = pipeline._estimate_pose(projected.reshape(4, 2))
    assert pose["pose_valid"]
    assert pose["reprojection_error_px"] < 1e-5
    np.testing.assert_allclose(pose["tvec_m"], tvec, atol=1e-6)
    estimated_rotation, _ = cv2.Rodrigues(np.array(pose["rvec_rad"]))
    expected_rotation, _ = cv2.Rodrigues(rvec)
    np.testing.assert_allclose(estimated_rotation, expected_rotation, atol=1e-6)


def test_perspective_rendered_tag_recovers_known_pose(calibration):
    pipeline = AprilTagPipeline({"threads": 1}, calibration)
    rvec = np.array([0.22, -0.35, 0.17])
    tvec = np.array([0.05, -0.03, 0.7])
    projected, _ = cv2.projectPoints(tag_object_points(0.1651), rvec, tvec,
                                      np.array(calibration["camera_matrix"], dtype=float), np.zeros(5))
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 7, 240)
    # Match decoded corners, including the dictionary's printed orientation.
    source = np.array([[240, 0], [0, 0], [0, 240], [240, 240]], dtype=np.float32)
    transform = cv2.getPerspectiveTransform(source, projected.reshape(4, 2).astype(np.float32))
    gray = cv2.warpPerspective(marker, transform, (640, 480), borderValue=255)
    tag = pipeline.process(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR))[0]
    assert tag["id"] == 7
    assert tag["pose_valid"]
    np.testing.assert_allclose(tag["tvec_m"], tvec, atol=0.005)
    estimated_rotation, _ = cv2.Rodrigues(np.array(tag["rvec_rad"]))
    expected_rotation, _ = cv2.Rodrigues(rvec)
    np.testing.assert_allclose(estimated_rotation, expected_rotation, atol=0.035)


def test_non_square_geometry_rejects_bad_pose(calibration):
    pipeline = AprilTagPipeline({"threads": 1, "max_reprojection_error_px": 0.1}, calibration)
    corners = np.array([[150, 330], [420, 350], [450, 110], [220, 100]], dtype=np.float64)
    pose = pipeline._estimate_pose(corners)
    assert not pose["pose_valid"]
    assert pose["pose_invalid_reason"] == "reprojection_error"
    assert "tvec_m" not in pose


def test_quality_filters_reject_low_margin_and_corrected_bits(calibration):
    pipeline = AprilTagPipeline({"threads": 1, "min_decision_margin": 40, "max_hamming": 0}, calibration)
    real = pipeline.detector.detect(cv2.cvtColor(tag_image(), cv2.COLOR_BGR2GRAY))[0]
    low_margin = SimpleNamespace(**vars(real))
    low_margin.decision_margin = 39
    bad_hamming = SimpleNamespace(**vars(real))
    bad_hamming.hamming = 1
    pipeline.detector = SimpleNamespace(detect=lambda *args, **kwargs: [low_margin, bad_hamming])
    assert pipeline.process(tag_image()) == []


@pytest.mark.parametrize("config", [{"tag_size_m": 0}, {"threads": 0}, {"threads": 1.5},
                                    {"quad_decimate": float("nan")}, {"max_hamming": 3},
                                    {"tag_family": "missing"}])
def test_invalid_settings_fail_before_native_detection(config):
    with pytest.raises(ValueError):
        AprilTagPipeline(config)


def test_invalid_frame_is_actionable():
    pipeline = AprilTagPipeline({"threads": 1})
    with pytest.raises(ValueError, match="uint8 BGR"):
        pipeline.process(np.zeros((480, 640, 4), dtype=np.uint8))
