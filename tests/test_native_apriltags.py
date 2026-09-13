"""Real native detector + synthetic calibrated geometry; no camera claims."""

from concurrent.futures import ThreadPoolExecutor
import json
import math

import cv2
import numpy as np
import pytest

native = pytest.importorskip("custom_vision._native", reason="run scripts/build_native.sh for native verification")
from custom_vision.apriltags import AprilTagPipeline, tag_object_points
from custom_vision.native_apriltags import NativeAprilTagPipeline, native_capabilities


@pytest.fixture
def intrinsics():
    return {"width": 640, "height": 480,
            "camera_matrix": [[600, 0, 320], [0, 600, 240], [0, 0, 1]],
            "dist_coeffs": [0, 0, 0, 0, 0]}


def marker_frame(rotation=0, color=False):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = np.rot90(cv2.aruco.generateImageMarker(dictionary, 7, 240), rotation)
    image = np.full((480, 640), 255, dtype=np.uint8)
    image[120:360, 200:440] = marker
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if color else image


@pytest.mark.parametrize("rotation", range(4))
@pytest.mark.parametrize("color", [False, True])
def test_native_matches_real_pupil_corner_convention(intrinsics, rotation, color):
    pipeline = NativeAprilTagPipeline({"threads": 1}, intrinsics)
    image = marker_frame(rotation, color)
    actual = pipeline.process(image)[0]
    reference = AprilTagPipeline({"threads": 1, "quad_decimate": 2}, intrinsics)
    expected = reference.process(marker_frame(rotation, True))[0]
    assert actual["id"] == expected["id"] == 7
    assert actual["hamming"] == 0
    np.testing.assert_allclose(actual["corners"], expected["corners"], atol=0.5)
    assert actual["pose_valid"]
    assert actual["tvec_m"] == pytest.approx([0, 0, 600 * 0.1651 / 240], abs=0.003)
    assert actual["reprojection_error_px"] < 0.5
    assert 0 <= actual["pose_ambiguity"] <= 1
    projected, _ = cv2.projectPoints(tag_object_points(0.1651), np.asarray(actual["rvec_rad"]),
                                     np.asarray(actual["tvec_m"]),
                                     np.asarray(intrinsics["camera_matrix"], dtype=float), np.zeros(5))
    np.testing.assert_allclose(projected.reshape(4, 2), actual["corners"], atol=0.5)
    json.dumps(actual, allow_nan=False)


@pytest.mark.parametrize("rotation", [0, math.pi / 2, math.pi, -math.pi / 2])
def test_exact_frontal_pose_avoids_ippe_pi_singularity(intrinsics, rotation):
    pipeline = NativeAprilTagPipeline({"threads": 1}, intrinsics)
    rvec = np.array([0, 0, rotation], dtype=float)
    tvec = np.array([0, 0, 1.0])
    points, _ = cv2.projectPoints(tag_object_points(0.1651), rvec, tvec,
                                  np.asarray(intrinsics["camera_matrix"], dtype=float), np.zeros(5))
    result = pipeline._estimate_pose(points.reshape(4, 2))
    assert result["pose_valid"]
    np.testing.assert_allclose(result["tvec_m"], tvec, atol=1e-6)
    np.testing.assert_allclose(cv2.Rodrigues(np.array(result["rvec_rad"]))[0], cv2.Rodrigues(rvec)[0], atol=1e-6)
    assert result["reprojection_error_px"] < 1e-5
    assert result["pose_ambiguity"] == pytest.approx(1.0, abs=1e-5)


def test_distorted_geometric_pose_and_alternate(intrinsics):
    intrinsics["dist_coeffs"] = [-0.22, 0.08, 0.002, -0.003, 0.01]
    pipeline = NativeAprilTagPipeline({"threads": 1}, intrinsics)
    rvec, tvec = np.array([0.22, -0.35, 0.17]), np.array([0.12, -0.06, 0.8])
    points, _ = cv2.projectPoints(tag_object_points(0.1651), rvec, tvec,
                                  np.asarray(intrinsics["camera_matrix"], dtype=float),
                                  np.asarray(intrinsics["dist_coeffs"]))
    result = pipeline._estimate_pose(points.reshape(4, 2))
    assert result["pose_valid"]
    np.testing.assert_allclose(result["tvec_m"], tvec, atol=1e-6)
    np.testing.assert_allclose(cv2.Rodrigues(np.array(result["rvec_rad"]))[0], cv2.Rodrigues(rvec)[0], atol=1e-6)
    assert result["pose_ambiguity"] < 0.01
    assert result["alternate_reprojection_error_px"] > 0.1
    assert not np.allclose(result["alternate_rvec_rad"], result["rvec_rad"], atol=0.01)


def test_perspective_raster_pose(intrinsics):
    pipeline = NativeAprilTagPipeline({"threads": 1}, intrinsics)
    rvec, tvec = np.array([0.22, -0.35, 0.17]), np.array([0.05, -0.03, 0.7])
    points, _ = cv2.projectPoints(tag_object_points(0.1651), rvec, tvec,
                                  np.asarray(intrinsics["camera_matrix"], dtype=float), np.zeros(5))
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 7, 240)
    source = np.array([[240, 0], [0, 0], [0, 240], [240, 240]], dtype=np.float32)
    transform = cv2.getPerspectiveTransform(source, points.reshape(4, 2).astype(np.float32))
    image = cv2.warpPerspective(marker, transform, (640, 480), borderValue=255)
    result = pipeline.process(image)[0]
    assert result["id"] == 7 and result["pose_valid"]
    np.testing.assert_allclose(result["tvec_m"], tvec, atol=0.005)
    np.testing.assert_allclose(cv2.Rodrigues(np.array(result["rvec_rad"]))[0], cv2.Rodrigues(rvec)[0], atol=0.035)


def test_2d_resolution_and_calibration_pose_gates(intrinsics):
    image = marker_frame()
    result = NativeAprilTagPipeline({"mode": "2d"}, intrinsics).process(image)[0]
    assert not result["pose_valid"] and result["pose_invalid_reason"] == "mode_2d"
    assert "tvec_m" not in result
    assert NativeAprilTagPipeline({}).process(image)[0]["pose_invalid_reason"] == "no_calibration"
    resized = cv2.resize(image, (800, 600), interpolation=cv2.INTER_NEAREST)
    assert NativeAprilTagPipeline({}, intrinsics).process(resized)[0]["pose_invalid_reason"] == "calibration_resolution_mismatch"


def test_non_square_reprojection_rejected(intrinsics):
    result = NativeAprilTagPipeline({"max_reprojection_error_px": 0.1}, intrinsics)._estimate_pose(
        np.array([[150, 330], [420, 350], [450, 110], [220, 100]], dtype=float))
    assert not result["pose_valid"] and result["pose_invalid_reason"] == "reprojection_error"
    assert "tvec_m" not in result


@pytest.mark.parametrize("value", [0, 127, 255])
def test_blank_frames(value):
    assert NativeAprilTagPipeline({}).process(np.full((480, 640), value, dtype=np.uint8)) == []


def test_quality_filter():
    assert NativeAprilTagPipeline({"min_decision_margin": 1000}).process(marker_frame()) == []


def test_phase_profile_and_explicit_contrast_threshold(intrinsics):
    pipeline = NativeAprilTagPipeline({"min_white_black_diff": 15}, intrinsics)
    assert pipeline.process(marker_frame())[0]["pose_valid"]
    profile = pipeline.last_profile
    assert profile["quad_count"] >= 1
    assert profile["phases_ms"]["threshold"] >= 0
    assert all(math.isfinite(value) and value >= 0 for value in profile["phases_ms"].values())
    with pytest.raises(ValueError):
        NativeAprilTagPipeline({"min_white_black_diff": 256})


def two_tag_frame(ids=(7, 8), dim_second=False):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    image = np.full((480, 640), 255, dtype=np.uint8)
    for index, tag_id in enumerate(ids):
        tag = cv2.aruco.generateImageMarker(dictionary, tag_id, 120)
        if index == 1 and dim_second:
            tag = np.where(tag > 0, 142, 130).astype(np.uint8)
        image[180:300, 80 + index * 320:200 + index * 320] = tag
    return image


def test_multitag_deferral_only_for_distinct_quality_valid_mapped_ids(intrinsics):
    pipeline = NativeAprilTagPipeline({"skip_single_when_multi": True, "known_tag_ids": [7, 8]}, intrinsics)
    detections = pipeline.process(two_tag_frame())
    assert {tag["id"] for tag in detections} == {7, 8}
    assert all(tag["pose_invalid_reason"] == "deferred_multitag" and not tag["pose_valid"] for tag in detections)
    for image in [two_tag_frame(ids=(7, 7)), two_tag_frame(ids=(7, 9)), two_tag_frame(dim_second=True)]:
        detections = pipeline.process(image)
        assert detections
        assert all(tag["pose_valid"] for tag in detections)
    missing_calibration = NativeAprilTagPipeline({"skip_single_when_multi": True, "known_tag_ids": [7, 8]})
    assert all(tag["pose_invalid_reason"] == "no_calibration" for tag in missing_calibration.process(two_tag_frame()))


@pytest.mark.parametrize("config", [{"threads": 0}, {"threads": True}, {"threads": 1.5},
                                    {"tag_size_m": 0}, {"quad_decimate": float("nan")},
                                    {"max_hamming": 3}, {"mode": "unknown"}, {"preprocess": "auto"},
                                    {"tag_family": "fake"}, {"refine_edges": "yes"}])
def test_configuration_validation(config):
    with pytest.raises(ValueError):
        NativeAprilTagPipeline(config)


@pytest.mark.parametrize("image", [np.zeros((5, 5), dtype=np.uint8),
                                    np.zeros((480, 640), dtype=np.float32),
                                    np.zeros((480, 640, 4), dtype=np.uint8),
                                    np.zeros((480, 640), dtype=np.uint8)[:, ::-1],
                                    np.zeros((480, 1280), dtype=np.uint8)[:, ::2]])
def test_bad_image_layout_rejected(image):
    with pytest.raises(ValueError):
        NativeAprilTagPipeline({}).process(image)


def test_padded_rows_borrowed_and_read_only_input_unchanged():
    padded = np.zeros((480, 800), dtype=np.uint8)
    padded[:, :640] = marker_frame()
    roi = padded[:, :640]
    roi.flags.writeable = False
    before = padded.copy()
    assert NativeAprilTagPipeline({}).process(roi)[0]["id"] == 7
    np.testing.assert_array_equal(padded, before)


def test_two_camera_threads_and_same_detector_are_safe(intrinsics):
    pipelines = [NativeAprilTagPipeline({"threads": 2}, intrinsics) for _ in range(2)]
    frames = [marker_frame(rotation) for rotation in range(4)]

    def detect(index):
        result = pipelines[index % 2].process(frames[index % 4])[0]
        assert result["id"] == 7 and result["pose_valid"]
        return result["tvec_m"]

    with ThreadPoolExecutor(max_workers=4) as executor:
        results = list(executor.map(detect, range(32)))
    assert len(results) == 32
    for pipeline in pipelines:
        timings = pipeline.last_timings
        assert all(math.isfinite(value) and value >= 0 for value in timings.values())
        assert timings["total_ms"] == pytest.approx(sum(timings[key] for key in ("preprocess_ms", "detect_ms", "pose_ms")))


def test_real_cuda_preprocess_equivalence_or_explicit_unavailability(intrinsics):
    capabilities = native_capabilities()
    assert capabilities["available"] and capabilities["gil_released"]
    assert capabilities["apriltag_device"] == "cpu" and capabilities["pose_device"] == "cpu"
    if not capabilities["cuda_preprocess"]:
        with pytest.raises((ValueError, RuntimeError), match="CUDA"):
            NativeAprilTagPipeline({"preprocess": "cuda"})
        return
    cpu = NativeAprilTagPipeline({"preprocess": "cpu"}, intrinsics)
    gpu = NativeAprilTagPipeline({"preprocess": "cuda"}, intrinsics)
    for rotation in range(4):
        image = marker_frame(rotation, True)
        expected, actual = cpu.process(image)[0], gpu.process(image)[0]
        assert expected["id"] == actual["id"]
        np.testing.assert_allclose(expected["corners"], actual["corners"], atol=0.02)
        np.testing.assert_allclose(expected["tvec_m"], actual["tvec_m"], atol=0.0001)
    assert gpu.last_timings["preprocess_ms"] > 0


def require_gpu_detector():
    capabilities = native_capabilities()
    if not capabilities.get("cuda_apriltag_compiled") or not capabilities.get("cuda_devices"):
        pytest.skip("optional cuAprilTags build required")


@pytest.mark.parametrize("rotation", range(4))
@pytest.mark.parametrize("color", [False, True])
def test_real_cuda_detector_rotations_quality_and_pose(intrinsics, rotation, color):
    require_gpu_detector()
    gpu = NativeAprilTagPipeline({"detector_device": "cuda"}, intrinsics)
    cpu = NativeAprilTagPipeline({}, intrinsics)
    image = marker_frame(rotation, color)
    expected, actual = cpu.process(image)[0], gpu.process(image)[0]
    assert actual["id"] == expected["id"] == 7 and actual["hamming"] == 0
    np.testing.assert_allclose(actual["corners"], expected["corners"], atol=0.5)
    assert actual["decision_margin"] == pytest.approx(expected["decision_margin"], abs=1)
    assert actual["pose_valid"]
    np.testing.assert_allclose(actual["tvec_m"], expected["tvec_m"], atol=0.002)
    assert gpu.last_profile["phases_ms"]["gpu_detect_decode"] > 0
    assert gpu.last_profile["phases_ms"]["cpu_verify_refine"] > 0


def test_gpu_rectification_maps_back_to_distorted_raw_corners(intrinsics):
    require_gpu_detector()
    intrinsics["dist_coeffs"] = [-0.22, 0.08, 0.002, -0.003, 0.01]
    matrix = np.asarray(intrinsics["camera_matrix"], dtype=float)
    distortion = np.asarray(intrinsics["dist_coeffs"])
    rvec, tvec = np.array([0.22, -0.35, 0.17]), np.array([0.12, -0.06, 0.8])
    ideal, _ = cv2.projectPoints(tag_object_points(0.1651), rvec, tvec, matrix, np.zeros(5))
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 7, 240)
    source = np.array([[240, 0], [0, 0], [0, 240], [240, 240]], dtype=np.float32)
    transform = cv2.getPerspectiveTransform(source, ideal.reshape(4, 2).astype(np.float32))
    ideal_image = cv2.warpPerspective(marker, transform, (640, 480), borderValue=255)
    yy, xx = np.indices((480, 640), dtype=np.float32)
    pixels = np.stack([xx, yy], axis=-1).reshape(-1, 1, 2)
    source_pixels = cv2.undistortPoints(pixels, matrix, distortion, P=matrix).reshape(480, 640, 2)
    image = cv2.remap(ideal_image, source_pixels[:, :, 0], source_pixels[:, :, 1], cv2.INTER_LINEAR, borderValue=255)
    gpu = NativeAprilTagPipeline({"detector_device": "cuda"}, intrinsics)
    result = gpu.process(image)[0]
    assert result["id"] == 7 and result["pose_valid"]
    np.testing.assert_allclose(result["tvec_m"], tvec, atol=0.005)
    expected_raw, _ = cv2.projectPoints(tag_object_points(0.1651), rvec, tvec, matrix, distortion)
    np.testing.assert_allclose(result["corners"], expected_raw.reshape(4, 2), atol=1)


@pytest.mark.parametrize("decimate", [1, 2, 3, 4])
@pytest.mark.parametrize("rotation", range(4))
def test_gpu_decimation_recovers_raw_corners_and_pose(intrinsics, decimate, rotation):
    require_gpu_detector()
    image = marker_frame(rotation)
    expected = NativeAprilTagPipeline({}, intrinsics).process(image)[0]
    gpu = NativeAprilTagPipeline({"detector_device": "cuda", "quad_decimate": decimate}, intrinsics)
    result = gpu.process(image)[0]
    np.testing.assert_allclose(result["corners"], expected["corners"], atol=0.5)
    np.testing.assert_allclose(result["tvec_m"], expected["tvec_m"], atol=0.002)
    assert result["pose_valid"] and result["reprojection_error_px"] < 0.5


def test_gpu_blank_frames_filters_resolution_changes_and_2d(intrinsics):
    require_gpu_detector()
    gpu = NativeAprilTagPipeline({"detector_device": "cuda"}, intrinsics)
    for value in [0, 127, 255]:
        assert gpu.process(np.full((480, 640), value, dtype=np.uint8)) == []
    assert gpu.process(marker_frame())[0]["pose_valid"]
    resized = cv2.resize(marker_frame(), (800, 600), interpolation=cv2.INTER_NEAREST)
    assert gpu.process(resized)[0]["pose_invalid_reason"] == "calibration_resolution_mismatch"
    assert gpu.process(marker_frame())[0]["pose_valid"]
    assert NativeAprilTagPipeline({"detector_device": "cuda"}).process(marker_frame())[0]["pose_invalid_reason"] == "no_calibration"
    assert NativeAprilTagPipeline({"detector_device": "cuda", "mode": "2d"}, intrinsics).process(marker_frame())[0]["pose_invalid_reason"] == "mode_2d"
    assert NativeAprilTagPipeline({"detector_device": "cuda", "min_decision_margin": 1000}).process(marker_frame()) == []
    with pytest.raises(ValueError, match="tag36h11"):
        NativeAprilTagPipeline({"detector_device": "cuda", "tag_family": "tag16h5"})
    with pytest.raises(ValueError, match="refine_edges=true"):
        NativeAprilTagPipeline({"detector_device": "cuda", "refine_edges": False})


def test_gpu_dual_camera_and_capacity_reporting(intrinsics):
    require_gpu_detector()
    pipelines = [NativeAprilTagPipeline({"detector_device": "cuda"}, intrinsics) for _ in range(2)]
    def work(index):
        tag = pipelines[index % 2].process(marker_frame(index % 4))[0]
        return tag["id"], tag["pose_valid"]
    with ThreadPoolExecutor(max_workers=4) as executor:
        assert list(executor.map(work, range(16))) == [(7, True)] * 16
    capped = NativeAprilTagPipeline({"detector_device": "cuda", "cuda_max_tags": 1})
    assert len(capped.process(two_tag_frame())) == 1
    assert capped.last_profile["capacity_reached"]


@pytest.mark.parametrize("device", ["cpu", "cuda"])
def test_close_releases_native_detector_and_is_idempotent(device):
    if device == "cuda":
        require_gpu_detector()
    pipeline = NativeAprilTagPipeline({"detector_device": device})
    assert pipeline.process(marker_frame())
    pipeline.close()
    pipeline.close()
    assert pipeline.detector is None
    with pytest.raises(RuntimeError, match="closed"):
        pipeline.process(marker_frame())
    with pytest.raises(RuntimeError, match="closed"):
        _ = pipeline.last_timings
