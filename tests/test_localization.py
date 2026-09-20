"""Synthetic *absolute* field scenes: coordinates are specified independently.

These establish geometry and solver behavior; they do not establish camera
accuracy, physical calibration, capture timing, or on-robot reliability.
"""

import copy
import json
import math

import cv2
import numpy as np
import pytest

from custom_vision.localization import Localization, estimate_tag_pose, validate_field_layout, validate_robot_to_camera


SIZE = .1651


@pytest.fixture
def calibration():
    return {"width": 1280, "height": 800,
            "camera_matrix": [[870., 0., 640.], [0., 870., 400.], [0., 0., 1.]],
            "dist_coeffs": [-.13, .04, .001, -.002, 0.]}


def transform(xyz, rpy=(0., 0., 0.)):
    """Test scene construction using independent axis rotations."""
    roll, pitch, yaw = np.deg2rad(rpy)
    rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = rz @ ry @ rx, xyz
    return result


def layout_for(tags):
    entries = []
    for tag_id, field_to_tag in tags.items():
        rvec = cv2.Rodrigues(field_to_tag[:3, :3])[0].reshape(3)
        angle = np.linalg.norm(rvec)
        q = np.r_[math.cos(angle / 2), rvec * math.sin(angle / 2) / angle] if angle else [1, 0, 0, 0]
        entries.append({"ID": tag_id, "pose": {
            "translation": dict(zip(("x", "y", "z"), field_to_tag[:3, 3].tolist())),
            "rotation": {"quaternion": dict(zip(("W", "X", "Y", "Z"), q))}}})
    return {"field": {"length": 16.5, "width": 8.2}, "tags": entries}


def project_scene(tags, field_to_camera, calibration):
    detections = []
    h = SIZE / 2
    # Physical printed corners TR, TL, BL, BR: tag +X out, +Y observer's
    # right, +Z up. This fixture deliberately does not use production bases.
    tag_corners = np.array([[0, h, h, 1], [0, -h, h, 1], [0, -h, -h, 1], [0, h, -h, 1]])
    for tag_id, field_to_tag in tags.items():
        camera_points = (np.linalg.inv(field_to_camera) @ field_to_tag @ tag_corners.T).T[:, :3]
        # Camera NWU -> optical: right=-left, down=-up, depth=forward.
        optical_points = np.column_stack((-camera_points[:, 1], -camera_points[:, 2], camera_points[:, 0]))
        assert np.min(optical_points[:, 2]) > 0
        corners = cv2.projectPoints(optical_points, np.zeros(3), np.zeros(3),
                                   np.array(calibration["camera_matrix"]), np.array(calibration["dist_coeffs"]))[0].reshape(4, 2)
        center = corners.mean(axis=0)
        detections.append({"id": tag_id, "corners": corners.tolist(), "center": center.tolist(),
                           "decision_margin": 100., "hamming": 0, "pose_valid": False})
    return detections


def pose_matrix(pose):
    w, x, y, z = pose["rotation_quaternion_wxyz"]
    # Quaternion point-vector formula, independent of production conversion.
    vec = np.array([x, y, z])
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    result = np.eye(4)
    result[:3, :3] = (w*w - vec @ vec) * np.eye(3) + 2 * np.outer(vec, vec) + 2 * w * skew
    result[:3, 3] = pose["translation_m"]
    return result


def test_deferred_single_pose_fallback_reports_actual_device_and_does_not_retry(calibration):
    localizer = Localization({}, calibration)
    calls = []

    def rejected_gpu_pose(corners):
        calls.append(corners)
        return {'pose_valid': False, 'pose_attempted': True, 'pose_device': 'cuda',
                'pose_invalid_reason': 'reprojection_error', 'pose_source': 'single_tag_cuda_ippe'}

    localizer.single_pose_solver = rejected_gpu_pose
    detection = {'id': 7, 'center': [640., 400.],
                 'corners': [[620., 420.], [660., 420.], [660., 380.], [620., 380.]],
                 'pose_valid': False, 'pose_attempted': False, 'pose_device': 'none',
                 'pose_invalid_reason': 'deferred_multitag'}
    result = localizer.enrich([detection], (800, 1280, 3))
    fallback = result['localization']['single_tag_fallback']
    assert fallback['calls'] == 1 and fallback['devices'] == ['cuda'] and fallback['pose_ms'] >= 0
    assert result['detections'][0]['pose_device'] == 'cuda'
    localizer._ensure_single(result['detections'][0])
    assert len(calls) == 1
    assert 'single_tag_fallback' not in localizer.enrich([], (800, 1280, 3))['localization']


@pytest.fixture
def scene():
    camera = transform([2., 3., .7], [1., -4., 17.])
    tags = {1: transform([5., 3.3, 1.3], [0., 0., 180.]),
            2: transform([5.4, 4.4, .8], [0., 0., 200.]),
            3: transform([4.8, 2.6, .9], [0., 0., 160.])}
    return camera, tags


def test_joint_nonplanar_pnp_recovers_absolute_field_camera(calibration, scene):
    camera, tags = scene
    localizer = Localization({}, calibration, layout_for(tags))
    result = localizer.enrich(project_scene(tags, camera, calibration), (800, 1280, 3))
    pose = result["localization"]
    assert pose["valid"] and pose["method"] == "multitag_pnp"
    assert pose["used_tag_ids"] == [1, 2, 3]
    assert pose["inlier_tag_count"] == 3
    assert pose["reprojection_error_px"] < 1e-6
    np.testing.assert_allclose(pose_matrix(pose["field_to_camera"]), camera, atol=1e-6)
    assert pose["field_to_robot"] is None
    assert pose["robot_pose_invalid_reason"] == "no_robot_to_camera"
    for detection in result["detections"]:
        assert detection["pose_source"] == "field_layout_multitag"
        np.testing.assert_allclose(pose_matrix(detection["camera_to_target"]),
                                   np.linalg.inv(camera) @ tags[detection["id"]], atol=1e-6)
        assert detection["robot_to_target"] is None
    json.dumps(result, allow_nan=False)


def test_joint_pose_does_not_reuse_prior_single_tag_alternate(calibration, scene):
    camera, tags = scene
    detections = project_scene(tags, camera, calibration)
    alternate_keys = {'alternate_rvec_rad', 'alternate_tvec_m', 'alternate_reprojection_error_px'}
    for detection in detections:
        detection.update(estimate_tag_pose(detection['corners'], SIZE,
                         np.asarray(calibration['camera_matrix']), np.asarray(calibration['dist_coeffs'])))
        assert detection['pose_valid'] and alternate_keys.issubset(detection)
    before = copy.deepcopy(detections)
    result = Localization({}, calibration, layout_for(tags)).enrich(detections, (800, 1280))
    assert result['localization']['valid']
    for detection in result['detections']:
        assert detection['pose_source'] == 'field_layout_multitag'
        assert detection['pose_ambiguity'] == result['localization']['ambiguity']
        assert not alternate_keys.intersection(detection)
    # The original single-tag packet is still suitable for independent POI
    # processing, and explicitly requested single poses keep their alternates.
    assert detections == before
    singles = Localization({'always_single_tag': True}, calibration, layout_for(tags)).enrich(
        detections, (800, 1280))
    assert all(alternate_keys.issubset(detection) for detection in singles['detections'])


def test_robot_mount_rotates_translation_and_uses_inverse(calibration, scene):
    camera, tags = scene
    mount = {"translation_m": [.35, -.17, .42], "rotation_rpy_deg": [5., -15., 25.]}
    robot_to_camera = transform(mount["translation_m"], mount["rotation_rpy_deg"])
    expected_robot = camera @ np.linalg.inv(robot_to_camera)
    result = Localization({}, calibration, layout_for(tags), mount).enrich(
        project_scene(tags, camera, calibration), (800, 1280))
    np.testing.assert_allclose(pose_matrix(result["localization"]["field_to_robot"]), expected_robot, atol=1e-6)
    for detection in result["detections"]:
        np.testing.assert_allclose(pose_matrix(detection["robot_to_target"]),
                                   np.linalg.inv(expected_robot) @ tags[detection["id"]], atol=1e-6)


def test_single_oblique_tag_field_pose(calibration, scene):
    camera, tags = scene
    tags = {1: tags[1]}
    result = Localization({}, calibration, layout_for(tags)).enrich(
        project_scene(tags, camera, calibration), (800, 1280))
    pose = result["localization"]
    assert pose["valid"] and pose["method"] == "single_tag_pnp"
    assert pose["ambiguity"] < .2
    np.testing.assert_allclose(pose_matrix(pose["field_to_camera"]), camera, atol=1e-5)
    assert "alternate_rvec_rad" in result["detections"][0]


def test_exact_headon_ambiguity_is_not_false_certainty(calibration):
    calibration["dist_coeffs"] = [0.] * 5
    camera, tags = transform([1., 2., 1.]), {7: transform([3., 2., 1.], [0, 0, 180])}
    result = Localization({}, calibration, layout_for(tags)).enrich(
        project_scene(tags, camera, calibration), (800, 1280))
    detection = result["detections"][0]
    assert detection["pose_valid"]
    assert detection["pose_ambiguity"] > .9
    assert detection["camera_to_target"]["translation_m"] == pytest.approx([2., 0., 0.], abs=1e-6)
    np.testing.assert_allclose(pose_matrix(detection["camera_to_target"])[:3, :3],
                               np.diag([-1., -1., 1.]), atol=1e-6)
    assert not result["localization"]["valid"]
    assert result["localization"]["field_to_camera"] is None
    assert result["localization"]["invalid_reason"] == "ambiguous_pose"


@pytest.mark.parametrize("rotation", [0, 1, 2, 3])
def test_rendered_decoded_corner_order_has_correct_absolute_wpilib_pose(rotation):
    from custom_vision.apriltags import AprilTagPipeline
    calibration = {"width": 640, "height": 480,
                   "camera_matrix": [[600., 0., 320.], [0., 600., 240.], [0., 0., 1.]],
                   "dist_coeffs": [0.] * 5}
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 7, 240)
    image = np.full((480, 640), 255, np.uint8)
    image[120:360, 200:440] = np.rot90(marker, rotation)
    # Detector without calibration lets the new solver itself establish pose.
    detections = AprilTagPipeline({"threads": 1}).process(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))
    result = Localization({}, calibration).enrich(detections, image.shape)
    pose = pose_matrix(result["detections"][0]["camera_to_target"])
    assert pose[:3, 3] == pytest.approx([600 * SIZE / 240, 0., 0.], abs=.003)
    # A CCW image turn is -roll about camera forward. An upright tag faces
    # camera with yaw pi. This checks physical printed orientation explicitly.
    expected_rotation = transform([0, 0, 0], [-rotation * 90, 0, 0])[:3, :3] @ np.diag([-1., -1., 1.])
    np.testing.assert_allclose(pose[:3, :3], expected_rotation, atol=.04)


def test_two_coplanar_oblique_tags(calibration):
    camera = transform([2., 3., .7], [0., 0., 15.])
    tags = {1: transform([5., 3.5, 1.3], [0., 0., 180.]),
            2: transform([5., 4.5, 1.3], [0., 0., 180.])}
    result = Localization({}, calibration, layout_for(tags)).enrich(project_scene(tags, camera, calibration), (800, 1280))
    assert result["localization"]["valid"]
    np.testing.assert_allclose(pose_matrix(result["localization"]["field_to_camera"]), camera, atol=1e-5)


def test_coplanar_frontoparallel_noisy_multitag_rejects_ambiguity(calibration):
    calibration["dist_coeffs"] = [0.] * 5
    camera = transform([1., 3., 1.])
    tags = {1: transform([8., 2.75, 1.], [0., 0., 180.]),
            2: transform([8., 3.25, 1.], [0., 0., 180.])}
    detections = project_scene(tags, camera, calibration)
    rng = np.random.default_rng(63)
    for detection in detections:
        detection["corners"] = (np.asarray(detection["corners"]) + rng.normal(0., .2, (4, 2))).tolist()
    result = Localization({}, calibration, layout_for(tags)).enrich(detections, (800, 1280))
    assert not result["localization"]["valid"]
    assert result["localization"]["invalid_reason"] == "ambiguous_pose"
    assert result["localization"]["field_to_camera"] is None


def test_whole_tag_outlier_rejected(calibration, scene):
    camera, tags = scene
    detections = project_scene(tags, camera, calibration)
    detections[2]["corners"] = (np.asarray(detections[2]["corners"]) + [70., -30.]).tolist()
    result = Localization({}, calibration, layout_for(tags)).enrich(detections, (800, 1280))
    pose = result["localization"]
    assert pose["valid"] and pose["used_tag_ids"] == [1, 2]
    assert pose["rejected_tag_ids"] == [3]
    np.testing.assert_allclose(pose_matrix(pose["field_to_camera"]), camera, atol=1e-5)


def test_two_inconsistent_tags_do_not_choose_arbitrary_single(calibration, scene):
    camera, tags = scene
    tags = {k: tags[k] for k in (1, 2)}
    detections = project_scene(tags, camera, calibration)
    detections[1]["corners"] = (np.asarray(detections[1]["corners"]) + [150., -75.]).tolist()
    result = Localization({}, calibration, layout_for(tags)).enrich(detections, (800, 1280))
    assert not result["localization"]["valid"]
    assert result["localization"]["invalid_reason"] == "inconsistent_tag_observations"


def test_multitag_direct_fit_does_not_need_single_pnp(calibration, scene, monkeypatch):
    camera, tags = scene
    def fail(*args, **kwargs):
        raise AssertionError("Single-tag PnP should not run in the accepted multi-tag fast path")
    monkeypatch.setattr("custom_vision.localization.estimate_tag_pose", fail)
    result = Localization({}, calibration, layout_for(tags)).enrich(project_scene(tags, camera, calibration), (800, 1280))
    assert result["localization"]["valid"]


def test_disabled_multitag_uses_lowest_ambiguity_single(calibration, scene, monkeypatch):
    camera, tags = scene
    localizer = Localization({"multitag": False}, calibration, layout_for(tags))
    def fail(*args, **kwargs):
        raise AssertionError("Disabled multi-tag must not execute joint PnP")
    monkeypatch.setattr(localizer, "_multitag", fail)
    result = localizer.enrich(project_scene(tags, camera, calibration), (800, 1280))
    assert result["localization"]["valid"]
    assert result["localization"]["method"] == "single_tag_pnp"
    assert len(result["localization"]["used_tag_ids"]) == 1
    assert result["localization"]["ambiguity"] == min(d["pose_ambiguity"] for d in result["detections"])
    np.testing.assert_allclose(pose_matrix(result["localization"]["field_to_camera"]), camera, atol=1e-5)


def test_duplicate_observations_are_not_a_multitag_result(calibration, scene):
    camera, tags = scene
    detection = project_scene({1: tags[1]}, camera, calibration)[0]
    result = Localization({}, calibration, layout_for(tags)).enrich([detection, detection], (800, 1280))
    assert not result["localization"]["valid"]
    assert result["localization"]["duplicate_tag_ids"] == [1]
    assert result["localization"]["invalid_reason"] == "duplicate_tag_id"


def test_unknown_id_still_has_relative_pose(calibration, scene):
    camera, tags = scene
    result = Localization({}, calibration, layout_for({2: tags[2]})).enrich(
        project_scene({1: tags[1]}, camera, calibration), (800, 1280))
    assert result["detections"][0]["camera_to_target"] is not None
    assert result["localization"]["invalid_reason"] == "no_known_tags"


def test_2d_mode_strips_any_prior_metric_pose(calibration, scene):
    camera, tags = scene
    source = Localization({}, calibration, layout_for(tags)).enrich(project_scene(tags, camera, calibration), (800, 1280))
    result = Localization({"mode": "2d"}, calibration, layout_for(tags)).enrich(source["detections"], (800, 1280))
    assert result["localization"]["invalid_reason"] == "mode_2d"
    for detection in result["detections"]:
        assert not detection["pose_valid"]
        assert detection["angle_source"] == "calibrated"
        assert detection["camera_to_target"] is None and detection["robot_to_target"] is None
        assert "tvec_m" not in detection and "rvec_rad" not in detection
        assert detection["area_pct"] > 0


def test_angles_undistort_and_use_wpilib_left_up_signs(calibration):
    optical_point = np.array([[-.3, -.2, 1.]])
    center = cv2.projectPoints(optical_point, np.zeros(3), np.zeros(3),
                               np.array(calibration["camera_matrix"]), np.array(calibration["dist_coeffs"]))[0].reshape(2)
    detection = {"id": 1, "center": center.tolist(),
                 "corners": (center + np.array([[-10, -10], [10, -10], [10, 10], [-10, 10]])).tolist()}
    result = Localization({"mode": "2d"}, calibration).enrich([detection], (800, 1280))["detections"][0]
    assert result["yaw_deg"] == pytest.approx(math.degrees(math.atan(.3)), abs=1e-5)
    assert result["pitch_deg"] == pytest.approx(math.degrees(math.atan2(.2, math.hypot(1., .3))), abs=1e-5)
    assert result["area_pct"] == pytest.approx(400 * 100 / (800 * 1280), rel=1e-5)


@pytest.mark.parametrize("provided,size,reason", [(False, (800, 1280), "no_calibration"),
                                                   (True, (600, 960), "calibration_resolution_mismatch")])
def test_uncalibrated_geometry_is_labeled_approximate(calibration, scene, provided, size, reason):
    camera, tags = scene
    result = Localization({}, calibration if provided else None, layout_for(tags)).enrich(
        project_scene(tags, camera, calibration), size)
    assert result["localization"]["invalid_reason"] == reason
    for detection in result["detections"]:
        assert detection["angle_source"] == "nominal_fov_approximate"
        assert not detection["pose_valid"] and detection["camera_to_target"] is None


@pytest.mark.parametrize("mutate", [
    lambda layout: layout["tags"].append(copy.deepcopy(layout["tags"][0])),
    lambda layout: layout["field"].update(length=0),
    lambda layout: layout["field"].update(width=float("nan")),
    lambda layout: layout["tags"][0].update(ID=True),
    lambda layout: layout["tags"][0]["pose"]["translation"].update(x=float("inf")),
    lambda layout: layout["tags"][0]["pose"]["rotation"]["quaternion"].update(W=2.),
    lambda layout: layout["tags"][0]["pose"].pop("rotation"),
    lambda layout: layout.update(tags=[]),
])
def test_invalid_maps_fail_before_processing(scene, mutate):
    layout = layout_for(scene[1])
    mutate(layout)
    with pytest.raises(ValueError):
        validate_field_layout(layout)


@pytest.mark.parametrize("mount", [{}, {"translation_m": [0, 0, 0]},
                                  {"translation_m": [0, 0, 0], "rotation_rpy_deg": [0, float("inf"), 0]}])
def test_mount_must_be_explicit_complete_finite(mount):
    with pytest.raises(ValueError):
        validate_robot_to_camera(mount)
    assert validate_robot_to_camera(None) is None


@pytest.mark.parametrize("config", [{"mode": "4d"}, {"max_ambiguity": 1.1}, {"tag_size_m": 0},
                                    {"hfov_deg": 180}, {"always_single_tag": "false"}])
def test_invalid_localization_config(config):
    with pytest.raises(ValueError):
        Localization(config)


def test_empty_frame_clears_localization(calibration, scene):
    localizer = Localization({}, calibration, layout_for(scene[1]))
    valid = localizer.enrich(project_scene(scene[1], scene[0], calibration), (800, 1280))
    assert valid["localization"]["valid"]
    result = localizer.enrich([], (800, 1280))
    assert not result["localization"]["valid"] and result["localization"]["field_to_camera"] is None


def test_pose_serialization_preserves_rotations_at_zero_pi_and_gimbal_lock():
    from custom_vision.localization import pose_dict
    rng = np.random.default_rng(1086)
    rotations = [[0., 0., 0.], [180., 0., 0.], [0., 180., 0.], [0., 0., 180.],
                 [30., 90., 50.], [30., -90., 50.], [179.999, 0., 0.]]
    rotations.extend(rng.uniform(-180., 180., (100, 3)).tolist())
    for angles in rotations:
        expected = transform([.3, -.7, 2.], angles)
        published = pose_dict(expected)
        w, x, y, z = published['rotation_quaternion_wxyz']
        assert w >= 0
        assert math.hypot(w, x, y, z) == pytest.approx(1., abs=1e-12)
        # Independently reconstruct the quaternion's rotation matrix.
        actual = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                           [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                           [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        np.testing.assert_allclose(actual, expected[:3, :3], atol=1e-9)
        np.testing.assert_allclose(transform(published['translation_m'], published['rotation_rpy_deg']),
                                   expected, atol=1e-9)
