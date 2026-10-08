"""Small calibrated-board geometry/identity checks; no captures or native fits."""
import copy
import json

import cv2
import numpy as np
import pytest

from custom_vision.calibration_diagnostics import build_board_diagnostics, POSE_CONVENTION
from custom_vision.calibration_session import Board


@pytest.fixture
def evidence():
    board = Board(7, 5, .03)
    calibration = {"width": 640, "height": 480,
                   "camera_matrix": [[600., 0., 320.], [0., 605., 240.], [0., 0., 1.]],
                   "dist_coeffs": [-.06, .015, .0005, -.0008, 0., .002, -.001, .0001],
                   "lensmodel": "LENSMODEL_OPENCV8", "distortion_model": "opencv_pinhole",
                   "board": {"cols": 7, "rows": 5, "square_size_m": .03}}
    session = {"board": calibration["board"].copy(), "width": 640, "height": 480, "views": []}
    poses = {"convention": POSE_CONVENTION, "robot_to_camera": None, "poses": []}
    # Two reproducible, tilted poses; records deliberately use noncontiguous IDs.
    rng = np.random.default_rng(1086)
    for index, frame_id in enumerate((8, 42)):
        rvec = rng.uniform(-.2, .2, 3)
        tvec = np.array([-.12 + .02 * index, -.06, .75 + .05 * index])
        observed = cv2.projectPoints(board.points(), rvec, tvec,
                                     np.asarray(calibration["camera_matrix"]),
                                     np.asarray(calibration["dist_coeffs"]))[0].reshape(-1, 2)
        transform = np.eye(4)
        transform[:3, :3] = cv2.Rodrigues(rvec)[0]
        transform[:3, 3] = tvec
        image = f"frames/frame{index:06d}.png"
        session["views"].append({"image": image, "source_frame_id": frame_id, "corners": observed.tolist()})
        poses["poses"].append({"image": image, "camera_cv_T_board": transform.tolist()})
    return session, calibration, poses


def test_retained_metric_geometry_and_exact_distorted_projection(evidence):
    session, calibration, poses = evidence
    before = copy.deepcopy(evidence)
    poses["poses"].reverse()  # Joining must follow image identity rather than list position.
    result = build_board_diagnostics(session, calibration, poses)
    assert result["schema_version"] == 1 and result["status"] == "available"
    assert result["frame"] == "camera_optical" and result["units"] == "m"
    assert result["image_size_px"] == [640, 480]
    assert [frame["frame_id"] for frame in result["frames"]] == [8, 42]
    board_points = Board(**session["board"]).points()
    for frame in result["frames"]:
        pose = next(p for p in poses["poses"] if p["image"] == frame["image"])
        transform = np.asarray(pose["camera_cv_T_board"])
        expected = board_points @ transform[:3, :3].T + transform[:3, 3]
        np.testing.assert_allclose(frame["corners_camera_m"], expected, atol=1e-12)
        np.testing.assert_allclose(frame["projected_corners_px"], frame["corners_px"], atol=1e-10)
        assert frame["reprojection_rms_px"] < 1e-10
        assert frame["width"] == 640 and frame["height"] == 480
    assert any("180-degree" in warning for warning in result["warnings"])
    assert any("not arbitrary scene" in warning for warning in result["warnings"])
    json.dumps(result, allow_nan=False)
    assert session == before[0] and calibration == before[1]
    assert poses["poses"] == list(reversed(before[2]["poses"]))


def test_reprojection_rms_is_euclidean_pixel_residual(evidence):
    session, calibration, poses = evidence
    for view in session["views"]:
        view["corners"] = (np.asarray(view["corners"]) + [3., 4.]).tolist()
    result = build_board_diagnostics(session, calibration, poses)
    assert all(frame["reprojection_rms_px"] == pytest.approx(5.) for frame in result["frames"])


def test_retained_final_native_observations_preserve_solve_order(evidence):
    session, calibration, poses = evidence
    for pose, view in zip(poses["poses"], session["views"]):
        pose["observed_corners_px"] = copy.deepcopy(view["corners"])
        pose["observed_corner_source"] = "solver_final_detection"
        # A second detector may reverse the symmetric board in the saved preview.
        view["corners"].reverse()
    result = build_board_diagnostics(session, calibration, poses)
    assert all(frame["reprojection_rms_px"] < 1e-10 for frame in result["frames"])
    for frame, pose in zip(result["frames"], poses["poses"]):
        assert frame["corners_px"] == pose["observed_corners_px"]
    assert any("retained native observations" in w for w in result["warnings"])
    assert not any("lack retained final detector" in w for w in result["warnings"])


def test_legacy_native_artifacts_disclose_preview_order_without_auto_flip(evidence):
    session, calibration, poses = evidence
    for view in session["views"]:
        view["corners"].reverse()
    result = build_board_diagnostics(session, calibration, poses)
    assert any("2 solved views lack retained final detector" in w for w in result["warnings"])
    assert all(frame["reprojection_rms_px"] > 50 for frame in result["frames"])
    assert all(frame["corners_px"] == view["corners"] for frame, view in zip(result["frames"], session["views"]))


@pytest.mark.parametrize("corners", [None, [], [[1., 2.]], [[0., 0.]] * 36, [[True, 1.]] * 35,
                                     [[640., 0.]] * 35, [[float("nan"), 0.]] * 35])
def test_malformed_native_observations_never_fall_back_to_snapshot_order(evidence, corners):
    session, calibration, poses = evidence
    poses["poses"][0]["observed_corners_px"] = corners
    poses["poses"][0]["observed_corner_source"] = "solver_final_detection"
    with pytest.raises(ValueError, match="Observed corners"):
        build_board_diagnostics(session, calibration, poses)


@pytest.mark.parametrize("source", [None, "preview_detection", "unknown"])
def test_native_observation_source_must_identify_actual_final_detector(evidence, source):
    session, calibration, poses = evidence
    poses["poses"][0]["observed_corners_px"] = session["views"][0]["corners"]
    if source is not None:
        poses["poses"][0]["observed_corner_source"] = source
    with pytest.raises(ValueError, match="solver_final_detection provenance"):
        build_board_diagnostics(session, calibration, poses)


@pytest.mark.parametrize("absent", ["poses", "calibration", "empty_poses"])
def test_missing_evidence_never_invents_pose(evidence, absent):
    session, calibration, poses = evidence
    if absent == "poses":
        poses = None
    elif absent == "calibration":
        calibration = None
    else:
        poses["poses"] = []
    result = build_board_diagnostics(session, calibration, poses)
    assert result["status"] == "unavailable" and result["frames"] == []
    assert result["board"] == session["board"]
    assert result["image_size_px"] == [640, 480]
    assert result["warnings"]


def test_subset_of_native_poses_excludes_unsolved_views(evidence):
    session, calibration, poses = evidence
    poses["poses"] = poses["poses"][:1]
    result = build_board_diagnostics(session, calibration, poses)
    assert result["status"] == "available"
    assert [frame["frame_id"] for frame in result["frames"]] == [8]
    assert any("1 selected views have no native pose" in w for w in result["warnings"])


def test_native_board_warp_is_disclosed_without_reconstruction(evidence):
    session, calibration, poses = evidence
    poses["calobject_warp"] = [.001, -.002]
    result = build_board_diagnostics(session, calibration, poses)
    assert any("without applying warp" in w for w in result["warnings"])
    poses["calobject_warp"] = [0., 0.]
    assert not any("without applying warp" in w for w in build_board_diagnostics(*evidence)["warnings"])


@pytest.mark.parametrize("kind", ["scale", "reflection", "nonhomogeneous", "nan", "bad_shape", "boolean", "string"])
def test_malformed_transforms_reject_entire_response(evidence, kind):
    session, calibration, poses = evidence
    transform = poses["poses"][1]["camera_cv_T_board"]
    if kind == "scale":
        transform[0][0] *= 2
    elif kind == "reflection":
        transform[:3] = (np.asarray(transform[:3]) * [-1, 1, 1, 1]).tolist()
    elif kind == "nonhomogeneous":
        transform[3][2] = .01
    elif kind == "nan":
        transform[0][3] = float("nan")
    elif kind == "bad_shape":
        transform.pop()
    elif kind == "boolean":
        transform[3][3] = True
    else:
        transform[3][3] = "1"
    with pytest.raises(ValueError, match="camera_cv_T_board"):
        build_board_diagnostics(session, calibration, poses)


@pytest.mark.parametrize("z", [-1., 0.])
def test_board_corners_must_be_in_front_of_camera(evidence, z):
    session, calibration, poses = evidence
    transform = np.eye(4)
    transform[2, 3] = z
    poses["poses"][0]["camera_cv_T_board"] = transform.tolist()
    with pytest.raises(ValueError, match="in front"):
        build_board_diagnostics(session, calibration, poses)


@pytest.mark.parametrize("kind", ["duplicate_pose", "unselected_pose", "duplicate_image", "duplicate_frame_id",
                                  "missing_frame_id", "unsafe_image", "wrong_convention", "robot_transform"])
def test_identity_and_coordinate_system_mismatches_rejected(evidence, kind):
    session, calibration, poses = evidence
    if kind == "duplicate_pose":
        poses["poses"].append(copy.deepcopy(poses["poses"][0]))
    elif kind == "unselected_pose":
        poses["poses"][0]["image"] = "frames/other.png"
    elif kind == "duplicate_image":
        session["views"][1]["image"] = session["views"][0]["image"]
    elif kind == "duplicate_frame_id":
        session["views"][1]["source_frame_id"] = session["views"][0]["source_frame_id"]
    elif kind == "missing_frame_id":
        del session["views"][0]["source_frame_id"]
    elif kind == "unsafe_image":
        session["views"][0]["image"] = "frames/../../private.png"
    elif kind == "wrong_convention":
        poses["convention"] = "wpilib_nwu"
    else:
        poses["robot_to_camera"] = np.eye(4).tolist()
    with pytest.raises(ValueError):
        build_board_diagnostics(session, calibration, poses)


@pytest.mark.parametrize("kind", ["calibration_resolution", "view_resolution", "calibration_board", "board_size",
                                  "lensmodel", "distortion_count", "matrix_string", "distortion_boolean",
                                  "corner_shape", "corner_nan", "corner_boolean", "corner_outside",
                                  "too_many_views", "too_many_poses", "too_many_corners"])
def test_untrusted_board_camera_and_observation_mismatches_rejected(evidence, kind):
    session, calibration, poses = evidence
    if kind == "calibration_resolution":
        calibration["width"] = 1280
    elif kind == "view_resolution":
        session["views"][0]["width"] = 1280
    elif kind == "calibration_board":
        calibration["board"]["square_size_m"] = .04
    elif kind == "board_size":
        session["board"]["cols"] = 41
    elif kind == "lensmodel":
        calibration["lensmodel"] = "LENSMODEL_SPLINED_STEREOGRAPHIC"
    elif kind == "distortion_count":
        calibration["dist_coeffs"] = [0.] * 5
    elif kind == "matrix_string":
        calibration["camera_matrix"][0][0] = "600"
    elif kind == "distortion_boolean":
        calibration["dist_coeffs"][0] = False
    elif kind == "corner_shape":
        session["views"][0]["corners"].pop()
    elif kind == "corner_nan":
        session["views"][0]["corners"][0][0] = float("nan")
    elif kind == "corner_boolean":
        session["views"][0]["corners"][0][0] = False
    elif kind == "corner_outside":
        session["views"][0]["corners"][0][0] = 640
    elif kind == "too_many_views":
        session["views"] = [session["views"][0]] * 181
    elif kind == "too_many_poses":
        poses["poses"] = [poses["poses"][0]] * 181
    else:
        session["views"][0]["corners"] *= 100
    with pytest.raises(ValueError):
        build_board_diagnostics(session, calibration, poses)


def test_malformed_pose_is_rejected_even_without_runtime_calibration(evidence):
    session, _, poses = evidence
    poses["poses"][0]["camera_cv_T_board"] = np.eye(3).tolist()
    with pytest.raises(ValueError):
        build_board_diagnostics(session, None, poses)


def test_mode_dimensions_and_board_configuration_are_retained_for_unavailable_view(evidence):
    session, _, _ = evidence
    session["board"] = {"cols": 3, "rows": 4, "square_size_m": .025}
    session["views"] = []
    session["width"], session["height"] = 1280, 800
    result = build_board_diagnostics(session, None, None)
    assert result["board"] == {"cols": 3, "rows": 4, "square_size_m": .025}
    assert result["image_size_px"] == [1280, 800]
    assert result["status"] == "unavailable" and result["frames"] == []
