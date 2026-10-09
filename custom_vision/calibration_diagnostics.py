"""Bounded board-only diagnostics from retained calibration evidence.

This module performs no capture, fitting, file access or activation. Its metric
geometry is the configured chessboard transformed by retained native poses, not
arbitrary scene reconstruction or a robot mount estimate. Untrusted evidence is
validated before any frame is returned; unavailable evidence has no fake pose.
"""
from __future__ import annotations

import math
from numbers import Real
from pathlib import PurePosixPath

import cv2
import numpy as np

from .calibration import validate_calibration
from .calibration_session import Board


MAX_FRAMES = 180
MAX_CORNERS = 40 * 40
POSE_CONVENTION = (
    "p_camera_cv = R @ p_board + t; camera +x right,+y down,+z forward; meters"
)
_WARNINGS = [
    "Camera optical axes: +x right, +y down, +z forward; geometry is in meters.",
    "Solved chessboard geometry is not arbitrary scene reconstruction or robot mount extrinsics.",
    "An ordinary chessboard can have ambiguous 180-degree corner orientation; physical corner numbering is not verified.",
]


def _object(value, name):
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be an object")
    return value


def _integer(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise ValueError(f"{name} must be an integer in [{low}, {high}]")
    return value


def _array(value, shape, name):
    # Reject coerced strings/bools and ragged or oversized input before casting.
    try:
        _check_array(value, shape)
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a numeric array with shape {shape}") from exc
    if not np.isfinite(array).all():
        raise ValueError(f"{name} must contain finite values")
    return array


def _check_array(value, shape):
    if not shape:
        if not isinstance(value, Real) or isinstance(value, (bool, np.bool_)):
            raise ValueError
        return
    if not isinstance(value, (list, tuple, np.ndarray)) or len(value) != shape[0]:
        raise ValueError
    for child in value:
        _check_array(child, shape[1:])


def _image(value):
    if (not isinstance(value, str) or not 1 <= len(value) <= 255
            or any(ord(c) < 32 for c in value) or "\\" in value
            or any(c in value for c in (":", "?", "#"))):
        raise ValueError("View image must be a safe relative image name")
    path = PurePosixPath(value)
    if (path.is_absolute() or path.as_posix() != value
            or any(part in (".", "..") for part in path.parts)):
        raise ValueError("View image must be a safe relative image name")
    return value


def _board(value):
    value = _object(value, "Session board")
    if set(value) != {"cols", "rows", "square_size_m"}:
        raise ValueError("Board requires cols, rows and measured square_size_m")
    cols = _integer(value["cols"], "Board cols", 3, 40)
    rows = _integer(value["rows"], "Board rows", 3, 40)
    square = value["square_size_m"]
    if (isinstance(square, bool) or not isinstance(square, Real)
            or not math.isfinite(square) or not 0 < square < 1):
        raise ValueError("Measured square_size_m must be finite and between 0 and 1 meter")
    return Board(cols, rows, float(square))


def _rigid_transform(value):
    transform = _array(value, (4, 4), "camera_cv_T_board")
    rotation = transform[:3, :3]
    if (not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6, rtol=0)):
        raise ValueError("camera_cv_T_board must be a right-handed rigid homogeneous transform")
    return transform


def _observed_corners(value, board, width, height):
    observed = _array(value, (board.cols * board.rows, 2), "Observed corners")
    if np.any(observed < 0) or np.any(observed >= [width, height]):
        raise ValueError("Observed corners lie outside the selected session image")
    return observed


def build_board_diagnostics(session, calibration, board_poses_or_none):
    """Return normalized, JSON-safe camera-optical board evidence.

    ``session`` is the selected raw session.json object. ``calibration`` is the
    exact OpenCV8 runtime export, including matching resolution and optional
    board metadata. ``board_poses_or_none`` is the retained native board-poses.json
    object; each pose may retain ``observed_corners_px`` in the final detector's
    solved order, labeled ``observed_corner_source='solver_final_detection'``.
    Poses are joined by exact image name, never array position. Missing
    poses/calibration return ``unavailable``; malformed evidence raises ValueError
    rather than partially mixing frames or silently supplying an identity pose.
    This response is diagnostic evidence and does not certify calibration trust.
    """
    session = _object(session, "Session")
    board = _board(session.get("board"))
    width = _integer(session.get("width"), "Session width", 1, 32768)
    height = _integer(session.get("height"), "Session height", 1, 32768)
    views = session.get("views")
    if not isinstance(views, list) or len(views) > MAX_FRAMES:
        raise ValueError(f"Session views must be a list with at most {MAX_FRAMES} frames")

    selected = {}
    frame_ids = set()
    for view in views:
        view = _object(view, "Selected view")
        image = _image(view.get("image"))
        frame_id = _integer(view.get("source_frame_id"), "View source_frame_id", 0, 2**53 - 1)
        if image in selected or frame_id in frame_ids:
            raise ValueError("Selected views must have unique image and source_frame_id identities")
        for name, expected in (("width", width), ("height", height)):
            if name in view and _integer(view[name], f"View {name}", 1, 32768) != expected:
                raise ValueError("View dimensions do not match the selected session")
        observed = _observed_corners(view.get("corners"), board, width, height)
        selected[image] = (frame_id, observed)
        frame_ids.add(frame_id)

    result = {"schema_version": 1, "frame": "camera_optical", "units": "m",
              "board": {"cols": board.cols, "rows": board.rows, "square_size_m": board.square_size_m},
              "image_size_px": [width, height], "status": "unavailable", "frames": [],
              "warnings": list(_WARNINGS)}
    if board_poses_or_none is None:
        result["warnings"].append("No retained native board poses are available; no 3D geometry was inferred.")
        return result

    artifact = _object(board_poses_or_none, "Native board poses")
    if artifact.get("convention") != POSE_CONVENTION:
        raise ValueError("Unsupported or missing native board pose coordinate convention")
    if artifact.get("robot_to_camera") is not None:
        raise ValueError("Board diagnostics require camera poses and cannot use robot mount transforms")
    poses = artifact.get("poses")
    if not isinstance(poses, list) or len(poses) > MAX_FRAMES:
        raise ValueError(f"Native poses must be a list with at most {MAX_FRAMES} frames")
    transforms = {}
    points = board.points()
    for pose in poses:
        pose = _object(pose, "Native pose")
        image = _image(pose.get("image"))
        if image not in selected:
            raise ValueError("Native pose image is not a selected session view")
        if image in transforms:
            raise ValueError("Native poses must have unique image identities")
        transform = _rigid_transform(pose.get("camera_cv_T_board"))
        camera_points = points @ transform[:3, :3].T + transform[:3, 3]
        if not np.isfinite(camera_points).all() or np.any(camera_points[:, 2] <= 0):
            raise ValueError("Every solved board corner must be finite and in front of the camera")
        if "observed_corners_px" in pose and pose.get("observed_corner_source") != "solver_final_detection":
            raise ValueError("Native observed corners require solver_final_detection provenance")
        native_observed = (_observed_corners(pose["observed_corners_px"], board, width, height)
                           if "observed_corners_px" in pose else None)
        transforms[image] = (transform, camera_points, native_observed)

    if "calobject_warp" in artifact:
        warp = _array(artifact["calobject_warp"], (2,), "Native calobject_warp")
        if np.any(warp != 0):
            result["warnings"].append(
                "The native solve includes board warp; diagnostics display the configured planar chessboard without applying warp.")
    if not poses:
        result["warnings"].append("The native artifact contains no solved board poses.")
        return result
    if calibration is None:
        result["warnings"].append("No exact runtime intrinsics are available; no calibrated 3D view was returned.")
        return result

    calibration = _object(calibration, "Calibration")
    # The native OpenCV8 export uses a flat eight-coefficient vector. Check its
    # original numbers before the shared validator can normalize/coerce them.
    matrix = _array(calibration.get("camera_matrix"), (3, 3), "Calibration camera_matrix")
    distortion = _array(calibration.get("dist_coeffs"), (8,), "Calibration dist_coeffs")
    calibration = validate_calibration(calibration)
    if (calibration["width"], calibration["height"]) != (width, height):
        raise ValueError("Calibration resolution does not match the selected session")
    if (len(calibration["dist_coeffs"]) != 8
            or calibration.get("lensmodel", "LENSMODEL_OPENCV8") != "LENSMODEL_OPENCV8"):
        raise ValueError("Board diagnostics require the exact OpenCV8 runtime projection model")
    if "board" in calibration and _board(calibration["board"]) != board:
        raise ValueError("Calibration board configuration does not match the selected session")
    missing_native_observations = 0
    changed_native_observations = False
    for image, (frame_id, snapshot_observed) in selected.items():
        if image not in transforms:
            continue
        transform, camera_points, native_observed = transforms[image]
        if native_observed is None:
            observed = snapshot_observed
            missing_native_observations += 1
        else:
            observed = native_observed
            changed_native_observations |= not np.allclose(observed, snapshot_observed, atol=1e-6, rtol=0)
        try:
            projected = cv2.projectPoints(camera_points, np.zeros(3), np.zeros(3), matrix,
                                          distortion)[0].reshape(-1, 2)
        except cv2.error as exc:
            raise ValueError("Runtime projection of solved board corners failed") from exc
        if not np.isfinite(projected).all():
            raise ValueError("Runtime projection produced nonfinite board corners")
        residual = projected - observed
        rms = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
        if not math.isfinite(rms):
            raise ValueError("Runtime reprojection residual is nonfinite")
        result["frames"].append({"frame_id": frame_id, "image": image,
                                 "transform_board_to_camera": transform.tolist(),
                                 "corners_camera_m": camera_points.tolist(),
                                 "corners_px": observed.tolist(),
                                 "projected_corners_px": projected.tolist(),
                                 "width": width, "height": height, "reprojection_rms_px": rms})
    if len(transforms) < len(selected):
        result["warnings"].append(
            f"{len(selected) - len(transforms)} selected views have no native pose and are excluded from the 3D view.")
    if missing_native_observations:
        result["warnings"].append(
            f"{missing_native_observations} solved views lack retained final detector observations; their RMS compares snapshot corners and may differ from the native fit residual or corner order.")
    if changed_native_observations:
        result["warnings"].append(
            "Native detector observations differ from snapshot observations; displayed corner order and residuals use the retained native observations without inferring physical board orientation.")
    result["status"] = "available"
    return result
