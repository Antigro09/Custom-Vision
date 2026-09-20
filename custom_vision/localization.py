"""Calibrated AprilTag geometry in WPILib NWU, independent of capture and NT.

``A_to_B`` is the pose of B expressed in A: its matrix maps B points into A.
All inputs/outputs use meters, except explicitly named degree/pixel fields.
The optical frame used only for OpenCV PnP is x-right/y-down/z-forward.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

import cv2
import numpy as np

from .calibration import validate_calibration


CV_TO_NWU = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
# Raw tag object corners decode upright printed TR, TL, BL, BR. This is NOT
# the camera optical-to-NWU change of basis. WP tag +X faces the observer.
RAW_FROM_WP_TAG = np.array([[0., -1., 0.], [0., 0., 1.], [-1., 0., 0.]])


def _number(value, name: str, *, positive: bool = False) -> float:
    if isinstance(value, (bool, str)):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(result) or (positive and result <= 0):
        raise ValueError(f"{name} must be finite" + (" and positive" if positive else ""))
    return result


def _vector(value, length: int, name: str) -> np.ndarray:
    if not isinstance(value, (list, tuple, np.ndarray)) or len(value) != length:
        raise ValueError(f"{name} must contain {length} numbers")
    result = np.array([_number(x, name) for x in value], dtype=np.float64)
    if result.shape != (length,):
        raise ValueError(f"{name} must contain {length} numbers")
    return result


def _quaternion_rotation(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([[1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
                     [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
                     [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]])


def _rotation_quaternion(rotation: np.ndarray) -> list[float]:
    # Rodrigues' axis-angle conversion remains stable around pi, unlike the
    # trace-only quaternion formula. Canonicalize sign for stable telemetry.
    x, y, z = map(float, cv2.Rodrigues(rotation)[0].reshape(3))
    angle = math.hypot(x, y, z)
    if angle < 1e-12:
        return [1., 0., 0., 0.]
    w = math.cos(angle / 2)
    scale = math.sin(angle / 2) / angle
    sign = -1. if w < 0 else 1.
    return [sign*w, sign*scale*x, sign*scale*y, sign*scale*z]


def _rpy_rotation(rpy_degrees: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = np.deg2rad(rpy_degrees)
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                            math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp, cp*sr, cp*cr]])


def _transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = rotation, translation.reshape(3)
    return transform


def invert_transform(transform: np.ndarray) -> np.ndarray:
    rotation = transform[:3, :3].T
    return _transform(rotation, -rotation @ transform[:3, 3])


def pose_dict(transform: np.ndarray) -> dict:
    rotation = transform[:3, :3]
    pitch = math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0]))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = 0.
        yaw = math.atan2(-rotation[0, 1], rotation[1, 1])
    return {"translation_m": transform[:3, 3].tolist(),
            "rotation_quaternion_wxyz": _rotation_quaternion(rotation),
            "rotation_rpy_deg": [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)],
            "frame": "wpilib_nwu"}


def validate_robot_to_camera(value: dict | None) -> dict | None:
    """Validate measured camera mount; absence never means an identity mount."""
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ValueError("robot_to_camera must be an object or null")
    return {"translation_m": _vector(value.get("translation_m"), 3,
                                     "robot_to_camera.translation_m").tolist(),
            "rotation_rpy_deg": _vector(value.get("rotation_rpy_deg"), 3,
                                       "robot_to_camera.rotation_rpy_deg").tolist()}


def validate_field_layout(value: dict) -> dict:
    """Accept WPILib AprilTagFieldLayout JSON, preserving its fixed origin.

    Field dimensions and translations are meters. Rotations are normalized
    WXYZ quaternions, allowing only small floating-point serialization drift.
    No season, field layout, or alliance transform is guessed.
    """
    if not isinstance(value, dict) or not isinstance(value.get("field"), dict):
        raise ValueError("Field layout requires a field object and tags list")
    field = {key: _number(value["field"].get(key), f"field.{key}", positive=True)
             for key in ("length", "width")}
    tags = value.get("tags")
    if not isinstance(tags, list) or not 1 <= len(tags) <= 1024:
        raise ValueError("Field layout tags must be a list with 1 to 1024 tags")
    result, ids = [], set()
    for entry in tags:
        if not isinstance(entry, dict):
            raise ValueError("Each field tag must be an object")
        tag_id = entry.get("ID")
        if isinstance(tag_id, bool) or not isinstance(tag_id, int) or not 0 <= tag_id <= 2**31 - 1:
            raise ValueError("Field tag ID must be a nonnegative integer")
        if tag_id in ids:
            raise ValueError(f"Duplicate field tag ID {tag_id}")
        ids.add(tag_id)
        try:
            translation = entry["pose"]["translation"]
            quaternion = entry["pose"]["rotation"]["quaternion"]
            xyz = [_number(translation[k], f"tag {tag_id} translation.{k}") for k in ("x", "y", "z")]
            q = np.array([_number(quaternion[k], f"tag {tag_id} quaternion.{k}")
                          for k in ("W", "X", "Y", "Z")])
        except (KeyError, TypeError) as exc:
            raise ValueError(f"Tag {tag_id} needs pose.translation xyz and rotation.quaternion WXYZ") from exc
        norm = float(np.linalg.norm(q))
        if abs(norm - 1.) > 1e-3:
            raise ValueError(f"Tag {tag_id} quaternion must have unit norm")
        q /= norm
        result.append({"ID": tag_id, "pose": {"translation": dict(zip(("x", "y", "z"), xyz)),
                       "rotation": {"quaternion": dict(zip(("W", "X", "Y", "Z"), q.tolist()))}}})
    return {"field": field, "tags": result}


def load_field_layout(path: str | Path) -> dict:
    return validate_field_layout(json.loads(Path(path).read_text(encoding="utf-8")))


def _raw_corners(size: float) -> np.ndarray:
    h = size / 2
    return np.array([[-h, h, 0.], [h, h, 0.], [h, -h, 0.], [-h, -h, 0.]])


def _rms(points: np.ndarray, observed: np.ndarray, rvec: np.ndarray, tvec: np.ndarray,
         matrix: np.ndarray, distortion: np.ndarray) -> tuple[float, np.ndarray]:
    projected = cv2.projectPoints(points, rvec, tvec, matrix, distortion)[0].reshape(-1, 2)
    squared = np.sum((projected - observed) ** 2, axis=1)
    return float(np.sqrt(np.mean(squared))), squared


def _ambiguity(errors: list[float]) -> float:
    if len(errors) < 2:
        return 0.
    best, alternate = sorted(errors)[:2]
    return 1. if alternate < 1e-7 else min(1., best / alternate)


def estimate_tag_pose(corners, tag_size_m, camera_matrix, dist_coeffs,
                      max_reprojection_error_px=3.) -> dict:
    """Solve both planar branches; refine only the best, retain honest ambiguity.

    Cyclically rebase the solver to avoid OpenCV IPPE's rotation-near-pi
    numerical singularity, then restore the tag's decoded coordinate frame.
    """
    invalid = {"pose_valid": False, "pose_invalid_reason": "pnp_failed"}
    points = _raw_corners(tag_size_m)
    observed = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    matrix, distortion = np.asarray(camera_matrix, dtype=np.float64), np.asarray(dist_coeffs, dtype=np.float64)
    if not np.isfinite(observed).all():
        return invalid
    index = int(np.argmax(observed[:, 1] - observed[:, 0]))
    restore = cv2.Rodrigues(np.array([0., 0., index * math.pi / 2]))[0]
    try:
        result = cv2.solvePnPGeneric(points, np.roll(observed, -index, axis=0), matrix,
                                     distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        candidates = []
        if result[0]:
            for rvec, tvec in zip(result[1], result[2]):
                rotation = cv2.Rodrigues(rvec)[0] @ restore
                rvec = cv2.Rodrigues(rotation)[0]
                if not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
                    continue
                if np.any(((rotation @ points.T).T + tvec.reshape(3))[:, 2] <= 0):
                    continue
                error, _ = _rms(points, observed, rvec, tvec, matrix, distortion)
                if math.isfinite(error):
                    candidates.append((error, rvec, tvec))
        if not candidates:
            return invalid
        candidates.sort(key=lambda candidate: candidate[0])
        ambiguity = _ambiguity([candidate[0] for candidate in candidates])
        error, rvec, tvec = candidates[0]
        ok, refined_r, refined_t = cv2.solvePnP(points, observed, matrix, distortion,
                                               rvec.copy(), tvec.copy(), True,
                                               flags=cv2.SOLVEPNP_ITERATIVE)
        if ok and np.isfinite(refined_r).all() and np.isfinite(refined_t).all():
            refined_rotation = cv2.Rodrigues(refined_r)[0]
            refined_error, _ = _rms(points, observed, refined_r, refined_t, matrix, distortion)
            if (refined_error <= error and
                    np.all(((refined_rotation @ points.T).T + refined_t.reshape(3))[:, 2] > 0)):
                error, rvec, tvec = refined_error, refined_r, refined_t
        if error > max_reprojection_error_px:
            return {"pose_valid": False, "pose_invalid_reason": "reprojection_error",
                    "reprojection_error_px": error, "pose_ambiguity": ambiguity}
        pose = {"pose_valid": True, "rvec_rad": rvec.reshape(3).tolist(),
                "tvec_m": tvec.reshape(3).tolist(), "distance_m": float(np.linalg.norm(tvec)),
                "reprojection_error_px": error, "pose_ambiguity": ambiguity,
                "pose_source": "single_tag_pnp"}
        if len(candidates) > 1:
            pose.update(alternate_rvec_rad=candidates[1][1].reshape(3).tolist(),
                        alternate_tvec_m=candidates[1][2].reshape(3).tolist(),
                        alternate_reprojection_error_px=candidates[1][0])
        return pose
    except cv2.error:
        return invalid


def _raw_to_camera_nwu(rvec, tvec) -> np.ndarray:
    return _transform(CV_TO_NWU @ cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))[0]
                      @ RAW_FROM_WP_TAG, CV_TO_NWU @ np.asarray(tvec, dtype=np.float64))


def _empty(reason: str) -> dict:
    return {"valid": False, "method": "none", "invalid_reason": reason,
            "field_to_camera": None, "field_to_robot": None, "used_tag_ids": [],
            "reprojection_error_px": None, "ambiguity": None, "inlier_tag_count": 0}


class Localization:
    """Per-camera single/multi-tag localization and robot-relative targeting."""

    def __init__(self, config: dict, calibration: dict | None = None,
                 field_layout: dict | None = None, robot_to_camera: dict | None = None):
        if not isinstance(config, dict):
            raise ValueError("Localization configuration must be an object")
        self.mode = config.get("mode", "3d")
        if self.mode not in ("2d", "3d"):
            raise ValueError("apriltags.mode must be '2d' or '3d'")
        self.pose_device = config.get('pose_device', 'cpu')
        if self.pose_device not in ('cpu', 'cuda'):
            raise ValueError('pose_device must be cpu or cuda')
        self.multitag_solver = None
        self.tag_size_m = _number(config.get("tag_size_m", .1651), "tag_size_m", positive=True)
        self.max_error = _number(config.get("max_reprojection_error_px", 3.),
                                 "max_reprojection_error_px", positive=True)
        self.max_ambiguity = _number(config.get("max_ambiguity", .2), "max_ambiguity")
        if not 0 <= self.max_ambiguity <= 1:
            raise ValueError("max_ambiguity must lie between 0 and 1")
        self.hfov = _number(config.get("hfov_deg", 81.), "hfov_deg", positive=True)
        self.vfov = _number(config.get("vfov_deg", 52.), "vfov_deg", positive=True)
        if self.hfov >= 180 or self.vfov >= 180:
            raise ValueError("Nominal FOV must be below 180 degrees")
        self.always_single_tag = config.get("always_single_tag", False)
        if not isinstance(self.always_single_tag, bool):
            raise ValueError("always_single_tag must be boolean")
        self.multitag = config.get("multitag", True)
        if not isinstance(self.multitag, bool):
            raise ValueError("multitag must be boolean")
        self.calibration = validate_calibration(calibration) if calibration is not None else None
        self.matrix = None if self.calibration is None else np.array(self.calibration["camera_matrix"])
        self.distortion = None if self.calibration is None else np.array(self.calibration["dist_coeffs"])
        mount = validate_robot_to_camera(robot_to_camera)
        self.robot_to_camera = None if mount is None else _transform(
            _rpy_rotation(np.array(mount["rotation_rpy_deg"])), np.array(mount["translation_m"]))
        self.field_layout = validate_field_layout(field_layout) if field_layout is not None else None
        self.field_tags, self.field_corners = {}, {}
        self.raw_corners = _raw_corners(self.tag_size_m)
        self.single_pose_calls = 0
        self.single_pose_ms = 0.
        self.single_pose_devices = set()
        wp_corners = self.raw_corners @ RAW_FROM_WP_TAG
        for tag in self.field_layout["tags"] if self.field_layout else []:
            xyz = np.array([tag["pose"]["translation"][k] for k in ("x", "y", "z")])
            q = np.array([tag["pose"]["rotation"]["quaternion"][k] for k in ("W", "X", "Y", "Z")])
            rotation = _quaternion_rotation(q)
            self.field_tags[tag["ID"]] = _transform(rotation, xyz)
            self.field_corners[tag["ID"]] = (rotation @ wp_corners.T).T + xyz

    def _angles(self, detection: dict, width: int, height: int, calibrated: bool) -> None:
        center = np.asarray(detection["center"], dtype=np.float64).reshape(1, 1, 2)
        if calibrated:
            x, y = cv2.undistortPoints(center, self.matrix, self.distortion).reshape(2)
            detection["angle_source"] = "calibrated"
        else:
            x = (center[0, 0, 0] - width / 2) * (2 * math.tan(math.radians(self.hfov) / 2) / width)
            y = (center[0, 0, 1] - height / 2) * (2 * math.tan(math.radians(self.vfov) / 2) / height)
            detection["angle_source"] = "nominal_fov_approximate"
        detection.update(yaw_deg=math.degrees(math.atan2(-x, 1.)),
                         pitch_deg=math.degrees(math.atan2(-y, math.hypot(1., x))),
                         area_pct=float(abs(cv2.contourArea(np.asarray(detection["corners"],
                                                                        dtype=np.float32)))) * 100 / (width * height))

    def _relative(self, detection: dict, transform: np.ndarray | None = None) -> None:
        if not detection.get("pose_valid"):
            return
        if transform is None:
            transform = _raw_to_camera_nwu(detection["rvec_rad"], detection["tvec_m"])
        detection["camera_to_target"] = pose_dict(transform)
        detection["robot_to_target"] = (None if self.robot_to_camera is None else
                                          pose_dict(self.robot_to_camera @ transform))
        detection["pose_ambiguous"] = detection.get("pose_ambiguity", 1.) > self.max_ambiguity

    def _ensure_single(self, detection: dict) -> None:
        needs_pose = ("pose_ambiguity" not in detection or (not detection.get("pose_valid")
                                                          and "rvec_rad" not in detection))
        # Never rerun a completed native solve (or silently fall back from CUDA)
        # just because its quality check rejected the result.
        if needs_pose and not detection.get("pose_attempted", False):
            solver = getattr(self, "single_pose_solver", None)
            if solver is None and self.pose_device == 'cuda':
                raise RuntimeError('CUDA single-tag solver is unavailable; no CPU fallback is allowed')
            started = time.perf_counter_ns()
            estimate = solver(detection["corners"]) if solver else estimate_tag_pose(
                detection["corners"], self.tag_size_m, self.matrix, self.distortion, self.max_error)
            self.single_pose_ms += (time.perf_counter_ns() - started) / 1e6
            self.single_pose_calls += 1
            self.single_pose_devices.add(estimate.get('pose_device', 'cpu'))
            detection.update(estimate)
            # An invalid completed solve must not run twice when robust joint
            # localization considers per-tag seeds and then emits its targets.
            detection['pose_device'] = estimate.get('pose_device', 'cpu')
            detection['pose_attempted'] = True
        detection.setdefault("pose_source", "single_tag_pnp")
        if detection.get("pose_valid"):
            detection.pop("pose_invalid_reason", None)
        self._relative(detection)

    def _result(self, field_to_camera: np.ndarray, method: str, ids: list[int],
                error: float, ambiguity: float, rejected_ids: list[int] | None = None) -> dict:
        valid = ambiguity <= self.max_ambiguity and error <= self.max_error
        result = {"valid": valid, "method": method, "used_tag_ids": ids,
                  "inlier_tag_count": len(ids), "reprojection_error_px": error,
                  "ambiguity": ambiguity, "field_to_camera": None, "field_to_robot": None,
                  "rejected_tag_ids": rejected_ids or []}
        if not valid:
            result["invalid_reason"] = "ambiguous_pose" if ambiguity > self.max_ambiguity else "reprojection_error"
            return result
        result["field_to_camera"] = pose_dict(field_to_camera)
        if self.robot_to_camera is not None:
            result["field_to_robot"] = pose_dict(field_to_camera @ invert_transform(self.robot_to_camera))
        else:
            result["robot_pose_invalid_reason"] = "no_robot_to_camera"
        return result

    def _multitag(self, known: list[dict]) -> tuple[dict, np.ndarray | None]:
        if self.pose_device == 'cuda':
            return self._multitag_cuda(known)
        points = np.concatenate([self.field_corners[d["id"]] for d in known])
        observed = np.concatenate([np.asarray(d["corners"], dtype=np.float64) for d in known])
        candidates = []

        def add(rvec, tvec):
            if not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
                return
            rotation = cv2.Rodrigues(rvec)[0]
            depths = ((rotation @ points.T).T + tvec.reshape(3))[:, 2].reshape(-1, 4)
            error, squared = _rms(points, observed, rvec, tvec, self.matrix, self.distortion)
            tag_errors = np.sqrt(squared.reshape(-1, 4).mean(axis=1))
            mask = (tag_errors <= self.max_error) & np.all(depths > 0, axis=1)
            # Do not let corner-level RANSAC turn three corners of a bad tag
            # into a reported multi-tag inlier; all four corners participate.
            count = int(mask.sum())
            score_error = float(np.sqrt(np.mean(squared.reshape(-1, 4)[mask]))) if count else error
            candidates.append((count, score_error, mask, rvec, tvec))

        try:
            coplanar = np.linalg.svd(points - points.mean(axis=0), compute_uv=False)[-1] < 1e-7
            flag = cv2.SOLVEPNP_IPPE if coplanar else cv2.SOLVEPNP_SQPNP
            solutions = cv2.solvePnPGeneric(points, observed, self.matrix, self.distortion, flags=flag)
            if solutions[0]:
                for rvec, tvec in zip(solutions[1], solutions[2]):
                    add(rvec, tvec)
            direct_all_inliers = bool(candidates) and max(candidate[0] for candidate in candidates) == len(known)
            if not candidates or max(candidate[0] for candidate in candidates) < len(known):
                # RANSAC is paid for only when the direct joint fit disagrees.
                ok, rvec, tvec, _ = cv2.solvePnPRansac(
                    points, observed, self.matrix, self.distortion, iterationsCount=60,
                    reprojectionError=self.max_error, confidence=.995, flags=cv2.SOLVEPNP_EPNP)
                if ok:
                    add(rvec, tvec)
                for detection in known:
                    self._ensure_single(detection)
                    if not detection.get("pose_valid"):
                        continue
                    camera_to_tag = _raw_to_camera_nwu(detection["rvec_rad"], detection["tvec_m"])
                    # field-to-camera = field-to-tag * inverse(camera-to-tag).
                    field_to_camera = self.field_tags[detection["id"]] @ invert_transform(camera_to_tag)
                    camera_to_field = invert_transform(field_to_camera)
                    add(cv2.Rodrigues(CV_TO_NWU.T @ camera_to_field[:3, :3])[0],
                        (CV_TO_NWU.T @ camera_to_field[:3, 3]).reshape(3, 1))
            if not candidates:
                return _empty("multitag_pnp_failed"), None
            candidates.sort(key=lambda candidate: (-candidate[0], candidate[1]))
            count, _, mask, rvec, tvec = candidates[0]
            if count < 2:
                return _empty("inconsistent_tag_observations"), None
            inlier_points = points.reshape(-1, 4, 3)[mask].reshape(-1, 3)
            inlier_observed = observed.reshape(-1, 4, 2)[mask].reshape(-1, 2)
            # Re-solve both branches on the complete accepted tags. Global
            # RANSAC's one solution is not evidence that a planar pose is unique.
            if count == len(known) and direct_all_inliers:
                branches = solutions
            else:
                planar_inliers = np.linalg.svd(inlier_points - inlier_points.mean(axis=0),
                                               compute_uv=False)[-1] < 1e-7
                branches = cv2.solvePnPGeneric(inlier_points, inlier_observed, self.matrix,
                                               self.distortion,
                                               flags=cv2.SOLVEPNP_IPPE if planar_inliers else cv2.SOLVEPNP_SQPNP)
            accepted = []
            if branches[0]:
                for branch_r, branch_t in zip(branches[1], branches[2]):
                    if not np.isfinite(branch_r).all() or not np.isfinite(branch_t).all():
                        continue
                    depths = ((cv2.Rodrigues(branch_r)[0] @ inlier_points.T).T + branch_t.reshape(3))[:, 2]
                    if np.any(depths <= 0):
                        continue
                    error, _ = _rms(inlier_points, inlier_observed, branch_r, branch_t,
                                    self.matrix, self.distortion)
                    if math.isfinite(error):
                        accepted.append((error, branch_r, branch_t))
            if not accepted:
                return _empty("multitag_pnp_failed"), None
            accepted.sort(key=lambda candidate: candidate[0])
            ambiguity = _ambiguity([candidate[0] for candidate in accepted])
            error, rvec, tvec = accepted[0]
            ok, refined_r, refined_t = cv2.solvePnP(inlier_points, inlier_observed, self.matrix,
                                                   self.distortion, rvec.copy(), tvec.copy(),
                                                   True, flags=cv2.SOLVEPNP_ITERATIVE)
            if ok and np.isfinite(refined_r).all() and np.isfinite(refined_t).all():
                refined_error, _ = _rms(inlier_points, inlier_observed, refined_r, refined_t,
                                        self.matrix, self.distortion)
                depths = ((cv2.Rodrigues(refined_r)[0] @ inlier_points.T).T + refined_t.reshape(3))[:, 2]
                if refined_error <= error and np.all(depths > 0):
                    error, rvec, tvec = refined_error, refined_r, refined_t
            _, squared = _rms(inlier_points, inlier_observed, rvec, tvec,
                               self.matrix, self.distortion)
            if np.any(np.sqrt(squared.reshape(-1, 4).mean(axis=1)) > self.max_error):
                return _empty("reprojection_error"), None
            camera_to_field = _transform(CV_TO_NWU @ cv2.Rodrigues(rvec)[0],
                                         CV_TO_NWU @ tvec.reshape(3))
            field_to_camera = invert_transform(camera_to_field)
            ids = [d["id"] for d, keep in zip(known, mask) if keep]
            rejected = [d["id"] for d, keep in zip(known, mask) if not keep]
            result = self._result(field_to_camera, "multitag_pnp", ids, error, ambiguity, rejected)
            return result, field_to_camera if result["valid"] else None
        except cv2.error:
            return _empty("multitag_pnp_failed"), None

    def _multitag_cuda(self, known: list[dict]) -> tuple[dict, np.ndarray | None]:
        """Use original mapped corners; all fitting and consensus stay on CUDA."""
        if self.multitag_solver is None:
            raise RuntimeError('CUDA MultiTag solver is unavailable; no CPU fallback is allowed')
        points = np.asarray([self.field_corners[d['id']] for d in known], dtype=np.float64)
        observed = np.asarray([d['corners'] for d in known], dtype=np.float64)
        started = time.perf_counter_ns()
        solved = self.multitag_solver(observed, points)
        wall_ms = (time.perf_counter_ns() - started) / 1e6
        timing = dict(solved.get('timings', {}), call_ms=wall_ms)
        if solved.get('pose_device') != 'cuda':
            raise RuntimeError('CUDA MultiTag solver returned unexpected execution device')
        if not solved.get('pose_valid'):
            result = _empty(solved.get('pose_invalid_reason', 'cuda_multitag_failed'))
            result.update(pose_device='cuda', gpu_timings=timing)
            return result, None
        # No solvePnP/projectPoints here: the GPU already checked every accepted
        # corner. These operations only change the output coordinate convention.
        rvec = np.asarray(solved['rvec_rad'], dtype=np.float64).reshape(3)
        tvec = np.asarray(solved['tvec_m'], dtype=np.float64).reshape(3)
        indices = solved['inlier_indices']
        if (not np.isfinite(rvec).all() or not np.isfinite(tvec).all()
                or len(indices) < 2 or len(set(indices)) != len(indices)
                or any(not isinstance(i, int) or not 0 <= i < len(known) for i in indices)):
            raise RuntimeError('CUDA MultiTag returned invalid geometric output')
        camera_to_field = _transform(CV_TO_NWU @ cv2.Rodrigues(rvec)[0], CV_TO_NWU @ tvec)
        field_to_camera = invert_transform(camera_to_field)
        accepted = set(indices)
        ids = [d['id'] for i, d in enumerate(known) if i in accepted]
        rejected = [d['id'] for i, d in enumerate(known) if i not in accepted]
        result = self._result(field_to_camera, 'multitag_pnp', ids,
                              solved['reprojection_error_px'], solved['pose_ambiguity'], rejected)
        result.update(pose_device='cuda', gpu_timings=timing,
                      coplanar=bool(solved.get('coplanar', False)),
                      tag_reprojection_errors_px={str(known[i]['id']): solved['per_tag_reprojection_error_px'][i]
                                                  for i in indices})
        return result, field_to_camera if result['valid'] else None

    def _derive_target(self, detection: dict, field_to_camera: np.ndarray, ambiguity: float,
                       gpu_error: float | None = None,
                       camera_to_field: np.ndarray | None = None) -> None:
        if camera_to_field is None:
            camera_to_field = invert_transform(field_to_camera)
        camera_to_target = camera_to_field @ self.field_tags[detection["id"]]
        raw_rotation = CV_TO_NWU.T @ camera_to_target[:3, :3] @ RAW_FROM_WP_TAG.T
        raw_translation = CV_TO_NWU.T @ camera_to_target[:3, 3]
        rvec = cv2.Rodrigues(raw_rotation)[0]
        if self.pose_device == 'cuda':
            if gpu_error is None or not math.isfinite(gpu_error) or not 0 <= gpu_error <= self.max_error:
                raise RuntimeError('CUDA field-derived target lacks its validated GPU residual')
            error = gpu_error
        else:
            error, _ = _rms(self.raw_corners, np.asarray(detection["corners"]), rvec,
                            raw_translation, self.matrix, self.distortion)
        # A preceding single-tag solve (required by POI) can have a different
        # alternate branch. It is not the alternate of this joint field pose
        # and must not be paired with the joint solution/ambiguity on the wire.
        for key in ("alternate_rvec_rad", "alternate_tvec_m", "alternate_reprojection_error_px"):
            detection.pop(key, None)
        detection.update(pose_valid=True, rvec_rad=rvec.reshape(3).tolist(), tvec_m=raw_translation.tolist(),
                         distance_m=float(np.linalg.norm(raw_translation)), reprojection_error_px=error,
                         pose_ambiguity=ambiguity, pose_source="field_layout_multitag", pose_device=self.pose_device)
        detection.pop("pose_invalid_reason", None)
        # The joint transform already has the exact target rotation. Avoid a
        # rotation-matrix -> Rodrigues -> rotation-matrix round trip per tag.
        self._relative(detection, camera_to_target)

    def enrich(self, detections: list, image_shape) -> dict:
        self.single_pose_calls, self.single_pose_ms = 0, 0.
        self.single_pose_devices.clear()
        if len(image_shape) < 2 or min(image_shape[:2]) <= 0:
            raise ValueError("image_shape must contain positive capture height and width")
        height, width = int(image_shape[0]), int(image_shape[1])
        calibrated = self.calibration is not None and (width, height) == (
            self.calibration["width"], self.calibration["height"])
        reason = ("mode_2d" if self.mode == "2d" else
                  "no_calibration" if self.calibration is None else
                  "calibration_resolution_mismatch" if not calibrated else None)
        output = []
        for source in detections:
            detection = dict(source)
            corners = np.asarray(detection.get("corners"), dtype=np.float64)
            center = np.asarray(detection.get("center"), dtype=np.float64)
            if corners.shape != (4, 2) or center.shape != (2,) or not np.isfinite(corners).all() or not np.isfinite(center).all():
                continue
            detection["camera_to_target"] = detection["robot_to_target"] = None
            self._angles(detection, width, height, calibrated)
            if reason:
                for key in ("rvec_rad", "tvec_m", "distance_m", "pose_ambiguity", "pose_ambiguous",
                            "alternate_rvec_rad", "alternate_tvec_m", "alternate_reprojection_error_px",
                            "reprojection_error_px", "pose_source"):
                    detection.pop(key, None)
                detection.update(pose_valid=False, pose_invalid_reason=reason,
                                 pose_device="none", pose_attempted=False)
            output.append(detection)
        if reason:
            return {"detections": output, "localization": _empty(reason)}
        ids = [d["id"] for d in output]
        duplicate_ids = {tag_id for tag_id in ids if ids.count(tag_id) > 1}
        known = [d for d in output if d["id"] in self.field_tags and d["id"] not in duplicate_ids]
        result, field_to_camera = _empty("no_field_layout" if not self.field_tags else "no_known_tags"), None
        if len(known) >= 2 and self.multitag:
            result, field_to_camera = self._multitag(known)
            result['pose_device'] = self.pose_device
        camera_to_field = None if field_to_camera is None else invert_transform(field_to_camera)
        for detection in output:
            if (field_to_camera is not None and not self.always_single_tag and
                    detection["id"] in result["used_tag_ids"]):
                self._derive_target(detection, field_to_camera, result["ambiguity"],
                                     result.get('tag_reprojection_errors_px', {}).get(str(detection['id'])),
                                     camera_to_field)
            else:
                self._ensure_single(detection)
            if detection["id"] in duplicate_ids:
                detection["localization_excluded_reason"] = "duplicate_tag_id"
            elif detection["id"] in result.get("rejected_tag_ids", []):
                detection["localization_excluded_reason"] = "multitag_outlier"
        if len(known) == 1 or (known and not self.multitag):
            # Disabled MultiTag selects the lowest-ambiguity usable tag.
            # Never silently use this fallback for contradictory joint inputs.
            detection = min(known, key=lambda d: (not d.get("pose_valid", False),
                                                   d.get("pose_ambiguity", 1.),
                                                   d.get("reprojection_error_px", math.inf)))
            if detection.get("pose_valid"):
                camera_to_tag = _raw_to_camera_nwu(detection["rvec_rad"], detection["tvec_m"])
                field_to_camera = self.field_tags[detection["id"]] @ invert_transform(camera_to_tag)
                result = self._result(field_to_camera, "single_tag_pnp", [detection["id"]],
                                      detection["reprojection_error_px"], detection["pose_ambiguity"])
                result['pose_device'] = detection.get('pose_device', 'cpu')
            else:
                result = _empty(detection.get("pose_invalid_reason", "pnp_failed"))
        if duplicate_ids:
            result["duplicate_tag_ids"] = sorted(duplicate_ids)
            if not known:
                result["invalid_reason"] = "duplicate_tag_id"
        if self.single_pose_calls:
            result['single_tag_fallback'] = {'calls': self.single_pose_calls,
                                            'pose_ms': self.single_pose_ms,
                                            'devices': sorted(self.single_pose_devices)}
        return {"detections": output, "localization": result}
