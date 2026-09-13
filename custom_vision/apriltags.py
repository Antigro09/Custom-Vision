"""AprilTag IDs and calibrated tag-to-camera poses.

Camera coordinates are OpenCV's +x right, +y down, +z forward. A pose maps
tag-local points into that camera frame; it is not a robot or field pose.
"""

from __future__ import annotations

import math

import cv2
import numpy as np

from .calibration import validate_calibration


def tag_object_points(tag_size_m: float) -> np.ndarray:
    """Return pupil/AprilTag corners in OpenCV IPPE_SQUARE order.

    Pupil's homography maps (-1,+1), (+1,+1), (+1,-1), (-1,-1) to
    detection.corners. Preserve decoded orientation; never sort by pixels.
    https://github.com/pupil-labs/apriltags/blob/main/src/pupil_apriltags/bindings.py
    https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html
    """
    half = tag_size_m / 2.0
    return np.array([[-half, half, 0], [half, half, 0],
                     [half, -half, 0], [-half, -half, 0]], dtype=np.float64)


def _positive_float(config: dict, key: str, default: float, *, zero_ok: bool = False) -> float:
    try:
        value = float(config.get(key, default))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"apriltags.{key} must be a finite number") from exc
    if not math.isfinite(value) or value < 0 or (value == 0 and not zero_ok):
        raise ValueError(f"apriltags.{key} must be {'nonnegative' if zero_ok else 'positive'}")
    return value


class AprilTagPipeline:
    """Detect tag36h11 by default; emit JSON-native results per frame.

    Calibration must describe the exact raw image resolution. Missing
    calibration and resolution mismatches retain pixel detections with an
    explicit invalid pose, rather than assuming a lens or scaling intrinsics.
    """

    def __init__(self, config: dict, calibration: dict | None = None):
        if not isinstance(config, dict):
            raise ValueError("apriltags configuration must be an object")
        self.mode = config.get("mode", "3d")
        if self.mode not in ("2d", "3d"):
            raise ValueError("AprilTag mode must be 2d or 3d")
        self.known_tag_ids = set(config.get("known_tag_ids", []))
        self.skip_single_when_multi = config.get("skip_single_when_multi", False)
        self.tag_size_m = _positive_float(config, "tag_size_m", 0.1651)
        self.min_decision_margin = _positive_float(config, "min_decision_margin", 30.0, zero_ok=True)
        self.max_reprojection_error_px = _positive_float(config, "max_reprojection_error_px", 3.0)
        decimate = _positive_float(config, "quad_decimate", 1.0)
        threads = config.get("threads", 2)
        max_hamming = config.get("max_hamming", 0)
        if isinstance(threads, bool) or not isinstance(threads, int) or threads < 1:
            raise ValueError("apriltags.threads must be a positive integer")
        if isinstance(max_hamming, bool) or not isinstance(max_hamming, int) or not 0 <= max_hamming <= 2:
            raise ValueError("apriltags.max_hamming must be an integer from 0 to 2")
        self.max_hamming = max_hamming
        family = config.get("tag_family", "tag36h11")
        supported = {"tag16h5", "tag25h9", "tag36h11", "tagCircle21h7", "tagCircle49h12",
                     "tagCustom48h12", "tagStandard41h12", "tagStandard52h13"}
        if not isinstance(family, str) or family not in supported:
            raise ValueError(f"Unsupported AprilTag family: {family!r}")
        self.calibration = validate_calibration(calibration) if calibration is not None else None
        self._object_points = tag_object_points(self.tag_size_m)
        self._camera_matrix = None
        self._dist_coeffs = None
        if self.calibration is not None:
            self._camera_matrix = np.asarray(self.calibration["camera_matrix"], dtype=np.float64)
            self._dist_coeffs = np.asarray(self.calibration["dist_coeffs"], dtype=np.float64)
        try:
            from pupil_apriltags import Detector
        except ImportError as exc:
            raise RuntimeError("AprilTag detection requires pupil-apriltags; run scripts/setup_jetson.sh") from exc
        self.detector = Detector(families=family, nthreads=threads, quad_decimate=decimate,
                                 refine_edges=1, debug=0)

    def _estimate_pose(self, corners: np.ndarray) -> dict:
        from .localization import estimate_tag_pose
        return estimate_tag_pose(corners, self.tag_size_m, self._camera_matrix,
                                 self._dist_coeffs, self.max_reprojection_error_px)

    def process(self, frame_bgr: np.ndarray) -> list[dict]:
        if (not isinstance(frame_bgr, np.ndarray) or frame_bgr.dtype != np.uint8
                or frame_bgr.ndim not in (2, 3) or (frame_bgr.ndim == 3 and frame_bgr.shape[2] != 3)
                or min(frame_bgr.shape[:2]) < 8):
            raise ValueError("AprilTag input must be a nonempty uint8 BGR image, at least 8x8")
        gray = frame_bgr if frame_bgr.ndim == 2 else cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape
        pose_reason = None
        if self.mode == "2d":
            pose_reason = "2d_mode"
        elif self.calibration is None:
            pose_reason = "no_calibration"
        elif (width, height) != (self.calibration["width"], self.calibration["height"]):
            pose_reason = "calibration_resolution_mismatch"
        results = []
        tags = self.detector.detect(np.ascontiguousarray(gray), estimate_tag_pose=False)
        if self.skip_single_when_multi and len({tag.tag_id for tag in tags if tag.tag_id in self.known_tag_ids and tag.hamming <= self.max_hamming and tag.decision_margin >= self.min_decision_margin}) >= 2 and not pose_reason:
            pose_reason = "deferred_multitag"
        for tag in tags:
            margin = float(tag.decision_margin)
            hamming = int(tag.hamming)
            if not math.isfinite(margin) or margin < self.min_decision_margin or hamming > self.max_hamming:
                continue
            corners = np.asarray(tag.corners, dtype=np.float64).reshape(4, 2)
            center = np.asarray(tag.center, dtype=np.float64).reshape(2)
            if not np.isfinite(corners).all() or not np.isfinite(center).all():
                continue
            detection = {"id": int(tag.tag_id), "decision_margin": margin, "hamming": hamming,
                         "center": center.tolist(), "corners": corners.tolist(), "pose_valid": False}
            if pose_reason:
                detection["pose_invalid_reason"] = pose_reason
            else:
                detection.update(self._estimate_pose(corners))
            results.append(detection)
        return results
