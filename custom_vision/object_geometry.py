"""Calibrated floor-plane ranging and short-lived robot-relative object association.

All metric results are in the robot's WPILib NWU frame at capture time. Without
odometry, association is deliberately bounded and never predicts missing objects.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading

import cv2
import numpy as np

from .calibration import validate_calibration
from .localization import CV_TO_NWU, _rpy_rotation, validate_robot_to_camera


_DEFAULTS = {
    "target_height_m": None, "anchor": "bbox_center", "min_range_m": 0.05,
    "max_range_m": 5.0, "min_downward_angle_deg": 8.0,
    "max_position_std_m": 0.30, "pixel_std_px": 2.0, "height_std_m": 0.015,
    "pitch_std_deg": 0.5, "mount_xy_std_m": 0.01,
    "tracking_gate_m": 0.30, "tracking_ttl_s": 0.15,
    "selection_hysteresis_m": 0.20, "intake_offset_m": [0.0, 0.0],
    "approach_standoff_m": 0.10, "reject_clipped_boxes": True, "max_targets": 100,
}


def _finite(value, name, lower, upper):
    if isinstance(value, (bool, str)) or not isinstance(value, (int, float, np.number)):
        raise ValueError(f"geometry.{name} must be a finite number")
    value = float(value)
    if not math.isfinite(value) or not lower <= value <= upper:
        raise ValueError(f"geometry.{name} must be in [{lower}, {upper}]")
    return value


def validate_geometry_settings(settings: dict | None) -> dict:
    """Normalize only geometry settings; reject misspellings instead of guessing."""
    if settings is None:
        settings = {}
    if not isinstance(settings, dict):
        raise ValueError("geometry must be an object")
    if any(not isinstance(key, str) for key in settings):
        raise ValueError("geometry setting names must be strings")
    unknown = set(settings) - set(_DEFAULTS)
    if unknown:
        raise ValueError(f"Unknown geometry setting(s): {', '.join(sorted(unknown))}")
    result = dict(_DEFAULTS, **settings)
    if result["target_height_m"] is not None:
        result["target_height_m"] = _finite(result["target_height_m"], "target_height_m", -3, 3)
    if result["anchor"] not in ("bbox_center", "bbox_bottom", "mask_centroid", "mask_bottom"):
        raise ValueError("geometry.anchor must be bbox_center, bbox_bottom, mask_centroid, or mask_bottom")
    bounds = {
        "min_range_m": (0, 20), "max_range_m": (0.01, 20),
        "min_downward_angle_deg": (1, 85), "max_position_std_m": (0.001, 5),
        "pixel_std_px": (0.01, 100), "height_std_m": (0, 1),
        "pitch_std_deg": (0, 15), "mount_xy_std_m": (0, 1),
        "tracking_gate_m": (0.001, 2), "tracking_ttl_s": (0.001, 0.5),
        "selection_hysteresis_m": (0, 5), "approach_standoff_m": (0, 5),
    }
    for key, (low, high) in bounds.items():
        result[key] = _finite(result[key], key, low, high)
    if result["min_range_m"] >= result["max_range_m"]:
        raise ValueError("geometry.min_range_m must be less than max_range_m")
    offset = result["intake_offset_m"]
    if not isinstance(offset, (list, tuple)) or len(offset) != 2:
        raise ValueError("geometry.intake_offset_m must be [forward_m, left_m]")
    result["intake_offset_m"] = [_finite(v, "intake_offset_m", -3, 3) for v in offset]
    if not isinstance(result["reject_clipped_boxes"], bool):
        raise ValueError("geometry.reject_clipped_boxes must be boolean")
    count = result["max_targets"]
    if isinstance(count, bool) or not isinstance(count, int) or not 1 <= count <= 256:
        raise ValueError("geometry.max_targets must be an integer in [1, 256]")
    return result


@dataclass
class _Track:
    track_id: int
    class_key: tuple
    xy: np.ndarray
    observed_s: float
    first_observed_s: float
    observations: int


class ObjectGeometry:
    """Estimate targets on an explicitly configured plane; preserve raw detections.

    ``enrich`` and ``reset`` may be called from separate runtime/watchdog threads.
    A missing calibration, measured mount, or target height permits 2D only.
    Positive mount pitch points a forward camera down, matching WPILib NWU.
    """

    def __init__(self, settings: dict | None, calibration: dict | None = None,
                 robot_to_camera: dict | None = None):
        self.settings = validate_geometry_settings(settings)
        self.calibration = validate_calibration(calibration) if calibration is not None else None
        self.robot_to_camera = validate_robot_to_camera(robot_to_camera)
        self._lock = threading.RLock()
        self._tracks: dict[int, _Track] = {}
        self._next_id = 1
        self._selected_id = None
        self._last_capture_s = None
        self._matrix = self._distortion = self._origin = self._rotation = None
        self._pitch_rotations = None
        if self.calibration is not None:
            self._matrix = np.asarray(self.calibration["camera_matrix"], np.float64)
            self._distortion = np.asarray(self.calibration["dist_coeffs"], np.float64)
        if self.robot_to_camera is not None:
            self._origin = np.asarray(self.robot_to_camera["translation_m"], np.float64)
            rpy = np.asarray(self.robot_to_camera["rotation_rpy_deg"], np.float64)
            self._rotation = _rpy_rotation(rpy) @ CV_TO_NWU
            # Central difference is in degrees; pitch_std_deg has the same unit.
            step = np.array([0., 0.01, 0.])
            self._pitch_rotations = ((_rpy_rotation(rpy + step) @ CV_TO_NWU),
                                     (_rpy_rotation(rpy - step) @ CV_TO_NWU))
        self._min_downward = math.sin(math.radians(self.settings["min_downward_angle_deg"]))

    def reset(self):
        """Forget target association on disconnect/restart; never reuse old IDs."""
        with self._lock:
            self._tracks.clear()
            self._selected_id = None
            self._last_capture_s = None

    def _prerequisite(self, image_shape):
        if self.calibration is None:
            return "missing_calibration"
        if tuple(image_shape[:2]) != (self.calibration["height"], self.calibration["width"]):
            return "calibration_resolution_mismatch"
        if self.robot_to_camera is None:
            return "missing_measured_camera_mount"
        height = self.settings["target_height_m"]
        if height is None:
            return "missing_target_height"
        if self._origin[2] <= height:
            return "camera_not_above_target_plane"
        return None

    def _anchor(self, detection, image_shape):
        h, w = image_shape[:2]
        try:
            box = np.asarray(detection["bbox_xyxy"], np.float64)
        except (ValueError, TypeError, KeyError):
            return None, "invalid_bounding_box"
        if (box.shape != (4,) or not np.isfinite(box).all() or
                box[0] >= box[2] or box[1] >= box[3]):
            return None, "invalid_bounding_box"
        if box[0] < 0 or box[1] < 0 or box[2] > w or box[3] > h:
            return None, "bounding_box_outside_image"
        if self.settings["reject_clipped_boxes"] and (box[0] <= 0 or box[1] <= 0 or box[2] >= w or box[3] >= h):
            return None, "clipped_detection"
        kind = self.settings["anchor"]
        if kind == "bbox_center":
            anchor = (box[:2] + box[2:]) / 2
        elif kind == "bbox_bottom":
            anchor = np.array([(box[0] + box[2]) / 2, box[3]])
        else:
            segmentation = detection.get("segmentation")
            if not isinstance(segmentation, dict):
                return None, "missing_segmentation_anchor"
            try:
                anchor = np.asarray(segmentation["centroid_px" if kind == "mask_centroid" else "bottom_px"], np.float64)
            except (KeyError, TypeError, ValueError):
                return None, "missing_segmentation_anchor"
        if anchor.shape != (2,) or not np.isfinite(anchor).all():
            return None, "invalid_anchor"
        if not (0 <= anchor[0] < w and 0 <= anchor[1] < h):
            return None, "anchor_outside_image"
        if not (box[0] <= anchor[0] <= box[2] and box[1] <= anchor[1] <= box[3]):
            return None, "anchor_outside_bounding_box"
        return anchor, None

    def _intersect_unchecked(self, optical_rays, rotation):
        rays = optical_rays @ rotation.T
        with np.errstate(divide="ignore", invalid="ignore"):
            scales = (self.settings["target_height_m"] - self._origin[2]) / rays[:, 2]
            points = self._origin + scales[:, None] * rays
        return points, rays, scales

    def _project(self, anchors):
        # Undistort all targets and finite-difference samples in one OpenCV call.
        samples = np.asarray(anchors, np.float64)[:, None, :] + np.array(
            [[0., 0.], [1., 0.], [-1., 0.], [0., 1.], [0., -1.]])[None, :, :]
        normalized = cv2.undistortPoints(samples.reshape(-1, 1, 2), self._matrix,
                                        self._distortion).reshape(-1, 2)
        optical = np.column_stack((normalized, np.ones(len(normalized))))
        points, rays, scales = self._intersect_unchecked(optical, self._rotation)
        points = points.reshape(-1, 5, 3)
        rays = rays.reshape(-1, 5, 3)[:, 0]
        scales = scales.reshape(-1, 5)[:, 0]
        plus = self._intersect_unchecked(optical[::5], self._pitch_rotations[0])[0]
        minus = self._intersect_unchecked(optical[::5], self._pitch_rotations[1])[0]
        output = []
        for index, p in enumerate(points):
            d = rays[index]
            down = -d[2] / np.linalg.norm(d)
            if down < self._min_downward:
                output.append((None, "ray_near_or_above_horizon"))
                continue
            if scales[index] <= 0 or not np.isfinite(p).all():
                output.append((None, "intersection_behind_camera"))
                continue
            point = p[0]
            xy_range = float(np.linalg.norm(point[:2]))
            if xy_range < 1e-6:
                output.append((None, "bearing_undefined_at_robot_origin"))
                continue
            if not self.settings["min_range_m"] <= xy_range <= self.settings["max_range_m"]:
                output.append((None, "outside_validated_range"))
                continue
            j_pixel = np.column_stack(((p[1, :2] - p[2, :2]) / 2,
                                       (p[3, :2] - p[4, :2]) / 2))
            # Combined camera-height/target-height error changes their separation.
            j_height = -d[:2] / d[2]
            j_pitch = (plus[index, :2] - minus[index, :2]) / 0.02
            covariance = (j_pixel @ j_pixel.T * self.settings["pixel_std_px"] ** 2
                          + np.outer(j_height, j_height) * self.settings["height_std_m"] ** 2
                          + np.outer(j_pitch, j_pitch) * self.settings["pitch_std_deg"] ** 2
                          + np.eye(2) * self.settings["mount_xy_std_m"] ** 2)
            if not np.isfinite(covariance).all():
                output.append((None, "uncertainty_unbounded"))
                continue
            largest_std = math.sqrt(max(0., float(np.linalg.eigvalsh(covariance)[-1])))
            if largest_std > self.settings["max_position_std_m"]:
                output.append((None, "position_uncertainty_too_large"))
                continue
            radial = point[:2] / xy_range
            bearing_j = np.array([-point[1], point[0]]) / xy_range ** 2
            target = {
                "valid": True, "frame": "robot_relative_at_capture_wpilib_nwu",
                "translation_m": point.tolist(), "range_xy_m": xy_range,
                "bearing_deg": math.degrees(math.atan2(point[1], point[0])),
                "anchor": self.settings["anchor"], "anchor_px": anchors[index].tolist(),
                "method": "calibrated_ray_target_height_plane", "approximate": True,
                "target_height_m": self.settings["target_height_m"],
                "uncertainty": {
                    "model": "first_order_configured_pixel_height_pitch_mount",
                    "covariance_xy_m2": covariance.tolist(),
                    "std_xy_m": np.sqrt(np.maximum(0., np.diag(covariance))).tolist(),
                    "max_position_std_m": largest_std,
                    "range_std_m": math.sqrt(max(0., float(radial @ covariance @ radial))),
                    "bearing_std_deg": math.degrees(math.sqrt(max(0., float(bearing_j @ covariance @ bearing_j)))),
                },
            }
            intake_vector = point[:2] - np.asarray(self.settings["intake_offset_m"])
            intake_range = float(np.linalg.norm(intake_vector))
            approach = (intake_vector * max(0., 1 - self.settings["approach_standoff_m"] / intake_range)
                        if intake_range > 1e-9 else np.zeros(2))
            target.update(range_from_intake_m=intake_range, approach={
                "translation_m": [float(approach[0]), float(approach[1]), 0.],
                "rotation_yaw_deg": 0., "frame": "robot_relative_at_capture_wpilib_nwu",
                "heading_policy": "maintain_capture_heading", "path_validated": False,
            })
            output.append((target, None))
        return output

    @staticmethod
    def _class_key(detection):
        value = detection.get("class_id")
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            return None
        label = detection.get("label")
        if not isinstance(label, str) or not label:
            return None
        return value, label

    def _associate(self, targets, capture_s):
        ttl = self.settings["tracking_ttl_s"]
        self._tracks = {key: t for key, t in self._tracks.items() if 0 <= capture_s - t.observed_s <= ttl}
        # Keep association work bounded even if each frame introduces new IDs.
        limit = 2 * self.settings["max_targets"]
        if len(self._tracks) > limit:
            recent = sorted(self._tracks.values(), key=lambda t: (-t.observed_s, -t.track_id))[:limit]
            self._tracks = {t.track_id: t for t in recent}
        # These are two-scalar distances, not large linear-algebra operations.
        # Group classes once and avoid allocating a NumPy array for every edge.
        tracks_by_class = {}
        for track_id, track in self._tracks.items():
            tracks_by_class.setdefault(track.class_key, []).append(
                (track_id, float(track.xy[0]), float(track.xy[1])))
        edges = []
        gate = self.settings["tracking_gate_m"]
        for index, target in enumerate(targets):
            x, y = map(float, target["translation_m"][:2])
            for track_id, tx, ty in tracks_by_class.get((target["class_id"], target["label"]), ()):
                distance = math.hypot(x - tx, y - ty)
                if distance <= gate:
                    edges.append((distance, track_id, index))
        # Sorting the geometric edges makes associations independent of detector
        # confidence/order, except intrinsically ambiguous co-located detections.
        assigned, consumed = {}, set()
        for _, track_id, index in sorted(edges):
            if track_id not in consumed and index not in assigned:
                assigned[index] = track_id
                consumed.add(track_id)
        for index, target in enumerate(targets):
            track_id = assigned.get(index)
            if track_id is None:
                track_id = self._next_id
                self._next_id += 1
                track = _Track(track_id, (target["class_id"], target["label"]),
                               np.asarray(target["translation_m"][:2]), capture_s, capture_s, 0)
                self._tracks[track_id] = track
            else:
                track = self._tracks[track_id]
            track.xy = np.asarray(target["translation_m"][:2])
            track.observed_s = capture_s
            track.observations += 1
            target.update(track_id=track_id, observed=True, predicted=False,
                          capture_monotonic_us=int(capture_s * 1e6),
                          track_observations=track.observations,
                          track_age_ms=(capture_s - track.first_observed_s) * 1000.)
        if len(self._tracks) > limit:
            recent = sorted(self._tracks.values(), key=lambda t: (-t.observed_s, -t.track_id))[:limit]
            self._tracks = {t.track_id: t for t in recent}
        # IDs can survive a brief miss, but a missed track is NEVER published or
        # selected. No camera-relative history masquerades as a stationary map.
        if not targets:
            self._selected_id = None
            return None
        nearest = min(targets, key=lambda t: (t["range_from_intake_m"], t["track_id"]))
        previous = next((t for t in targets if t["track_id"] == self._selected_id), None)
        selected = previous if (previous and previous["range_from_intake_m"] <=
                                nearest["range_from_intake_m"] + self.settings["selection_hysteresis_m"]) else nearest
        self._selected_id = selected["track_id"]
        return selected

    def enrich(self, detections, image_shape, capture_monotonic_s):
        """Return one coherent frame; no wall-clock reads or inference-time timestamp."""
        capture_s = _finite(capture_monotonic_s, "capture_monotonic_s", 0, 1e12)
        if (not isinstance(image_shape, (list, tuple)) or len(image_shape) < 2 or
                any(isinstance(v, bool) or not isinstance(v, (int, np.integer)) or v <= 0 for v in image_shape[:2])):
            raise ValueError("image_shape requires positive [height, width, ...]")
        if not isinstance(detections, (list, tuple)) or any(not isinstance(d, dict) for d in detections):
            raise ValueError("detections must be a list of detection objects")
        with self._lock:
            result = [dict(d) for d in detections]
            problem = self._prerequisite(image_shape)
            if self._last_capture_s is not None and capture_s <= self._last_capture_s:
                problem = "non_monotonic_capture_timestamp"
                self._tracks.clear()
                self._selected_id = None
            self._last_capture_s = max(capture_s, self._last_capture_s or 0.)
            anchors, indices = [], []
            for index, detection in enumerate(result):
                reason = problem
                if reason is None and self._class_key(detection) is None:
                    reason = "missing_or_invalid_class_identity"
                if reason is None and len(anchors) >= self.settings["max_targets"]:
                    reason = "target_limit_exceeded"
                anchor = None
                if reason is None:
                    anchor, reason = self._anchor(detection, image_shape)
                detection["robot_relative"] = {"valid": False, "invalid_reason": reason}
                if reason is None:
                    anchors.append(anchor)
                    indices.append(index)
            targets = []
            if anchors:
                for index, (target, reason) in zip(indices, self._project(anchors)):
                    if target is None:
                        result[index]["robot_relative"]["invalid_reason"] = reason
                    else:
                        target.update(detection_index=index, class_id=result[index]["class_id"], label=result[index]["label"])
                        confidence = result[index].get("confidence")
                        if isinstance(confidence, (int, float)) and not isinstance(confidence, bool) and math.isfinite(confidence):
                            target["confidence"] = float(confidence)
                        if isinstance(result[index].get("confidence_kind"), str):
                            target["confidence_kind"] = result[index]["confidence_kind"]
                        result[index]["robot_relative"] = target
                        targets.append(target)
            if problem is not None:
                self._tracks.clear()
                self._selected_id = None
            selected = self._associate(targets, capture_s)
            return {"detections": result, "objects": {
                "valid": bool(targets), "targets": targets, "selected_target": selected,
                "selected_track_id": selected["track_id"] if selected else None,
                "capture_monotonic_us": int(capture_s * 1e6),
                "invalid_reason": None if targets else (problem or "no_valid_targets"),
                "frame": "robot_relative_at_capture_wpilib_nwu", "motion_compensated": False,
            }}
