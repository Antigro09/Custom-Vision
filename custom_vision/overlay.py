"""Preview-only drawing; never rotate raw processing/calibration coordinates."""

from __future__ import annotations

import cv2
import numpy as np

from .calibration import validate_calibration


def annotate(frame: np.ndarray, detections: list[dict], calibration: dict | None = None,
             tag_size_m: float = .1651, objects: dict | None = None,
             *, poi: dict | None = None, box_depth_ratio: float = .5) -> np.ndarray:
    """Copy a captured frame and draw tag poses or object target overlays.

    Call this in the preview worker after publication, only at preview FPS.
    The box projects toward the observer from the tag's visible face. The raw
    pose frame's +Z points into the tag, so box height is negative raw Z.
    """
    result = (cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR) if frame.ndim == 2 else frame.copy())
    height, width = frame.shape[:2]
    matrix, distortion = None, None
    if calibration is not None:
        checked = validate_calibration(calibration)
        if (width, height) == (checked["width"], checked["height"]):
            matrix, distortion = np.array(checked["camera_matrix"]), np.array(checked["dist_coeffs"])
    if not np.isfinite(box_depth_ratio) or not .1 <= box_depth_ratio <= 2.:
        raise ValueError("box_depth_ratio must be in [0.1, 2]")
    half = tag_size_m / 2
    square = np.array([[-half, half, 0.], [half, half, 0.], [half, -half, 0.], [-half, -half, 0.]])
    cube = np.vstack((square, square + [0., 0., -tag_size_m * box_depth_ratio]))
    axes = np.array([[0., 0., 0.], [0., 0., -tag_size_m / 2],
                     [-tag_size_m / 2, 0., 0.], [0., tag_size_m / 2, 0.]])
    for detection in detections:
        corners = np.asarray(detection.get("corners", []), dtype=np.float64)
        if corners.shape != (4, 2) or not np.isfinite(corners).all():
            continue
        polygon = np.clip(np.rint(corners), -1e6, 1e6).astype(np.int32)
        cv2.polylines(result, [polygon], True, (80, 220, 80), 2, cv2.LINE_AA)
        center = np.rint(np.asarray(detection.get("center", corners.mean(axis=0)))).astype(int)
        label = f"ID {detection['id']}"
        if detection.get("pose_ambiguous"):
            label += " ambiguous"
        elif detection.get("distance_m") is not None:
            label += f" {detection['distance_m']:.2f}m"
        cv2.putText(result, label, (int(center[0]) + 5, int(center[1]) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, (40, 240, 240), 1, cv2.LINE_AA)
        if matrix is None or not detection.get("pose_valid"):
            continue
        try:
            rvec, tvec = np.asarray(detection["rvec_rad"]), np.asarray(detection["tvec_m"])
            rotation = cv2.Rodrigues(rvec)[0]
            if np.any(((rotation @ cube.T).T + tvec.reshape(3))[:, 2] <= 0):
                continue
            projected = cv2.projectPoints(np.vstack((cube, axes)), rvec, tvec, matrix, distortion)[0].reshape(-1, 2)
            if not np.isfinite(projected).all():
                continue
            projected = np.clip(np.rint(projected), -1e6, 1e6).astype(np.int32)
            cv2.polylines(result, [projected[4:8]], True, (255, 180, 20), 2, cv2.LINE_AA)
            for index in range(4):
                cv2.line(result, tuple(projected[index]), tuple(projected[index + 4]),
                         (255, 180, 20), 1, cv2.LINE_AA)
            for index, color in enumerate(((0, 0, 255), (0, 255, 0), (255, 0, 0)), 9):
                cv2.line(result, tuple(projected[8]), tuple(projected[index]), color, 2, cv2.LINE_AA)
        except (KeyError, ValueError, cv2.error):
            continue
    if matrix is not None and poi:
        for target in poi.get("targets", []):
            if not target.get("geometry_valid") or not target.get("in_image"):
                continue
            pixel = np.asarray(target.get("pixel"), dtype=float)
            if pixel.shape != (2,) or not np.isfinite(pixel).all():
                continue
            x, y = np.rint(pixel).astype(int)
            if not (0 <= x < width and 0 <= y < height):
                continue
            color = (200, 80, 255) if target.get("valid") else (0, 180, 255)
            cv2.drawMarker(result, (x, y), color, cv2.MARKER_CROSS, 22, 2, cv2.LINE_AA)
            label = f"POI {target.get('name', '')}" + ("" if target.get("valid") else " PREVIEW ONLY")
            cv2.putText(result, label, (x + 12, max(15, y - 10)), cv2.FONT_HERSHEY_SIMPLEX,
                        .45, color, 1, cv2.LINE_AA)
    _draw_objects(result, detections, objects)
    return result


def _draw_objects(result: np.ndarray, detections: list[dict], objects: dict | None) -> None:
    """Draw original-frame boxes and optional approximate segmentation contours."""
    objects = objects or {}
    targets = {target.get("detection_index"): target for target in objects.get("targets", [])}
    for index, detection in enumerate(detections):
        try:
            box = np.asarray(detection.get("bbox_xyxy", []), dtype=np.float64)
            if box.shape != (4,) or not np.isfinite(box).all() or np.any(box[2:] <= box[:2]):
                continue
            target = detection.get("robot_relative") or targets.get(index, {})
            track_id = target.get("track_id")
            selected = (objects.get("valid") is True and target.get("valid") is True
                        and track_id is not None and track_id == objects.get("selected_track_id"))
            color = (80, 235, 130) if selected else (255, 180, 60)
            box_pixels = np.clip(np.rint(box), -1e6, 1e6).astype(np.int32)
            cv2.rectangle(result, tuple(box_pixels[:2]), tuple(box_pixels[2:]),
                          color, 3 if selected else 2, cv2.LINE_AA)
            segmentation = detection.get("segmentation") or {}
            contour = np.asarray(segmentation.get("contour_px", []), dtype=np.float64)
            if contour.ndim == 2 and contour.shape[1:] == (2,) and len(contour) >= 3 and np.isfinite(contour).all():
                pixels = np.clip(np.rint(contour), -1e6, 1e6).astype(np.int32)
                cv2.polylines(result, [pixels], True, color, 1, cv2.LINE_AA)
            label = str(detection.get("label", "object"))[:80]
            if track_id is not None:
                label += f" #{track_id}"
            distance = target.get("range_xy_m")
            if objects.get("valid") is True and target.get("valid") is True and isinstance(distance, (int, float)) and np.isfinite(distance):
                label += f" {distance:.2f}m"
            if selected:
                label += " selected"
            x = max(0, min(result.shape[1] - 1, int(box_pixels[0])))
            y = max(14, min(result.shape[0] - 1, int(box_pixels[1]) - 6))
            cv2.putText(result, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, .45,
                        color, 1, cv2.LINE_AA)
        except (TypeError, ValueError, cv2.error):
            continue
