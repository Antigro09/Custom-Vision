"""Preview-only drawing; never rotate raw processing/calibration coordinates."""

from __future__ import annotations

import cv2
import numpy as np

from .calibration import validate_calibration


def annotate(frame: np.ndarray, detections: list[dict], calibration: dict | None = None,
             tag_size_m: float = .1651) -> np.ndarray:
    """Copy a captured frame and draw decoded polygons plus calibrated 3D boxes.

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
    half = tag_size_m / 2
    square = np.array([[-half, half, 0.], [half, half, 0.], [half, -half, 0.], [-half, -half, 0.]])
    cube = np.vstack((square, square + [0., 0., -tag_size_m / 2]))
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
    return result
