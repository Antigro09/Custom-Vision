"""Pinhole camera calibration from chessboard images at one capture resolution."""

from __future__ import annotations

import argparse
import glob
import json
import math
from pathlib import Path

import cv2
import numpy as np


def validate_calibration(calibration: dict) -> dict:
    """Validate and normalize a JSON calibration; never supply dummy intrinsics."""
    if not isinstance(calibration, dict):
        raise ValueError("Calibration must be a JSON object")
    for key in ("width", "height"):
        value = calibration.get(key)
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError(f"Calibration {key} must be a positive integer capture resolution")
    if calibration.get("distortion_model", "opencv_pinhole") != "opencv_pinhole":
        raise ValueError("Only opencv_pinhole calibration is supported; fisheye requires its own model")
    try:
        matrix = np.asarray(calibration["camera_matrix"], dtype=np.float64)
        distortion = np.asarray(calibration["dist_coeffs"], dtype=np.float64)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Calibration requires numeric camera_matrix and dist_coeffs") from exc
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("camera_matrix must be a finite 3x3 matrix")
    if (matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.allclose(matrix[2], [0, 0, 1])
            or not np.allclose([matrix[0, 1], matrix[1, 0]], [0, 0])):
        raise ValueError("camera_matrix must contain positive fx/fy, zero skew, and last row [0,0,1]")
    if not (0 <= matrix[0, 2] < calibration["width"] and 0 <= matrix[1, 2] < calibration["height"]):
        raise ValueError("Calibration principal point must lie inside the capture resolution")
    if (distortion.ndim not in (1, 2) or distortion.size not in (4, 5, 8, 12, 14)
            or (distortion.ndim == 2 and 1 not in distortion.shape) or not np.isfinite(distortion).all()):
        raise ValueError("dist_coeffs must be a finite vector with 4, 5, 8, 12, or 14 values")
    normalized = dict(calibration)
    normalized.update(camera_matrix=matrix.tolist(), dist_coeffs=distortion.reshape(-1).tolist(),
                      distortion_model="opencv_pinhole")
    return normalized


def load_calibration(path: str | Path) -> dict:
    return validate_calibration(json.loads(Path(path).read_text(encoding="utf-8")))


def calibrate_images(image_paths: list[str | Path], board_cols: int, board_rows: int,
                     square_size_m: float, *, min_views: int = 10, max_rms_px: float = 1.0) -> dict:
    """Detect inner corners, fit intrinsics, and report rejected views and RMS.

    At least ten diverse views are recommended. Duplicate views are excluded;
    all readable input images must have exactly the same width and height.
    """
    for label, value in (("board_cols", board_cols), ("board_rows", board_rows)):
        if isinstance(value, bool) or not isinstance(value, int) or value < 3:
            raise ValueError(f"{label} must be an integer >= 3 (inner corners, not squares)")
    if isinstance(min_views, bool) or not isinstance(min_views, int) or min_views < 3:
        raise ValueError("min_views must be an integer >= 3; 10 or more is recommended")
    if not math.isfinite(square_size_m) or square_size_m <= 0:
        raise ValueError("square_size_m must be finite and positive")
    if not math.isfinite(max_rms_px) or max_rms_px <= 0:
        raise ValueError("max_rms_px must be finite and positive")
    paths = list(dict.fromkeys(Path(path).resolve() for path in image_paths))
    if len(paths) < min_views:
        raise ValueError(f"Need at least {min_views} distinct images; received {len(paths)}")
    object_template = np.zeros((board_cols * board_rows, 3), dtype=np.float32)
    object_template[:, :2] = np.mgrid[0:board_cols, 0:board_rows].T.reshape(-1, 2) * square_size_m
    image_points = []
    accepted = []
    skipped = []
    image_size = None
    for path in paths:
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            raise ValueError(f"Cannot read calibration image: {path}")
        size = (gray.shape[1], gray.shape[0])
        if image_size is not None and size != image_size:
            raise ValueError(f"Calibration image resolution mismatch: {path} is {size}, expected {image_size}")
        image_size = size
        found, corners = cv2.findChessboardCornersSB(
            gray, (board_cols, board_rows), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            skipped.append({"image": str(path), "reason": "chessboard_not_found"})
            continue
        # Reversed ordering can represent the same symmetric board view.
        duplicate = any(min(np.max(np.linalg.norm(corners - other, axis=2)),
                            np.max(np.linalg.norm(corners[::-1] - other, axis=2))) < 2.0
                        for other in image_points)
        if duplicate:
            skipped.append({"image": str(path), "reason": "duplicate_view"})
            continue
        image_points.append(corners.astype(np.float32))
        accepted.append(str(path))
    if len(image_points) < min_views:
        raise ValueError(f"Only {len(image_points)} usable, distinct chessboard views; need {min_views}. "
                         "Capture different positions, distances, and tilts at one resolution.")
    objects = [object_template.copy() for _ in image_points]
    try:
        rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
            objects, image_points, image_size, None, None)
    except cv2.error as exc:
        raise ValueError(f"OpenCV could not calibrate the supplied images: {exc}") from exc
    if not math.isfinite(rms) or rms > max_rms_px:
        raise ValueError(f"Calibration RMS {rms:.3f}px exceeds allowed {max_rms_px:.3f}px; "
                         "check board dimensions, motion blur, coverage, and lens focus")
    per_view = []
    for path, observed, rvec, tvec in zip(accepted, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(object_template, rvec, tvec, matrix, distortion)
        error = float(np.sqrt(np.mean(np.sum((observed - projected) ** 2, axis=2))))
        per_view.append({"image": path, "rms_px": error})
    return validate_calibration({
        "width": image_size[0], "height": image_size[1],
        "camera_matrix": matrix.tolist(), "dist_coeffs": distortion.reshape(-1).tolist(),
        "distortion_model": "opencv_pinhole", "rms_error_px": float(rms),
        "board": {"inner_cols": board_cols, "inner_rows": board_rows, "square_size_m": square_size_m},
        "accepted_views": len(image_points), "per_view_errors": per_view, "skipped_images": skipped,
    })


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", required=True, nargs="+", help="Image paths or quoted glob patterns")
    parser.add_argument("--board-cols", type=int, required=True, help="Number of inner chessboard corners across")
    parser.add_argument("--board-rows", type=int, required=True, help="Number of inner chessboard corners down")
    parser.add_argument("--square-size-m", type=float, required=True, help="Measured square side in meters")
    parser.add_argument("--min-views", type=int, default=10)
    parser.add_argument("--max-rms-px", type=float, default=1.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    paths = []
    for pattern in args.images:
        matches = sorted(glob.glob(pattern))
        if not matches:
            parser.error(f"No images match {pattern!r}")
        paths.extend(matches)
    try:
        result = calibrate_images(paths, args.board_cols, args.board_rows, args.square_size_m,
                                  min_views=args.min_views, max_rms_px=args.max_rms_px)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    print(f"Saved {args.output}: {result['accepted_views']} views, "
          f"{result['width']}x{result['height']}, RMS {result['rms_error_px']:.3f}px")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
