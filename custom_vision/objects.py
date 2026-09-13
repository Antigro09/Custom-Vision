"""Monochrome/color candidates and raw YOLO detection with camera-relative bearings."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class LetterboxTransform:
    original_width: int
    original_height: int
    scale_x: float
    scale_y: float
    pad_left: int
    pad_top: int

    def restore_boxes(self, boxes: np.ndarray) -> np.ndarray:
        """Map xyxy boxes to original image coordinates and clip to its bounds."""
        restored = np.array(boxes, dtype=np.float32, copy=True)
        restored[:, [0, 2]] = (restored[:, [0, 2]] - self.pad_left) / self.scale_x
        restored[:, [1, 3]] = (restored[:, [1, 3]] - self.pad_top) / self.scale_y
        restored[:, [0, 2]] = np.clip(restored[:, [0, 2]], 0, self.original_width)
        restored[:, [1, 3]] = np.clip(restored[:, [1, 3]], 0, self.original_height)
        return restored


def _input_size(value) -> tuple[int, int]:
    if isinstance(value, int) and not isinstance(value, bool):
        value = (value, value)
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError("input_size must be an integer or [width, height].")
    if any(isinstance(x, bool) or not isinstance(x, int) or x <= 0 for x in value):
        raise ValueError("input_size dimensions must be positive integers.")
    return tuple(value)


def letterbox(frame_bgr: np.ndarray, input_size: int | tuple[int, int]) -> tuple[np.ndarray, LetterboxTransform]:
    """Resize to [width,height] with centered 114-gray padding, retaining exact scale."""
    width, height = _input_size(input_size)
    original_height, original_width = frame_bgr.shape[:2]
    ratio = min(width / original_width, height / original_height)
    resized_width = max(1, min(width, round(original_width * ratio)))
    resized_height = max(1, min(height, round(original_height * ratio)))
    pad_left = (width - resized_width) // 2
    pad_top = (height - resized_height) // 2
    resized = cv2.resize(frame_bgr, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    padded = cv2.copyMakeBorder(
        resized, pad_top, height - resized_height - pad_top,
        pad_left, width - resized_width - pad_left,
        cv2.BORDER_CONSTANT, value=(114, 114, 114),
    )
    return padded, LetterboxTransform(
        original_width, original_height, resized_width / original_width,
        resized_height / original_height, pad_left, pad_top,
    )


def classwise_nms(boxes: np.ndarray, scores: np.ndarray, class_ids: np.ndarray,
                  iou_threshold: float = 0.45, max_detections: int = 100) -> list[int]:
    """Greedy NMS only suppresses overlapping boxes of the same class."""
    if not 0 <= iou_threshold <= 1 or max_detections < 1:
        raise ValueError("NMS requires IoU in [0,1] and a positive detection limit.")
    order = np.argsort(-scores, kind="stable")
    keep = []
    while order.size and len(keep) < max_detections:
        current = int(order[0])
        keep.append(current)
        remaining = order[1:]
        if not remaining.size:
            break
        top_left = np.maximum(boxes[current, :2], boxes[remaining, :2])
        bottom_right = np.minimum(boxes[current, 2:], boxes[remaining, 2:])
        intersection = np.prod(np.maximum(bottom_right - top_left, 0), axis=1)
        area_current = np.prod(np.maximum(boxes[current, 2:] - boxes[current, :2], 0))
        area_remaining = np.prod(np.maximum(boxes[remaining, 2:] - boxes[remaining, :2], 0), axis=1)
        union = area_current + area_remaining - intersection
        iou = np.divide(intersection, union, out=np.zeros_like(intersection), where=union > 0)
        suppress = (class_ids[remaining] == class_ids[current]) & (iou > iou_threshold)
        order = remaining[~suppress]
    return keep


def decode_yolo_output(output: np.ndarray, labels: list[str], transform: LetterboxTransform,
                       confidence_threshold: float = 0.35, iou_threshold: float = 0.45,
                       max_detections: int = 100) -> list[dict]:
    """Decode YOLOv8/YOLO11 raw [1,4+nc,N] (pixel xywh, class probabilities)."""
    output = np.asarray(output)
    if output.ndim != 3 or output.shape[:2] != (1, 4 + len(labels)):
        raise ValueError(
            f"Expected raw YOLO output [1,{4 + len(labels)},N], got {output.shape}. "
            "Export a YOLOv8/YOLO11 detection model with nms=False and matching labels."
        )
    if not labels or not 0 <= confidence_threshold <= 1:
        raise ValueError("YOLO needs labels and a confidence threshold in [0,1].")
    candidates = output[0].T.astype(np.float32, copy=False)
    # Invalid model output must never leak NaN/Infinity onto NetworkTables or JSON.
    candidates = candidates[np.all(np.isfinite(candidates), axis=1)]
    if not len(candidates):
        return []
    class_ids = np.argmax(candidates[:, 4:], axis=1)
    scores = candidates[np.arange(len(candidates)), 4 + class_ids]
    if np.any((scores < 0) | (scores > 1)):
        raise ValueError("YOLO class scores must be probabilities in [0,1]; logits are unsupported.")
    valid = (scores >= confidence_threshold) & (candidates[:, 2] > 0) & (candidates[:, 3] > 0)
    candidates, scores, class_ids = candidates[valid], scores[valid], class_ids[valid]
    if not len(candidates):
        return []
    # Bound CPU NMS work for an unexpectedly noisy model at startup.
    order = np.argsort(-scores, kind="stable")[:3000]
    candidates, scores, class_ids = candidates[order], scores[order], class_ids[order]
    boxes = np.concatenate((candidates[:, :2] - candidates[:, 2:4] / 2,
                            candidates[:, :2] + candidates[:, 2:4] / 2), axis=1)
    boxes = transform.restore_boxes(boxes)
    valid = (boxes[:, 2] > boxes[:, 0]) & (boxes[:, 3] > boxes[:, 1])
    boxes, scores, class_ids = boxes[valid], scores[valid], class_ids[valid]
    keep = classwise_nms(boxes, scores, class_ids, iou_threshold, max_detections)
    return [
        _detection(int(class_ids[i]), labels[int(class_ids[i])], float(scores[i]), boxes[i],
                   transform.original_width, transform.original_height)
        for i in keep
    ]


def _detection(class_id: int, label: str, confidence: float, box,
               width: int, height: int) -> dict:
    x1, y1, x2, y2 = (float(value) for value in box)
    return {
        "class_id": int(class_id), "label": label, "confidence": float(confidence),
        "bbox_xyxy": [x1, y1, x2, y2], "center": [(x1 + x2) / 2, (y1 + y2) / 2],
        "area_fraction": float((x2 - x1) * (y2 - y1) / (width * height)),
    }


class ObjectPipeline:
    """Configured shape/color baseline or trained object detector.

    Contour confidence is circularity; HSV confidence is color-pixel occupancy.
    Neither heuristic is a learned probability. Bearings are camera-relative
    degrees: yaw positive right, pitch positive up.
    """

    def __init__(self, config: dict, calibration: dict | None = None) -> None:
        self.config = dict(config)
        self.backend = config.get("backend", "contour")
        self._model = None
        self._calibration = None
        self.max_detections = int(config.get("max_detections", 100))
        if self.max_detections < 1:
            raise ValueError("max_detections must be positive.")
        if calibration is not None:
            try:
                matrix = np.asarray(calibration["camera_matrix"], dtype=np.float64)
                distortion = np.asarray(calibration.get("dist_coeffs", []), dtype=np.float64).reshape(-1)
                size = (int(calibration["width"]), int(calibration["height"]))
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError("Object bearings require calibration camera_matrix, width, height, and optional dist_coeffs.") from exc
            if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)) or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
                raise ValueError("Calibration camera_matrix must be finite 3x3 with positive focal lengths.")
            if distortion.size not in (0, 4, 5, 8, 12, 14) or not np.all(np.isfinite(distortion)):
                raise ValueError("dist_coeffs must contain 0, 4, 5, 8, 12, or 14 finite values.")
            if min(size) <= 0:
                raise ValueError("Calibration width and height must be positive.")
            self._calibration = (matrix, distortion if distortion.size else None, size)
        if self.backend in ("contour", "hsv"):
            self.label = str(config.get("label", "ball_candidate" if self.backend == "contour" else "game_piece_color"))
            self.min_area_px = float(config.get("min_area_px", 150))
            if not math.isfinite(self.min_area_px) or self.min_area_px <= 0:
                raise ValueError("min_area_px must be finite and positive.")
            kernel_size = int(config.get("morphology_kernel", 3))
            if kernel_size < 0 or (kernel_size > 0 and kernel_size % 2 == 0):
                raise ValueError("morphology_kernel must be zero or a positive odd integer.")
            self._kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8) if kernel_size else None
            if self.backend == "hsv":
                self.hsv_lower = self._hsv_bound(config.get("hsv_lower", [5, 100, 80]))
                self.hsv_upper = self._hsv_bound(config.get("hsv_upper", [30, 255, 255]))
                if np.any(self.hsv_lower[1:] > self.hsv_upper[1:]):
                    raise ValueError("HSV saturation/value lower bounds must not exceed upper bounds.")
            else:
                self.threshold = int(config.get("threshold", 180))
                self.polarity = config.get("polarity", "bright")
                self.max_area_fraction = float(config.get("max_area_fraction", 0.25))
                self.min_circularity = float(config.get("min_circularity", 0.55))
                if not 0 <= self.threshold <= 255:
                    raise ValueError("contour threshold must be in [0,255].")
                if self.polarity not in ("bright", "dark"):
                    raise ValueError("contour polarity must be bright or dark.")
                if not 0 < self.max_area_fraction <= 1 or not 0 <= self.min_circularity <= 1:
                    raise ValueError("max_area_fraction must be in (0,1]; min_circularity must be in [0,1].")
        elif self.backend in ("tensorrt", "opencv_onnx"):
            self.labels = config.get("labels", [])
            if not isinstance(self.labels, list) or not self.labels or any(not isinstance(label, str) or not label for label in self.labels):
                raise ValueError("Neural object detection requires a nonempty labels list in training class order.")
            self.input_size = _input_size(config.get("input_size", 640))
            self.confidence_threshold = float(config.get("confidence_threshold", 0.35))
            self.iou_threshold = float(config.get("iou_threshold", 0.45))
            if not 0 <= self.confidence_threshold <= 1 or not 0 <= self.iou_threshold <= 1:
                raise ValueError("confidence_threshold and iou_threshold must be in [0,1].")
            model_path = config.get("model_path")
            if not isinstance(model_path, str) or not model_path.strip():
                raise ValueError(f"The {self.backend} backend requires model_path for a trained detector.")
            path = Path(model_path).expanduser()
            if not path.is_file():
                raise FileNotFoundError(f"Object detection model does not exist: {path}")
            if self.backend == "tensorrt":
                from .tensorrt_backend import TensorRTEngine

                self._model = TensorRTEngine(path, len(self.labels))
                expected = (1, 3, self.input_size[1], self.input_size[0])
                if self._model.input_shape != expected:
                    actual = self._model.input_shape
                    self._model.close()
                    raise ValueError(f"Configured input_size requires engine input {expected}; got {actual}.")
            else:
                self._model = cv2.dnn.readNetFromONNX(str(path))
                self._model.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                self._model.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        else:
            raise ValueError(f"Unsupported object backend {self.backend!r}; use contour, hsv, tensorrt, or opencv_onnx.")

    @staticmethod
    def _hsv_bound(value) -> np.ndarray:
        values = np.asarray(value)
        if values.shape != (3,) or not np.issubdtype(values.dtype, np.number):
            raise ValueError("HSV bounds must be [hue, saturation, value] integers.")
        if not np.all(np.isfinite(values)) or np.any(values != np.floor(values)) or np.any(values < 0) or np.any(values > [179, 255, 255]):
            raise ValueError("HSV bounds use OpenCV hue 0..179 and saturation/value 0..255.")
        return values.astype(np.uint8)

    def process(self, frame_bgr: np.ndarray) -> list[dict]:
        if not isinstance(frame_bgr, np.ndarray) or frame_bgr.dtype != np.uint8 or frame_bgr.ndim not in (2, 3) or min(frame_bgr.shape[:2]) <= 0:
            raise ValueError("ObjectPipeline.process requires a nonempty uint8 grayscale [H,W] or BGR [H,W,3] image.")
        if frame_bgr.ndim == 2:
            frame_bgr = cv2.cvtColor(frame_bgr, cv2.COLOR_GRAY2BGR)
        if frame_bgr.shape[2] != 3:
            raise ValueError("BGR input must have exactly three channels.")
        height, width = frame_bgr.shape[:2]
        if self._calibration is not None and self._calibration[2] != (width, height):
            raise ValueError(f"Frame is {width}x{height}, but object calibration is {self._calibration[2]}; recalibrate at capture resolution.")
        if self.backend == "contour":
            detections = self._process_contour(frame_bgr)
        elif self.backend == "hsv":
            detections = self._process_hsv(frame_bgr)
        else:
            padded, transform = letterbox(frame_bgr, self.input_size)
            tensor = np.ascontiguousarray(padded[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255.0
            if self.backend == "tensorrt":
                output = self._model.infer(tensor)
            else:
                self._model.setInput(tensor)
                output = self._model.forward()
            detections = decode_yolo_output(output, self.labels, transform, self.confidence_threshold, self.iou_threshold, self.max_detections)
        if self._calibration is not None and detections:
            matrix, distortion, _size = self._calibration
            centers = np.asarray([detection["center"] for detection in detections], dtype=np.float64).reshape(-1, 1, 2)
            rays = cv2.undistortPoints(centers, matrix, distortion).reshape(-1, 2)
            for detection, (x, y) in zip(detections, rays):
                detection["yaw_deg"] = float(math.degrees(math.atan(x)))
                detection["pitch_deg"] = float(math.degrees(math.atan(-y)))
        return detections

    def _process_contour(self, frame_bgr: np.ndarray) -> list[dict]:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        mode = cv2.THRESH_BINARY if self.polarity == "bright" else cv2.THRESH_BINARY_INV
        _threshold, mask = cv2.threshold(gray, self.threshold, 255, mode)
        if self._kernel is not None:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        contours, _hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        height, width = gray.shape
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area_px or area / (width * height) > self.max_area_fraction:
                continue
            perimeter = cv2.arcLength(contour, True)
            circularity = min(1.0, 4 * math.pi * area / (perimeter * perimeter)) if perimeter > 0 else 0.0
            if circularity < self.min_circularity:
                continue
            x, y, box_width, box_height = cv2.boundingRect(contour)
            result = _detection(0, self.label, circularity,
                                [x, y, x + box_width, y + box_height], width, height)
            result["confidence_kind"] = "shape_circularity"
            detections.append(result)
        detections.sort(key=lambda detection: detection["area_fraction"], reverse=True)
        return detections[:self.max_detections]

    def _process_hsv(self, frame_bgr: np.ndarray) -> list[dict]:
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        if self.hsv_lower[0] <= self.hsv_upper[0]:
            mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        else:
            # A lower hue above upper hue wraps around red, e.g. 170..10.
            upper_high = self.hsv_upper.copy()
            upper_high[0] = 179
            lower_low = self.hsv_lower.copy()
            lower_low[0] = 0
            mask = cv2.bitwise_or(cv2.inRange(hsv, self.hsv_lower, upper_high), cv2.inRange(hsv, lower_low, self.hsv_upper))
        if self._kernel is not None:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        _count, _components, stats, _centers = cv2.connectedComponentsWithStats(mask, connectivity=8)
        height, width = frame_bgr.shape[:2]
        detections = []
        for x, y, box_width, box_height, area in stats[1:]:
            if area >= self.min_area_px:
                result = _detection(0, self.label, area / (box_width * box_height),
                                    [x, y, x + box_width, y + box_height], width, height)
                result["confidence_kind"] = "color_fill_fraction"
                detections.append(result)
        detections.sort(key=lambda detection: detection["area_fraction"], reverse=True)
        return detections[:self.max_detections]

    def close(self) -> None:
        if self.backend == "tensorrt" and self._model is not None:
            self._model.close()
