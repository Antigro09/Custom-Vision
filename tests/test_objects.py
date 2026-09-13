import json

import cv2
import numpy as np
import pytest

from custom_vision.objects import (
    ObjectPipeline,
    classwise_nms,
    decode_yolo_output,
    letterbox,
)


def calibration(width=320, height=240):
    return {
        "width": width, "height": height,
        "camera_matrix": [[200.0, 0, width / 2], [0, 200.0, height / 2], [0, 0, 1]],
        "dist_coeffs": [0, 0, 0, 0, 0],
    }


def test_monochrome_ball_candidate_rejects_line_and_noise():
    image = np.zeros((240, 320), np.uint8)
    cv2.circle(image, (90, 100), 25, 240, -1)
    cv2.rectangle(image, (170, 60), (300, 65), 240, -1)
    cv2.circle(image, (40, 30), 3, 255, -1)
    pipeline = ObjectPipeline({"backend": "contour", "min_area_px": 100, "min_circularity": 0.7})
    detections = pipeline.process(image)
    assert len(detections) == 1
    assert detections[0]["label"] == "ball_candidate"
    assert detections[0]["center"] == pytest.approx([90.5, 100.5], abs=1)
    assert detections[0]["confidence_kind"] == "shape_circularity"
    assert 0.7 <= detections[0]["confidence"] <= 1
    assert "yaw_deg" not in detections[0]
    json.dumps(detections, allow_nan=False)


def test_monochrome_rejects_large_round_background():
    image = np.zeros((240, 320), np.uint8)
    cv2.circle(image, (160, 120), 100, 255, -1)
    assert ObjectPipeline({"max_area_fraction": 0.1}).process(image) == []


def test_monochrome_dark_polarity_and_holes():
    image = np.full((240, 320), 255, np.uint8)
    cv2.circle(image, (100, 100), 25, 20, -1)
    cv2.circle(image, (95, 95), 5, 255, -1)
    detections = ObjectPipeline({"polarity": "dark", "threshold": 100}).process(image)
    assert len(detections) == 1
    assert detections[0]["confidence_kind"] == "shape_circularity"


def test_hsv_color_target_filters_background_and_small_noise():
    image = np.zeros((240, 320, 3), np.uint8)
    image[60:110, 200:240] = (0, 140, 255)
    image[10:12, 10:12] = (0, 140, 255)
    image[100:150, 20:80] = (255, 0, 0)
    pipeline = ObjectPipeline({"backend": "hsv", "label": "orange_piece", "min_area_px": 50})
    detections = pipeline.process(image)
    assert len(detections) == 1
    detection = detections[0]
    assert detection["bbox_xyxy"] == [200, 60, 240, 110]
    assert detection["center"] == [220, 85]
    assert detection["label"] == "orange_piece"
    assert detection["area_fraction"] == pytest.approx(2000 / 76800)
    assert detection["confidence"] == 1.0
    assert detection["confidence_kind"] == "color_fill_fraction"


def test_hsv_supports_red_hue_wraparound():
    hsv = np.zeros((100, 200, 3), np.uint8)
    hsv[20:60, 10:50] = [175, 230, 230]
    hsv[20:60, 80:120] = [5, 230, 230]
    hsv[20:60, 140:180] = [50, 230, 230]
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    pipeline = ObjectPipeline({"backend": "hsv", "hsv_lower": [170, 100, 100], "hsv_upper": [10, 255, 255]})
    assert len(pipeline.process(bgr)) == 2


def test_calibrated_bearings_positive_right_and_up():
    image = np.zeros((240, 320, 3), np.uint8)
    image[60:110, 200:240] = (0, 140, 255)
    pipeline = ObjectPipeline({"backend": "hsv"}, calibration())
    detection = pipeline.process(image)[0]
    assert detection["yaw_deg"] == pytest.approx(np.degrees(np.arctan(60 / 200)))
    assert detection["pitch_deg"] == pytest.approx(np.degrees(np.arctan(35 / 200)))


def test_calibration_undistorts_bearing():
    settings = calibration()
    settings["dist_coeffs"] = [0.3, 0, 0, 0, 0]
    image = np.zeros((240, 320, 3), np.uint8)
    image[60:110, 260:300] = (0, 140, 255)
    detection = ObjectPipeline({"backend": "hsv"}, settings).process(image)[0]
    assert 0 < detection["yaw_deg"] < np.degrees(np.arctan(120 / 200))


def test_calibration_resolution_mismatch_fails_even_without_targets():
    pipeline = ObjectPipeline({}, calibration())
    with pytest.raises(ValueError, match="recalibrate"):
        pipeline.process(np.zeros((480, 640, 3), np.uint8))


@pytest.mark.parametrize("shape,size", [((720, 1280), 640), ((481, 639), (640, 480)), ((777, 211), (320, 320))])
def test_letterbox_exact_inverse_including_rounding(shape, size):
    height, width = shape
    image = np.zeros((height, width, 3), np.uint8)
    padded, transform = letterbox(image, size)
    expected_size = (size, size) if isinstance(size, int) else size
    assert padded.shape == (expected_size[1], expected_size[0], 3)
    boxes = np.array([[10, 20, width - 20, height - 30]], dtype=np.float32)
    transformed = boxes.copy()
    transformed[:, [0, 2]] = transformed[:, [0, 2]] * transform.scale_x + transform.pad_left
    transformed[:, [1, 3]] = transformed[:, [1, 3]] * transform.scale_y + transform.pad_top
    np.testing.assert_allclose(transform.restore_boxes(transformed), boxes, atol=1e-4)
    if transform.pad_top:
        assert (padded[0] == 114).all()


def test_nms_suppresses_same_class_but_preserves_different_class():
    boxes = np.array([[0, 0, 100, 100], [5, 5, 105, 105], [0, 0, 100, 100], [200, 200, 240, 240]], np.float32)
    scores = np.array([0.9, 0.8, 0.7, 0.6], np.float32)
    class_ids = np.array([0, 0, 1, 0])
    assert classwise_nms(boxes, scores, class_ids) == [0, 2, 3]
    assert classwise_nms(boxes, scores, class_ids, max_detections=2) == [0, 2]


def test_yolo_decode_unpads_clips_and_filters_invalid_predictions():
    _, transform = letterbox(np.zeros((100, 200, 3), np.uint8), 200)
    # Columns: xywh plus scores for two classes. Original frame has 50px top padding.
    output = np.array([
        [50, 51, 50, 190, 50, 50, 50],
        [100, 101, 100, 100, 100, 100, 100],
        [40, 40, 40, 40, -2, 20, 20],
        [20, 20, 20, 40, 20, 20, 20],
        [0.9, 0.8, 0.1, 0.6, 0.9, np.nan, 0.2],
        [0.1, 0.2, 0.7, 0.1, 0.1, 0.1, 0.1],
    ], dtype=np.float32)[None]
    detections = decode_yolo_output(output, ["ball", "other"], transform)
    assert len(detections) == 3
    assert detections[0]["bbox_xyxy"] == [30, 40, 70, 60]
    assert detections[1]["label"] == "other"
    assert detections[2]["bbox_xyxy"] == [170, 30, 200, 70]
    json.dumps(detections, allow_nan=False)


@pytest.mark.parametrize("shape", [(1, 8400, 6), (1, 7, 8400), (1, 300, 6), (6, 10)])
def test_yolo_rejects_wrong_class_count_or_output_contract(shape):
    _, transform = letterbox(np.zeros((100, 200, 3), np.uint8), 200)
    with pytest.raises(ValueError, match="raw YOLO"):
        decode_yolo_output(np.zeros(shape, np.float32), ["a", "b"], transform)


def test_yolo_empty_output_and_all_padding_boxes():
    _, transform = letterbox(np.zeros((100, 200, 3), np.uint8), 200)
    assert decode_yolo_output(np.zeros((1, 5, 0), np.float32), ["ball"], transform) == []
    padding_box = np.array([[[50], [10], [20], [10], [0.9]]], np.float32)
    assert decode_yolo_output(padding_box, ["ball"], transform) == []


def test_neural_pipeline_rgb_normalization_and_letterbox(monkeypatch, tmp_path):
    class FakeNetwork:
        def setPreferableBackend(self, _backend):
            pass

        def setPreferableTarget(self, _target):
            pass

        def setInput(self, tensor):
            self.tensor = tensor

        def forward(self):
            return np.array([[[100], [100], [100], [50], [0.9]]], np.float32)

    network = FakeNetwork()
    monkeypatch.setattr(cv2.dnn, "readNetFromONNX", lambda _path: network)
    model = tmp_path / "detector.onnx"
    model.touch()
    pipeline = ObjectPipeline({"backend": "opencv_onnx", "model_path": str(model), "labels": ["ball"], "input_size": 200})
    image = np.zeros((100, 200, 3), np.uint8)
    image[:] = (0, 128, 255)
    detections = pipeline.process(image)
    assert network.tensor.shape == (1, 3, 200, 200)
    np.testing.assert_allclose(network.tensor[0, :, 100, 100], [1, 128 / 255, 0])
    np.testing.assert_allclose(network.tensor[0, :, 0, 0], [114 / 255] * 3)
    assert detections[0]["bbox_xyxy"] == [50, 25, 150, 75]


def test_missing_model_has_actionable_error(tmp_path):
    with pytest.raises(ValueError, match="model_path"):
        ObjectPipeline({"backend": "tensorrt", "labels": ["ball"]})
    with pytest.raises(FileNotFoundError, match="does not exist"):
        ObjectPipeline({"backend": "tensorrt", "labels": ["ball"], "model_path": str(tmp_path / "missing.engine")})


@pytest.mark.parametrize("config", [
    {"backend": "unknown"}, {"threshold": 300}, {"min_area_px": -1},
    {"max_area_fraction": 0}, {"min_circularity": 1.1}, {"polarity": "other"},
    {"morphology_kernel": 4}, {"backend": "hsv", "hsv_lower": [180, 0, 0]},
    {"backend": "tensorrt", "labels": []},
])
def test_invalid_configuration_rejected(config):
    with pytest.raises(ValueError):
        ObjectPipeline(config)


@pytest.mark.parametrize("frame", [np.zeros((10, 10), np.float32), np.zeros((10, 10, 4), np.uint8), np.zeros((0, 10), np.uint8)])
def test_invalid_frame_rejected(frame):
    with pytest.raises(ValueError):
        ObjectPipeline({}).process(frame)
