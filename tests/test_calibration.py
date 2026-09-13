"""Calibrate actual synthetic chessboard images with known lens intrinsics."""

import copy
import json

import cv2
import numpy as np
import pytest

from custom_vision.calibration import calibrate_images, load_calibration, main, validate_calibration


@pytest.fixture
def valid_calibration():
    return {"width": 640, "height": 480,
            "camera_matrix": [[600, 0, 320], [0, 600, 240], [0, 0, 1]],
            "dist_coeffs": [0, 0, 0, 0, 0]}


@pytest.fixture(scope="module")
def chessboard_images(tmp_path_factory):
    directory = tmp_path_factory.mktemp("chessboards")
    paths = []
    # Render outer squares; inner intersections are [0..6, 0..4] * 25mm.
    scale = 3
    matrix = np.array([[600 * scale, 0, 320 * scale],
                       [0, 600 * scale, 240 * scale], [0, 0, 1]], dtype=float)
    for index in range(12):
        frame = np.full((480 * scale, 640 * scale), 230, dtype=np.uint8)
        rvec = np.array([(-0.3 + 0.2 * (index % 4)), (-0.35 + 0.3 * (index // 4)),
                         (-0.12 + 0.08 * (index % 3))])
        tvec = np.array([-0.1 + 0.02 * (index % 3), -0.07 + 0.015 * (index // 3),
                         0.45 + 0.04 * (index % 4)])
        for y in range(-1, 5):
            for x in range(-1, 7):
                points = np.array([[x, y, 0], [x + 1, y, 0], [x + 1, y + 1, 0], [x, y + 1, 0]], dtype=float) * 0.025
                pixels, _ = cv2.projectPoints(points, rvec, tvec, matrix, np.zeros(5))
                cv2.fillConvexPoly(frame, np.rint(pixels.reshape(4, 2)).astype(np.int32),
                                  0 if (x + y) % 2 == 0 else 255)
        image = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)
        path = directory / f"board_{index:02d}.png"
        assert cv2.imwrite(str(path), image)
        paths.append(path)
    return paths


def test_real_chessboard_calibration_and_cli(chessboard_images, tmp_path):
    output = tmp_path / "calibration.json"
    assert main(["--images", str(chessboard_images[0].parent / "*.png"), "--board-cols", "7",
                 "--board-rows", "5", "--square-size-m", "0.025", "--output", str(output)]) == 0
    result = load_calibration(output)
    assert (result["width"], result["height"]) == (640, 480)
    assert result["accepted_views"] >= 10
    assert result["rms_error_px"] < 0.4
    matrix = np.array(result["camera_matrix"])
    assert matrix[0, 0] == pytest.approx(600, rel=0.025)
    assert matrix[1, 1] == pytest.approx(600, rel=0.025)
    assert matrix[0, 2] == pytest.approx(320, abs=10)
    assert matrix[1, 2] == pytest.approx(240, abs=10)
    assert result["distortion_model"] == "opencv_pinhole"
    json.dumps(result, allow_nan=False)


def test_mixed_resolutions_rejected(chessboard_images, tmp_path):
    smaller = tmp_path / "smaller.png"
    assert cv2.imwrite(str(smaller), np.full((240, 320), 255, dtype=np.uint8))
    with pytest.raises(ValueError, match="resolution mismatch"):
        calibrate_images([chessboard_images[0], smaller, chessboard_images[1]], 7, 5, 0.025, min_views=3)


def test_identical_views_do_not_count_as_diverse(chessboard_images, tmp_path):
    image = cv2.imread(str(chessboard_images[0]), cv2.IMREAD_GRAYSCALE)
    paths = []
    for i in range(3):
        path = tmp_path / f"copy_{i}.png"
        cv2.imwrite(str(path), image)
        paths.append(path)
    with pytest.raises(ValueError, match="Only 1 usable"):
        calibrate_images(paths, 7, 5, 0.025, min_views=3)


@pytest.mark.parametrize("field,value", [("width", None), ("width", 640.5), ("height", -1),
                                         ("camera_matrix", [[0, 0, 320], [0, 600, 240], [0, 0, 1]]),
                                         ("dist_coeffs", [0, float("nan"), 0, 0, 0]),
                                         ("dist_coeffs", [0, 0, 0]),
                                         ("distortion_model", "fisheye")])
def test_invalid_calibration_is_rejected(valid_calibration, field, value):
    calibration = copy.deepcopy(valid_calibration)
    calibration[field] = value
    with pytest.raises(ValueError):
        validate_calibration(calibration)


def test_insufficient_inputs_and_invalid_board_are_rejected():
    with pytest.raises(ValueError, match="at least 10"):
        calibrate_images([], 7, 5, 0.025)
    with pytest.raises(ValueError, match="inner corners"):
        calibrate_images([], 0, 5, 0.025)
