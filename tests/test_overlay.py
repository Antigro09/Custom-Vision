import cv2
import numpy as np

from custom_vision.localization import estimate_tag_pose
from custom_vision.overlay import annotate


def fixture():
    calibration = {"width": 640, "height": 480,
                   "camera_matrix": [[600., 0., 320.], [0., 600., 240.], [0., 0., 1.]],
                   "dist_coeffs": [0.] * 5}
    h = .1651 / 2
    points = np.array([[-h, h, 0.], [h, h, 0.], [h, -h, 0.], [-h, -h, 0.]])
    corners = cv2.projectPoints(points, np.array([.1, -.2, 2.9]), np.array([.08, .02, 1.]),
                                np.array(calibration["camera_matrix"]), np.zeros(5))[0].reshape(4, 2)
    detection = {"id": 7, "corners": corners.tolist(), "center": corners.mean(axis=0).tolist()}
    detection.update(estimate_tag_pose(corners, .1651, calibration["camera_matrix"], calibration["dist_coeffs"]))
    return calibration, detection


def test_overlay_projects_3d_box_in_raw_pixels_without_mutating_frame():
    calibration, detection = fixture()
    frame = np.zeros((480, 640, 3), np.uint8)
    annotated = annotate(frame, [detection], calibration)
    two_d = annotate(frame, [detection])
    assert not frame.any()
    assert np.count_nonzero(annotated) > np.count_nonzero(two_d)
    assert np.count_nonzero(annotated != two_d) > 100
    # Top box corners extend from tag toward camera (negative raw +Z).
    h = .1651 / 2
    top = np.array([[-h, h, -h], [h, h, -h], [h, -h, -h], [-h, -h, -h]])
    projected = cv2.projectPoints(top, np.array(detection["rvec_rad"]), np.array(detection["tvec_m"]),
                                  np.array(calibration["camera_matrix"]), np.zeros(5))[0].reshape(4, 2)
    for point in projected:
        x, y = np.rint(point).astype(int)
        assert annotated[y-2:y+3, x-2:x+3].any()


def test_mismatched_calibration_draws_only_2d():
    calibration, detection = fixture()
    calibration["width"] = 800
    frame = np.zeros((480, 640, 3), np.uint8)
    assert np.array_equal(annotate(frame, [detection], calibration), annotate(frame, [detection]))


def test_grayscale_preview_converts_without_changing_source():
    _, detection = fixture()
    frame = np.zeros((480, 640), np.uint8)
    result = annotate(frame, [detection])
    assert result.shape == (480, 640, 3)
    assert not frame.any()


def object_fixture():
    target = {"valid": True, "detection_index": 0, "track_id": 12,
              "translation_m": [1.5, .2, .1], "range_xy_m": 1.51}
    detection = {"label": "piece", "bbox_xyxy": [100, 120, 200, 230],
                 "segmentation": {"contour_px": [[120, 145], [180, 145], [155, 205]],
                                  "approximate": True}, "robot_relative": target}
    return detection, {"valid": True, "targets": [target], "selected_track_id": 12,
                       "selected_target": target}


def test_object_overlay_box_contour_selection_and_raw_pixels(monkeypatch):
    detection, objects = object_fixture()
    frame = np.zeros((300, 400), np.uint8)
    labels = []
    real_put_text = cv2.putText

    def put_text(image, text, *args, **kwargs):
        labels.append(text)
        return real_put_text(image, text, *args, **kwargs)

    monkeypatch.setattr(cv2, "putText", put_text)
    result = annotate(frame, [detection], objects=objects)
    assert not frame.any()
    assert result.shape == (300, 400, 3)
    assert result[120, 150].any()  # Bounding box edge in raw image coordinates.
    assert result[145, 150].any()  # Segmentation edge, distinct from the box.
    assert labels == ["piece #12 1.51m selected"]
    assert detection["bbox_xyxy"] == [100, 120, 200, 230]


def test_object_overlay_suppresses_invalid_range_and_selection(monkeypatch):
    detection, objects = object_fixture()
    objects["valid"] = False
    labels = []
    monkeypatch.setattr(cv2, "putText", lambda _frame, text, *_args, **_kwargs: labels.append(text))
    annotate(np.zeros((300, 400, 3), np.uint8), [detection], objects=objects)
    assert labels == ["piece #12"]


def test_object_overlay_works_without_geometry_and_ignores_invalid_boxes():
    frame = np.zeros((300, 400, 3), np.uint8)
    detection = {"label": "candidate", "bbox_xyxy": [100, 120, 200, 230]}
    result = annotate(frame, [detection])
    assert result.any()
    for invalid in ([1, 2, float("nan"), 4], [1, 2, 0, 4], [1, 2, 3], None):
        assert np.array_equal(annotate(frame, [{"bbox_xyxy": invalid}]), frame)


def test_object_overlay_preserves_apriltag_projection():
    calibration, tag = fixture()
    detection, objects = object_fixture()
    frame = np.zeros((480, 640, 3), np.uint8)
    tags_only = annotate(frame, [tag], calibration)
    mixed = annotate(frame, [tag, detection], calibration, objects=objects)
    # Every drawn tag pixel is unchanged by the separate object annotation.
    tag_pixels = tags_only.any(axis=2)
    assert np.array_equal(tags_only[tag_pixels], mixed[tag_pixels])
    assert np.count_nonzero(mixed) > np.count_nonzero(tags_only)
