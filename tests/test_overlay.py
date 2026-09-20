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


def test_box_depth_ratio_changes_only_projected_depth(monkeypatch):
    calibration, detection = fixture()
    frame = np.zeros((480, 640, 3), np.uint8)
    projections = []
    project_points = cv2.projectPoints

    def record_projection(points, *args, **kwargs):
        projections.append(points.copy())
        return project_points(points, *args, **kwargs)

    monkeypatch.setattr(cv2, "projectPoints", record_projection)
    short = annotate(frame, [detection], calibration, box_depth_ratio=.5)
    cube = annotate(frame, [detection], calibration, box_depth_ratio=1.)
    assert len(projections) == 2
    np.testing.assert_array_equal(projections[0][:4], projections[1][:4])
    np.testing.assert_allclose(projections[0][4:8, 2], -.1651 / 2)
    np.testing.assert_allclose(projections[1][4:8, 2], -.1651)
    np.testing.assert_allclose(np.ptp(projections[1][:8], axis=0), [.1651] * 3)
    assert not np.array_equal(short, cube)
    assert not frame.any()


def test_poi_overlay_uses_raw_pixels_and_distinguishes_preview_only(monkeypatch):
    calibration, _ = fixture()
    frame = np.zeros((480, 640, 3), np.uint8)
    labels = []
    monkeypatch.setattr(cv2, "putText", lambda _frame, text, *_args, **_kwargs: labels.append(text))
    poi = {"valid": True, "targets": [
        {"name": "aim", "pixel": [200, 180], "valid": True, "geometry_valid": True, "in_image": True},
        {"name": "uncertain", "pixel": [400, 280], "valid": False, "geometry_valid": True, "in_image": True},
    ]}
    result = annotate(frame, [], calibration, poi=poi)
    np.testing.assert_array_equal(result[180, 200], [200, 80, 255])
    np.testing.assert_array_equal(result[280, 400], [0, 180, 255])
    assert labels == ["POI aim", "POI uncertain PREVIEW ONLY"]
    assert not frame.any()


def test_poi_overlay_rejects_unprojectable_or_mismatched_geometry():
    calibration, _ = fixture()
    frame = np.zeros((480, 640, 3), np.uint8)
    entries = [
        {"pixel": [200, 180], "geometry_valid": False, "in_image": True},
        {"pixel": [200, 180], "geometry_valid": True, "in_image": False},
        {"pixel": [float("nan"), 180], "geometry_valid": True, "in_image": True},
        {"pixel": [200, 180, 0], "geometry_valid": True, "in_image": True},
        {"pixel": [-1, 180], "geometry_valid": True, "in_image": True},
    ]
    assert np.array_equal(annotate(frame, [], calibration, poi={"targets": entries}), frame)
    valid = {"targets": [{"pixel": [200, 180], "geometry_valid": True, "in_image": True}]}
    assert np.array_equal(annotate(frame, [], poi=valid), frame)
    calibration["width"] = 800
    assert np.array_equal(annotate(frame, [], calibration, poi=valid), frame)
