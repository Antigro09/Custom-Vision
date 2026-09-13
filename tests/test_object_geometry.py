import json
import math
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import pytest

from custom_vision.localization import CV_TO_NWU, _rpy_rotation
from custom_vision.object_geometry import ObjectGeometry, validate_geometry_settings


CALIBRATION = {"width": 1280, "height": 800,
               "camera_matrix": [[800., 0., 640.], [0., 800., 400.], [0., 0., 1.]],
               "dist_coeffs": [0., 0., 0., 0., 0.]}
MOUNT = {"translation_m": [0., 0., 1.], "rotation_rpy_deg": [0., 45., 0.]}
SHAPE = (800, 1280, 3)


def pipeline(settings=None, calibration=CALIBRATION, mount=MOUNT):
    return ObjectGeometry({"target_height_m": 0., **(settings or {})}, calibration, mount)


def detection(pixel=(640., 400.), class_id=0, label="piece"):
    x, y = pixel
    return {"class_id": class_id, "label": label, "confidence": .9,
            "bbox_xyxy": [x - 12., y - 12., x + 12., y + 12.]}


def projected(point, mount=MOUNT, calibration=CALIBRATION, **kwargs):
    optical = CV_TO_NWU.T @ _rpy_rotation(np.asarray(mount["rotation_rpy_deg"])).T @ (
        np.asarray(point, float) - np.asarray(mount["translation_m"]))
    assert optical[2] > 0
    pixel = cv2.projectPoints(optical.reshape(1, 3), np.zeros(3), np.zeros(3),
                              np.asarray(calibration["camera_matrix"]),
                              np.asarray(calibration["dist_coeffs"]))[0].reshape(2)
    return detection(pixel, **kwargs)


def only_target(result):
    assert result["objects"]["valid"], result
    assert len(result["objects"]["targets"]) == 1
    return result["objects"]["targets"][0]


def test_measured_ray_intersection_uses_robot_mount_and_height():
    mount = {"translation_m": [.3, -.2, 1.1], "rotation_rpy_deg": [3., 37., 12.]}
    expected = [1.4, .15, .08]
    pipe = pipeline({"target_height_m": .08}, mount=mount)
    original = projected(expected, mount)
    target = only_target(pipe.enrich([original], SHAPE, 23.456789))
    assert target["translation_m"] == pytest.approx(expected, abs=1e-8)
    assert target["range_xy_m"] == pytest.approx(math.hypot(*expected[:2]))
    assert target["bearing_deg"] == pytest.approx(math.degrees(math.atan2(.15, 1.4)))
    assert target["capture_monotonic_us"] == 23456789
    assert target["frame"] == "robot_relative_at_capture_wpilib_nwu"
    assert target["approximate"] and target["observed"] and not target["predicted"]
    assert "robot_relative" not in original
    assert target["detection_index"] == 0
    json.dumps(target, allow_nan=False)


def test_positive_pitch_looks_down_and_left_bearing_is_positive():
    pipe = pipeline()
    result = pipe.enrich([detection(), detection((560., 400.))], SHAPE, 1.)
    center, left = result["objects"]["targets"]
    assert center["translation_m"] == pytest.approx([1., 0., 0.])
    assert left["translation_m"][1] > 0 and left["bearing_deg"] > 0


def test_rear_camera_produces_negative_robot_x_not_invalid_camera_depth():
    mount = {"translation_m": [-.2, 0., 1.], "rotation_rpy_deg": [0., 45., 180.]}
    target = only_target(pipeline(mount=mount).enrich([detection()], SHAPE, 1.))
    assert target["translation_m"] == pytest.approx([-1.2, 0., 0.], abs=1e-8)
    assert abs(target["bearing_deg"]) == pytest.approx(180.)


def test_distorted_image_ray_round_trip():
    calibration = dict(CALIBRATION, dist_coeffs=[.18, -.03, .002, -.001, .002])
    point = [1.3, .4, 0.]
    target = only_target(pipeline(calibration=calibration).enrich(
        [projected(point, calibration=calibration)], SHAPE, 1.))
    assert target["translation_m"] == pytest.approx(point, abs=2e-6)


@pytest.mark.parametrize("calibration,mount,settings,shape,reason", [
    (None, MOUNT, {}, SHAPE, "missing_calibration"),
    (CALIBRATION, None, {}, SHAPE, "missing_measured_camera_mount"),
    (CALIBRATION, MOUNT, {"target_height_m": None}, SHAPE, "missing_target_height"),
    (CALIBRATION, MOUNT, {}, (400, 640), "calibration_resolution_mismatch"),
    (CALIBRATION, MOUNT, {"target_height_m": 1.}, SHAPE, "camera_not_above_target_plane"),
])
def test_missing_real_inputs_never_substitutes_metric_pose(calibration, mount, settings, shape, reason):
    result = pipeline(settings, calibration, mount).enrich([detection()], shape, 1.)
    assert not result["objects"]["valid"]
    assert result["objects"]["invalid_reason"] == reason
    assert result["objects"]["targets"] == []
    assert result["objects"]["selected_target"] is None
    assert result["detections"][0]["bbox_xyxy"] == detection()["bbox_xyxy"]
    assert result["detections"][0]["robot_relative"] == {"valid": False, "invalid_reason": reason}


@pytest.mark.parametrize("pixel", [(640., 130.), (640., 80.)])
def test_near_horizon_and_upward_rays_rejected(pixel):
    mount = {"translation_m": [0., 0., 1.], "rotation_rpy_deg": [0., 20., 0.]}
    result = pipeline(mount=mount).enrich([detection(pixel)], SHAPE, 1.)
    assert result["detections"][0]["robot_relative"]["invalid_reason"] == "ray_near_or_above_horizon"


@pytest.mark.parametrize("settings", [{"max_range_m": .9}, {"min_range_m": 1.1}])
def test_configured_range_limits(settings):
    result = pipeline(settings).enrich([detection()], SHAPE, 1.)
    assert result["detections"][0]["robot_relative"]["invalid_reason"] == "outside_validated_range"


def test_uncertainty_increases_with_pitch_error_and_range():
    tight = pipeline({"pitch_std_deg": .05})
    broad = pipeline({"pitch_std_deg": 2., "max_position_std_m": 2.})
    one = only_target(tight.enrich([projected([1., 0., 0.])], SHAPE, 1.))
    two = only_target(broad.enrich([projected([1., 0., 0.])], SHAPE, 1.))
    far = only_target(broad.enrich([projected([2., 0., 0.])], SHAPE, 1.1))
    assert one["uncertainty"]["max_position_std_m"] < two["uncertainty"]["max_position_std_m"]
    assert two["uncertainty"]["max_position_std_m"] < far["uncertainty"]["max_position_std_m"]
    assert np.linalg.eigvalsh(np.asarray(far["uncertainty"]["covariance_xy_m2"])).min() >= 0


def test_high_position_uncertainty_rejected():
    result = pipeline({"pitch_std_deg": 4., "max_position_std_m": .05}).enrich([detection()], SHAPE, 1.)
    assert result["detections"][0]["robot_relative"]["invalid_reason"] == "position_uncertainty_too_large"


def test_bbox_bottom_is_explicit_approximate_anchor():
    target = only_target(pipeline({"anchor": "bbox_bottom"}).enrich([detection()], SHAPE, 1.))
    assert target["anchor_px"] == [640., 412.]
    assert target["translation_m"][0] < 1.
    assert target["anchor"] == "bbox_bottom" and target["approximate"]


@pytest.mark.parametrize("anchor,key", [("mask_centroid", "centroid_px"), ("mask_bottom", "bottom_px")])
def test_segmentation_anchor_uses_original_capture_coordinates(anchor, key):
    det = detection()
    det["segmentation"] = {key: [645., 405.], "approximate": True}
    target = only_target(pipeline({"anchor": anchor}).enrich([det], SHAPE, 1.))
    assert target["anchor_px"] == [645., 405.]
    assert target["translation_m"][1] < 0


def test_segmentation_anchor_never_silently_falls_back_to_box():
    result = pipeline({"anchor": "mask_centroid"}).enrich([detection()], SHAPE, 1.)
    assert result["detections"][0]["robot_relative"]["invalid_reason"] == "missing_segmentation_anchor"


@pytest.mark.parametrize("box,reason", [
    ([0, 20, 100, 80], "clipped_detection"),
    ([-1, 20, 100, 80], "bounding_box_outside_image"),
    ([40, 20, 10, 80], "invalid_bounding_box"),
    ([1, 2, 3], "invalid_bounding_box"),
])
def test_invalid_or_clipped_boxes_do_not_range(box, reason):
    result = pipeline().enrich([{**detection(), "bbox_xyxy": box}], SHAPE, 1.)
    assert result["detections"][0]["robot_relative"]["invalid_reason"] == reason


def test_track_ids_survive_order_changes_and_never_cross_classes():
    pipe = pipeline()
    a = projected([1., -.2, 0.])
    b = projected([1., .2, 0.])
    first = pipe.enrich([a, b], SHAPE, 1.)["objects"]["targets"]
    second = pipe.enrich([b, a], SHAPE, 1.04)["objects"]["targets"]
    assert second[0]["track_id"] == first[1]["track_id"]
    assert second[1]["track_id"] == first[0]["track_id"]
    third = only_target(pipe.enrich([{**a, "class_id": 1}], SHAPE, 1.08))
    assert third["track_id"] not in [t["track_id"] for t in first]
    assert second[0]["track_observations"] == 2


def test_same_class_far_jump_and_expired_tracks_get_new_ids():
    pipe = pipeline({"tracking_gate_m": .15})
    first = only_target(pipe.enrich([projected([1., 0., 0.])], SHAPE, 1.))
    jump = only_target(pipe.enrich([projected([1., .3, 0.])], SHAPE, 1.03))
    assert first["track_id"] != jump["track_id"]
    expired = only_target(pipe.enrich([projected([1., .3, 0.])], SHAPE, 1.5))
    assert expired["track_id"] != jump["track_id"]


def test_missed_frames_publish_no_prediction_or_selected_target():
    pipe = pipeline()
    before = only_target(pipe.enrich([detection()], SHAPE, 1.))
    missing = pipe.enrich([], SHAPE, 1.03)["objects"]
    assert not missing["valid"] and missing["selected_target"] is None and missing["targets"] == []
    after = only_target(pipe.enrich([detection()], SHAPE, 1.06))
    assert after["track_id"] == before["track_id"]
    assert "velocity_mps" not in after


def test_reset_drops_tracks_without_reusing_ids():
    pipe = pipeline()
    first = only_target(pipe.enrich([detection()], SHAPE, 1.))
    pipe.reset()
    second = only_target(pipe.enrich([detection()], SHAPE, 1.01))
    assert second["track_id"] > first["track_id"]
    assert second["track_observations"] == 1


@pytest.mark.parametrize("stamp", [1., .9])
def test_duplicate_or_backward_timestamp_cannot_look_fresh(stamp):
    pipe = pipeline()
    pipe.enrich([detection()], SHAPE, 1.)
    stale = pipe.enrich([detection()], SHAPE, stamp)["objects"]
    assert stale["invalid_reason"] == "non_monotonic_capture_timestamp"
    assert not stale["valid"] and stale["selected_target"] is None
    later = only_target(pipe.enrich([detection()], SHAPE, 1.01))
    assert later["track_observations"] == 1


def test_selection_has_hysteresis_but_only_among_current_observations():
    pipe = pipeline({"tracking_gate_m": .6, "selection_hysteresis_m": .2})
    a = projected([1., -.1, 0.])
    b = projected([1.1, .15, 0.])
    first = pipe.enrich([a, b], SHAPE, 1.)["objects"]
    first_id = first["selected_track_id"]
    close = pipe.enrich([projected([1.08, -.1, 0.]), projected([1.01, .15, 0.])], SHAPE, 1.03)["objects"]
    assert close["selected_track_id"] == first_id
    farther = pipe.enrich([projected([1.4, -.1, 0.]), projected([1.01, .15, 0.])], SHAPE, 1.06)["objects"]
    assert farther["selected_track_id"] != first_id
    missing = pipe.enrich([projected([1.4, -.1, 0.])], SHAPE, 1.09)["objects"]
    assert missing["selected_track_id"] == first_id


def test_intake_offset_approach_preserves_capture_heading_without_motion_command():
    pipe = pipeline({"intake_offset_m": [.4, .2], "approach_standoff_m": .1})
    target = only_target(pipe.enrich([projected([1.4, .2, 0.])], SHAPE, 1.))
    assert target["range_from_intake_m"] == pytest.approx(1.)
    assert target["approach"]["translation_m"] == pytest.approx([.9, 0., 0.])
    assert target["approach"]["rotation_yaw_deg"] == 0.
    assert not target["approach"]["path_validated"]
    near = only_target(pipe.enrich([projected([.45, .2, 0.])], SHAPE, 1.03))
    assert near["approach"]["translation_m"] == [0., 0., 0.]


def test_geometry_work_bounded_by_configured_target_limit():
    result = pipeline({"max_targets": 1}).enrich([detection(), detection((600., 400.))], SHAPE, 1.)
    assert len(result["objects"]["targets"]) == 1
    assert result["detections"][1]["robot_relative"]["invalid_reason"] == "target_limit_exceeded"


@pytest.mark.parametrize("settings", [
    {"target_height_m": "0.1"}, {"target_height_m": -3.01}, {"target_height_m": float("nan")},
    {"max_range_m": float("inf")}, {"min_range_m": 3, "max_range_m": 1},
    {"anchor": "circle"}, {"pixel_std_px": 0}, {"tracking_ttl_s": 30},
    {"intake_offset_m": [1]}, {"intake_offset_m": [True, 0]},
    {"reject_clipped_boxes": 1}, {"max_targets": True}, {"max_targets": 999}, {"taget_height_m": .1},
])
def test_configuration_validation_rejects_unsafe_or_misspelled_settings(settings):
    with pytest.raises(ValueError, match="geometry"):
        validate_geometry_settings(settings)


def test_concurrent_reset_and_enrich_remain_coherent():
    pipe = pipeline()
    with ThreadPoolExecutor(max_workers=2) as pool:
        reset = pool.submit(lambda: [pipe.reset() for _ in range(100)])
        results = [pipe.enrich([detection()], SHAPE, 1. + index * .01) for index in range(100)]
        reset.result()
    assert all(result["objects"]["valid"] for result in results)
    assert all(result["objects"]["targets"][0]["capture_monotonic_us"] == result["objects"]["capture_monotonic_us"] for result in results)


def test_tracking_history_has_fixed_memory_bound_under_new_ids():
    pipe = pipeline({"max_targets": 1, "tracking_ttl_s": .5})
    for index in range(20):
        pipe.enrich([detection(class_id=index)], SHAPE, 1. + index * .001)
        assert len(pipe._tracks) <= 2


def test_unknown_class_identity_is_not_implicitly_one_shared_track_class():
    det = detection()
    del det["class_id"]
    result = pipeline().enrich([det], SHAPE, 1.)
    assert result["detections"][0]["robot_relative"]["invalid_reason"] == "missing_or_invalid_class_identity"


def test_target_plane_can_be_below_robot_coordinate_origin():
    mount = {"translation_m": [.2, .1, .8], "rotation_rpy_deg": [0., 45., 0.]}
    # Robot CAD origin is 0.20m above the floor; a 0.05m-radius ball is at z=-0.15.
    expected = [1.15, .1, -.15]
    target = only_target(pipeline({"target_height_m": -.15}, mount=mount).enrich(
        [projected(expected, mount)], SHAPE, 1.))
    assert target["translation_m"] == pytest.approx(expected, abs=1e-8)
    assert target["target_height_m"] == -.15


def test_origin_has_no_defined_bearing_even_with_zero_configured_minimum_range():
    mount = {"translation_m": [0., 0., 1.], "rotation_rpy_deg": [0., 90., 0.]}
    result = pipeline({"min_range_m": 0.}, mount=mount).enrich([detection()], SHAPE, 1.)
    assert not result["objects"]["valid"]
    assert result["objects"]["selected_target"] is None
    assert result["detections"][0]["robot_relative"] == {
        "valid": False, "invalid_reason": "bearing_undefined_at_robot_origin"}
