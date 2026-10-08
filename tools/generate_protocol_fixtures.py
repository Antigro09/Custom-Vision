#!/usr/bin/env python3
"""Camera-free, CPU-bounded producer fixtures through Publisher.publish.

Run from the checkout with its test Python. Geometry is synthetic, clocks/NT
transport are stubs, and no detector, camera, service, or hardware is started.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import cv2
import numpy as np

from custom_vision.localization import Localization
from custom_vision.object_geometry import ObjectGeometry
from custom_vision.objects import LetterboxTransform, decode_yolo_output
from custom_vision.poi import PointOfInterestTracker
from custom_vision.publisher import Publisher

PROFILE = "custom-vision-schema2-2026.1"
SHAPE = (800, 1280, 3)
CAPTURE_US = 1234567890123
PUBLISH_UNIX_US = 1791456000123456
CALIBRATION = {
    "width": 1280, "height": 800,
    "camera_matrix": [[870., 0., 640.], [0., 870., 400.], [0., 0., 1.]],
    "dist_coeffs": [-.13, .04, .001, -.002, 0.],
}
MOUNT = {"translation_m": [.35, -.17, .42], "rotation_rpy_deg": [5., -15., 25.]}
OBJECT_CALIBRATION = {
    "width": 1280, "height": 800,
    "camera_matrix": [[800., 0., 640.], [0., 800., 400.], [0., 0., 1.]],
    "dist_coeffs": [0.] * 5,
}
OBJECT_MOUNT = {"translation_m": [0., 0., 1.], "rotation_rpy_deg": [0., 45., 0.]}


class StubTopic:
    def publish(self, _options):
        return self

    def set(self, value):
        self.value = copy.deepcopy(value)


class StubInstance:
    def __init__(self):
        self.offset = 2000000
        self.connected = True
        self.topics = {}
        self.current_table = None

    def isConnected(self):
        return self.connected

    def getServerTimeOffset(self):
        return self.offset

    def getTable(self, path):
        self.current_table = path
        return self

    def __getattr__(self, name):
        if name.startswith("get") and name.endswith("Topic"):
            return lambda key: self.topics.setdefault((self.current_table, key), StubTopic())
        raise AttributeError(name)

    def flush(self):
        pass


class StubClock:
    """Deterministic injected clock, with explicit microsecond units."""
    def capture_server_us(self, instance, capture_monotonic_us, correction_ms=0):
        if not instance.isConnected() or instance.offset is None:
            return None
        nt_now_us = CAPTURE_US + 30000
        host_now_us = CAPTURE_US + 30000
        return nt_now_us + instance.offset - (host_now_us - capture_monotonic_us) - int(correction_ms * 1000)


def transform(xyz, rpy=(0., 0., 0.)):
    # Independent scene rotations, rather than the production basis helper.
    roll, pitch, yaw = np.deg2rad(rpy)
    rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = rz @ ry @ rx, xyz
    return result


def layout_for(tags):
    entries = []
    for tag_id, pose in tags.items():
        rvec = cv2.Rodrigues(pose[:3, :3])[0].reshape(3)
        angle = np.linalg.norm(rvec)
        q = np.r_[math.cos(angle / 2), rvec * math.sin(angle / 2) / angle] if angle else [1, 0, 0, 0]
        entries.append({"ID": tag_id, "pose": {
            "translation": dict(zip(("x", "y", "z"), pose[:3, 3].tolist())),
            "rotation": {"quaternion": dict(zip(("W", "X", "Y", "Z"), q))},
        }})
    return {"field": {"length": 16.5, "width": 8.2}, "tags": entries}


def project_scene(tags, camera, calibration):
    half = .1651 / 2
    corners = np.array([[0, half, half, 1], [0, -half, half, 1], [0, -half, -half, 1], [0, half, -half, 1]])
    detections = []
    for tag_id, tag in tags.items():
        points = (np.linalg.inv(camera) @ tag @ corners.T).T[:, :3]
        optical = np.column_stack((-points[:, 1], -points[:, 2], points[:, 0]))
        pixels = cv2.projectPoints(optical, np.zeros(3), np.zeros(3),
                                  np.asarray(calibration["camera_matrix"]), np.asarray(calibration["dist_coeffs"]))[0].reshape(4, 2)
        detections.append({"id": tag_id, "corners": pixels.tolist(), "center": pixels.mean(axis=0).tolist(),
                           "decision_margin": 100., "hamming": 0, "pose_valid": False})
    return detections


def public_revisions(calibration, mount, layout):
    # Import the runtime's one revision method; fixture identifiers are not an
    # independently invented compatibility contract.
    from custom_vision.revisions import geometry_revisions
    return geometry_revisions({"calibration_data": calibration, "robot_to_camera": mount},
                              {"field_layout_data": layout})


def packet(kind="apriltag", frame_id=42, boot_id="fixture-boot-a", calibration=CALIBRATION, mount=MOUNT, layout=None):
    return dict(
        schema_version=2, protocol_profile=PROFILE, boot_id=boot_id,
        pipeline="front_tags" if kind == "apriltag" else "front_objects", type=kind,
        mode="3d" if kind == "apriltag" else "detect", backend="pupil" if kind == "apriltag" else "contour",
        detector_device="cpu", pose_device="cpu" if kind == "apriltag" else "none", input_kind="synthetic",
        connected=True, frame_id=frame_id, capture_monotonic_us=CAPTURE_US,
        publish_unix_us=PUBLISH_UNIX_US, latency_ms=30., timestamp_source="host_frame_read_complete",
        capture_latency_offset_ms=0., detections=[], error=None, preview_settings={},
        frame_size=[1280, 800], processing_ms=0., detector_ms=0., localization_ms=0., queue_ms=0.,
        native_timings={}, fps=50., dropped_frames=0,
        **public_revisions(calibration, mount, layout),
    )


def make_cases():
    """Use implemented geometry with controlled synthetic observations."""
    camera = transform([2., 3., .7], [1., -4., 17.])
    tags = {1: transform([5., 3.3, 1.3], [0., 0., 180.]),
            2: transform([5.4, 4.4, .8], [0., 0., 200.]),
            3: transform([4.8, 2.6, .9], [0., 0., 160.])}
    layout = layout_for(tags)
    cases = []

    def add(name, payload, expected, **metadata):
        cases.append(dict(name=name, payload=payload, expected=expected, **metadata))

    def localized(name, observed, *, calibration=CALIBRATION, mount=MOUNT, field=layout, settings=None, expected=None):
        payload = packet(calibration=calibration, mount=mount, layout=field)
        payload.update(Localization(settings or {}, calibration, field, mount).enrich(observed, SHAPE))
        add(name, payload, expected or {"localization_valid": True, "field_robot_valid": mount is not None})
        return payload

    one = {1: tags[1]}
    single = localized("single_tag", project_scene(one, camera, CALIBRATION))
    localized("multitag", project_scene(tags, camera, CALIBRATION))
    localized("camera_only_localization", project_scene(tags, camera, CALIBRATION), mount=None)
    localized("missing_calibration", project_scene(one, camera, CALIBRATION), calibration=None,
              expected={"localization_valid": False, "localization_reason": "no_calibration"})
    localized("missing_field_layout", project_scene(one, camera, CALIBRATION), field=None,
              expected={"localization_valid": False, "localization_reason": "no_field_layout", "tag_geometry_valid": True})
    flat_cal = dict(CALIBRATION, dist_coeffs=[0.] * 5)
    flat_camera, flat_tags = transform([1., 2., 1.]), {7: transform([3., 2., 1.], [0., 0., 180.])}
    localized("ambiguous_tag", project_scene(flat_tags, flat_camera, flat_cal), calibration=flat_cal,
              field=layout_for(flat_tags), expected={"localization_valid": False, "localization_reason": "ambiguous_pose", "tag_geometry_valid": True})

    poi_settings = dict(enabled=True, calibration_verified=True,
                        targets=[dict(name="aim", tag_id=1, offset_m=[0., .2, .4])])
    # Single solves exist before optional field enrichment. POI itself refuses
    # field-derived tag poses; this reproduces the runtime's independent order.
    raw_singles = Localization({}, CALIBRATION).enrich(project_scene(one, camera, CALIBRATION), SHAPE)["detections"]
    for mounted in (False, True):
        payload = packet(mount=MOUNT if mounted else None)
        payload["detections"] = copy.deepcopy(raw_singles)
        payload["localization"] = Localization({}, CALIBRATION).enrich([], SHAPE)["localization"]
        payload["poi"] = PointOfInterestTracker(poi_settings, CALIBRATION, MOUNT if mounted else None).process(raw_singles, SHAPE, CAPTURE_US / 1e6)
        add("poi_with_robot" if mounted else "poi_camera_only", payload,
            {"localization_valid": False, "poi_valid": True, "poi_robot_valid": mounted})
    unverified = packet(mount=None)
    unverified["detections"] = copy.deepcopy(raw_singles)
    unverified["poi"] = PointOfInterestTracker(dict(poi_settings, calibration_verified=False), CALIBRATION).process(raw_singles, SHAPE, CAPTURE_US / 1e6)
    add("poi_unverified_calibration", unverified, {"poi_valid": False, "poi_reason": "calibration_not_verified"})

    detections = [dict(class_id=0, label="synthetic_piece", confidence=.912345678,
                       bbox_xyxy=[628., 388., 652., 412.]),
                  dict(class_id=0, label="synthetic_piece", confidence=.876543219,
                       bbox_xyxy=[548., 388., 572., 412.])]
    for name, calibration, mount, height, reason in (
        ("objects_compact_covariance_selection", OBJECT_CALIBRATION, OBJECT_MOUNT, 0., None),
        ("objects_2d", None, None, None, "missing_calibration"),
        ("objects_missing_mount", OBJECT_CALIBRATION, None, 0., "missing_measured_camera_mount"),
        ("objects_missing_target_height", OBJECT_CALIBRATION, OBJECT_MOUNT, None, "missing_target_height"),
    ):
        payload = packet("object", calibration=calibration, mount=mount)
        payload.update(ObjectGeometry({"target_height_m": height}, calibration, mount).enrich(detections, SHAPE, CAPTURE_US / 1e6))
        add(name, payload, {"objects_valid": reason is None, "objects_reason": reason})

    # Real bounded YOLO output decoding supplies absent, empty and mask-limited
    # segmentation representations without loading a detector or neural model.
    decode_transform = LetterboxTransform(32, 32, 1., 1., 0, 0, 32, 32)
    mask_output = np.array([[[0, 0, 32, 32, .9, 0, 1], [1, 1, 31, 31, .8, 0, 1]]], np.float32)
    proto = np.ones((1, 1, 8, 8), np.float32)
    for name, output, prototypes, statuses in (
        ("segment_valid_mask_limit", mask_output, proto, ["valid", "mask_limit"]),
        ("segment_empty_masks", mask_output, -proto, ["empty", "empty"]),
        ("segment_missing_masks", mask_output[:, :, :6], None, [None, None]),
    ):
        decoded = decode_yolo_output(output, ["synthetic_piece"], decode_transform,
                                     output_format="yolo26_end2end", prototypes=prototypes,
                                     max_masks=2 if name == "segment_empty_masks" else 1)
        payload = packet("object", calibration=None, mount=None)
        payload.update(mode="segment", backend="synthetic_yolo_output", frame_size=[32, 32])
        payload.update(ObjectGeometry({"anchor": "mask_centroid"}, None, None).enrich(decoded, (32, 32, 3), CAPTURE_US / 1e6))
        add(name, payload, {"objects_valid": False, "objects_reason": "missing_calibration", "segmentation_statuses": statuses})

    empty = packet(layout=layout)
    empty.update(Localization({}, CALIBRATION, layout, MOUNT).enrich([], SHAPE))
    add("empty_tags", empty, {"localization_valid": False, "detections_empty": True})
    empty_object = packet("object", calibration=OBJECT_CALIBRATION, mount=OBJECT_MOUNT)
    empty_object.update(ObjectGeometry({"target_height_m": 0.}, OBJECT_CALIBRATION, OBJECT_MOUNT).enrich([], SHAPE, CAPTURE_US / 1e6))
    add("empty_objects", empty_object, {"objects_valid": False, "detections_empty": True})

    # A same-capture invalidation remains a new publication. frame_id is
    # deliberately unchanged; actual Publisher owns packet_seq allocation.
    for name, reason in (("same_frame_watchdog", "No fresh processed frame for 100 ms"),
                         ("repeated_invalidation", "No fresh processed frame for 100 ms"),
                         ("camera_error", "Camera read failed"),
                         ("reload_invalidation", "Configuration reload"),
                         ("shutdown_invalidation", "Runtime shutdown")):
        invalid = packet(layout=layout)
        invalid.update(connected=False, detections=[], error=reason, fps=0., pose_device="none",
                       localization={"valid": False, "field_to_camera": None, "field_to_robot": None,
                                     "used_tag_ids": [], "invalid_reason": reason},
                       objects={"valid": False, "targets": [], "selected_target": None,
                                "selected_track_id": None, "motion_compensated": False, "invalid_reason": reason},
                       poi={"valid": False, "selected_name": None, "targets": [], "invalid_reason": reason})
        add(name, invalid, {"connected": False, "detections_empty": True, "all_actionable_topics_clear": True},
            same_frame_as="single_tag" if name in ("same_frame_watchdog", "repeated_invalidation") else None)
    add("new_boot", dict(copy.deepcopy(single), boot_id="fixture-boot-b"), {"localization_valid": True, "new_session": True})
    add("unsynchronized_time", copy.deepcopy(single), {"localization_valid": True, "time_sync_valid": False}, offset=None)
    measured_zero = copy.deepcopy(single)
    measured_zero["timing"] = dict(clock_domain="nt_server", timestamp_unit="us", capture_event="host_frame_read_complete",
                                   capture_correction_verified=True, capture_correction_uncertainty_ms=.25)
    add("measured_zero_correction", measured_zero, {"time_sync_valid": True, "capture_correction_verified": True})
    unknown_nonzero = copy.deepcopy(single)
    unknown_nonzero.update(capture_latency_offset_ms=4.123456)
    unknown_nonzero["timing"] = dict(clock_domain="nt_server", timestamp_unit="us", capture_event="host_frame_read_complete",
                                     capture_correction_verified=False, capture_correction_uncertainty_ms=None)
    add("unverified_nonzero_correction", unknown_nonzero, {"time_sync_valid": True, "capture_correction_verified": False})
    # The first four files form a replay: valid capture, same-capture watchdog,
    # repeated invalidation, then a new boot. Other files are independent
    # producer snapshots sharing the publisher to exercise sequence allocation.
    lifecycle = ("single_tag", "same_frame_watchdog", "repeated_invalidation", "new_boot")
    return [next(case for case in cases if case["name"] == name) for name in lifecycle] + [case for case in cases if case["name"] not in lifecycle]


def produce():
    pub = Publisher({"enabled": False, "table": "/CustomVision/fixture"}, clock_adapter=StubClock())
    pub.instance = StubInstance()
    records = []
    # Runtime diagnostics are wall-time dependent; run their existing timing
    # reads against a deterministic synthetic clock for reproducible fixtures.
    with patch("custom_vision.localization.time.perf_counter_ns", return_value=0):
        cases = make_cases()
    for case in cases:
        pub.instance.offset = case.get("offset", 2000000)
        host = case["payload"]
        pub.publish(host)
        path = "/CustomVision/fixture/" + host["pipeline"]
        topics = {key: copy.deepcopy(topic.value) for (table, key), topic in pub.instance.topics.items() if table == path}
        encoded = topics["result"]
        records.append(dict(name=case["name"], wire=encoded, packet=json.loads(encoded), host=host,
                            typed={key: value for key, value in topics.items() if key != "result"},
                            expected=case["expected"], same_frame_as=case.get("same_frame_as")))
    return records


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def generate(destination, contract_status="pending_runtime_alignment"):
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    records = produce()
    revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    source_files = ("custom_vision/publisher.py", "custom_vision/app.py", "custom_vision/config.py", "custom_vision/camera.py",
                    "custom_vision/clock.py", "custom_vision/nt_clock.py",
                    "custom_vision/revisions.py", "custom_vision/localization.py", "custom_vision/object_geometry.py",
                    "custom_vision/poi.py", "custom_vision/objects.py", "tools/generate_protocol_fixtures.py")
    manifest = dict(manifest_version=1, contract_status=contract_status, profile=PROFILE,
                    producer_revision=revision,
                    producer_sources_sha256={name: sha256((ROOT / name).read_bytes()) for name in source_files if (ROOT / name).exists()},
                    generation=dict(command=".venv/bin/python tools/generate_protocol_fixtures.py --contract-status " + contract_status, hardware=False,
                                    observations="synthetic CPU geometry", transport="stub NT topics; injected microsecond clock",
                                    serialization="Publisher.publish -> wire_packet -> json.dumps(allow_nan=False,separators=(',',':'))",
                                    opencv_version=cv2.__version__, numpy_version=np.__version__),
                    fixtures=[])
    for record in records:
        filename = record["name"] + ".json"
        # Exactly the coherent string bytes emitted by Publisher, plus a final
        # file newline. Hash includes the newline, not a reformatted JSON tree.
        data = (record["wire"] + "\n").encode()
        (destination / filename).write_bytes(data)
        manifest["fixtures"].append(dict(name=record["name"], path="fixtures/" + filename,
                                         sha256=sha256(data), byte_count=len(data), expected=record["expected"],
                                         same_frame_as=record["same_frame_as"], typed_topics=record["typed"]))
    (destination.parent / "fixture-manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n")
    invalid_dir = destination.parent / "consumer-invalid"
    invalid_dir.mkdir(exist_ok=True)
    invalid_manifest = dict(manifest_version=1, producer_fixture=False, cases=[])
    by_name = {record["name"]: record["packet"] for record in records}
    mutations = (
        ("negative_packet_seq", "single_tag", "packet_seq", -1, False, "negative publication sequence"),
        ("overflow_packet_seq", "single_tag", "packet_seq", 2**53, False, "sequence exceeds exact JSON bound"),
        ("string_timestamp", "single_tag", "capture_server_us", "1234567890123", False, "timestamp must be integer microseconds"),
        ("wrong_timing_unit", "single_tag", "timing.timestamp_unit", "ns", False, "JSON timestamps always use microseconds"),
        ("dangling_selection", "objects_compact_covariance_selection", "objects.selected_track_id", 999, True, "selection must resolve in canonical targets"),
        ("dangling_reference", "objects_compact_covariance_selection", "detections.0.robot_relative.track_id", 999, True, "compact reference must resolve in canonical targets"),
    )
    for name, source, field, value, schema_valid, reason in mutations:
        malformed = copy.deepcopy(by_name[source])
        container = malformed
        parts = field.split(".")
        for part in parts[:-1]:
            container = container[int(part)] if isinstance(container, list) else container[part]
        container[parts[-1]] = value
        data = (json.dumps(malformed, allow_nan=False, separators=(",", ":")) + "\n").encode()
        filename = name + ".json"
        (invalid_dir / filename).write_bytes(data)
        invalid_manifest["cases"].append(dict(name=name, path=filename, derived_from=source, sha256=sha256(data),
                                             expected_schema_valid=schema_valid, expected_consumer_accepted=False, reason=reason))
    (invalid_dir / "manifest.json").write_text(json.dumps(invalid_manifest, indent=2) + "\n")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "protocol" / "fixtures")
    parser.add_argument("--contract-status", choices=("pending_runtime_alignment", "matched_runtime"),
                        default="pending_runtime_alignment",
                        help="Mark matched only after runtime alignment and protocol tests pass.")
    args = parser.parse_args()
    summary = generate(args.output, args.contract_status)
    print(f"Generated {len(summary['fixtures'])} producer fixtures ({summary['contract_status']}).")
