"""Normative schema/profile and actual camera-free producer wire corpus."""
import copy
from decimal import Decimal
import hashlib
import importlib.util
import json
from pathlib import Path

import jsonschema
import pytest
from referencing import Registry, Resource

from custom_vision.publisher import wire_packet


ROOT = Path(__file__).resolve().parents[1]
PROTOCOL = ROOT / "protocol"
BASE = json.loads((PROTOCOL / "schema2.json").read_text())
PROFILE = json.loads((PROTOCOL / "custom-vision-schema2-2026.1.json").read_text())
REGISTRY = Registry().with_resources((
    (BASE["$id"], Resource.from_contents(BASE)),
    (PROFILE["$id"], Resource.from_contents(PROFILE)),
))
VALIDATOR = jsonschema.Draft202012Validator(PROFILE, registry=REGISTRY)
MANIFEST = json.loads((PROTOCOL / "fixture-manifest.json").read_text())
FIXTURES = {entry["name"]: entry for entry in MANIFEST["fixtures"]}


def fixture(name):
    return json.loads((PROTOCOL / FIXTURES[name]["path"]).read_text())


def generator():
    spec = importlib.util.spec_from_file_location("protocol_fixture_generator", ROOT / "tools" / "generate_protocol_fixtures.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_normative_schemas_are_valid_and_base_accepts_deployed_envelope():
    jsonschema.Draft202012Validator.check_schema(BASE)
    jsonschema.Draft202012Validator.check_schema(PROFILE)
    legacy = fixture("single_tag")
    for key in ("protocol_profile", "packet_seq", "calibration_revision", "mount_revision", "field_layout_revision", "timing"):
        legacy.pop(key)
    jsonschema.Draft202012Validator(BASE).validate(legacy)
    assert not VALIDATOR.is_valid(legacy)


@pytest.mark.parametrize("name", FIXTURES)
def test_golden_bytes_hash_schema_units_and_expected_outcomes(name):
    entry = FIXTURES[name]
    data = (PROTOCOL / entry["path"]).read_bytes()
    assert hashlib.sha256(data).hexdigest() == entry["sha256"]
    assert len(data) == entry["byte_count"]
    packet = json.loads(data)
    VALIDATOR.validate(packet)
    assert packet["protocol_profile"] == MANIFEST["profile"]
    assert type(packet["capture_monotonic_us"]) is int
    assert type(packet["publish_unix_us"]) is int
    assert packet["capture_monotonic_us"] == 1234567890123
    assert packet["publish_unix_us"] == 1791456000123456
    assert packet["timestamp_source"] == "host_frame_read_complete"
    assert packet["timing"]["timestamp_unit"] == "us"
    assert packet["timing"]["clock_domain"] == "nt_server"
    assert packet["timing"]["capture_event"] == "host_frame_read_complete"
    topics = entry["typed_topics"]
    assert topics["packet_seq"] == packet["packet_seq"]
    assert topics["frame_id"] == packet["frame_id"]
    assert topics["capture_server_us"] == (packet["capture_server_us"] or 0)
    assert topics["time_sync_valid"] == packet["time_sync_valid"]
    expected = entry["expected"]
    if "localization_valid" in expected:
        assert packet["localization"]["valid"] == expected["localization_valid"]
    if "localization_reason" in expected:
        assert packet["localization"]["invalid_reason"] == expected["localization_reason"]
    if "field_robot_valid" in expected:
        assert bool(packet["localization"]["field_to_robot"]) == expected["field_robot_valid"]
        assert topics["pose_valid"] == expected["field_robot_valid"]
    if "tag_geometry_valid" in expected:
        assert packet["detections"][0]["pose_valid"] == expected["tag_geometry_valid"]
    if "objects_valid" in expected:
        assert packet["objects"]["valid"] == expected["objects_valid"]
        assert topics["target_valid"] == expected["objects_valid"]
    if "objects_reason" in expected:
        assert packet["objects"]["invalid_reason"] == expected["objects_reason"]
    if "poi_valid" in expected:
        assert packet["poi"]["valid"] == expected["poi_valid"]
        assert topics["poi_valid"] == expected["poi_valid"]
    if "poi_robot_valid" in expected:
        assert bool(packet["poi"]["targets"][0]["robot_translation_m"]) == expected["poi_robot_valid"]
    if "poi_reason" in expected:
        assert packet["poi"]["targets"][0]["invalid_reason"] == expected["poi_reason"]
    if "segmentation_statuses" in expected:
        assert packet["mode"] == "segment"
        assert [d.get("segmentation_status") for d in packet["detections"]] == expected["segmentation_statuses"]
        for detection in packet["detections"]:
            if detection.get("segmentation_status") in ("empty", "mask_limit"):
                assert detection["segmentation"] is None
            elif detection.get("segmentation_status") == "valid":
                assert detection["segmentation"]["approximate"] is True
            else:
                assert "segmentation" not in detection
    if "time_sync_valid" in expected:
        assert packet["time_sync_valid"] == expected["time_sync_valid"]
    if "capture_correction_verified" in expected:
        assert packet["timing"]["capture_correction_verified"] == expected["capture_correction_verified"]
    if expected.get("detections_empty"):
        assert packet["detections"] == []
        assert topics["has_target"] is False
    if expected.get("all_actionable_topics_clear"):
        assert packet["connected"] is False
        for key in ("pose_valid", "target_valid", "poi_valid", "has_target", "time_sync_valid"):
            assert topics[key] is False
        for key in ("field_to_robot", "used_tag_ids", "selected_target_robot", "approach_robot_xy", "object_track_ids",
                    "poi_camera_xyz", "poi_robot_xyz", "poi_robot_yaw_deg", "tag_ids"):
            assert topics[key] == []
        assert topics["count"] == 0 and topics["selected_track_id"] == 0


def assert_typed_topics(actual, expected):
    """Integers/structure stay exact; full-precision geometry allows CPU roundoff."""
    if isinstance(expected, dict):
        assert set(actual) == set(expected)
        for key in expected:
            assert_typed_topics(actual[key], expected[key])
    elif isinstance(expected, list):
        assert len(actual) == len(expected)
        for value, reference in zip(actual, expected):
            assert_typed_topics(value, reference)
    elif isinstance(expected, float):
        assert actual == pytest.approx(expected, rel=1e-10, abs=1e-10)
    else:
        assert type(actual) is type(expected)
        assert actual == expected


def test_corpus_reproduces_exact_wire_bytes_through_real_publisher():
    records = generator().produce()
    assert {record["name"] for record in records} == set(FIXTURES)
    for record in records:
        assert (record["wire"] + "\n").encode() == (PROTOCOL / FIXTURES[record["name"]]["path"]).read_bytes()
        assert_typed_topics(record["typed"], FIXTURES[record["name"]]["typed_topics"])


def test_same_frame_invalidation_precedes_measurement_dedup_and_boot_resets_seq():
    observed, invalid, repeated, reboot = [fixture(name) for name in
                                         ("single_tag", "same_frame_watchdog", "repeated_invalidation", "new_boot")]
    assert observed["frame_id"] == invalid["frame_id"] == repeated["frame_id"]
    assert observed["boot_id"] == invalid["boot_id"] == repeated["boot_id"]
    assert observed["packet_seq"] == 0
    assert observed["packet_seq"] < invalid["packet_seq"] < repeated["packet_seq"]
    assert reboot["boot_id"] != observed["boot_id"] and reboot["packet_seq"] == 0
    # Minimal semantic replay: apply accepted publication validity first, then
    # dedupe capture geometry. The duplicate frame must still clear actions.
    seen_captures = set()
    active_pose = None
    last_sequence = -1
    for packet in (observed, invalid, repeated):
        assert packet["packet_seq"] > last_sequence
        last_sequence = packet["packet_seq"]
        if not packet["connected"]:
            active_pose = None
        key = packet["boot_id"], packet["frame_id"]
        if key not in seen_captures:
            seen_captures.add(key)
            if packet["connected"] and packet["localization"]["valid"]:
                active_pose = packet["localization"]["field_to_robot"]
    assert len(seen_captures) == 1 and active_pose is None


def test_float_rounding_keeps_integer_timestamps_and_expanded_dashboard_payload():
    records = generator().produce()
    for record in records:
        decimal_packet = json.loads(record["wire"], parse_float=Decimal)

        def visit(value):
            if isinstance(value, Decimal):
                assert value.as_tuple().exponent >= -6
            elif isinstance(value, dict):
                for item in value.values():
                    visit(item)
            elif isinstance(value, list):
                for item in value:
                    visit(item)

        visit(decimal_packet)
    compact = next(record for record in records if record["name"] == "objects_compact_covariance_selection")
    host, wire = compact["host"], compact["packet"]
    assert host["objects"]["selected_target"] in host["objects"]["targets"]
    assert host["detections"][0]["robot_relative"] is host["objects"]["targets"][0]
    assert host["detections"][0]["confidence"] == .912345678
    assert wire["detections"][0]["confidence"] == .912346
    assert "selected_target" not in wire["objects"]
    targets = {target["track_id"]: target for target in wire["objects"]["targets"]}
    assert wire["objects"]["selected_track_id"] in targets
    for detection in wire["detections"]:
        assert set(detection["robot_relative"]) == {"valid", "track_id"}
        assert detection["robot_relative"]["track_id"] in targets
    for target in targets.values():
        assert target["approximate"] and target["observed"] and not target["predicted"]
        assert not target["approach"]["path_validated"]
        assert len(target["uncertainty"]["covariance_xy_m2"]) == 2
    assert host["objects"]["motion_compensated"] is False


def test_zero_measured_correction_and_nonzero_unknown_are_independent_metadata():
    zero, unknown = fixture("measured_zero_correction"), fixture("unverified_nonzero_correction")
    assert zero["capture_latency_offset_ms"] == 0
    assert zero["timing"]["capture_correction_verified"] is True
    assert zero["timing"]["capture_correction_uncertainty_ms"] == .25
    assert unknown["capture_latency_offset_ms"] > 0
    assert unknown["timing"]["capture_correction_verified"] is False
    assert unknown["timing"]["capture_correction_uncertainty_ms"] is None
    assert zero["capture_server_us"] - unknown["capture_server_us"] == 4123


def test_optional_timing_and_failure_diagnostics_can_be_missing_but_null_is_not_zero_pose():
    packet = fixture("camera_error")
    packet.pop("timing")
    VALIDATOR.validate(packet)
    assert packet["localization"]["field_to_robot"] is None
    assert "method" not in packet["localization"]
    invalid = copy.deepcopy(packet)
    invalid["localization"]["field_to_robot"] = {"translation_m": [0., 0., 0.],
        "rotation_quaternion_wxyz": [1., 0., 0., 0.], "rotation_rpy_deg": [0., 0., 0.], "frame": "wpilib_nwu"}
    assert not VALIDATOR.is_valid(invalid)


@pytest.mark.parametrize("key,value", [("packet_seq", -1), ("packet_seq", 2**53),
                                        ("packet_seq", "1"), ("capture_server_us", "123"),
                                        ("capture_monotonic_us", -1), ("mount_revision", "/home/private/camera.yaml")])
def test_profile_rejects_malformed_numbers_and_nonopaque_revisions(key, value):
    invalid = fixture("single_tag")
    invalid[key] = value
    assert not VALIDATOR.is_valid(invalid)


@pytest.mark.parametrize("field,value", [("timestamp_unit", "ns"), ("capture_event", "exposure"),
                                         ("clock_domain", "unix"), ("capture_correction_verified", 1),
                                         ("capture_correction_uncertainty_ms", -1)])
def test_profile_rejects_incompatible_timing_units_and_metadata(field, value):
    invalid = fixture("single_tag")
    invalid["timing"][field] = value
    assert not VALIDATOR.is_valid(invalid)


def test_malformed_consumer_corpus_is_separate_and_has_explicit_rejection_outcomes():
    invalid_dir = PROTOCOL / "consumer-invalid"
    manifest = json.loads((invalid_dir / "manifest.json").read_text())
    assert manifest["producer_fixture"] is False
    for entry in manifest["cases"]:
        data = (invalid_dir / entry["path"]).read_bytes()
        assert hashlib.sha256(data).hexdigest() == entry["sha256"]
        assert VALIDATOR.is_valid(json.loads(data)) == entry["expected_schema_valid"]
        assert entry["expected_consumer_accepted"] is False
        assert entry["reason"]


@pytest.mark.parametrize("mutation", ["selection", "reference", "duplicate"])
def test_schema_valid_but_incoherent_compact_targets_are_rejected_at_wire_boundary(mutation):
    invalid = fixture("objects_compact_covariance_selection")
    if mutation == "selection":
        invalid["objects"]["selected_track_id"] = 999
    elif mutation == "reference":
        invalid["detections"][0]["robot_relative"]["track_id"] = 999
    else:
        invalid["objects"]["targets"].append(copy.deepcopy(invalid["objects"]["targets"][0]))
    # JSON Schema cannot express these cross-list references. Producer and
    # consumer semantic checks must still reject them.
    VALIDATOR.validate(invalid)
    with pytest.raises(ValueError):
        wire_packet(invalid)
