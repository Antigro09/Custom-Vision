"""Host-side latency regressions; no cameras, GPU, or robot are simulated as real."""
import builtins
import copy
import json
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from custom_vision import app, publisher
from custom_vision.object_geometry import ObjectGeometry


def _legacy_packet(payload):
    result = publisher.wire_values(payload)
    if isinstance(result.get("objects"), dict):
        result["objects"].pop("selected_target", None)
        for detection in result.get("detections", []):
            target = detection.get("robot_relative")
            if isinstance(target, dict) and target.get("valid"):
                detection["robot_relative"] = {"valid": True, "track_id": target["track_id"]}
    return result


@pytest.mark.parametrize("count", [0, 1, 16, 32])
def test_wire_compaction_matches_legacy_without_mutating_aliases(count):
    targets = [{"valid": True, "track_id": i + 1, "translation_m": [1.23456789, i / 13, 0.],
                "capture_monotonic_us": 1234567890123456,
                "uncertainty": {"range_std_m": .0123456789}} for i in range(count)]
    payload = {"detections": [{"robot_relative": target} for target in targets] +
                            [{"robot_relative": {"valid": False, "reason": "horizon"}}],
               "objects": {"targets": targets, "selected_target": targets[0] if targets else None,
                           "selected_track_id": 1 if targets else None}}
    before = copy.deepcopy(payload)
    actual = publisher.wire_packet(payload)
    assert json.dumps(actual, separators=(",", ":")) == json.dumps(_legacy_packet(payload), separators=(",", ":"))
    assert payload == before
    if targets:
        assert payload["detections"][0]["robot_relative"] is targets[0]
        assert payload["objects"]["selected_target"] is targets[0]
        actual["objects"]["targets"][0]["translation_m"][0] = 99
        assert targets[0]["translation_m"][0] == before["objects"]["targets"][0]["translation_m"][0]


@pytest.mark.parametrize("objects", [None, "unused", {}])
def test_wire_compaction_preserves_tag_and_empty_payloads(objects):
    payload = {"detections": [{"id": 7, "center": (1.23456789, 2.)}], "objects": objects}
    assert publisher.wire_packet(payload) == _legacy_packet(payload)


def test_discarded_duplicate_targets_are_not_recursively_rounded():
    calls = []

    class CountedFloat(float):
        def __round__(self, digits=None):
            calls.append(1)
            return super().__round__(digits)

    target = {"valid": True, "track_id": 7, "translation_m": [CountedFloat(1.23456789)]}
    payload = {"detections": [{"robot_relative": target}],
               "objects": {"targets": [target], "selected_target": target, "selected_track_id": 7}}
    assert publisher.wire_packet(payload)["objects"]["targets"][0]["translation_m"] == [1.234568]
    assert len(calls) == 1, "Only the canonical target should be traversed."


@pytest.mark.parametrize("stdout", [False, True])
def test_offline_publication_does_not_import_ntcore(monkeypatch, capsys, stdout):
    original_import = builtins.__import__

    def without_ntcore(name, *args, **kwargs):
        if name == "ntcore":
            raise ImportError("ntcore deliberately unavailable")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", without_ntcore)
    pub = publisher.Publisher({"enabled": False}, stdout=stdout)
    payload = {"pipeline": "offline", "connected": True, "detections": [], "capture_monotonic_us": 100}
    pub.publish(payload)
    assert payload["capture_server_us"] is None and payload["time_sync_valid"] is False
    output = capsys.readouterr().out
    assert bool(output) is stdout
    if stdout:
        assert json.loads(output) == payload
    pub.close()


def test_disabled_outputs_do_not_serialize(monkeypatch):
    def unexpected(*args, **kwargs):
        raise AssertionError("No output consumer requires JSON serialization")

    monkeypatch.setattr(publisher.json, "dumps", unexpected)
    pub = publisher.Publisher({"enabled": False})
    pub.publish({"connected": True, "detections": []})
    pub.close()


def _worker_fixture(monkeypatch, clock, *, capture=100., count=1):
    runtime = app.Runtime.__new__(app.Runtime)
    runtime.stop = threading.Event()
    runtime.max_frames = 1
    runtime.closed = runtime.had_error = False
    runtime.lock = threading.RLock()
    runtime.dashboard = None
    payloads = []
    runtime.publisher = SimpleNamespace(publish=lambda value: payloads.append(copy.deepcopy(value)))
    group = []
    runtime.states = {}
    for index in range(count):
        cfg = {"name": f"camera_{index}", "type": "object", "settings": {}, "camera": {"source": 0}}
        detector = SimpleNamespace(process=Mock(return_value=[]), geometry=SimpleNamespace(
            reset=Mock(), enrich=Mock(return_value={"detections": [], "objects": {"valid": False}})))
        runtime.states[cfg["name"]] = {"last_frame": capture, "frame_id": 0, "failed": False}
        group.append((cfg, detector))
    runtime.config = {"max_frame_age_ms": 500, "pipelines": [cfg for cfg, _ in group]}
    camera = SimpleNamespace(last_capture_monotonic=capture, release=Mock(),
                             read=Mock(return_value=(True, np.zeros((24, 32, 3), np.uint8))))
    monkeypatch.setattr(app.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(app, "open_camera", lambda _: camera)
    return runtime, group, payloads, camera


def test_expired_input_is_rejected_before_inference(monkeypatch):
    runtime, group, payloads, camera = _worker_fixture(monkeypatch, [100.6])
    runtime.camera_worker(group)
    group[0][1].process.assert_not_called()
    group[0][1].geometry.enrich.assert_not_called()
    group[0][1].geometry.reset.assert_called_once()
    assert payloads[0]["detections"] == [] and not payloads[0]["connected"]
    assert "500 ms" in payloads[0]["error"]
    camera.release.assert_called_once()


def test_shared_camera_rechecks_age_before_each_pipeline(monkeypatch):
    clock = [100.]
    runtime, group, payloads, _ = _worker_fixture(monkeypatch, clock, count=2)

    def slow(_):
        clock[0] += .6
        return []

    group[0][1].process.side_effect = slow
    runtime.camera_worker(group)
    group[0][1].process.assert_called_once()
    group[1][1].process.assert_not_called()
    assert len(payloads) == 2 and all(not value["connected"] for value in payloads)


def test_fresh_input_still_runs_each_pipeline(monkeypatch):
    runtime, group, payloads, _ = _worker_fixture(monkeypatch, [100.1], count=2)
    runtime.camera_worker(group)
    for _, detector in group:
        detector.process.assert_called_once()
        detector.geometry.enrich.assert_called_once()
        detector.geometry.reset.assert_not_called()
    assert len(payloads) == 2 and all(value["connected"] for value in payloads)


def test_shutdown_does_not_start_another_shared_camera_inference(monkeypatch):
    runtime, group, payloads, camera = _worker_fixture(monkeypatch, [100.], count=2)
    group[0][1].process.side_effect = lambda _: runtime.stop.set() or []
    runtime.camera_worker(group)
    group[0][1].process.assert_called_once()
    group[1][1].process.assert_not_called()
    assert not payloads
    camera.release.assert_called_once()


def _target(x, y, class_id=0):
    return {"class_id": class_id, "label": str(class_id), "translation_m": [x, y, 0.],
            "range_from_intake_m": float(np.hypot(x, y))}


@pytest.mark.parametrize("distance,match", [(.299999999, True), (.3, True), (.300000001, False)])
def test_association_inclusive_gate_boundary(distance, match):
    geometry = ObjectGeometry({"tracking_gate_m": .3})
    first = [_target(0., 0.)]
    geometry._associate(first, 1.)
    second = [_target(distance, 0.)]
    geometry._associate(second, 1.01)
    assert (first[0]["track_id"] == second[0]["track_id"]) is match


def test_association_equal_distances_use_stable_track_id_tie_break():
    geometry = ObjectGeometry({"tracking_gate_m": .3})
    first = [_target(-.1, 0.), _target(.1, 0.), _target(0., 0., 1)]
    geometry._associate(first, 1.)
    second = [_target(0., 0.), _target(0., 0., 1)]
    geometry._associate(second, 1.01)
    assert second[0]["track_id"] == first[0]["track_id"]
    assert second[1]["track_id"] == first[2]["track_id"]
