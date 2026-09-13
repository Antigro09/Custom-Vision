"""Runtime configuration persistence, upload validation, and HTTP reload handoff."""
import copy
import json
from pathlib import Path
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest
import yaml

from custom_vision.control import RuntimeController
from custom_vision.dashboard import Dashboard


def config():
    return {"pipelines": [{"name": "front_tags", "type": "apriltag", "enabled": True,
                           "camera": {"source": 0, "width": 640, "height": 480, "fps": 30},
                           "settings": {"backend": "pupil", "mode": "2d"}, "robot_to_camera": None}],
            "dashboard": {"enabled": False, "port": 5801}, "networktables": {"enabled": False}}


def calibration():
    return {"width": 640, "height": 480, "camera_matrix": [[500, 0, 320], [0, 500, 240], [0, 0, 1]],
            "dist_coeffs": [0, 0, 0, 0, 0]}


def layout():
    return {"field": {"length": 16.54, "width": 8.02}, "tags": [
        {"ID": 7, "pose": {"translation": {"x": 3., "y": 2., "z": 1.},
                            "rotation": {"quaternion": {"W": 1., "X": 0., "Y": 0., "Z": 0.}}}}]}


@pytest.fixture
def controller(tmp_path):
    path = tmp_path / "config" / "vision.yaml"
    path.parent.mkdir()
    path.write_text(yaml.safe_dump(config()))
    restarted = threading.Event()
    return RuntimeController(path, restarted.set), restarted


def test_config_apply_is_atomic_and_preserves_input(controller):
    instance, restarted = controller
    original = instance.get_config()
    submitted = copy.deepcopy(original)
    submitted["pipelines"][0]["settings"]["threads"] = 3
    submitted["pipelines"][0]["calibration_data"] = {"discard": True}
    submitted["field_layout_data"] = {"discard": True}
    snapshot = copy.deepcopy(submitted)
    validations = []
    instance.validate_runtime = validations.append
    result = instance.apply_config(submitted)
    assert result["saved"] and result["restart_required"]
    assert submitted == snapshot
    saved = instance.get_config()
    assert saved["pipelines"][0]["settings"]["threads"] == 3
    assert "calibration_data" not in saved["pipelines"][0]
    assert "field_layout_data" not in saved
    assert saved["pipelines"][0]["robot_to_camera"] is None
    assert len(validations) == 1
    assert not list(instance.path.parent.glob(".pending-*"))
    assert restarted.wait(1)


def test_invalid_config_and_backend_initialization_do_not_replace_file(controller):
    instance, restarted = controller
    original_bytes = instance.path.read_bytes()
    bad = instance.get_config()
    bad["pipelines"][0]["settings"]["threads"] = 0
    with pytest.raises(ValueError):
        instance.apply_config(bad)
    assert instance.path.read_bytes() == original_bytes
    def reject(_):
        raise RuntimeError("Backend unavailable")
    instance.validate_runtime = reject
    with pytest.raises(RuntimeError, match="Backend unavailable"):
        instance.apply_config(instance.get_config())
    assert instance.path.read_bytes() == original_bytes
    assert not restarted.is_set()


def test_calibration_upload_is_validated_before_artifact_or_config_write(controller):
    instance, restarted = controller
    original_bytes = instance.path.read_bytes()
    bad = calibration(); bad["camera_matrix"][0][0] = -500
    with pytest.raises(ValueError):
        instance.upload_calibration("front_tags", bad)
    with pytest.raises(ValueError, match="Unknown pipeline"):
        instance.upload_calibration("missing", calibration())
    assert instance.path.read_bytes() == original_bytes
    assert not (instance.path.parent.parent / "calibration").exists()
    assert not restarted.is_set()


def test_valid_calibration_is_content_addressed_and_previous_version_survives(controller):
    instance, restarted = controller
    first = calibration()
    instance.upload_calibration("front_tags", first)
    first_path = (instance.path.parent / instance.get_config()["pipelines"][0]["calibration"]).resolve()
    assert first_path.parent == instance.path.parent.parent / "calibration"
    assert json.loads(first_path.read_text())["camera_matrix"] == first["camera_matrix"]
    instance.upload_calibration("front_tags", first)
    assert (instance.path.parent / instance.get_config()["pipelines"][0]["calibration"]).resolve() == first_path
    second = calibration(); second["camera_matrix"][0][0] = 510
    instance.upload_calibration("front_tags", second)
    second_path = (instance.path.parent / instance.get_config()["pipelines"][0]["calibration"]).resolve()
    assert second_path != first_path and first_path.exists()
    assert json.loads(first_path.read_text())["camera_matrix"][0][0] == 500
    assert restarted.wait(1)


def test_field_upload_requires_complete_wpilib_layout_and_no_duplicate_ids(controller):
    instance, restarted = controller
    original = instance.path.read_bytes()
    for invalid in ({"tags": []}, {"field": {"length": 1, "width": 1}, "tags": []},
                    {**layout(), "tags": layout()["tags"] * 2}):
        with pytest.raises(ValueError):
            instance.upload_field_layout(invalid)
    assert instance.path.read_bytes() == original
    assert not restarted.is_set()
    instance.upload_field_layout(layout())
    saved = instance.get_config()
    assert json.loads((instance.path.parent / saved["field_layout"]).read_text()) == layout()
    assert restarted.wait(1)


def test_http_save_returns_before_reload_and_subsequent_server_reads_saved_config(controller):
    instance, restarted = controller
    dashboard = Dashboard({"host": "127.0.0.1", "port": 0}, instance)
    address = f"http://127.0.0.1:{dashboard.server.server_port}"
    try:
        with urlopen(address + "/api/config", timeout=2) as response:
            setup = json.load(response)
        updated = setup["config"]
        updated["pipelines"][0]["preview"] = {"rotation_deg": 270}
        request = Request(address + "/api/config", data=json.dumps(updated).encode(), method="POST",
                          headers={"Content-Type": "application/json", "X-Custom-Vision-CSRF": setup["csrf_token"], "Origin": address})
        with urlopen(request, timeout=2) as response:
            result = json.load(response)
        assert result["saved"]
        assert restarted.wait(1)
    finally:
        dashboard.close()
    # This exercises the HTTP/controller handoff without opening real cameras or
    # claiming a physical capture restart has been validated.
    replacement = Dashboard({"host": "127.0.0.1", "port": 0}, instance)
    try:
        with urlopen(f"http://127.0.0.1:{replacement.server.server_port}/api/config", timeout=2) as response:
            setup = json.load(response)
        assert setup["config"]["pipelines"][0]["preview"]["rotation_deg"] == 270
    finally:
        replacement.close()
