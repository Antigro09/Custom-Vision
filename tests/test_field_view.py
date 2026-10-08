"""Read-only field geometry and receipt ages use real local dashboard HTTP."""
import copy
import json
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from custom_vision import dashboard as dashboard_module
from custom_vision.dashboard import Dashboard


@pytest.fixture
def dashboard():
    instance = Dashboard({"host": "127.0.0.1", "port": 0})
    yield instance
    instance.close()


def get(dashboard, path):
    with urlopen(f"http://127.0.0.1:{dashboard.server.server_port}{path}", timeout=3) as response:
        return response.read(), dict(response.headers)


def packet(**extra):
    return {"pipeline": "front_tags", "boot_id": "synthetic-boot", "frame_id": 3,
            "capture_monotonic_us": 10_123_456, "publish_unix_us": 1_800_000_000_000_001,
            "connected": True, "detections": [], "input_kind": "synthetic", **extra}


def layout():
    return {"field": {"length": 10., "width": 5.}, "tags": [{
        "ID": 3, "pose": {"translation": {"x": 2., "y": 1., "z": 1.3},
                           "rotation": {"quaternion": {"W": 1., "X": 0., "Y": 0., "Z": 0.}}}}]}


def mount():
    return {"translation_m": [.2, -.1, .7], "rotation_rpy_deg": [0., -10., 0.]}


def test_field_view_preserves_status_and_packet_values(dashboard):
    incoming = packet(localization={"valid": False, "field_to_camera": None,
                                    "field_to_robot": None, "invalid_reason": "no_calibration"})
    original = copy.deepcopy(incoming)
    dashboard.update(incoming)
    status_before, _ = get(dashboard, "/api/status")
    body, headers = get(dashboard, "/api/field-view")
    view = json.loads(body)
    assert set(view) == {"version", "results", "receipt_age_ms", "field_layout", "mounts"}
    assert view["version"] == 1
    assert view["results"] == json.loads(status_before) == {"front_tags": original}
    assert view["results"]["front_tags"]["publish_unix_us"] == 1_800_000_000_000_001
    assert incoming == original and dashboard.results["front_tags"] is incoming
    assert get(dashboard, "/api/status")[0] == status_before
    assert view["receipt_age_ms"]["front_tags"] >= 0
    assert view["field_layout"] is None and view["mounts"] == {}
    assert headers["Cache-Control"] == "no-store"
    assert headers["Content-Security-Policy"] == (
        "default-src 'self'; img-src 'self' blob:; frame-ancestors 'none'; "
        "base-uri 'none'; form-action 'self'")


def test_receipt_age_advances_on_reads_and_resets_only_on_update(dashboard, monkeypatch):
    clock = [20.]
    monkeypatch.setattr(dashboard_module.time, "monotonic", lambda: clock[0])
    dashboard.update(packet())
    assert dashboard.field_view()["receipt_age_ms"]["front_tags"] == 0
    clock[0] = 20.125
    assert dashboard.field_view()["receipt_age_ms"]["front_tags"] == 125
    clock[0] = 21.
    assert dashboard.field_view()["receipt_age_ms"]["front_tags"] == 1000
    # A same-frame invalidation is a fresh receipt but keeps its actual packet.
    invalid = packet(connected=False, error="Watchdog invalidation", capture_monotonic_us=10_123_456)
    dashboard.update(invalid)
    assert dashboard.field_view()["receipt_age_ms"]["front_tags"] == 0
    clock[0] = 21.5
    assert dashboard.field_view()["receipt_age_ms"]["front_tags"] == 500
    assert dashboard.field_view()["results"]["front_tags"] == invalid


def test_pipeline_receipt_ages_remain_independent(dashboard, monkeypatch):
    clock = [10.]
    monkeypatch.setattr(dashboard_module.time, "monotonic", lambda: clock[0])
    dashboard.update(packet())
    clock[0] = 10.5
    dashboard.update(packet(pipeline="rear_tags", connected=False, error="No camera"))
    clock[0] = 11.
    assert dashboard.field_view()["receipt_age_ms"] == {"front_tags": 1000, "rear_tags": 500}


def test_geometry_is_allowlisted_copied_and_nullable(dashboard):
    field = layout()
    field["path"] = "/not-a-served-path"
    field["field"]["private_note"] = "omit"
    mounts = {"front_tags": {**mount(), "credential": "omit"}, "rear_tags": None}
    dashboard.set_field_geometry(field, mounts)
    field["tags"][0]["pose"]["translation"]["x"] = 99.
    mounts["front_tags"]["translation_m"][0] = 99.
    view = json.loads(get(dashboard, "/api/field-view")[0])
    assert view["field_layout"] == layout()
    assert view["mounts"] == {"front_tags": mount(), "rear_tags": None}
    assert "omit" not in json.dumps(view)
    dashboard.set_field_geometry(None, {"front_tags": None})
    assert dashboard.field_view()["field_layout"] is None
    assert dashboard.field_view()["mounts"] == {"front_tags": None}


def test_invalid_geometry_does_not_replace_cached_context(dashboard):
    dashboard.set_field_geometry(layout(), {"front_tags": mount()})
    bad_layout = layout()
    bad_layout["field"]["width"] = float("nan")
    for field, mounts in ((bad_layout, {}), (layout(), []),
                          (layout(), {"../other": None}),
                          (layout(), {"front_tags": {"translation_m": [1, 2], "rotation_rpy_deg": [0, 0, 0]}})):
        with pytest.raises(ValueError):
            dashboard.set_field_geometry(field, mounts)
    assert dashboard.field_view()["field_layout"] == layout()
    assert dashboard.field_view()["mounts"] == {"front_tags": mount()}


def test_camera_only_pose_is_not_replaced_with_robot_or_identity(dashboard):
    camera = {"translation_m": [2., 1., 1.3], "rotation_quaternion_wxyz": [1., 0., 0., 0.],
              "rotation_rpy_deg": [0., 0., 0.], "frame": "wpilib_nwu"}
    incoming = packet(localization={"valid": True, "field_to_camera": camera, "field_to_robot": None,
                                    "robot_pose_invalid_reason": "no_robot_to_camera"})
    dashboard.set_field_geometry(layout(), {"front_tags": None})
    dashboard.update(incoming)
    view = json.loads(get(dashboard, "/api/field-view")[0])
    assert view["results"]["front_tags"] == incoming
    assert view["results"]["front_tags"]["localization"]["field_to_robot"] is None
    assert view["mounts"]["front_tags"] is None


def test_field_view_is_read_only(dashboard):
    request = Request(f"http://127.0.0.1:{dashboard.server.server_port}/api/field-view",
                      data=b"{}", headers={"Content-Type": "application/json"}, method="POST")
    with pytest.raises(HTTPError) as error:
        urlopen(request, timeout=3)
    assert error.value.code == 404
    assert dashboard.field_view()["results"] == {}


def test_runtime_supplies_actual_layout_and_nullable_mounts_without_capture(monkeypatch):
    from custom_vision import app
    captured = {}

    class ReadOnlyDashboard:
        def __init__(self, config, controller=None):
            pass

        def set_field_geometry(self, field, mounts):
            captured.update(field_layout=field, mounts=mounts)

        def set_renderer(self, callback):
            pass

    monkeypatch.setattr(app, "Dashboard", ReadOnlyDashboard)
    monkeypatch.setattr(app, "make_detector", lambda *args: object())
    config = {"field_layout_data": layout(), "networktables": {"enabled": False},
              "dashboard": {"enabled": True}, "pipelines": [
                  {"name": "front_tags", "type": "apriltag", "camera": {"source": 0},
                   "settings": {}, "robot_to_camera": mount()},
                  {"name": "rear_tags", "type": "apriltag", "camera": {"source": 1},
                   "settings": {}, "enabled": False}]}
    runtime = app.Runtime(config)
    try:
        assert captured == {"field_layout": layout(), "mounts": {"front_tags": mount(), "rear_tags": None}}
        assert runtime.threads == []
    finally:
        runtime.publisher.close()


@pytest.mark.parametrize("filename", ["field_model.js", "field_renderer.js", "field_scene.js",
                                    "field_dashboard.js", "field.css"])
def test_field_static_assets_use_exact_whitelist(dashboard, monkeypatch, tmp_path, filename):
    monkeypatch.setattr(dashboard_module, "_STATIC", tmp_path)
    (tmp_path / filename).write_text("local asset")
    body, headers = get(dashboard, f"/static/{filename}")
    assert body == b"local asset"
    assert headers["Content-Type"].startswith("text/css" if filename.endswith(".css") else "text/javascript")
    (tmp_path / "private.txt").write_text("not served")
    for path in ("/static/private.txt", "/static/../private.txt", "/static/%2e%2e/private.txt"):
        with pytest.raises(HTTPError) as error:
            get(dashboard, path)
        assert error.value.code == 404


def test_read_only_preview_controller_never_enables_writes():
    class ReadOnly:
        writable = False
        def get_config(self): return {"pipelines": []}
    instance = Dashboard({"host": "127.0.0.1", "port": 0}, ReadOnly())
    try:
        assert json.loads(get(instance, "/api/config")[0])["writable"] is False
        request = Request(f"http://127.0.0.1:{instance.server.server_port}/api/config",
                          data=b'{}', headers={"Content-Type": "application/json"}, method="POST")
        with pytest.raises(HTTPError) as error:
            urlopen(request, timeout=3)
        assert error.value.code == 405
    finally:
        instance.close()


def test_field_client_geometry_and_receipt_regressions():
    from pathlib import Path
    import shutil
    import subprocess
    node = shutil.which("node")
    if node is None: pytest.skip("Node unavailable for field client checks")
    scripts = sorted(Path(__file__).parent.glob("field_*.test.cjs"))
    assert len(scripts) == 3
    result = subprocess.run([node, "--test", *map(str, scripts)], capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
