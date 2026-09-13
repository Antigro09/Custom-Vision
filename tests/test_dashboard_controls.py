"""Real HTTP, preview scheduling/race tests, and representative UVC output."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import shutil
import subprocess
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import cv2
import numpy as np
import pytest

from custom_vision.dashboard import Dashboard
from custom_vision import device_controls as controls


def test_browser_request_and_status_regressions():
    node = shutil.which("node")
    if not node:
        pytest.skip("Node is unavailable for browser client regression tests")
    script = Path(__file__).with_name("dashboard_client.test.cjs")
    result = subprocess.run([node, "--test", str(script)], capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr


MODE_OUTPUT = """
ioctl: VIDIOC_ENUM_FMT
    Type: Video Capture
    [0]: 'MJPG' (Motion-JPEG, compressed)
        Size: Discrete 1280x800
            Interval: Discrete 0.008s (120.000 fps)
            Interval: Discrete 0.017s (60.000 fps)
        Size: Discrete 640x400
            Interval: Discrete 0.005s (200.000 fps)
    [1]: 'YUYV' (YUYV 4:2:2)
        Size: Discrete 640x480
            Interval: Discrete 0.033s (30.000 fps)
        Size: Stepwise 32x32 - 4096x4096 with step 2/2
            Interval: Discrete 0.033s (30.000 fps)
"""
CONTROL_OUTPUT = """
User Controls
                     brightness 0x00980900 (int)    : min=0 max=255 step=1 default=128 value=128
                           gain 0x00980913 (int)    : min=0 max=100 step=2 default=0 value=10
          white_balance_automatic 0x0098090c (bool)   : default=1 value=1
                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value=1
                                1: Manual Mode
                                3: Aperture Priority Mode
         exposure_time_absolute 0x009a0902 (int)    : min=1 max=10000 step=1 default=156 value=156 flags=inactive
                  camera_status 0x00980920 (int)    : min=0 max=255 step=1 default=0 value=1 flags=read-only
"""


class Controller:
    def __init__(self):
        self.calls = []

    def get_config(self):
        return {"pipelines": [{"name": "front_tags", "type": "apriltag"}], "dashboard": {"port": 5801}}

    def apply_config(self, data):
        if "pipelines" not in data:
            raise ValueError("At least one pipeline is required")
        self.calls.append(("config", data))
        return {"ok": True, "restart_required": False}

    def upload_calibration(self, pipeline, data):
        self.calls.append(("calibration", pipeline, data))
        return {"ok": True}

    def upload_field_layout(self, data):
        self.calls.append(("field", data))
        return {"ok": True}


@pytest.fixture
def dashboard():
    instance = Dashboard({"host": "127.0.0.1", "port": 0, "stream_fps": 10}, Controller())
    yield instance
    instance.close()


def base(dashboard):
    return f"http://127.0.0.1:{dashboard.server.server_port}"


def get(dashboard, path):
    with urlopen(base(dashboard) + path, timeout=3) as response:
        return response.read()


def post(dashboard, path, data, *, token=True, headers=None):
    initial = json.loads(get(dashboard, "/api/config"))
    fields = {"Content-Type": "application/json"}
    if token:
        fields["X-Custom-Vision-CSRF"] = initial["csrf_token"]
    fields.update(headers or {})
    request = Request(base(dashboard) + path, data=json.dumps(data).encode(), headers=fields, method="POST")
    with urlopen(request, timeout=3) as response:
        return json.load(response)


def payload(connected=True, **extra):
    return {"pipeline": "front_tags", "connected": connected, "detections": [], "frame_id": 42, **extra}


def test_uvc_modes_are_complete_tuples_not_cartesian_product():
    modes = controls.parse_modes(MODE_OUTPUT)
    assert len(modes) == 4
    assert modes[0] == {"fourcc": "MJPG", "width": 1280, "height": 800, "fps": 120}
    assert modes[-1] == {"fourcc": "YUYV", "width": 640, "height": 480, "fps": 30}
    assert not any(m["fourcc"] == "YUYV" and m["fps"] == 120 for m in modes)


def test_uvc_controls_only_allow_advertised_ranges_and_menus():
    specs = controls.parse_controls(CONTROL_OUTPUT)
    assert controls.validate_controls({"brightness": 37, "auto_exposure": 1, "white_balance_automatic": 0}, specs)
    for values in ({"focus": 12}, {"gain": 3}, {"brightness": 256}, {"brightness": True},
                   {"auto_exposure": 2}, {"exposure_time_absolute": 10}, {"camera_status": 0}):
        with pytest.raises(ValueError):
            controls.validate_controls(values, specs)
    exposure = next(c for c in specs if c["name"] == "auto_exposure")
    assert exposure["menu"] == {"1": "Manual Mode", "3": "Aperture Priority Mode"}


def test_uvc_invalid_control_batch_does_not_write(monkeypatch):
    calls = []
    def run(source, *args):
        calls.append((source, args))
        return CONTROL_OUTPUT
    monkeypatch.setattr(controls, "_run", run)
    with pytest.raises(ValueError):
        controls.apply_controls(0, {"brightness": 80, "focus": 10})
    assert calls == [("/dev/video0", ("--list-ctrls-menus",))]
    assert controls.apply_controls(0, {"brightness": 128, "auto_exposure": 1}) == {"auto_exposure": 1, "brightness": 128}
    assert calls[-2][1] == ("--set-ctrl", "auto_exposure=1,brightness=128")


def test_uvc_source_rejects_non_device_paths_and_shell_text():
    assert controls.device_path(1) == "/dev/video1"
    for source in (True, -1, "/etc/passwd", "0; touch /tmp/unwanted", "videotestsrc ! appsink"):
        with pytest.raises(ValueError):
            controls.device_path(source)


def test_dashboard_serves_self_contained_ui_and_no_arbitrary_files(dashboard):
    page = get(dashboard, "/")
    assert b"Live preview" in page and b"Robot &amp; field" in page
    assert b"collectSettings" in get(dashboard, "/static/app.js")
    assert b"--accent" in get(dashboard, "/static/style.css")
    for path in ("/static/../../PROJECT_MEMORY.md", "/frame/../anything", "/frame/%2Fetc%2Fpasswd"):
        with pytest.raises(HTTPError) as error:
            get(dashboard, path)
        assert error.value.code == 404


def test_dashboard_configuration_uploads_and_validation(dashboard):
    assert post(dashboard, "/api/config", {"pipelines": []})["ok"]
    assert post(dashboard, "/api/calibration", {"pipeline": "front_tags", "data": {"width": 1280}})["ok"]
    assert post(dashboard, "/api/field-layout", {"data": {"tags": []}})["ok"]
    assert len(dashboard.controller.calls) == 3
    for path, data in (("/api/config", {}), ("/api/calibration", {"pipeline": "../escape", "data": {}}),
                       ("/api/field-layout", {"data": []})):
        with pytest.raises(HTTPError) as error:
            post(dashboard, path, data)
        assert error.value.code == 400
    assert len(dashboard.controller.calls) == 3


def test_dashboard_blocks_cross_origin_and_missing_tokens(dashboard):
    for headers, token in (({}, False), ({"Origin": "https://unrelated.example"}, True),
                           ({"Sec-Fetch-Site": "cross-site"}, True), ({"Content-Type": "text/plain"}, True)):
        with pytest.raises(HTTPError) as error:
            post(dashboard, "/api/config", {"pipelines": []}, headers=headers, token=token)
        assert error.value.code in (403, 415)
    assert dashboard.controller.calls == []
    assert post(dashboard, "/api/config", {"pipelines": []}, headers={"Origin": base(dashboard)})["ok"]


def test_dashboard_rejects_oversized_body_before_controller(dashboard):
    with pytest.raises(HTTPError) as error:
        post(dashboard, "/api/config", {}, headers={"Content-Length": str(1024 * 1024 + 1)})
    assert error.value.code == 413
    assert dashboard.controller.calls == []


def test_dashboard_read_only_without_controller():
    dashboard = Dashboard({"host": "127.0.0.1", "port": 0})
    try:
        assert json.loads(get(dashboard, "/api/config"))["writable"] is False
        with pytest.raises(HTTPError) as error:
            post(dashboard, "/api/config", {})
        assert error.value.code == 405
    finally:
        dashboard.close()


def test_preview_is_demand_driven_and_rotation_does_not_mutate_input(dashboard):
    rendered = threading.Event()
    frame = np.zeros((50, 80, 3), np.uint8)
    def render(data, incoming):
        assert incoming is frame
        assert threading.current_thread() is dashboard.encoder_thread
        rendered.set()
        return incoming
    dashboard.set_renderer(render)
    dashboard.update(payload(preview_settings={"rotation_deg": 90}), frame)
    assert not rendered.wait(.05), "No viewer should mean no rendering work"
    jpeg = get(dashboard, "/frame/front_tags")
    assert rendered.is_set()
    decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (80, 50, 3)
    assert frame.shape == (50, 80, 3)
    assert dashboard.preview_stats["front_tags"]["rotation_deg"] == 90


def test_disconnect_drops_inflight_encode_and_update_does_not_wait(dashboard):
    started, proceed = threading.Event(), threading.Event()
    frame = np.zeros((50, 80, 3), np.uint8)
    def render(data, incoming):
        started.set()
        assert proceed.wait(3)
        return incoming
    dashboard.set_renderer(render)
    dashboard.update(payload(), frame)
    def read_frame():
        try:
            return get(dashboard, "/frame/front_tags")
        except HTTPError as exc:
            return exc.code
    with ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(read_frame)
        assert started.wait(2)
        try:
            # This must return before the blocked renderer is released.
            dashboard.update(payload(False, error="Disconnected"))
            assert future.result(timeout=1) == 404
        finally:
            proceed.set()
    with dashboard.condition:
        dashboard.condition.wait_for(lambda: not dashboard.images, timeout=1)
    assert "front_tags" not in dashboard.images
    assert json.loads(get(dashboard, "/api/status"))["front_tags"]["connected"] is False


def test_preview_encodes_latest_pending_frame(dashboard):
    seen = []
    dashboard.set_renderer(lambda data, frame: (seen.append(data["frame_id"]) or frame))
    frame = np.zeros((12, 16, 3), np.uint8)
    for i in range(100):
        dashboard.update(payload(frame_id=i), frame)
    assert get(dashboard, "/frame/front_tags").startswith(b"\xff\xd8")
    assert seen == [99]
