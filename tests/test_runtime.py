"""Runtime contracts tested without a physical camera or robot network."""

import copy
import json
import socket
import threading
import time
from urllib.error import HTTPError
from urllib.request import urlopen

import numpy as np
import pytest
import yaml

from custom_vision import app
from custom_vision.camera import LatestFrameCapture
from custom_vision.config import load_config
from custom_vision.dashboard import Dashboard
from custom_vision.publisher import Publisher


def write_config(tmp_path, value):
    path = tmp_path / "vision.yaml"
    path.write_text(yaml.safe_dump(value))
    return path


def base_config():
    return {"pipelines": [{"name": "front_tags", "type": "apriltag",
                           "camera": {"source": 0}, "settings": {}}]}


def test_config_resolves_inputs_relative_to_yaml(tmp_path):
    calibration = {"fx": 800, "fy": 800, "cx": 640, "cy": 400,
                   "dist_coeffs": [0, 0, 0, 0, 0]}
    (tmp_path / "camera.json").write_text(json.dumps(calibration))
    config = base_config()
    pipeline = config["pipelines"][0]
    pipeline["camera"]["source"] = "recording.avi"
    pipeline["calibration"] = "camera.json"
    pipeline["settings"]["model_path"] = "models/game_piece.onnx"
    result = load_config(write_config(tmp_path, config))
    actual = result["pipelines"][0]
    assert actual["camera"]["source"] == str(tmp_path / "recording.avi")
    assert actual["settings"]["model_path"] == str(tmp_path / "models/game_piece.onnx")
    assert actual["calibration_data"] == calibration
    assert result["networktables"]["enabled"] is False


@pytest.mark.parametrize("team", [None, True, 0, -1, "1086", 100000])
def test_enabled_networktables_requires_valid_destination(tmp_path, team):
    config = base_config()
    config["networktables"] = {"enabled": True, "team": team}
    with pytest.raises(ValueError, match="team"):
        load_config(write_config(tmp_path, config))


def test_duplicate_names_rejected(tmp_path):
    config = base_config()
    config["pipelines"].append(copy.deepcopy(config["pipelines"][0]))
    with pytest.raises(ValueError, match="unique"):
        load_config(write_config(tmp_path, config))


@pytest.mark.parametrize("camera", [
    {"backend": "invalid"}, {"fourcc": "MJPEG"}, {"width": True},
    {"height": 720.5}, {"source": -1},
])
def test_invalid_camera_settings_rejected_before_capture(tmp_path, camera):
    config = base_config()
    config["pipelines"][0]["camera"].update(camera)
    with pytest.raises(ValueError):
        load_config(write_config(tmp_path, config))


class RecordingPublisher:
    def __init__(self):
        self.payloads = []
        self.closed = False

    def publish(self, payload):
        self.payloads.append(copy.deepcopy(payload))

    def close(self):
        self.closed = True


class FakeCamera:
    def __init__(self, reads):
        self.reads = iter(reads)
        self.released = False

    def read(self):
        return next(self.reads)

    def release(self):
        self.released = True


class FakeDetector:
    def process(self, frame):
        return [{"id": 7, "center": [16, 12]}]


def fake_runtime(*, max_frames=1):
    """Exercise capture/publication without importing an inference backend."""
    runtime = app.Runtime.__new__(app.Runtime)
    runtime.config = base_config()
    pipeline = runtime.config["pipelines"][0]
    runtime.stop = threading.Event()
    runtime.max_frames = max_frames
    runtime.threads = []
    runtime.closed = False
    runtime.had_error = False
    runtime.lock = threading.RLock()
    runtime.states = {pipeline["name"]: {"last_frame": time.monotonic(),
                                        "frame_id": 0, "failed": False}}
    runtime.groups = {"0": [(pipeline, FakeDetector())]}
    runtime.publisher = RecordingPublisher()
    runtime.dashboard = None
    return runtime


def test_camera_failure_clears_previous_target(monkeypatch):
    runtime = fake_runtime(max_frames=2)
    camera = FakeCamera([(True, np.zeros((24, 32, 3), np.uint8)), (False, None)])
    monkeypatch.setattr(app, "open_camera", lambda _: camera)
    runtime.camera_worker(runtime.groups["0"])
    first, failed = runtime.publisher.payloads
    assert first["connected"] and first["detections"][0]["id"] == 7
    assert failed["connected"] is False
    assert failed["detections"] == []
    assert "Camera read failed" in failed["error"]
    assert camera.released


def test_slow_inference_cannot_publish_expired_target(monkeypatch):
    runtime = fake_runtime()
    clock = [100.0]
    camera = FakeCamera([(True, np.zeros((24, 32, 3), np.uint8))])

    class SlowDetector:
        def process(self, frame):
            clock[0] += 0.6
            return [{"id": 7, "center": [16, 12]}]

    runtime.groups["0"][0] = (runtime.groups["0"][0][0], SlowDetector())
    monkeypatch.setattr(app.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(app, "open_camera", lambda _: camera)
    runtime.camera_worker(runtime.groups["0"])
    [payload] = runtime.publisher.payloads
    assert payload["connected"] is False
    assert payload["detections"] == []
    assert "500 ms" in payload["error"]


def test_watchdog_clears_target_while_camera_read_is_blocked(monkeypatch):
    runtime = fake_runtime(max_frames=2)
    allow_read = threading.Event()
    captured_target = threading.Event()
    got_watchdog = threading.Event()
    camera = FakeCamera([(True, np.zeros((24, 32, 3), np.uint8))])
    original_publish = runtime.publisher.publish

    def publish(payload):
        original_publish(payload)
        if payload["detections"]:
            captured_target.set()
        if payload["error"] == "No fresh frame within 500 ms":
            got_watchdog.set()
            allow_read.set()

    reads = [0]

    def read():
        reads[0] += 1
        if reads[0] == 1:
            return True, np.zeros((24, 32, 3), np.uint8)
        allow_read.wait(3)
        return False, None

    camera.read = read
    runtime.publisher.publish = publish
    monkeypatch.setattr(app, "open_camera", lambda _: camera)
    runner = threading.Thread(target=runtime.run)
    try:
        runner.start()
        assert captured_target.wait(2)
        assert got_watchdog.wait(2)
    finally:
        allow_read.set()
        runtime.stop.set()
        runner.join(timeout=3)
    assert not runner.is_alive()
    assert runtime.publisher.closed
    final = runtime.publisher.payloads[-1]
    assert final["connected"] is False and final["detections"] == []
    assert final["error"] == "Runtime stopped"
    assert camera.released


def test_stop_during_capture_does_not_publish_new_targets(monkeypatch):
    runtime = fake_runtime()

    class StopDuringRead(FakeCamera):
        def read(self):
            runtime.stop.set()
            return True, np.zeros((24, 32, 3), np.uint8)

    camera = StopDuringRead([])
    monkeypatch.setattr(app, "open_camera", lambda _: camera)
    runtime.camera_worker(runtime.groups["0"])
    assert not any(payload["detections"] for payload in runtime.publisher.payloads)
    assert camera.released


def test_latest_capture_overwrites_unconsumed_frames():
    first = np.full((8, 8, 3), 10, np.uint8)
    newest = np.full((8, 8, 3), 20, np.uint8)
    drained = threading.Event()
    end_input = threading.Event()

    class BatchCamera(FakeCamera):
        def read(self):
            try:
                return super().read()
            except StopIteration:
                # Starting the third read proves both queued frames were received.
                drained.set()
                end_input.wait(3)
                return False, None

    camera = BatchCamera([(True, first), (True, newest)])
    capture = LatestFrameCapture(camera)
    try:
        assert drained.wait(2)
        ok, delivered = capture.read()
        assert ok
        np.testing.assert_array_equal(delivered, newest)
        assert capture.last_capture_monotonic > 0
    finally:
        end_input.set()
        capture.release()
    assert camera.released
    assert not capture.thread.is_alive()


def test_latest_capture_failure_blocks_buffered_stale_frame():
    camera = FakeCamera([(True, np.full((8, 8, 3), 20, np.uint8)), (False, None)])
    capture = LatestFrameCapture(camera)
    try:
        capture.thread.join(timeout=2)
        assert not capture.thread.is_alive()
        # The successful frame was never consumed, but must not survive failure.
        ok, delivered = capture.read()
        assert ok is False
        assert delivered is None
        assert camera.released
    finally:
        capture.release()


def payload(*, connected=True, detections=None):
    return {"schema_version": 1, "pipeline": "front_tags", "type": "apriltag",
            "connected": connected, "frame_id": 1, "latency_ms": 5.0,
            "detections": detections if detections is not None else [{"id": 7}],
            "error": None if connected else "Camera disconnected"}


def test_dashboard_removes_disconnected_preview():
    dashboard = Dashboard({"host": "127.0.0.1", "port": 0})
    address = "http://127.0.0.1:%d" % dashboard.server.server_port
    try:
        dashboard.update(payload(), np.zeros((24, 32, 3), np.uint8))
        with urlopen(address + "/frame/front_tags", timeout=2) as response:
            assert response.headers["Content-Type"] == "image/jpeg"
            assert response.read().startswith(b"\xff\xd8")
        dashboard.update(payload(connected=False, detections=[]))
        with urlopen(address + "/api/status", timeout=2) as response:
            status = json.load(response)
            assert status["front_tags"]["connected"] is False
            assert status["front_tags"]["detections"] == []
        with pytest.raises(HTTPError) as error:
            urlopen(address + "/frame/front_tags", timeout=2)
        assert error.value.code == 404
    finally:
        dashboard.close()


def unused_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def wait_until(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_nt4_round_trip_clears_all_target_topics(tmp_path):
    ntcore = pytest.importorskip("ntcore")
    server = ntcore.NetworkTableInstance.create()
    publisher = Publisher({"enabled": False})
    subscriptions = []
    try:
        port = unused_port()
        server.startServer(str(tmp_path / "nt.json"), "127.0.0.1", 0, port)
        publisher.instance = ntcore.NetworkTableInstance.create()
        publisher.instance.setServer("127.0.0.1", port)
        publisher.instance.startClient4("custom-vision-test")
        table = server.getTable("/CustomVision/front_tags")
        result = table.getStringTopic("result").subscribe("")
        connected = table.getBooleanTopic("connected").subscribe(False)
        has_target = table.getBooleanTopic("has_target").subscribe(False)
        count = table.getIntegerTopic("count").subscribe(-1)
        ids = table.getIntegerArrayTopic("tag_ids").subscribe([])
        subscriptions.extend([result, connected, has_target, count, ids])
        assert wait_until(publisher.instance.isConnected), "Local NT4 connection failed"
        publisher.publish(payload())
        assert wait_until(lambda: count.get() == 1 and list(ids.get()) == [7])
        assert wait_until(lambda: connected.get() and has_target.get())
        assert json.loads(result.get())["detections"] == [{"id": 7}]
        publisher.publish(payload(connected=False, detections=[]))
        assert wait_until(lambda: count.get() == 0 and list(ids.get()) == []
                          and not has_target.get() and not connected.get())
        assert wait_until(lambda: json.loads(result.get())["detections"] == [])
    finally:
        publisher.close()
        for subscription in subscriptions:
            subscription.close()
        server.stopServer()
        ntcore.NetworkTableInstance.destroy(server)
