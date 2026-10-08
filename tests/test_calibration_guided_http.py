"""Local routing and ownership boundaries; no camera or solver is started."""
import json
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from custom_vision.dashboard import Dashboard


class GuidedJobs:
    def __init__(self):
        self.calls = []
        self.closed = False

    def capabilities(self): return {"available": True, "live_capture": True}
    def list_candidates(self):
        return [{"candidate_id": "candidate", "quality_status": "needs_review", "reviewed": False}]
    def capture_sources(self):
        return {"sources": [{"source_id": "recorded", "kind": "recorded", "available": True}]}
    def capture_status(self): return {"capture_id": "capture", "state": "preview"}
    def capture_start(self, spec):
        self.calls.append(("start", spec))
        return self.capture_status()
    def capture_snapshot(self, spec):
        self.calls.append(("snapshot", spec))
        return {"session_id": "session", "snapshot_id": "snapshot"}
    def capture_stop(self, spec):
        self.calls.append(("stop", spec))
        return {"capture_id": spec["capture_id"], "state": "stopped"}
    def capture_frame(self, identifier, frame_id=None):
        if identifier != "capture" or frame_id not in (None, 42):
            raise RuntimeError("Displayed frame no longer belongs to this capture")
        self.calls.append(("frame", identifier, frame_id))
        return {"body": b"frame", "content_type": "image/jpeg", "file_name": "frame.jpg"}
    def snapshots(self, identifier):
        return {"snapshots": [{"frame_id": 42, "session_id": identifier}], "mosaic_url": "mosaic"}
    def mosaic(self, identifier):
        return {"body": b"contact sheet", "content_type": "image/jpeg", "file_name": f"{identifier}-mosaic.jpg"}
    def diagnostics(self, identifier):
        return {"status": "available", "frame": "camera_optical", "units": "m", "candidate_id": identifier}
    def review(self, identifier, spec):
        self.calls.append(("review", identifier, spec))
        return {"candidate_id": identifier, "reviewed": True}
    def import_candidate(self, spec):
        self.calls.append(("import", spec))
        return {"candidate_id": "imported", "reviewed": False}
    def candidate_metadata(self, identifier):
        return {"camera": {"physical_id": "camera"}, "mode": {"width": 640, "height": 480},
                "review": {"confirmed": True}, "quality_status": "heuristics_passed_not_hardware_validated",
                "source_kind": "v4l2", "capture_revision": 4, "stale": False}
    def candidate_data(self, identifier): return {"width": 640, "height": 480}
    def close(self): self.closed = True


class Controller:
    writable = True
    def __init__(self): self.calls = []
    def calibration_status(self):
        return [{"pipeline": "front", "active": False, "state": "mode_mismatch", "physical_verification": False}]
    def activate_calibration_candidate(self, pipeline, data, metadata):
        self.calls.append((pipeline, data, metadata))
        return {"activation_id": "activation"}


@pytest.fixture
def service():
    jobs = GuidedJobs()
    dashboard = Dashboard({"host": "127.0.0.1", "port": 0, "calibration_only": True}, calibration_jobs=jobs)
    yield dashboard, jobs
    dashboard.close()
    assert jobs.closed


def request(instance, path, data=None, headers=None):
    target = f"http://127.0.0.1:{instance.server.server_port}{path}"
    values = {"Content-Type": "application/json", "X-Custom-Vision-CSRF": instance._csrf}
    values.update(headers or {})
    raw = None if data is None else json.dumps(data).encode()
    with urlopen(Request(target, data=raw, headers=values), timeout=3) as response:
        body = response.read()
        return (json.loads(body) if response.headers.get_content_type() == "application/json" else body), dict(response.headers)


def test_capture_lifecycle_and_exact_displayed_frame(service):
    dashboard, jobs = service
    assert request(dashboard, "/api/calibration/capture/sources")[0]["sources"][0]["kind"] == "recorded"
    assert request(dashboard, "/api/calibration/capture/status")[0]["state"] == "preview"
    start = {"session_id": "session", "source_id": "recorded", "confirmed": True}
    assert request(dashboard, "/api/calibration/capture/start", start)[0]["capture_id"] == "capture"
    assert jobs.calls[-1] == ("start", start)
    assert request(dashboard, "/api/calibration/capture/capture/frame?frame_id=42")[0] == b"frame"
    assert jobs.calls[-1] == ("frame", "capture", 42)
    snapshot = {"capture_id": "capture", "frame_id": 42}
    assert request(dashboard, "/api/calibration/capture/snapshot", snapshot)[0]["session_id"] == "session"
    assert jobs.calls[-1] == ("snapshot", snapshot)
    assert request(dashboard, "/api/calibration/capture/stop", {"capture_id": "capture"})[0]["state"] == "stopped"


def test_preview_rejects_ambiguous_or_inexact_frame_queries(service):
    dashboard, jobs = service
    queries = ["frame_id=42&frame_id=43", "frame_id=42.0", "frame_id=-1", "frame_id=true",
               "frame_id=", "frame_id=042", "frame_id=9007199254740992", "other=42"]
    for query in queries:
        with pytest.raises(HTTPError) as error:
            request(dashboard, "/api/calibration/capture/capture/frame?" + query)
        assert error.value.code == 400
    assert not jobs.calls
    for suffix in ["capture/frame?frame_id=43", "other/frame?frame_id=42"]:
        with pytest.raises(HTTPError) as error:
            request(dashboard, "/api/calibration/capture/" + suffix)
        assert error.value.code == 409


def test_snapshot_artifacts_and_board_diagnostics_have_separate_routes(service):
    dashboard, _ = service
    assert request(dashboard, "/api/calibration/sessions/session/snapshots")[0]["snapshots"][0]["frame_id"] == 42
    body, headers = request(dashboard, "/api/calibration/sessions/session/mosaic")
    assert body == b"contact sheet" and headers["Content-Disposition"] == 'inline; filename="session-mosaic.jpg"'
    assert headers["Cache-Control"] == "no-store" and headers["X-Content-Type-Options"] == "nosniff"
    diagnostics = request(dashboard, "/api/calibration/candidates/candidate/diagnostics")[0]
    assert diagnostics["frame"] == "camera_optical" and diagnostics["units"] == "m"


def test_import_and_review_are_offline_writes_and_do_not_activate(service):
    dashboard, jobs = service
    spec = {"calibration": {"width": 640, "height": 480}, "confirmed": False}
    assert request(dashboard, "/api/calibration/candidates/import", spec)[0]["reviewed"] is False
    assert jobs.calls[-1] == ("import", spec)
    assert request(dashboard, "/api/calibration/candidates/candidate/review", {"confirmed": True})[0]["reviewed"]
    assert jobs.calls[-1] == ("review", "candidate", {"confirmed": True})
    assert dashboard.controller is None


def test_status_and_activation_keep_full_review_and_capture_provenance(service):
    dashboard, jobs = service
    status = request(dashboard, "/api/calibration/status")[0]
    assert status["runtime_activation_available"] is False and status["default_state"] == "offline_no_runtime"
    assert status["latest_candidates"][0]["quality_status"] == "needs_review"
    dashboard.controller = Controller()
    status = request(dashboard, "/api/calibration/status")[0]
    assert status["runtime_activation_available"] and status["pipelines"][0]["state"] == "mode_mismatch"
    request(dashboard, "/api/calibration/candidates/candidate/activate", {"pipeline": "front", "confirmed": True})
    assert dashboard.controller.calls[0][2] == jobs.candidate_metadata("candidate")


@pytest.mark.parametrize("headers", [
    {"X-Custom-Vision-CSRF": "wrong"}, {"Origin": "http://elsewhere.invalid"}, {"Sec-Fetch-Site": "cross-site"}])
def test_guided_writes_keep_existing_origin_and_setup_token_guards(service, headers):
    dashboard, jobs = service
    for route in ["capture/start", "capture/snapshot", "capture/stop", "candidates/import", "candidates/candidate/review"]:
        with pytest.raises(HTTPError) as error:
            request(dashboard, "/api/calibration/" + route, {"confirmed": True}, headers)
        assert error.value.code == 403
    assert not jobs.calls


def test_dedicated_page_loads_modules_but_combined_page_is_unchanged(service):
    dashboard, _ = service
    page = request(dashboard, "/")[0]
    for name in ["calibration_capture_ui.js", "calibration_diagnostics.js"]:
        assert name.encode() in page
        assert request(dashboard, "/static/" + name)[1]["Content-Type"].startswith("text/javascript")
    dashboard.config["calibration_only"] = False
    page = request(dashboard, "/")[0]
    assert b"field_dashboard.js" in page and b"calibration_capture_ui.js" not in page


@pytest.fixture
def launcher_service(monkeypatch):
    from tools import calibration_dashboard as launcher
    import sys

    instances = []
    class Jobs:
        def __init__(self, root, **kwargs): instances.append((root, kwargs)); self.closed = False
        def close(self): self.closed = True
    class FakeDashboard:
        def __init__(self, config, controller, calibration_jobs):
            assert config["host"] == "127.0.0.1" and controller.writable is False
            self.jobs = calibration_jobs
            self.server = SimpleNamespace(server_port=config["port"])
        def close(self): self.jobs.close()
    class Interrupt:
        def wait(self): raise KeyboardInterrupt

    monkeypatch.setattr(launcher, "CalibrationJobs", Jobs)
    monkeypatch.setattr(launcher, "Dashboard", FakeDashboard)
    monkeypatch.setattr(launcher.threading, "Event", Interrupt)
    monkeypatch.setitem(sys.modules, "custom_vision.calibration_guided", SimpleNamespace(GuidedCalibrationJobs=Jobs))
    return launcher, instances


def test_launcher_requires_explicit_guided_and_synthetic_flags(monkeypatch, tmp_path, launcher_service):
    import sys
    launcher, instances = launcher_service
    for flags, expected in [([], None), (["--guided-capture"], []),
                             (["--guided-capture", "--synthetic-preview"], "synthetic")]:
        monkeypatch.setattr(sys, "argv", ["calibration_dashboard", "--data", str(tmp_path),
                                         "--solver-python", "isolated/bin/python", *flags])
        launcher.main()
        kwargs = instances[-1][1]
        assert kwargs["solver_python"] == "isolated/bin/python"
        if expected is None: assert "sources" not in kwargs
        elif expected == []: assert kwargs["sources"] == []
        else: assert len(kwargs["sources"]) == 1 and kwargs["sources"][0].kind == "synthetic"
    count = len(instances)
    monkeypatch.setattr(sys, "argv", ["calibration_dashboard", "--synthetic-preview"])
    with pytest.raises(SystemExit) as error: launcher.main()
    assert error.value.code == 2 and len(instances) == count


def test_launcher_registers_linux_camera_only_without_opening_it(monkeypatch, tmp_path, launcher_service):
    import sys
    from custom_vision import calibration_capture as capture
    launcher, instances = launcher_service

    def forbidden(*args, **kwargs):
        raise AssertionError("Registration opened or enumerated a camera")

    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.setattr(capture.cv2, "VideoCapture", forbidden)
    from custom_vision import dashboard as dashboard_module
    monkeypatch.setattr(dashboard_module, "discover_devices", forbidden)
    monkeypatch.setattr(sys, "argv", ["calibration_dashboard", "--guided-capture", "--synthetic-preview",
        "--data", str(tmp_path), "--camera-device", "/dev/video9", "--camera-physical-id", "test-camera",
        "--camera-width", "640", "--camera-height", "480", "--camera-fps", "30"])
    launcher.main()
    sources = instances[-1][1]["sources"]
    assert [source.kind for source in sources] == ["synthetic", "v4l2"]
    registered = sources[1].public()
    assert registered["camera"]["physical_id"] == "test-camera"
    assert registered["mode"] == {"width": 640, "height": 480, "crop": None, "binning": None,
                                  "focus": {"kind": "unknown", "value": None, "locked": False}}
    assert "/dev/video9" not in json.dumps(registered)
    # The optional frame rate is a request to the reader, not verified mode data.
    assert "fps" not in registered["mode"]


@pytest.mark.parametrize("flags", [
    ["--camera-device", "/dev/video9"],
    ["--guided-capture", "--camera-device", "/dev/video9"],
    ["--guided-capture", "--camera-physical-id", "camera"],
    ["--guided-capture", "--camera-device", "/dev/video9", "--camera-physical-id", "camera", "--camera-width", "640"],
    ["--guided-capture", "--camera-device", "/dev/video9", "--camera-physical-id", "camera", "--camera-width", "1280", "--camera-height", "1024"],
    ["--guided-capture", "--camera-device", "/dev/video9", "--camera-physical-id", "camera", "--camera-width", "0", "--camera-height", "480"],
    ["--guided-capture", "--camera-device", "/dev/video9", "--camera-physical-id", "camera", "--camera-width", "640", "--camera-height", "480", "--camera-fps", "nan"],
    ["--guided-capture", "--camera-device", "/dev/video9", "--camera-physical-id", "camera", "--camera-width", "640", "--camera-height", "480", "--camera-fps", "241"],
    ["--guided-capture", "--camera-device", "/dev/random", "--camera-physical-id", "camera", "--camera-width", "640", "--camera-height", "480"],
])
def test_launcher_rejects_missing_or_unsupported_camera_configuration(monkeypatch, launcher_service, flags):
    import sys
    launcher, instances = launcher_service
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.setattr(sys, "argv", ["calibration_dashboard", *flags])
    with pytest.raises(SystemExit) as error: launcher.main()
    assert error.value.code == 2 and not instances


def test_launcher_rejects_mac_physical_registration(monkeypatch, launcher_service, capsys):
    import sys
    launcher, instances = launcher_service
    monkeypatch.setattr(sys, "platform", "darwin")
    monkeypatch.setattr(sys, "argv", ["calibration_dashboard", "--guided-capture", "--camera-device", "/dev/video9",
                                      "--camera-physical-id", "camera", "--camera-width", "640", "--camera-height", "480"])
    with pytest.raises(SystemExit) as error: launcher.main()
    assert error.value.code == 2 and not instances
    assert "Mac camera registration is unsupported" in capsys.readouterr().err
