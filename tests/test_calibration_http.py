"""Real local HTTP routing with a synthetic service adapter, no native solve."""
import json
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from custom_vision.dashboard import Dashboard


class Jobs:
    def __init__(self): self.calls = []; self.closed = False
    def capabilities(self): return {"available": True, "live_capture": False, "mrcal": {"available": False}}
    def add_asset(self, data): self.calls.append(("asset", data["file_name"])); return {"asset_id": "synthetic-asset"}
    def list_assets(self): return []
    def create_session(self, data): return {"session_id": "synthetic-session"}
    def list_sessions(self): return [{"session_id": "synthetic-session"}]
    def get_session(self, identifier):
        return {"session_id": identifier, "camera": {"physical_id": None}, "mode": {"width": 640, "height": 480}}
    def create_job(self, data): self.calls.append(("job", data)); return {"job_id": "synthetic-job"}
    def list_jobs(self): return []
    def get_job(self, identifier): return {"job_id": identifier, "status": "completed"}
    def cancel_job(self, identifier): self.calls.append(("cancel", identifier)); return {"cancel_requested": True}
    def list_candidates(self): return []
    def get_candidate(self, identifier): return {"candidate_id": identifier, "session_id": "synthetic-session"}
    def candidate_data(self, identifier): return {"width": 640, "height": 480}
    def get_artifact(self, identifier, name):
        if name != "opencv8/full/model.cameramodel": raise ValueError("Unsafe artifact")
        return {"body": b"synthetic native model", "content_type": "application/octet-stream", "file_name": "model.cameramodel"}
    def get_session_artifact(self, identifier, name):
        if name != "selection-preview.jpg": raise ValueError("Unsafe artifact")
        return {"body": b"synthetic jpeg", "content_type": "image/jpeg", "file_name": name}
    def close(self): self.closed = True


class Controller:
    def __init__(self, writable=True): self.writable = writable; self.calls = []
    def get_config(self): return {"pipelines": []}
    def list_calibration_activations(self): return [{"activation_id": "synthetic-activation"}]
    def activate_calibration_candidate(self, pipeline, data, metadata):
        self.calls.append((pipeline, data, metadata)); return {"activation_id": "synthetic-activation"}
    def restore_calibration_activation(self, identifier): self.calls.append(identifier); return {"restored": True}


@pytest.fixture
def service():
    jobs = Jobs()
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


def test_combined_preview_serves_both_without_camera_discovery_or_activation(monkeypatch):
    from custom_vision import dashboard as module
    from tools.field_dashboard_preview import ReplayConfig

    def forbidden_discovery():
        raise AssertionError("An offline preview must not inspect cameras")

    monkeypatch.setattr(module, "discover_devices", forbidden_discovery)
    jobs = Jobs()
    preview = Dashboard({"host": "127.0.0.1", "port": 0, "offline_preview": True},
                        ReplayConfig(None), calibration_jobs=jobs)
    try:
        preview.update({"pipeline": "front_tags", "input_kind": "synthetic", "frame_id": 1})
        page, _ = request(preview, "/")
        assert b'field-canvas' in page and b'calibration-workspace' in page
        assert b'calibration_dashboard.js' in page and b'field_dashboard.js' in page
        setup, _ = request(preview, "/api/config")
        assert setup["writable"] is False
        assert "SYNTHETIC LOCAL PREVIEW" in setup["config"]["dashboard"]["preview_notice"]
        assert request(preview, "/api/devices")[0]["devices"] == []
        assert request(preview, "/api/devices")[0]["devices"] == []
        assert request(preview, "/api/field-view")[0]["results"]["front_tags"]["input_kind"] == "synthetic"
        assert request(preview, "/api/calibration/capabilities")[0]["runtime_activation"] is False
        assert request(preview, "/api/calibration/assets", {"file_name": "synthetic.png"})[0]["asset_id"] == "synthetic-asset"
        with pytest.raises(HTTPError) as error:
            request(preview, "/api/config", {"pipelines": []})
        assert error.value.code == 405
        with pytest.raises(HTTPError) as error:
            request(preview, "/api/calibration/candidates/synthetic/activate", {"pipeline": "front_tags", "confirmed": True})
        assert error.value.code == 400
    finally:
        preview.close()
    assert jobs.closed


def test_offline_jobs_write_without_runtime_controller(service):
    dashboard, jobs = service
    caps, _ = request(dashboard, "/api/calibration/capabilities")
    assert caps["desktop_mode"] and caps["runtime_activation"] is False and caps["live_capture"] is False
    assert request(dashboard, "/api/calibration/jobs", {"session_id": "synthetic-session", "operation": "select"})[0]["job_id"]
    assert request(dashboard, "/api/calibration/jobs/synthetic-job/cancel", {})[0]["cancel_requested"]
    assert jobs.calls[-1] == ("cancel", "synthetic-job")


def test_offline_page_loads_only_calibration_application(service):
    dashboard, _ = service
    body, _ = request(dashboard, "/")
    text = body.decode()
    assert 'src="/static/calibration_dashboard.js"' in text
    assert 'src="/static/app.js"' not in text and "field_renderer.js" not in text
    assert "setup-form" not in text and "Offline Camera Calibration" in text


def test_media_upload_has_separate_bounded_body_route(service):
    dashboard, jobs = service
    data = {"file_name": "synthetic.png", "data_base64": "a" * (1024 * 1024 + 1), "kind": "image"}
    assert request(dashboard, "/api/calibration/assets", data)[0]["asset_id"]
    # Declare an oversized non-media body without sending it: the server rejects
    # before reading, which can otherwise race a client's large write/broken pipe.
    import http.client
    connection = http.client.HTTPConnection("127.0.0.1", dashboard.server.server_port, timeout=3)
    try:
        connection.request("POST", "/api/calibration/sessions", headers={
            "Content-Length": str(1024 * 1024 + 1), "Content-Type": "application/json",
            "X-Custom-Vision-CSRF": dashboard._csrf})
        assert connection.getresponse().status == 413
    finally: connection.close()


@pytest.mark.parametrize("headers", [
    {"X-Custom-Vision-CSRF": "wrong"}, {"Origin": "http://elsewhere.invalid"}, {"Sec-Fetch-Site": "cross-site"}])
def test_calibration_writes_preserve_existing_csrf_and_origin_guards(service, headers):
    dashboard, jobs = service
    with pytest.raises(HTTPError) as error: request(dashboard, "/api/calibration/assets", {"file_name": "x"}, headers)
    assert error.value.code == 403 and not jobs.calls


def test_artifact_routes_decode_safe_relative_name_and_do_not_expose_arbitrary_files(service):
    dashboard, _ = service
    body, headers = request(dashboard, "/api/calibration/candidates/synthetic/artifacts/opencv8%2Ffull%2Fmodel.cameramodel")
    assert body == b"synthetic native model" and headers["Content-Disposition"] == 'attachment; filename="model.cameramodel"'
    assert request(dashboard, "/api/calibration/sessions/synthetic/artifacts/selection-preview.jpg")[1]["Content-Disposition"].startswith("inline")
    with pytest.raises(HTTPError) as error: request(dashboard, "/api/calibration/candidates/synthetic/artifacts/..%2Fprivate")
    assert error.value.code == 400


def test_candidate_activation_needs_writable_controller_and_explicit_confirmation(service):
    dashboard, _ = service
    route = "/api/calibration/candidates/synthetic/activate"
    with pytest.raises(HTTPError): request(dashboard, route, {"pipeline": "front", "confirmed": True})
    dashboard.controller = Controller(False)
    assert request(dashboard, "/api/calibration/capabilities")[0]["runtime_activation"] is False
    with pytest.raises(HTTPError): request(dashboard, route, {"pipeline": "front", "confirmed": True})
    dashboard.controller = Controller()
    with pytest.raises(HTTPError): request(dashboard, route, {"pipeline": "front"})
    assert dashboard.controller.calls == []
    result = request(dashboard, route, {"pipeline": "front", "confirmed": True})[0]
    assert result["activation_id"]
    assert dashboard.controller.calls[0][2] == {"camera": {"physical_id": None}, "mode": {"width": 640, "height": 480}}
    assert request(dashboard, "/api/calibration/activations/synthetic/restore", {"confirmed": True})[0]["restored"]


def test_normal_runtime_has_no_calibration_manager_or_jobs():
    instance = Dashboard({"host": "127.0.0.1", "port": 0})
    try:
        assert request(instance, "/api/calibration/capabilities")[0]["available"] is False
        with pytest.raises(HTTPError) as error: request(instance, "/api/calibration/jobs", {})
        assert error.value.code == 405
    finally: instance.close()


def test_offline_device_refresh_never_enumerates_cameras(service, monkeypatch):
    from custom_vision import dashboard as module
    def forbidden(): raise AssertionError("offline application enumerated cameras")
    monkeypatch.setattr(module, "discover_devices", forbidden)
    dashboard, _ = service
    dashboard._device_cache_until = 0
    assert request(dashboard, "/api/devices")[0]["devices"] == []


def test_parallel_media_body_read_is_bounded_to_one_upload(service):
    dashboard, jobs = service
    dashboard._calibration_upload.acquire()
    try:
        with pytest.raises(HTTPError) as error:
            request(dashboard, "/api/calibration/assets", {"file_name": "synthetic.png"})
        assert error.value.code == 409 and jobs.calls == []
    finally: dashboard._calibration_upload.release()


def test_browser_job_monitor_and_profile_contract():
    from pathlib import Path
    import shutil
    import subprocess
    node = shutil.which("node")
    if node is None: pytest.skip("Node unavailable for calibration client checks")
    result = subprocess.run([node, "--test", str(Path(__file__).with_name("calibration_dashboard.test.cjs"))],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
