"""CPU calibration job contracts. Process fakes are synthetic lifecycle checks.

Rendered chessboards exercise real OpenCV detection/fitting, not camera accuracy.
No camera is opened and absent mrcal tools are never installed or simulated as a
native qualification.
"""
import base64
import copy
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from custom_vision.calibration_jobs import CalibrationJobs, ProcessAdapter, _PROBE


def asset(manager, body=b"synthetic-placeholder", name="test.png", kind="image"):
    return manager.add_asset({"file_name": name, "kind": kind,
                              "data_base64": base64.b64encode(body).decode("ascii")})


def session(manager, identifiers):
    return manager.create_session({"board": {"cols": 7, "rows": 5, "square_size_m": .025},
        "camera": {"physical_id": None, "user_label": "Synthetic offline test", "identity_source": "manual"},
        "mode": {"width": None, "height": None, "crop": {"x": 0, "y": 0, "width": 640, "height": 480},
                 "binning": [1, 1], "focus": {"kind": "fixed", "value": None, "locked": True}},
        "input": {"kind": "images", "asset_ids": identifiers, "timestamps_asset_id": None},
        "selection": {"novelty": 0, "max_sharpness_px": 5}})


def terminal(manager, identifier, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = manager.get_job(identifier)
        if result["status"] in {"completed", "canceled", "failed", "unavailable"}:
            return result
        time.sleep(.02)
    pytest.fail("Bounded local job did not finish")


@pytest.fixture
def manager(tmp_path):
    instance = CalibrationJobs(tmp_path / "store")
    yield instance
    instance.close()


def test_measured_capabilities_and_venv_invocation_path_are_retained(manager, tmp_path):
    assert manager.python == str(Path(sys.executable).absolute())
    assert manager.capabilities()["versions"]["opencv"] == cv2.__version__
    assert manager.capabilities()["solvers"]["opencv"]["available"]
    caps = manager.capabilities()
    caps["versions"]["opencv"] = "changed-copy"
    assert manager.capabilities()["versions"]["opencv"] == cv2.__version__
    fake_link = tmp_path / "venv" / "bin" / "python"
    fake_link.parent.mkdir(parents=True)
    fake_link.symlink_to(sys.executable)
    other = CalibrationJobs(tmp_path / "other", python=fake_link, solver_python=fake_link)
    try:
        assert other.python == str(fake_link)
        assert other.solver_python == str(fake_link)
    finally:
        other.close()


def test_probe_requires_actual_import_and_accepts_distro_modules_without_metadata(monkeypatch):
    modules = {"cv2": SimpleNamespace(__version__="distro-opencv"), "numpy": SimpleNamespace(__version__="distro-numpy"),
               "mrcal": SimpleNamespace(), "scipy": SimpleNamespace(__version__="distro-scipy")}
    import importlib
    import shutil
    monkeypatch.setattr(importlib, "import_module", lambda name: modules[name])
    monkeypatch.setattr(shutil, "which", lambda name: "/configured/bin/" + name)
    namespace = {}
    exec(_PROBE, namespace)
    result = namespace["result"]
    assert result["mrcal_importable"] and result["mrcal"] == "unknown"
    assert result["opencv"] == "distro-opencv"
    def broken(name):
        if name == "mrcal":
            raise ImportError("native shared object unavailable")
        return modules[name]
    monkeypatch.setattr(importlib, "import_module", broken)
    namespace = {}
    exec(_PROBE, namespace)
    assert namespace["result"]["mrcal_importable"] is False
    assert namespace["result"]["mrcal"] is None


@pytest.mark.parametrize("spec", [
    {"file_name": "../secret.png", "kind": "image", "data_base64": "eA=="},
    {"file_name": "image.png", "kind": "image", "data_base64": "not base64"},
    {"file_name": "image.png", "kind": "image", "data_base64": ""},
    {"file_name": "camera", "kind": "video", "data_base64": "eA=="},
    {"file_name": "file.py", "kind": "image", "data_base64": "eA=="},
])
def test_assets_reject_paths_invalid_formats_and_empty_data(manager, spec):
    with pytest.raises(ValueError):
        manager.add_asset(spec)


def test_assets_are_opaque_retained_and_store_quota_is_bounded(tmp_path):
    manager = CalibrationJobs(tmp_path / "store", total_asset_bytes=5)
    try:
        result = asset(manager, b"hello")
        assert len(result["asset_id"]) == 32 and result["size_bytes"] == 5
        assert set(result) == {"asset_id", "file_name", "kind", "size_bytes", "sha256"}
        with pytest.raises(ValueError, match="store limit"):
            asset(manager, b"!")
        assert manager.list_assets() == [result]
    finally:
        manager.close()
    reopened = CalibrationJobs(tmp_path / "store", total_asset_bytes=5)
    try:
        assert reopened.list_assets() == [result]
    finally:
        reopened.close()


def test_session_identity_unknown_mode_and_no_measured_time_are_explicit(manager):
    record = session(manager, [asset(manager)["asset_id"]])
    assert record["mode"]["width"] is None and record["width"] is None
    assert record["camera"]["physical_id"] is None
    assert record["mode"]["focus"] == {"kind": "fixed", "value": None, "locked": True}
    assert any("OBS" in warning for warning in record["warnings"])
    assert any("timestamp" in warning for warning in record["warnings"])
    assert manager.get_session(record["session_id"])["candidate_ids"] == []
    for identifier in ("../outside", "unknown", None):
        with pytest.raises(ValueError):
            manager.get_session(identifier)
    with pytest.raises(ValueError, match="before solving"):
        manager.create_job({"session_id": record["session_id"], "operation": "solve"})
    with pytest.raises(ValueError, match="executables"):
        manager.create_job({"session_id": record["session_id"], "operation": "select", "options": {"python": "/bin/sh"}})


class SyntheticProcess:
    def __init__(self, ignore_soft=False):
        self.returncode = None
        self.pid = 12345
        self.ignore_soft = ignore_soft
    def poll(self):
        return self.returncode
    def wait(self, timeout=None):
        return self.returncode


class SyntheticAdapter:
    """A fake process exercises cancellation state without executing a solver."""
    def __init__(self, ignore_soft=False):
        self.process = SyntheticProcess(ignore_soft)
        self.started = threading.Event()
        self.calls = []
        self.argv = self.env = None
    def start(self, argv, *, cwd, log, env):
        self.argv, self.env = argv, env
        self.started.set()
        return self.process
    def terminate(self, process, *, force=False):
        assert process is self.process
        self.calls.append(force)
        if force or not process.ignore_soft:
            process.returncode = -9 if force else -15


def test_synthetic_process_cancellation_is_persistent_and_one_job_is_active(tmp_path):
    adapter = SyntheticAdapter()
    manager = CalibrationJobs(tmp_path / "store", process_adapter=adapter)
    try:
        record = session(manager, [asset(manager)["asset_id"]])
        job = manager.create_job({"session_id": record["session_id"], "operation": "select"})
        assert adapter.started.wait(2)
        with pytest.raises(RuntimeError, match="already active"):
            manager.create_job({"session_id": record["session_id"], "operation": "select"})
        assert manager.cancel_job(job["job_id"])["cancel_requested"]
        result = terminal(manager, job["job_id"])
        assert result["status"] == "canceled" and result["cancel_requested"]
        assert adapter.calls == [False, True]
        assert adapter.argv[:3] == [str(Path(sys.executable).absolute()), "-m", "custom_vision.calibration_browser_worker"]
        assert all(adapter.env[name] == "1" for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"))
    finally:
        manager.close()
    restored = CalibrationJobs(tmp_path / "store")
    try:
        assert restored.get_job(job["job_id"])["status"] == "canceled"
    finally:
        restored.close()


def test_synthetic_uncooperative_process_is_killed_after_bounded_timeout(tmp_path):
    adapter = SyntheticAdapter(ignore_soft=True)
    manager = CalibrationJobs(tmp_path / "store", process_adapter=adapter)
    try:
        record = session(manager, [asset(manager)["asset_id"]])
        job = manager.create_job({"session_id": record["session_id"], "operation": "select", "options": {"timeout_s": .1}})
        result = terminal(manager, job["job_id"], timeout=3)
        assert result["status"] == "failed" and result["stage"] == "timeout"
        assert False in adapter.calls and True in adapter.calls
    finally:
        manager.close()


@pytest.fixture(scope="module")
def rendered_images():
    """Three small synthetic board images with varied known poses."""
    images = []
    matrix = np.array([[1800., 0, 960.], [0, 1800., 720.], [0, 0, 1.]])
    for index in (0, 5, 10):
        frame = np.full((1440, 1920), 230, np.uint8)
        rvec = np.array([-.3 + .2 * (index % 4), -.35 + .3 * (index // 4), -.12 + .08 * (index % 3)])
        tvec = np.array([-.1 + .02 * (index % 3), -.07 + .015 * (index // 3), .45 + .04 * (index % 4)])
        for y in range(-1, 5):
            for x in range(-1, 7):
                points = np.array([[x, y, 0], [x + 1, y, 0], [x + 1, y + 1, 0], [x, y + 1, 0]], float) * .025
                pixels = cv2.projectPoints(points, rvec, tvec, matrix, np.zeros(5))[0].reshape(4, 2)
                cv2.fillConvexPoly(frame, np.rint(pixels).astype(np.int32), 0 if (x + y) % 2 == 0 else 255)
        image = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)
        ok, encoded = cv2.imencode(".png", image)
        assert ok
        images.append(encoded.tobytes())
    return images


def test_real_small_image_selection_retains_lossless_frames_coverage_and_declared_mode(manager, rendered_images):
    ids = [asset(manager, body, f"board-{index}.png")["asset_id"] for index, body in enumerate(rendered_images[:3])]
    record = session(manager, ids)
    job = manager.create_job({"session_id": record["session_id"], "operation": "select"})
    completed = terminal(manager, job["job_id"])
    assert completed["status"] == "completed", completed["error"]
    saved = manager.get_session(record["session_id"])
    assert saved["status"] == "selected" and (saved["width"], saved["height"]) == (640, 480)
    assert saved["declared_mode"]["width"] is None and saved["declared_mode"]["height"] is None
    assert saved["mode_dimensions_source"] == "decoded"
    assert saved["mode"]["crop"] == record["mode"]["crop"] and saved["mode"]["focus"] == record["mode"]["focus"]
    assert saved["summary"]["processed_frames"] == 3
    assert saved["summary"]["accepted_views"] >= 2
    assert np.asarray(saved["summary"]["coverage_grid"]).shape == (6, 8)
    assert saved["summary"]["timestamp_source"] == "image_sequence_index_not_capture_time"
    first = saved["views"][0]
    body = manager.get_session_artifact(record["session_id"], first["image"])["body"]
    decoded = cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_UNCHANGED)
    original = cv2.imdecode(np.frombuffer(rendered_images[first["source_frame_id"]], np.uint8), cv2.IMREAD_UNCHANGED)
    np.testing.assert_array_equal(decoded, original)
    assert manager.get_session_artifact(record["session_id"], "selection-preview.jpg")["content_type"] == "image/jpeg"
    with pytest.raises(ValueError, match="artifact"):
        manager.get_session_artifact(record["session_id"], "../../assets/secret")
    native = manager.create_job({"session_id": record["session_id"], "operation": "solve", "solver": "mrcal"})
    assert native["status"] == "unavailable" and "timestamp" in native["error"]


def test_real_opencv_baseline_candidate_is_explicit_and_recovers_after_service_restart(tmp_path, rendered_images):
    manager = CalibrationJobs(tmp_path / "store")
    try:
        record = session(manager, [asset(manager, body, f"board-{index}.png")["asset_id"]
                                   for index, body in enumerate(rendered_images)])
        selected = manager.create_job({"session_id": record["session_id"], "operation": "select"})
        assert terminal(manager, selected["job_id"])["status"] == "completed"
        job = manager.create_job({"session_id": record["session_id"], "operation": "solve", "solver": "opencv",
                                  "options": {"min_views": 3}})
        completed = terminal(manager, job["job_id"])
        assert completed["status"] == "completed", completed["error"]
        identifier = completed["result"]["candidate_id"]
        candidate = manager.get_candidate(identifier)
        assert candidate["quality_status"] == "baseline_not_hardware_validated"
        assert candidate["report"]["training_rms_px"] < .4
        assert candidate["report"]["holdout"]["status"] == "not_computed"
        assert candidate["report"]["uncertainty"]["status"] == "not_computed"
        assert candidate["calibration"]["robot_mount_calibrated"] is False
        assert candidate["calibration"]["calibration_verified"] is False
        assert len(candidate["calibration"]["dist_coeffs"]) == 5  # actual baseline, not relabeled OPENCV8
        assert "directory" not in candidate
        assert manager.candidate_data(identifier) == candidate["calibration"]
        exported = manager.get_artifact(identifier, "intrinsics.candidate.json")
        assert json.loads(exported["body"])["quality_status"] == "baseline_not_hardware_validated"
        assert manager.get_session(record["session_id"])["latest_candidate_id"] == identifier
    finally:
        manager.close()
    restored = CalibrationJobs(tmp_path / "store")
    try:
        assert restored.get_session(record["session_id"])["candidate_ids"] == [identifier]
        assert restored.list_candidates()[0]["candidate_id"] == identifier
    finally:
        restored.close()


def test_real_first_accepted_frame_is_reviewable_after_synthetic_cancel(tmp_path, rendered_images, monkeypatch):
    from custom_vision import calibration_browser_worker as worker
    adapter = SyntheticAdapter()
    manager = CalibrationJobs(tmp_path / "store", process_adapter=adapter)
    worker._CANCELED.clear()
    try:
        record = session(manager, [asset(manager, body, f"board-{index}.png")["asset_id"]
                                  for index, body in enumerate(rendered_images)])
        job = manager.create_job({"session_id": record["session_id"], "operation": "select"})
        assert adapter.started.wait(2)
        job_dir = manager.root / "jobs" / job["job_id"]
        request = json.loads((job_dir / "request.json").read_text())
        process = worker.Session.process
        def cancel_after_first(self, *args, **kwargs):
            result = process(self, *args, **kwargs)
            if result[2]:
                worker._CANCELED.set()
            return result
        monkeypatch.setattr(worker.Session, "process", cancel_after_first)
        with pytest.raises(InterruptedError):
            worker.select(request, manager.root, job_dir)
        manager.cancel_job(job["job_id"])
        assert terminal(manager, job["job_id"])["status"] == "canceled"
        partial = manager.get_session(record["session_id"])
        assert partial["status"] == "incomplete"
        assert partial["summary"]["accepted_views"] == 1
        assert partial["summary"]["processed_frames"] == 1
        assert (partial["width"], partial["height"]) == (640, 480)
        assert partial["mode_dimensions_source"] == "decoded"
        assert partial["declared_mode"]["width"] is None
        image = manager.get_session_artifact(record["session_id"], partial["views"][0]["image"])
        assert image["body"].startswith(b"\x89PNG")
        assert manager.get_session_artifact(record["session_id"], "selection-preview.jpg")["body"].startswith(b"\xff\xd8")
        with pytest.raises(ValueError, match="complete saved session"):
            manager.create_job({"session_id": record["session_id"], "operation": "solve"})
        acquisition = json.loads(
            (manager.root / "sessions" / record["session_id"] / ("selection-" + job["job_id"]) / "session.json").read_text())
        assert acquisition["complete"] is False
    finally:
        worker._CANCELED.clear()
        manager.close()


@pytest.mark.parametrize("kind", ["fixed", "unknown"])
def test_contradictory_focus_metadata_is_rejected(manager, kind):
    record = session(manager, [asset(manager)["asset_id"]])
    spec = {key: copy.deepcopy(record[key]) for key in ("board", "camera", "mode", "input", "selection")}
    spec["mode"]["focus"] = {"kind": kind, "value": 12, "locked": False}
    with pytest.raises(ValueError, match="Only manual focus"):
        manager.create_session(spec)


def test_three_frame_video_uses_aligned_host_timestamp_sidecar(manager, rendered_images, tmp_path):
    video = tmp_path / "source.avi"
    writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"FFV1"), 2., (640, 480), False)
    assert writer.isOpened()
    for body in rendered_images:
        writer.write(cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_GRAYSCALE))
    writer.release()
    video_id = asset(manager, video.read_bytes(), "source.avi", "video")["asset_id"]
    rows = "".join(json.dumps({"frame_id": index, "elapsed_s": float(index)}) + "\n" for index in range(3))
    clock_id = asset(manager, rows.encode(), "raw-timestamps.jsonl", "timestamps")["asset_id"]
    record = manager.create_session({"board": {"cols": 7, "rows": 5, "square_size_m": .025},
        "input": {"kind": "video", "asset_ids": [video_id], "timestamps_asset_id": clock_id},
        "selection": {"novelty": 0, "max_sharpness_px": 5}})
    job = manager.create_job({"session_id": record["session_id"], "operation": "select"})
    completed = terminal(manager, job["job_id"])
    assert completed["status"] == "completed", completed["error"]
    saved = manager.get_session(record["session_id"])
    assert saved["summary"]["processed_frames"] == 3
    assert saved["summary"]["timestamp_source"] == "recorded_host_read_complete"
    assert [view["source_time_s"] for view in saved["views"]] == [0., 1., 2.]
    assert saved["declared_mode"]["width"] is None
    assert saved["mode"]["width"] == 640
    acquisition = json.loads((manager.root / "sessions" / record["session_id"] /
                              ("selection-" + job["job_id"]) / "session.json").read_text())
    assert acquisition["acquisition"]["measured_capture_time_verified"] is False


def test_declared_mode_mismatch_retains_actual_decoded_dimensions(manager, rendered_images):
    initial = session(manager, [asset(manager, rendered_images[0])["asset_id"]])
    spec = {key: copy.deepcopy(initial[key]) for key in ("board", "camera", "mode", "input", "selection")}
    spec["mode"].update(width=1280, height=800)
    record = manager.create_session(spec)
    job = manager.create_job({"session_id": record["session_id"], "operation": "select"})
    completed = terminal(manager, job["job_id"])
    assert completed["status"] == "failed" and "declared mode" in completed["error"]
    saved = manager.get_session(record["session_id"])
    assert saved["status"] == "incomplete"
    assert saved["declared_mode"]["width"] == 1280
    assert saved["mode"]["width"] == saved["width"] == 640
    assert saved["mode_dimensions_source"] == "decoded"
    retry = manager.create_job({"session_id": record["session_id"], "operation": "select"})
    retried = terminal(manager, retry["job_id"])
    assert retried["status"] == "failed" and "declared mode" in retried["error"]
    assert manager.get_session(record["session_id"])["declared_mode"]["width"] == 1280


def test_legacy_saved_mode_is_normalized_conservatively_in_views_and_worker_requests(tmp_path):
    adapter = SyntheticAdapter()
    manager = CalibrationJobs(tmp_path / "store", process_adapter=adapter)
    try:
        record = session(manager, [asset(manager)["asset_id"]])
        path = manager.root / "sessions" / record["session_id"] / "record.json"
        legacy = json.loads(path.read_text())
        legacy.pop("declared_mode")
        legacy.pop("mode_dimensions_source")
        legacy["mode"].update(width=640, height=480)
        path.write_text(json.dumps(legacy))
        restored = manager.get_session(record["session_id"])
        assert restored["mode"]["width"] == 640
        assert restored["declared_mode"]["width"] is None
        assert restored["declared_mode"]["height"] is None
        assert restored["declared_mode"]["crop"] == legacy["mode"]["crop"]
        assert restored["declared_mode"]["binning"] == legacy["mode"]["binning"]
        assert restored["declared_mode"]["focus"] == legacy["mode"]["focus"]
        assert restored["mode_dimensions_source"] == "unknown"
        job = manager.create_job({"session_id": record["session_id"], "operation": "select"})
        assert adapter.started.wait(2)
        request = json.loads((manager.root / "jobs" / job["job_id"] / "request.json").read_text())
        assert request["session"]["declared_mode"]["width"] is None
        assert request["session"]["mode_dimensions_source"] == "unknown"
        assert request["session"]["mode"]["width"] == 640
        manager.cancel_job(job["job_id"])
        assert terminal(manager, job["job_id"])["status"] == "canceled"
    finally:
        manager.close()


def test_selected_rgba_image_retains_unannotated_alpha(manager, rendered_images):
    gray = cv2.imdecode(np.frombuffer(rendered_images[0], np.uint8), cv2.IMREAD_GRAYSCALE)
    rgba = np.dstack((gray, gray, gray, np.full_like(gray, 201)))
    ok, encoded = cv2.imencode(".png", rgba)
    assert ok
    record = session(manager, [asset(manager, encoded.tobytes())["asset_id"]])
    job = manager.create_job({"session_id": record["session_id"], "operation": "select"})
    completed = terminal(manager, job["job_id"])
    assert completed["status"] == "completed", completed["error"]
    saved = manager.get_session(record["session_id"])
    body = manager.get_session_artifact(record["session_id"], saved["views"][0]["image"])["body"]
    np.testing.assert_array_equal(cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_UNCHANGED), rgba)
