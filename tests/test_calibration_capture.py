"""Bounded acquisition contracts; no physical devices are opened."""
import threading
import time
import subprocess
import sys

import cv2
import numpy as np
import pytest

from custom_vision.calibration_capture import (CalibrationCapture, CaptureSource, SyntheticChessboard,
                                              synthetic_source, recorded_source, v4l2_source)
from custom_vision.calibration_session import Board, detect_board
from custom_vision.camera_ownership import (CameraBusyError, LeasedCapture, acquire_camera, physical_device)


def session(identifier="session-one", board=None):
    return {"session_id": identifier, "board": board or {"cols": 7, "rows": 5, "square_size_m": .03},
            "camera": {"physical_id": None, "user_label": "Recorded camera", "identity_source": "manual", "capture_history": None},
            "mode": {"width": 640, "height": 480, "crop": None, "binning": None,
                     "focus": {"kind": "unknown", "value": None, "locked": False}}}


def detected(_frame, board):
    x, y = np.meshgrid(np.arange(board.cols) * 30 + 100, np.arange(board.rows) * 30 + 100)
    return np.stack((x, y), axis=-1).reshape(-1, 2).astype(float), {"sharpness_px": 1., "contrast": 180.}


class Reader:
    def __init__(self, *, shape=(480, 640), after_first=None):
        self.count = 0
        self.shape = shape
        self.closed = False
        self.after_first = after_first

    def read(self):
        self.count += 1
        if self.count > 1 and self.after_first:
            self.after_first()
        return (False, None) if self.closed else (True, np.full(self.shape, 180, np.uint8))

    def release(self):
        self.closed = True
        return True


def wait(manager, predicate=lambda status: status["frame"] is not None):
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        status = manager.status()
        if predicate(status):
            return status
        time.sleep(.005)
    raise AssertionError(manager.status())


@pytest.fixture
def manager():
    reader = Reader()
    source = recorded_source("recorded-id", "Recorded test (not live)", lambda _: reader)
    value = CalibrationCapture([source], detector=detected, preview_fps=10)
    yield value
    value.close()


def test_sources_are_explicit_and_do_not_discover_or_open_cameras(monkeypatch):
    monkeypatch.setattr(cv2, "VideoCapture", lambda *args: pytest.fail("No camera may be opened"))
    manager = CalibrationCapture()
    assert manager.list_sources() == []
    assert manager.status()["state"] == "idle"
    manager = CalibrationCapture([synthetic_source()])
    public = manager.list_sources()[0]
    assert public["synthetic"] is True and public["camera"]["physical_id"] is None
    assert "device" not in public and "factory" not in public
    manager.close()


@pytest.mark.parametrize("change", ["unconfirmed", "board", "mode", "camera", "unknown_source"])
def test_prerequisites_before_any_source_read(change):
    opened = []
    source = recorded_source("registered", "Recorded", lambda _: opened.append(True))
    manager = CalibrationCapture([source], detector=detected)
    spec = session()
    if change == "board":
        spec["board"]["cols"] = 2
    if change == "mode":
        spec["mode"]["width"] = 1280
    if change == "camera":
        spec["camera"]["physical_id"] = "another-camera"
    with pytest.raises(ValueError):
        manager.start(spec, "unregistered" if change == "unknown_source" else "registered", confirmed=change != "unconfirmed")
    assert not opened
    manager.close()


def test_snapshot_is_lossless_separate_overlay_and_transactional(manager):
    status = manager.start(session(), "recorded-id", confirmed=True)
    status = wait(manager)
    assert status["frame"]["corner_count"] == 35 and status["snapshot_eligible"]
    assert status["source"]["kind"] == "recorded"
    identifier = status["capture_id"]
    jpeg, mime = manager.frame(identifier)
    assert mime == "image/jpeg" and jpeg[:2] == b"\xff\xd8"
    snapshot = manager.snapshot(identifier, status["frame"]["frame_id"])
    assert manager.status()["snapshot_count"] == 0
    assert manager.snapshot(identifier, snapshot["frame_id"])["snapshot_id"] == snapshot["snapshot_id"]
    raw = cv2.imdecode(np.frombuffer(snapshot["raw_image_bytes"], np.uint8), cv2.IMREAD_UNCHANGED)
    overlay = cv2.imdecode(np.frombuffer(snapshot["overlay_image_bytes"], np.uint8), cv2.IMREAD_UNCHANGED)
    assert raw.shape == (480, 640) and np.all(raw == 180)
    assert overlay.shape == (480, 640, 3) and not np.all(overlay == 180)
    committed = manager.commit_snapshot(identifier, snapshot["snapshot_id"])
    assert committed["snapshot_count"] == 1 and committed["coverage_fraction"] > 0
    assert manager.commit_snapshot(identifier, snapshot["snapshot_id"])["snapshot_count"] == 1
    with pytest.raises(ValueError, match="Already covered"):
        manager.snapshot(identifier, wait(manager)["frame"]["frame_id"])


def test_storage_failure_discards_reservation_without_count_or_coverage(manager):
    manager.start(session(), "recorded-id", confirmed=True)
    status = wait(manager)
    snapshot = manager.snapshot(status["capture_id"], status["frame"]["frame_id"])
    status = manager.discard_snapshot(status["capture_id"], snapshot["snapshot_id"])
    assert status["snapshot_count"] == 0 and status["coverage_fraction"] == 0
    assert status["snapshot_eligible"]


def test_capture_session_board_changes_require_stop_and_create_new_epoch(manager):
    first = manager.start(session(), "recorded-id", confirmed=True)
    assert manager.start(session(), "recorded-id", confirmed=True)["capture_id"] == first["capture_id"]
    for modified in (session("session-two"), session(board={"cols": 8, "rows": 5, "square_size_m": .03})):
        with pytest.raises(CameraBusyError):
            manager.start(modified, "recorded-id", confirmed=True)
    status = manager.stop(first["capture_id"])
    assert status["state"] == "stopped" and status["frame"] is None
    assert manager.stop(first["capture_id"])["state"] == "stopped"
    # A fresh reader is needed for restart; the closed old reader is not adopted.
    manager._sources = [recorded_source("recorded-id", "Recorded", lambda _: Reader())]
    second = manager.start(session("session-two"), "recorded-id", confirmed=True)
    assert second["capture_id"] != first["capture_id"] and second["snapshot_count"] == 0
    with pytest.raises(ValueError, match="different"):
        manager.frame(first["capture_id"])


def test_wrong_capture_or_expired_frame_cannot_be_saved(manager):
    manager.start(session(), "recorded-id", confirmed=True)
    status = wait(manager)
    with pytest.raises(ValueError, match="different"):
        manager.snapshot("wrong", status["frame"]["frame_id"])
    for bad in (-1, 10_000, True):
        with pytest.raises(ValueError):
            manager.snapshot(status["capture_id"], bad)
    old = manager._frames[-1]
    old["received_at"] -= 3
    manager._state = "offline"
    with pytest.raises(ValueError, match="stale, offline"):
        manager.snapshot(status["capture_id"], old["frame_id"])


def test_recent_displayed_frame_can_be_saved_when_newer_frame_arrives(manager):
    manager.start(session(), "recorded-id", confirmed=True)
    status = wait(manager)
    first = status["frame"]["frame_id"]
    exact_first = manager.frame(status["capture_id"], first)
    wait(manager, lambda status: status["frame"] is not None and status["frame"]["frame_id"] > first)
    assert manager.frame(status["capture_id"], first) == exact_first
    assert manager.snapshot(status["capture_id"], first)["frame_id"] == first
    with pytest.raises(ValueError, match="expired"):
        manager.frame(status["capture_id"], -1)
    with pytest.raises(ValueError, match="integer"):
        manager.frame(status["capture_id"], True)


def test_interrupted_blocking_reader_cannot_be_reopened_and_stale_frame_clears():
    unblock = threading.Event()
    blocked = threading.Event()

    def block():
        blocked.set()
        unblock.wait(1.5)

    reader = Reader(after_first=block)
    manager = CalibrationCapture([recorded_source("recorded-id", "Recorded", lambda _: reader)],
                                 detector=detected, preview_fps=10, stale_after_s=.1, stop_timeout_s=.01)
    try:
        status = manager.start(session(), "recorded-id", confirmed=True)
        wait(manager)
        assert blocked.wait(.5)
        stale = wait(manager, lambda status: status["state"] == "stale")
        assert stale["frame"] is None and stale["timing"] is None and not stale["snapshot_eligible"]
        with pytest.raises(ValueError, match="stale"):
            manager.frame(status["capture_id"])
        assert manager.stop(status["capture_id"])["state"] == "stopping"
        with pytest.raises(CameraBusyError):
            manager.start(session("other"), "recorded-id", confirmed=True)
    finally:
        unblock.set()
        manager.close()
    assert reader.closed


def test_actual_mode_mismatch_fails_closed_and_releases():
    reader = Reader(shape=(240, 320))
    manager = CalibrationCapture([recorded_source("recorded-id", "Recorded", lambda _: reader)], detector=detected)
    status = manager.start(session(), "recorded-id", confirmed=True)
    status = wait(manager, lambda status: status["state"] == "mode_mismatch")
    assert status["frame"] is None and not status["snapshot_eligible"]
    assert reader.closed
    manager.close()


def test_offline_source_clears_actionable_preview():
    class Ended(Reader):
        def read(self):
            return False, None
    manager = CalibrationCapture([recorded_source("recorded-id", "Recorded", lambda _: Ended())], detector=detected)
    status = manager.start(session(), "recorded-id", confirmed=True)
    status = wait(manager, lambda status: status["state"] == "offline")
    assert status["frame"] is None and status["timing"] is None and not status["connected"]
    assert manager.stop(status["capture_id"])["state"] == "stopped"
    manager.close()


@pytest.mark.parametrize("failed_state", ["offline", "mode_mismatch", "error"])
def test_explicit_stop_finalizes_released_terminal_reader(failed_state):
    class Terminal(Reader):
        def read(self):
            if failed_state == "error":
                raise RuntimeError("Disconnected")
            if failed_state == "offline":
                return False, None
            return True, np.zeros((240, 320), np.uint8)
    reader = Terminal()
    manager = CalibrationCapture([recorded_source("recorded-id", "Recorded", lambda _: reader)], detector=detected)
    status = manager.start(session(), "recorded-id", confirmed=True)
    wait(manager, lambda value: value["state"] == failed_state)
    assert manager.stop(status["capture_id"])["state"] == "stopped"
    assert reader.closed
    manager.close()


def test_commit_can_follow_stop_but_pending_transaction_blocks_session_switch(manager):
    manager.start(session(), "recorded-id", confirmed=True)
    status = wait(manager)
    snapshot = manager.snapshot(status["capture_id"], status["frame"]["frame_id"])
    manager.stop(status["capture_id"])
    with pytest.raises(RuntimeError, match="pending snapshot"):
        manager.start(session("other"), "recorded-id", confirmed=True)
    assert manager.commit_snapshot(status["capture_id"], snapshot["snapshot_id"])["snapshot_count"] == 1


def test_resume_saved_real_corners_preserves_coverage_dedup_and_frame_sequence():
    spec = session()
    spec.update(width=640, height=480, views=[{"corners": detected(None, Board(7, 5, .03))[0].tolist(),
                                            "source_frame_id": 91, "source_time_s": 123.}])
    manager = CalibrationCapture([recorded_source("recorded-id", "Recorded", lambda _: Reader())], detector=detected)
    manager.start(spec, "recorded-id", confirmed=True)
    try:
        status = wait(manager)
        assert status["frame"]["frame_id"] > 91
        assert status["snapshot_count"] == 1 and status["coverage_fraction"] > 0
        assert not status["snapshot_eligible"] and "Already covered" in status["reason"]
    finally:
        manager.close()


def test_existing_session_cannot_switch_to_another_source_without_acquisition():
    called = []
    manager = CalibrationCapture([recorded_source("one", "Recorded", lambda _: called.append(True))])
    spec = session()
    spec["input"] = {"kind": "capture", "source_id": "original"}
    with pytest.raises(ValueError, match="different registered source"):
        manager.start(spec, "one", confirmed=True)
    assert not called
    manager.close()


def test_live_snapshot_honors_session_selection_quality_and_view_limit():
    spec = session()
    spec["selection"] = {"max_views": 1, "min_contrast": 200.}
    manager = CalibrationCapture([recorded_source("recorded-id", "Recorded", lambda _: Reader())], detector=detected)
    manager.start(spec, "recorded-id", confirmed=True)
    try:
        status = wait(manager)
        assert not status["snapshot_eligible"] and "Low board contrast" in status["reason"]
        with pytest.raises(ValueError, match="Low board contrast"):
            manager.snapshot(status["capture_id"], status["frame"]["frame_id"])
    finally:
        manager.close()


def test_synthetic_raster_has_real_detected_corners_and_unannotated_raw():
    reader = SyntheticChessboard(Board(7, 5, .03))
    ok, image = reader.read()
    corners, metrics = detect_board(image, Board(7, 5, .03))
    assert ok and image.shape == (480, 640) and len(corners) == 35
    assert metrics["contrast"] > 40 and metrics["sharpness_px"] < 3
    reader.release()
    assert reader.read() == (False, None)


def test_read_completion_fallback_does_not_invent_hardware_timestamp(manager):
    manager.start(session(), "recorded-id", confirmed=True)
    status = wait(manager)
    timing = status["timing"]
    assert timing["timestamp_source"] == "host_frame_read_complete"
    assert timing["physical_exposure_timestamp"] is False and timing["exposure_duration_ns"] is None
    assert timing["measurement_kind"] == "recorded"
    assert isinstance(timing["capture_monotonic_ns"], int)
    assert "not a live sensor" in timing["note"]


@pytest.mark.parametrize("verified,metadata,expected", [
    (False, {"exposure_start_monotonic_ns": 100, "exposure_duration_ns": 20, "clock_mapping_validated": True, "clock_domain": "host_monotonic"}, "host_frame_read_complete"),
    (True, {"exposure_start_monotonic_ns": 100, "exposure_duration_ns": 20, "clock_mapping_validated": False, "clock_domain": "host_monotonic"}, "host_frame_read_complete"),
    (True, {"exposure_start_monotonic_ns": 100, "exposure_duration_ns": 20, "clock_mapping_validated": True, "clock_domain": "other"}, "host_frame_read_complete"),
    (True, {"exposure_start_monotonic_ns": 200, "exposure_duration_ns": 20, "clock_mapping_validated": True, "clock_domain": "host_monotonic"}, "host_frame_read_complete"),
    (True, {"exposure_start_monotonic_ns": 100, "exposure_duration_ns": 21, "clock_mapping_validated": True, "clock_domain": "host_monotonic", "rolling_shutter_assumption": "rolling_shutter_midpoint_approximation"}, "hardware_exposure_midpoint"),
])
def test_exposure_midpoint_requires_actual_duration_start_and_validated_mapping(verified, metadata, expected):
    # This exercises a trusted fake backend's metadata, never opens a device.
    source = CaptureSource("test", "test", "v4l2", lambda _: None, device="/dev/video-test",
                           exposure_clock_mapping_verified=verified)
    manager = CalibrationCapture([source])
    manager._source = source
    reader = Reader()
    reader.last_timing = metadata
    timing = manager._timing(reader, 150)
    assert timing["timestamp_source"] == expected
    if expected == "hardware_exposure_midpoint":
        assert timing["capture_monotonic_ns"] == 110 and timing["exposure_duration_ns"] == 21
        assert timing["rolling_shutter_assumption"] == "rolling_shutter_midpoint_approximation"


def test_physical_lease_alias_conflict_release_and_foreign_failclosed(tmp_path):
    device = tmp_path / "camera-alias"
    device.symlink_to("/dev/video-lease-test")
    assert physical_device(str(device)) is None  # only explicitly selected /dev inputs are physical
    free = lambda _: {"supported": True, "busy": False, "reason": None}
    lease = acquire_camera("/dev/video-lease-test", "runtime", checker=free, lock_root=tmp_path / "locks")
    try:
        with pytest.raises(CameraBusyError, match="owned"):
            acquire_camera("/dev/video-lease-test", "calibration", checker=free, lock_root=tmp_path / "locks")
    finally:
        lease.release()
    with pytest.raises(CameraBusyError, match="unavailable"):
        acquire_camera("/dev/video-lease-test", "calibration", require_foreign_check=True,
                       checker=lambda _: {"supported": False, "busy": None, "reason": "unavailable"}, lock_root=tmp_path / "locks")
    with pytest.raises(CameraBusyError, match="foreign"):
        acquire_camera("/dev/video-lease-test", "calibration", checker=lambda _: {"supported": True, "busy": True, "reason": "foreign"}, lock_root=tmp_path / "locks")


def test_leased_reader_keeps_ownership_until_release_reports_success(tmp_path):
    lease = acquire_camera("/dev/video-lease-test", "runtime", checker=lambda _: {"supported": True, "busy": False}, lock_root=tmp_path / "locks")
    class Slow:
        complete = False
        def release(self):
            return self.complete
    reader = Slow()
    owned = LeasedCapture(reader, lease)
    assert owned.release() is False and not lease.released
    reader.complete = True
    assert owned.release() is True and lease.released


def test_cooperating_other_process_cannot_acquire_same_physical_device(tmp_path):
    directory = tmp_path / "leases"
    free = lambda _: {"supported": True, "busy": False}
    lease = acquire_camera("/dev/video-other-process-test", "runtime", checker=free, lock_root=directory)
    code = '''import sys
from custom_vision.camera_ownership import acquire_camera, CameraBusyError
try:
 lease=acquire_camera('/dev/video-other-process-test','calibration',checker=lambda _:{'supported':True,'busy':False},lock_root=sys.argv[1])
except CameraBusyError:
 sys.exit(42)
lease.release()
sys.exit(0)
'''
    try:
        result = subprocess.run([sys.executable, "-c", code, str(directory)], capture_output=True, timeout=2)
        assert result.returncode == 42, result.stderr.decode()
    finally:
        lease.release()
    result = subprocess.run([sys.executable, "-c", code, str(directory)], capture_output=True, timeout=2)
    assert result.returncode == 0, result.stderr.decode()


def test_failed_release_keeps_manager_closed_to_new_acquisition():
    class FailedRelease(Reader):
        def read(self):
            return False, None
        def release(self):
            return False
    manager = CalibrationCapture([recorded_source("recorded-id", "Recorded", lambda _: FailedRelease())])
    manager.start(session(), "recorded-id", confirmed=True)
    wait(manager, lambda status: status["state"] == "error")
    with pytest.raises(CameraBusyError, match="not released ownership"):
        manager.start(session("new"), "recorded-id", confirmed=True)
    manager.close()


@pytest.mark.parametrize("views,width,height", [
    ([{"source_frame_id": 1, "corners": [[1, 2]]}], 640, 480),
    ([{"source_frame_id": 1, "corners": detected(None, Board(7, 5, .03))[0].tolist()}], 320, 240),
    ([{"source_frame_id": True, "corners": detected(None, Board(7, 5, .03))[0].tolist()}], 640, 480),
])
def test_invalid_saved_capture_cannot_seed_resume_or_open_reader(views, width, height):
    called = []
    manager = CalibrationCapture([recorded_source("recorded-id", "Recorded", lambda _: called.append(True))])
    spec = session()
    spec.update(views=views, width=width, height=height)
    with pytest.raises(ValueError):
        manager.start(spec, "recorded-id", confirmed=True)
    assert not called
    manager.close()


def test_no_mac_webcam_registration(monkeypatch):
    monkeypatch.setattr("custom_vision.calibration_capture.sys.platform", "darwin")
    monkeypatch.setattr(cv2, "VideoCapture", lambda *args: pytest.fail("No webcam may be opened"))
    with pytest.raises(ValueError, match="Linux V4L2"):
        v4l2_source("camera", "Camera", {"source": 0, "width": 640, "height": 480}, camera={"physical_id": "x"})


def test_runtime_does_not_open_device_when_guided_capture_holds_lease(monkeypatch):
    from custom_vision import app
    monkeypatch.setattr(app, "physical_device", lambda *args: "/dev/video-shared-test")
    monkeypatch.setattr(app, "acquire_camera", lambda *args: (_ for _ in ()).throw(CameraBusyError("owned")))
    monkeypatch.setattr(cv2, "VideoCapture", lambda *args: pytest.fail("Owned camera must not be opened"))
    with pytest.raises(CameraBusyError):
        app.open_camera({"source": "/dev/video-shared-test", "width": 640, "height": 480, "fps": 10})


def test_runtime_releases_lease_when_reader_constructor_fails(monkeypatch):
    from custom_vision import app
    class Lease:
        released = False
        def release(self):
            self.released = True
    lease = Lease()
    monkeypatch.setattr(app, "physical_device", lambda *args: "/dev/video-shared-test")
    monkeypatch.setattr(app, "acquire_camera", lambda *args: lease)
    def fail(*args):
        raise RuntimeError("Unplugged")
    monkeypatch.setattr(cv2, "VideoCapture", fail)
    with pytest.raises(RuntimeError):
        app.open_camera({"source": "/dev/video-shared-test", "width": 640, "height": 480, "fps": 10})
    assert lease.released
