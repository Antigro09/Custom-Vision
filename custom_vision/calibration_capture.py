"""Explicit, bounded chessboard preview and transactional snapshot acquisition.

Sources are registered by the server, never filesystem paths or camera indices
from HTTP. Only the worker owns read/release. Raw lossless images stay separate
from actual detected-corner overlays. No calibration solve or activation occurs.
"""
from __future__ import annotations

from collections import deque
import copy
from dataclasses import asdict, dataclass, field
import math
import sys
import threading
import time
import uuid

import cv2
import numpy as np

from .calibration_session import Board, Selector, detect_board, grid_coverage
from .camera_ownership import acquire_camera, CameraBusyError, physical_device


_MAX_PIXELS = 1024 * 1024
_MODE_KEYS = ("width", "height", "crop", "binning", "focus")


def _mode(mode):
    if not isinstance(mode, dict):
        raise ValueError("Select an explicit camera mode before preview")
    result = {key: copy.deepcopy(mode.get(key)) for key in _MODE_KEYS}
    result["focus"] = result["focus"] or {"kind": "unknown", "value": None, "locked": False}
    for key in ("width", "height"):
        value = result[key]
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 8192:
            raise ValueError("Select explicit integer mode dimensions before preview")
    if result["width"] * result["height"] > _MAX_PIXELS:
        raise ValueError("Guided preview is bounded to one megapixel; select a supported mode")
    return result


def _selection(board, session):
    settings = session.get("selection", {})
    if not isinstance(settings, dict):
        raise ValueError("Session selection settings must be an object")
    values = {}
    for source, destination, default, low, high, integer in (
        ("max_views", "max_views", 180, 1, 180, True),
        ("interval_s", "min_interval", .7, .001, 60, False),
        ("novelty", "novelty", .025, 0, 1, False),
        ("max_sharpness_px", "max_sharpness", 3., .01, 100, False),
        ("min_contrast", "min_contrast", 40., 0, 255, False)):
        value = settings.get(source, default)
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                or not low <= value <= high or (integer and not isinstance(value, int))):
            raise ValueError("Session selection setting is outside supported bounds")
        values[destination] = value
    return Selector(board, **values)


@dataclass(frozen=True)
class CaptureSource:
    source_id: str
    label: str
    kind: str
    factory: object
    camera: dict = field(default_factory=lambda: {"physical_id": None})
    mode: dict = field(default_factory=lambda: {"width": 640, "height": 480})
    device: str | None = None
    available: bool = True
    exposure_clock_mapping_verified: bool = False
    foreign_checker: object = None

    def public(self):
        # Device paths, file paths, factories and clock authority are private.
        return {"source_id": self.source_id, "label": self.label, "kind": self.kind,
                "available": bool(self.available), "synthetic": self.kind == "synthetic",
                "camera": copy.deepcopy(self.camera), "mode": _mode(self.mode)}


class SyntheticChessboard:
    """A changing rasterized chessboard; detection uses the same real SB detector."""
    def __init__(self, board, width=640, height=480):
        self.board, self.width, self.height = board, width, height
        self.index, self.closed = 0, False
        step = 20
        grid = np.indices(((board.rows + 1) * step, (board.cols + 1) * step))
        self.texture = np.where((grid[0] // step + grid[1] // step) % 2 == 0, 15, 245).astype(np.uint8)

    def read(self):
        if self.closed:
            return False, None
        index = self.index
        self.index += 1
        # Movement is synthetic and deliberately varies coverage/scale/tilt.
        angle = .16 * math.sin(index * .18)
        scale = .65 + .13 * math.cos(index * .14)
        aspect = (self.board.cols + 1) / (self.board.rows + 1)
        height = min(self.height * scale, self.width * scale / aspect)
        width = height * aspect
        cx = self.width * (.5 + .13 * math.sin(index * .17))
        cy = self.height * (.5 + .10 * math.sin(index * .23))
        vertices = np.array([[-width/2, -height/2], [width/2, -height/2],
                             [width/2, height/2], [-width/2, height/2]], np.float32)
        vertices[0, 1] += height * .10 * math.sin(index * .11)
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]], np.float32)
        vertices = vertices @ rotation.T + [cx, cy]
        h, w = self.texture.shape
        transform = cv2.getPerspectiveTransform(np.array([[0, 0], [w-1, 0], [w-1, h-1], [0, h-1]], np.float32),
                                               vertices.astype(np.float32))
        image = cv2.warpPerspective(self.texture, transform, (self.width, self.height),
                                    flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=180)
        return True, image

    def release(self):
        self.closed = True
        return True


def synthetic_source(source_id="synthetic_chessboard", *, label="Synthetic chessboard (no camera)", width=640, height=480):
    mode = {"width": width, "height": height, "crop": None, "binning": None,
            "focus": {"kind": "unknown", "value": None, "locked": False}}
    _mode(mode)
    return CaptureSource(source_id, label, "synthetic",
                         lambda session: SyntheticChessboard(Board(**session["board"]), width, height),
                         camera={"physical_id": None}, mode=mode)


def recorded_source(source_id, label, reader_factory, *, camera=None, mode=None):
    """The owner resolves uploaded assets and supplies a trusted reader factory."""
    return CaptureSource(source_id, label, "recorded", reader_factory,
                         camera=camera or {"physical_id": None}, mode=mode or {"width": 640, "height": 480})


def v4l2_source(source_id, label, settings, *, camera):
    """Optional server configuration only. No camera is opened by registration."""
    if sys.platform != "linux" or settings.get("backend", "v4l2") != "v4l2":
        raise ValueError("Guided physical capture requires explicitly configured Linux V4L2")
    device = physical_device(settings.get("source"), "v4l2")
    if device is None:
        raise ValueError("Select a trusted Linux camera device")
    configured = copy.deepcopy(settings)
    mode = _mode(configured)

    def factory(_session):
        # This manager already holds the lease. Do not call app.open_camera here.
        reader = cv2.VideoCapture(device, cv2.CAP_V4L2)
        try:
            if not reader.isOpened():
                raise RuntimeError("Selected camera is unavailable")
            if configured.get("fourcc"):
                reader.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*configured["fourcc"]))
            reader.set(cv2.CAP_PROP_FRAME_WIDTH, mode["width"])
            reader.set(cv2.CAP_PROP_FRAME_HEIGHT, mode["height"])
            reader.set(cv2.CAP_PROP_FPS, configured.get("fps", 10))
            reader.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            return reader
        except Exception:
            reader.release()
            raise

    return CaptureSource(source_id, label, "v4l2", factory, camera=camera, mode=mode, device=device)


class CalibrationCapture:
    def __init__(self, sources=(), *, preview_fps=5, stale_after_s=1.5,
                 clock=time.monotonic, detector=detect_board, stop_timeout_s=1.5):
        if not 0 < preview_fps <= 10 or not .1 <= stale_after_s <= 10 or not 0 <= stop_timeout_s <= 5:
            raise ValueError("Capture preview/stale/stop bounds are invalid")
        self._sources = sources
        self.period, self.stale_after_s = 1 / preview_fps, stale_after_s
        self.clock, self.detector, self.stop_timeout_s = clock, detector, stop_timeout_s
        self.lock = threading.RLock()
        self.closed = False
        self._thread = self._reader = self._lease = None
        self._stop = threading.Event()
        self._capture_id = self._session = self._source = self._selector = None
        self._state, self._reason = "idle", "Select a source and board before starting preview"
        self._frames = deque(maxlen=10)
        self._pending = None
        self._committed = deque(maxlen=10)
        self._signature = None
        self._release_failed = False
        self._initial_frame_id = 0

    def _registered(self):
        sources = self._sources() if callable(self._sources) else self._sources
        sources = list(sources.values()) if isinstance(sources, dict) else list(sources)
        if any(not isinstance(source, CaptureSource) for source in sources):
            raise ValueError("Capture sources must be server-registered CaptureSource records")
        identifiers = [source.source_id for source in sources]
        if len(set(identifiers)) != len(identifiers):
            raise ValueError("Capture source IDs must be unique")
        return sources

    def list_sources(self):
        return [source.public() for source in self._registered()]

    def start(self, session, source_id, *, confirmed=False):
        if confirmed is not True:
            raise ValueError("Explicit confirmation of the selected source is required")
        if not isinstance(session, dict) or not isinstance(session.get("session_id"), str) or not session["session_id"]:
            raise ValueError("Select a calibration session before preview")
        try:
            board = Board(**session.get("board", {}))
        except (TypeError, AttributeError) as exc:
            raise ValueError("Configure board inner-corner dimensions and measured square spacing before preview") from exc
        mode = _mode(session.get("mode"))
        source = next((source for source in self._registered() if source.source_id == source_id), None)
        if source is None or not source.available:
            raise ValueError("Selected capture source is unavailable or unknown")
        if source.kind not in ("synthetic", "recorded", "v4l2") or not callable(source.factory):
            raise ValueError("Capture source has an unsupported kind or factory")
        if source.kind == "v4l2" and (sys.platform != "linux" or physical_device(source.device, "v4l2") is None):
            raise ValueError("Physical preview requires explicitly configured Linux V4L2")
        if mode != _mode(source.mode):
            raise ValueError("Session mode does not match the explicitly selected source")
        camera = session.get("camera", {})
        if not isinstance(camera, dict) or camera.get("physical_id") != source.camera.get("physical_id"):
            raise ValueError("Session camera identity does not match the selected source")
        saved_input = session.get("input", {})
        if saved_input.get("kind") == "capture" and saved_input.get("source_id") != source_id:
            raise ValueError("Existing capture session belongs to a different registered source")
        signature = (session["session_id"], asdict(board), mode, source_id, camera.get("physical_id"),
                     copy.deepcopy(session.get("selection", {})))
        with self.lock:
            if self.closed:
                raise RuntimeError("Calibration capture manager is closed")
            if self._thread and self._thread.is_alive():
                if self._signature == signature and not self._stop.is_set():
                    return self.status()
                raise CameraBusyError("Stop the existing preview before changing source, board or session")
            if self._release_failed or (self._lease is not None and not self._lease.released):
                raise CameraBusyError("Previous camera reader has not released ownership")
            if self._pending is not None:
                raise RuntimeError("Commit or discard the pending snapshot before switching sessions")
            self._capture_id = uuid.uuid4().hex
            self._session, self._source, self._signature = copy.deepcopy(session), source, signature
            self._selector = _selection(board, session)
            self._seed_selection(session, mode)
            self._frames.clear()
            self._committed.clear()
            self._stop = threading.Event()
            self._state, self._reason = "starting", "Opening the explicitly selected source"
            self._thread = threading.Thread(target=self._run, name="calibration-preview", daemon=True)
            self._thread.start()
            return self.status()

    def _seed_selection(self, session, mode):
        """Resume coverage/dedup from saved measured corners, never invented views."""
        self._initial_frame_id = 0
        views = session.get("views", [])
        if not isinstance(views, list) or len(views) > self._selector.max_views:
            raise ValueError("Saved capture views are invalid or exceed the view limit")
        if views and (session.get("width"), session.get("height")) != (mode["width"], mode["height"]):
            raise ValueError("Saved capture dimensions differ from the selected mode")
        for view in views:
            corners = np.asarray(view.get("corners"), np.float64)
            if (corners.shape != (self._selector.board.cols * self._selector.board.rows, 2)
                    or not np.isfinite(corners).all() or np.any(corners < 0)
                    or np.any(corners >= [mode["width"], mode["height"]])):
                raise ValueError("Saved capture has invalid measured corners")
            frame_id = view.get("source_frame_id")
            if isinstance(frame_id, bool) or not isinstance(frame_id, int) or frame_id < 0:
                raise ValueError("Saved capture has invalid source frame identity")
            self._initial_frame_id = max(self._initial_frame_id, frame_id)
            self._selector.accepted.append(corners / [mode["width"], mode["height"]])
            self._selector.coverage |= grid_coverage(corners, mode["width"], mode["height"])
        # Monotonic values can cross a process restart, and media preview time is
        # not original acquisition time. Never reuse a saved time for scheduling.
        self._selector.last_stamp = -math.inf

    def _timing(self, reader, read_ns):
        source = self._source
        result = {"timestamp_source": "host_frame_read_complete", "capture_event": "host_frame_read_complete",
                  "clock_domain": "host_monotonic", "timestamp_unit": "ns",
                  "capture_monotonic_ns": read_ns, "host_read_complete_monotonic_ns": read_ns,
                  "physical_exposure_timestamp": False, "exposure_duration_ns": None,
                  "rolling_shutter_assumption": "unknown", "clock_mapping_validated": False,
                  "measurement_kind": "live" if source.kind == "v4l2" else source.kind,
                  "note": "Host read completion includes transport/decode; exposure time is unmeasured"}
        if source.kind != "v4l2":
            result["note"] = "Synthetic/recorded host preview time is not a live sensor exposure timestamp"
            return result
        metadata = getattr(reader, "last_timing", None)
        if not isinstance(metadata, dict):
            return result
        duration = metadata.get("exposure_duration_ns")
        start = metadata.get("exposure_start_monotonic_ns")
        valid_duration = isinstance(duration, int) and not isinstance(duration, bool) and 0 <= duration <= 60_000_000_000
        if valid_duration:
            result["exposure_duration_ns"] = duration
        rolling = metadata.get("rolling_shutter_assumption")
        if rolling in ("unknown", "global_shutter", "rolling_shutter_midpoint_approximation"):
            result["rolling_shutter_assumption"] = rolling
        if (source.exposure_clock_mapping_verified and metadata.get("clock_mapping_validated") is True
                and metadata.get("clock_domain") == "host_monotonic" and valid_duration
                and isinstance(start, int) and not isinstance(start, bool) and start >= 0
                and start + duration <= read_ns):
            result.update(timestamp_source="hardware_exposure_midpoint", capture_event="exposure_midpoint",
                          capture_monotonic_ns=start + duration // 2, physical_exposure_timestamp=True,
                          clock_mapping_validated=True,
                          note="Backend exposure start/duration mapped to host monotonic by a validated mapping; midpoint rounded down to ns")
        return result

    def _run(self):
        reader = lease = None
        try:
            source = self._source
            if source.kind == "v4l2":
                lease = acquire_camera(source.device, "guided-calibration", require_foreign_check=True,
                                       checker=source.foreign_checker)
                with self.lock:
                    self._lease = lease
            if self._stop.is_set():
                return
            reader = source.factory(copy.deepcopy(self._session))
            with self.lock:
                self._reader = reader
            frame_id = self._initial_frame_id
            while not self._stop.is_set():
                iteration = self.clock()
                ok, raw = reader.read()
                read_ns = time.monotonic_ns()
                received_at = self.clock()
                if self._stop.is_set():
                    break
                if not ok or raw is None:
                    with self.lock:
                        self._state, self._reason = "offline", "Selected source ended or disconnected; snapshots disabled"
                    break
                if (not isinstance(raw, np.ndarray) or raw.dtype != np.uint8 or raw.ndim not in (2, 3)
                        or (raw.ndim == 3 and raw.shape[2] not in (3, 4))):
                    raise ValueError("Source returned an unsupported frame")
                height, width = raw.shape[:2]
                mode = self._session["mode"]
                if (width, height) != (mode["width"], mode["height"]) or width * height > _MAX_PIXELS:
                    with self.lock:
                        self._state, self._reason = "mode_mismatch", "Actual frame dimensions differ from the selected session mode"
                    break
                raw = raw.copy()
                corners, metrics = self.detector(raw, self._selector.board)
                corners = None if corners is None else np.asarray(corners, np.float64).copy()
                overlay = raw.copy() if raw.ndim == 3 else cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
                if overlay.ndim == 3 and overlay.shape[2] == 4:
                    overlay = cv2.cvtColor(overlay, cv2.COLOR_BGRA2BGR)
                if corners is not None:
                    # Green dots are the actual detector coordinates, not an ideal lattice.
                    if corners.shape != (self._selector.board.cols * self._selector.board.rows, 2) or not np.isfinite(corners).all():
                        corners, metrics = None, {"reason": "Detector returned invalid corner data"}
                    else:
                        cv2.drawChessboardCorners(overlay, (self._selector.board.cols, self._selector.board.rows),
                                                  corners.astype(np.float32).reshape(-1, 1, 2), True)
                        for point in corners:
                            cv2.circle(overlay, tuple(np.rint(point).astype(int)), 3, (0, 255, 0), 1, cv2.LINE_AA)
                cv2.putText(overlay, "SYNTHETIC" if source.kind == "synthetic" else ("RECORDED" if source.kind == "recorded" else "CHESSBOARD PREVIEW"),
                            (12, 24), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 0), 2)
                encoded, jpeg = cv2.imencode(".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not encoded:
                    raise RuntimeError("Preview JPEG encoding failed")
                frame_id += 1
                record = {"frame_id": frame_id, "width": width, "height": height, "raw": raw,
                          "overlay": overlay, "jpeg": jpeg.tobytes(), "corners": corners,
                          "metrics": copy.deepcopy(metrics), "received_at": received_at,
                          "timing": self._timing(reader, read_ns)}
                with self.lock:
                    if self._stop.is_set():
                        break
                    self._frames.append(record)
                    self._state, self._reason = "preview", None
                self._stop.wait(max(0, self.period - (self.clock() - iteration)))
        except Exception as exc:
            with self.lock:
                self._state = "error"
                self._reason = (f"Camera ownership refused: {exc}; snapshots disabled" if isinstance(exc, CameraBusyError)
                                else f"Selected source failed ({type(exc).__name__}); snapshots disabled")
        finally:
            released = True
            if reader is not None:
                try:
                    released = reader.release() is not False
                except Exception:
                    released = False
            if lease is not None and released:
                lease.release()
            with self.lock:
                self._reader = None
                if not released:
                    self._release_failed = True
                    self._state, self._reason = "error", "Reader did not release its device; ownership retained"
                elif self._stop.is_set():
                    self._state, self._reason = "stopped", "Preview stopped; saved snapshots retained"

    def _fresh(self):
        return bool(self._state == "preview" and self._frames and
                    0 <= self.clock() - self._frames[-1]["received_at"] <= self.stale_after_s)

    def _eligible(self, record):
        candidate = copy.deepcopy(self._selector)
        accepted, reason = candidate.consider(record["corners"], record["metrics"], record["raw"].shape,
                                               record["received_at"], force=True)
        return accepted, reason, candidate

    def status(self):
        with self.lock:
            fresh = self._fresh()
            state, reason = self._state, self._reason
            if state == "preview" and not fresh:
                state, reason = "stale", "Preview is stale; wait for a new frame before saving"
            frame = None
            eligible = False
            if fresh:
                record = self._frames[-1]
                eligible, reason, _ = self._eligible(record)
                frame = {"frame_id": record["frame_id"], "width": record["width"], "height": record["height"],
                         "corner_count": 0 if record["corners"] is None else len(record["corners"]),
                         "corners_px": [] if record["corners"] is None else record["corners"].tolist(),
                         "metrics": copy.deepcopy(record["metrics"]),
                         "overlay_url": f"/api/calibration/capture/{self._capture_id}/frame",
                         "age_ms": max(0, (self.clock() - record["received_at"]) * 1000)}
            if self._pending:
                eligible, reason = False, "Snapshot save is pending"
            return {"capture_id": self._capture_id, "session_id": self._session.get("session_id") if self._session else None,
                    "source_epoch": self._capture_id, "state": state, "source": self._source.public() if self._source else None,
                    "board": asdict(self._selector.board) if self._selector else None,
                    "frame": frame, "timing": copy.deepcopy(self._frames[-1]["timing"]) if fresh else None,
                    "snapshot_eligible": eligible, "reason": reason, "connected": fresh,
                    "snapshot_count": len(self._selector.accepted) if self._selector else 0,
                    "coverage_grid": self._selector.coverage.astype(int).tolist() if self._selector else [[0] * 8 for _ in range(6)],
                    "coverage_fraction": float(self._selector.coverage.mean()) if self._selector else 0,
                    "ownership": {"cooperating_process_lease": self._lease is not None and not self._lease.released,
                                  "foreign_reader_check": copy.deepcopy(self._lease.external_check) if self._lease else None,
                                  "limitation": "Foreign applications can acquire a nonexclusive UVC device after the check" if self._source and self._source.kind == "v4l2" else None}}

    def _check_capture(self, capture_id):
        if capture_id is None or capture_id != self._capture_id:
            raise ValueError("Preview belongs to a different capture session")

    def frame(self, capture_id, frame_id=None):
        with self.lock:
            self._check_capture(capture_id)
            if not self._fresh():
                raise ValueError("Preview is stale, offline or stopped")
            if frame_id is None:
                record = self._frames[-1]
            else:
                if isinstance(frame_id, bool) or not isinstance(frame_id, int):
                    raise ValueError("Preview needs an integer frame ID")
                record = next((value for value in self._frames if value["frame_id"] == frame_id), None)
                if record is None or not 0 <= self.clock() - record["received_at"] <= 2:
                    raise ValueError("Requested preview frame expired")
            return record["jpeg"], "image/jpeg"

    def snapshot(self, capture_id, frame_id):
        with self.lock:
            self._check_capture(capture_id)
            if isinstance(frame_id, bool) or not isinstance(frame_id, int):
                raise ValueError("Snapshot needs the displayed integer frame ID")
            if not self._fresh():
                raise ValueError("Cannot save a stale, offline or stopped preview")
            if self._pending is not None:
                if self._pending["data"]["frame_id"] == frame_id:
                    return copy.deepcopy(self._pending["data"])
                raise RuntimeError("Commit or discard the pending snapshot first")
            record = next((value for value in self._frames if value["frame_id"] == frame_id), None)
            if record is None or not 0 <= self.clock() - record["received_at"] <= 2:
                raise ValueError("Displayed frame expired; wait for the current preview")
            accepted, reason, selector = self._eligible(record)
            if not accepted:
                raise ValueError(reason)
            raw_ok, raw = cv2.imencode(".png", record["raw"], [cv2.IMWRITE_PNG_COMPRESSION, 1])
            overlay_ok, overlay = cv2.imencode(".png", record["overlay"], [cv2.IMWRITE_PNG_COMPRESSION, 1])
            if not raw_ok or not overlay_ok:
                raise RuntimeError("Lossless snapshot encoding failed")
            data = {"snapshot_id": uuid.uuid4().hex, "capture_id": capture_id, "session_id": self._session["session_id"],
                    "source_epoch": capture_id, "frame_id": record["frame_id"],
                    "width": record["width"], "height": record["height"],
                    "dimensions": [record["width"], record["height"]],
                    "raw_image_bytes": raw.tobytes(), "overlay_image_bytes": overlay.tobytes(),
                    "corners": record["corners"].tolist(), "metrics": copy.deepcopy(record["metrics"]),
                    "timing": copy.deepcopy(record["timing"]), "source": self._source.public(),
                    "board": asdict(self._selector.board), "reason": reason}
            self._pending = {"data": data, "selector": selector}
            return copy.deepcopy(data)

    def commit_snapshot(self, capture_id, snapshot_id):
        with self.lock:
            self._check_capture(capture_id)
            if snapshot_id in self._committed:
                return self.status()
            if self._pending is None or self._pending["data"]["snapshot_id"] != snapshot_id:
                raise ValueError("Unknown pending snapshot")
            self._selector = self._pending["selector"]
            self._pending = None
            self._committed.append(snapshot_id)
            return self.status()

    def discard_snapshot(self, capture_id, snapshot_id):
        with self.lock:
            self._check_capture(capture_id)
            if self._pending and self._pending["data"]["snapshot_id"] == snapshot_id:
                self._pending = None
            elif snapshot_id not in self._committed:
                raise ValueError("Unknown pending snapshot")
            return self.status()

    def stop(self, capture_id):
        with self.lock:
            self._check_capture(capture_id)
            self._stop.set()
            if self._thread and self._thread.is_alive():
                self._state, self._reason = "stopping", "Waiting for the selected reader to release; new acquisition blocked"
            thread = self._thread
        if thread and thread is not threading.current_thread():
            thread.join(self.stop_timeout_s)
        with self.lock:
            if (not thread or not thread.is_alive()) and not self._release_failed and (self._lease is None or self._lease.released):
                self._state, self._reason = "stopped", "Preview stopped; saved snapshots retained"
        return self.status()

    def close(self):
        with self.lock:
            self.closed = True
            capture_id = self._capture_id
        if capture_id is not None:
            self.stop(capture_id)
