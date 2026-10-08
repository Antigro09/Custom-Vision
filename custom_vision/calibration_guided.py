"""Explicit source preview and durable chessboard evidence for the desktop lab.

Sources are registered by the launcher or derived from uploaded local videos.
HTTP clients never supply camera indices, device paths, URLs or executables.
Synthetic/recorded previews do not establish physical exposure timing.
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import uuid
from urllib.parse import quote

import cv2
import numpy as np

from .calibration import validate_calibration
from .calibration_capture import CalibrationCapture, recorded_source
from .calibration_diagnostics import build_board_diagnostics
from .calibration_jobs import CalibrationJobs, _now, _object
from .calibration_review import candidate_review_status, review_candidate
from .calibration_session import grid_coverage, write_json


class GuidedCalibrationJobs(CalibrationJobs):
    def __init__(self, root="data/calibration-guided", *, sources=(), capture=None, **kwargs):
        super().__init__(root, **kwargs)
        self._sources = tuple(sources)
        self._video_sources = {}
        self._importing = False
        self.capture = capture or CalibrationCapture(sources=self._registered_sources)
        # An interrupted preview retains its images, but is never resumed silently.
        for record in self._list("sessions", "record.json"):
            if record["input"]["kind"] == "capture" and record["status"] == "capturing":
                self._finish_record(record)

    def _registered_sources(self):
        result = list(self._sources)
        for asset in self.list_assets():
            if asset["kind"] != "video":
                continue
            identifier = asset["asset_id"]
            if identifier not in self._video_sources:
                saved = self._read("assets", identifier, "asset.json")
                path = self._safe(self._directory("assets", identifier) / saved["stored_file"])
                reader = cv2.VideoCapture(str(path))  # trusted uploaded file, never a camera/URL
                try:
                    width, height = reader.get(cv2.CAP_PROP_FRAME_WIDTH), reader.get(cv2.CAP_PROP_FRAME_HEIGHT)
                    if (not reader.isOpened() or not all(math.isfinite(v) and v.is_integer() and 1 <= v <= 8192 for v in (width, height))
                            or width * height > 1024**2):
                        self._video_sources[identifier] = None
                    else:
                        mode = {"width": int(width), "height": int(height), "crop": None, "binning": None,
                                "focus": {"kind": "unknown", "value": None, "locked": False}}
                        self._video_sources[identifier] = recorded_source("video_" + identifier,
                            label="Recorded · " + asset["file_name"],
                            reader_factory=lambda board, path=path: cv2.VideoCapture(str(path)), mode=mode)
                finally:
                    reader.release()
            if self._video_sources[identifier] is not None:
                result.append(self._video_sources[identifier])
        return result

    def capabilities(self):
        caps = super().capabilities()
        caps.update(live_capture=True, inputs=["images", "video", "capture"],
                    guided_capture={"explicit_source_required": True, "max_pixels": 1024**2,
                                    "max_preview_fps": 5, "timestamp_unit": "ns",
                                    "native_validation_without_measured_time": "image_groups"})
        return caps

    def _validate_input(self, source):
        if source.get("kind") == "import" and self._importing:
            return {"kind": "import", "asset_ids": [], "timestamps_asset_id": None}
        if source.get("kind") != "capture":
            return super()._validate_input(source)
        if source.get("asset_ids", []) != [] or source.get("timestamps_asset_id") is not None:
            raise ValueError("Capture sessions cannot mix uploaded inputs or timestamp sidecars")
        source_id = source.get("source_id")
        if not any(item["source_id"] == source_id and item["available"] for item in self.capture.list_sources()):
            raise ValueError("Explicitly select an available registered capture source")
        return {"kind": "capture", "source_id": source_id, "asset_ids": [], "timestamps_asset_id": None}

    def create_session(self, spec):
        with self.lock:
            record = super().create_session(spec)
            record["capture_revision"] = 0
            if record["input"]["kind"] == "capture":
                source = next(item for item in self.capture.list_sources() if item["source_id"] == record["input"]["source_id"])
                record.update(source_kind=source["kind"], synthetic=source["synthetic"])
                record["warnings"].append("Preview timestamps describe this host read, not recorded sensor exposure or synchronized robot time")
            write_json(self._directory("sessions", record["session_id"]) / "record.json", record)
            return self.get_session(record["session_id"])

    def capture_sources(self):
        with self.lock:
            return {"sources": self.capture.list_sources(), "auto_capture": False}

    def capture_status(self):
        return self.capture.status()

    def capture_start(self, spec):
        spec = _object(spec, "capture start")
        with self.lock:
            if self._active:
                raise RuntimeError("Finish or cancel the solver before acquisition")
            record = self._session_record(spec.get("session_id"))
            if record["input"]["kind"] != "capture" or spec.get("source_id") != record["input"]["source_id"]:
                raise ValueError("Capture session belongs to a different source")
            status = self.capture.start(record, spec["source_id"], confirmed=spec.get("confirmed"))
            record["status"] = "capturing"
            try:
                write_json(self._directory("sessions", record["session_id"]) / "record.json", record)
            except Exception:
                self.capture.stop(status["capture_id"])
                raise
            return status

    def capture_frame(self, capture_id, frame_id=None):
        body, content_type = self.capture.frame(capture_id, frame_id=frame_id)
        return {"body": body, "content_type": content_type, "file_name": "chessboard-preview.jpg"}

    def _saved_data(self, record, *, complete):
        return {"schema_version": 1, "board": record["board"],
                "source": {"kind": record["source_kind"], "source_id": record["input"]["source_id"]},
                "created_utc": record["created_utc"], "width": record["width"], "height": record["height"],
                "views": record["views"], "complete": complete,
                "coverage_grid": record["summary"]["coverage_grid"],
                "corner_cell_coverage_fraction": record["summary"]["coverage_fraction"],
                "camera_identity": record["camera"], "capture_mode": record["mode"],
                "declared_capture_mode": record["declared_mode"], "physical_validation": False,
                "capture_revision": record["capture_revision"],
                "corner_detector_preview": "opencv_findChessboardCornersSB",
                "acquisition": {"timestamp_source": record["summary"]["timestamp_source"],
                                "measured_capture_time_verified": False},
                "extrinsics_reference": "board poses only; robot mount not determined"}

    def _snapshot_public(self, record, view):
        prefix = "/api/calibration/sessions/" + record["session_id"] + "/artifacts/"
        return {**copy.deepcopy(view), "frame_id": view["source_frame_id"],
                "width": record["width"], "height": record["height"], "corners_px": view["corners"],
                "image_url": prefix + quote(view["image"], safe="/"),
                "overlay_url": prefix + quote(view.get("overlay", view["image"]), safe="/")}

    def capture_snapshot(self, spec):
        spec = _object(spec, "snapshot")
        if type(spec.get("frame_id")) is not int or not 0 <= spec["frame_id"] <= 2**53 - 1:
            raise ValueError("Snapshot needs an exact nonnegative integer displayed frame ID")
        with self.lock:
            status = self.capture.status()
            if status["capture_id"] != spec.get("capture_id"):
                raise ValueError("Snapshot belongs to a different capture")
            record = self._session_record(status["session_id"])
            existing = next((v for v in record["views"] if v.get("capture_id") == spec["capture_id"]
                             and v["source_frame_id"] == spec.get("frame_id")), None)
            if existing:
                return {"session": self.get_session(record["session_id"]),
                        "snapshot": self._snapshot_public(record, existing), "capture": status}
            if self._active:
                raise RuntimeError("Cannot change observations while a solver is active")
            saved = self.capture.snapshot(spec.get("capture_id"), spec.get("frame_id"))
            previous = copy.deepcopy(record)
            paths = []
            directory = None
            try:
                if len(record["views"]) >= record["selection"]["max_views"]:
                    raise ValueError("Session view budget reached")
                total = sum(p.stat().st_size for p in (self.root / "sessions").rglob("*") if p.is_file())
                if total + len(saved["raw_image_bytes"]) + len(saved["overlay_image_bytes"]) > self.total_asset_bytes:
                    raise ValueError("Captured image store budget reached")
                if "selected_directory" not in record:
                    directory = self._directory("sessions", record["session_id"]) / "capture"
                    (directory / "frames").mkdir(parents=True, exist_ok=True)
                    (directory / "overlays").mkdir(exist_ok=True)
                    record["selected_directory"] = str(directory.relative_to(self.root))
                directory = self._safe(self.root / record["selected_directory"])
                count = len(record["views"])
                image, overlay = f"frames/frame{count:06d}.png", f"overlays/frame{count:06d}.png"
                for name, body in ((image, saved["raw_image_bytes"]), (overlay, saved["overlay_image_bytes"])):
                    path = self._safe(directory / name)
                    paths.append(path)
                    path.write_bytes(body)
                # Host monotonic read time is real, but recorded/synthetic playback
                # is explicitly not original sensor time or synchronized capture.
                view = {"image": image, "overlay": overlay, "source_frame_id": saved["frame_id"],
                        "source_time_s": saved["timing"]["capture_monotonic_ns"] / 1e9,
                        "corners": saved["corners"], **saved["metrics"], "timing": saved["timing"],
                        "capture_id": saved["capture_id"], "source_epoch": saved["source_epoch"],
                        "snapshot_id": saved["snapshot_id"]}
                record["views"].append(view)
                record.update(width=saved["width"], height=saved["height"], capture_revision=record["capture_revision"] + 1,
                              mode_dimensions_source="decoded")
                coverage = np.zeros((6, 8), bool)
                for value in record["views"]:
                    coverage |= grid_coverage(value["corners"], record["width"], record["height"])
                timing_sources = {value["timing"]["timestamp_source"] for value in record["views"]}
                timestamp_source = (next(iter(timing_sources)) if len(timing_sources) == 1 else "mixed_capture_events")
                record["summary"].update(accepted_views=len(record["views"]), processed_frames=saved["frame_id"],
                    coverage_grid=coverage.astype(int).tolist(), coverage_fraction=float(coverage.mean()),
                    preview_artifact=overlay,
                    timestamp_source=timestamp_source if record["source_kind"] == "v4l2" else "preview_host_read_complete_not_sensor_time")
                write_json(directory / "session.json", self._saved_data(record, complete=False))
                write_json(self._directory("sessions", record["session_id"]) / "record.json", record)
                self.capture.commit_snapshot(saved["capture_id"], saved["snapshot_id"])
            except Exception:
                for path in paths:
                    path.unlink(missing_ok=True)
                self.capture.discard_snapshot(saved["capture_id"], saved["snapshot_id"])
                # Each manifest is atomic. Restore both authoritative revisions;
                # startup also reconciles interrupted captures from record.json.
                write_json(self._directory("sessions", previous["session_id"]) / "record.json", previous)
                if previous.get("selected_directory"):
                    write_json(self._safe(self.root / previous["selected_directory"]) / "session.json",
                               self._saved_data(previous, complete=False))
                elif directory is not None:
                    (directory / "session.json").unlink(missing_ok=True)
                raise
            return {"session": self.get_session(record["session_id"]),
                    "snapshot": self._snapshot_public(record, view), "capture": self.capture.status()}

    def _finish_record(self, record):
        record["status"] = "selected" if record["views"] else "new"
        if record.get("selected_directory"):
            write_json(self._safe(self.root / record["selected_directory"]) / "session.json", self._saved_data(record, complete=True))
        write_json(self._directory("sessions", record["session_id"]) / "record.json", record)

    def capture_stop(self, spec):
        with self.lock:
            status = self.capture.stop(_object(spec, "capture stop").get("capture_id"))
            if status["state"] == "stopped":
                self._finish_record(self._session_record(status["session_id"]))
            return status

    def create_job(self, spec):
        with self.lock:
            status = self.capture.status()
            if status.get("state") not in ("idle", "stopped", "closed") and status.get("capture_id"):
                raise ValueError("Stop the owned preview before solving a saved session")
            record = self._session_record(spec.get("session_id"))
            if record["input"]["kind"] == "capture" and spec.get("operation") == "select":
                raise ValueError("Guided snapshots are already selected; review then solve")
            return super().create_job(spec)

    def _native_validation_basis(self, session):
        # Acquisition from the same board movement is not independent recapture.
        # Image grouping allows intrinsic holdout without fabricating timestamps.
        if session["input"]["kind"] == "capture":
            return "image_groups"
        return super()._native_validation_basis(session)

    def snapshots(self, identifier):
        with self.lock:
            record = self._session_record(identifier)
            return {"snapshots": [self._snapshot_public(record, view) for view in record["views"]],
                    "coverage_grid": record["summary"]["coverage_grid"],
                    "coverage_fraction": record["summary"]["coverage_fraction"],
                    "mosaic_url": "/api/calibration/sessions/" + identifier + "/mosaic"}

    def mosaic(self, identifier):
        with self.lock:
            snapshots = self.snapshots(identifier)["snapshots"]
            if not snapshots:
                raise ValueError("Session has no saved snapshots")
            cols, tile_w, tile_h = min(6, len(snapshots)), 192, 164
            image = np.full((math.ceil(len(snapshots) / cols) * tile_h, cols * tile_w, 3), 20, np.uint8)
            for index, value in enumerate(snapshots):
                body = self.get_session_artifact(identifier, value.get("overlay", value["image"]))["body"]
                frame = cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_COLOR)
                if frame is None or frame.shape[0] * frame.shape[1] > 16 * 1024**2:
                    raise ValueError("Invalid saved mosaic image")
                # Imported offline selections get an actual measured-corner overlay.
                if not value.get("overlay"):
                    for x, y in value["corners"]:
                        cv2.circle(frame, (round(x), round(y)), 3, (30, 255, 60), -1)
                scale = min(tile_w / frame.shape[1], (tile_h - 20) / frame.shape[0])
                frame = cv2.resize(frame, (max(1, round(frame.shape[1] * scale)), max(1, round(frame.shape[0] * scale))))
                x, y = index % cols * tile_w, index // cols * tile_h
                image[y:y+frame.shape[0], x:x+frame.shape[1]] = frame
                cv2.putText(image, f"Frame {value['frame_id']}", (x + 5, y + tile_h - 5), cv2.FONT_HERSHEY_SIMPLEX, .4, (230, 230, 230), 1)
            ok, encoded = cv2.imencode(".png", image)
            if not ok:
                raise OSError("Mosaic encoding failed")
            return {"body": encoded.tobytes(), "content_type": "image/png", "file_name": "chessboard-mosaic.png"}

    def candidate_metadata(self, identifier):
        with self.lock:
            candidate = super().get_candidate(identifier)
            record = self._session_record(candidate["session_id"])
            return {key: copy.deepcopy(candidate.get(key, record.get(key))) for key in
                    ("camera", "mode", "quality_status", "report", "review", "source_kind", "synthetic")} | {
                    "capture_revision": candidate.get("capture_revision"),
                    "stale": candidate.get("capture_revision") is not None and candidate["capture_revision"] != record.get("capture_revision")}

    def get_candidate(self, identifier):
        candidate = super().get_candidate(identifier)
        metadata = self.candidate_metadata(identifier)
        status = candidate_review_status(candidate["calibration"], metadata)
        return {**candidate, "stale": metadata["stale"], "reviewed": status["reviewed"], "review_status": status}

    def review(self, identifier, spec):
        with self.lock:
            candidate = self._read("candidates", identifier, "candidate.json")
            checked = review_candidate(self.candidate_data(identifier), self.candidate_metadata(identifier),
                                       confirmed=_object(spec, "review").get("confirmed"))
            candidate["review"] = checked["review"]
            write_json(self._directory("candidates", identifier) / "candidate.json", candidate)
            return self.get_candidate(identifier)

    def import_candidate(self, spec):
        spec = _object(spec, "candidate import")
        calibration = validate_calibration(copy.deepcopy(spec.get("calibration")))
        # Never inherit a claimed physical acknowledgement from an imported file.
        calibration.update(calibration_verified=False, robot_mount_calibrated=False)
        quality = spec.get("quality_status", calibration.get("quality_status"))
        if quality is not None:
            if not isinstance(quality, str) or len(quality) > 128:
                raise ValueError("Invalid imported quality status")
            if calibration.get("quality_status") not in (None, quality):
                raise ValueError("Imported quality statuses disagree")
            calibration["quality_status"] = quality
        report = _object(spec.get("report") or {"status": quality, "issues": [], "warnings": ["Imported result; physical verification unavailable"]}, "report")
        provenance = calibration.get("calibration_provenance", {})
        synthetic = any(item.get("synthetic") is True or item.get("source_kind") == "synthetic"
                        for item in (spec, calibration, report, provenance) if isinstance(item, dict))
        calibration["synthetic"] = synthetic
        report = copy.deepcopy(report)
        report["synthetic"] = synthetic
        json.dumps({"calibration": calibration, "report": report}, allow_nan=False)
        if report.get("status") not in (None, quality):
            raise ValueError("Imported report quality disagrees")
        mode = _object(spec.get("mode"), "mode")
        if (mode.get("width"), mode.get("height")) != (calibration["width"], calibration["height"]):
            raise ValueError("Imported calibration dimensions do not match declared mode")
        with self.lock:
            self._importing = True
            try:
                session = self.create_session({"board": {"cols": 7, "rows": 5, "square_size_m": .025},
                    "camera": spec.get("camera"), "mode": mode, "input": {"kind": "import"}})
            finally:
                self._importing = False
            # Importing a lens model supplies no measured chessboard observations.
            session_record = self._session_record(session["session_id"])
            session_record.update(board=None, board_provenance="unknown", source_kind="import", synthetic=synthetic)
            write_json(self._directory("sessions", session["session_id"]) / "record.json", session_record)
            identifier, job_id = uuid.uuid4().hex, uuid.uuid4().hex
            directory = self._directory("candidates", identifier)
            directory.mkdir()
            job_dir = self._directory("jobs", job_id)
            job_dir.mkdir()
            self._save_job({"job_id": job_id, "session_id": session["session_id"], "operation": "import",
                "solver": "imported", "created_utc": _now(), "status": "completed", "stage": "imported", "revision": 0,
                "result": {"candidate_id": identifier}, "error": None})
            write_json(directory / "intrinsics.candidate.json", calibration)
            write_json(directory / "report.json", report)
            artifacts = [{"name": name, "size_bytes": (directory / name).stat().st_size, "content_type": "application/json"}
                         for name in ("intrinsics.candidate.json", "report.json")]
            record = {"candidate_id": identifier, "session_id": session["session_id"], "job_id": job_id,
                "created_utc": _now(), "solver": "imported", "quality_status": quality, "camera": session["camera"],
                "mode": session["mode"], "calibration": calibration, "report": report, "artifacts": artifacts,
                "directory": str(directory.relative_to(self.root)), "source_kind": "import", "synthetic": synthetic,
                "capture_revision": None}
            write_json(directory / "candidate.json", record)
            return self.get_candidate(identifier)

    def diagnostics(self, identifier):
        with self.lock:
            candidate = self.get_candidate(identifier)
            record = self._session_record(candidate["session_id"])
            if record.get("board") is None:
                return {"schema_version": 1, "status": "unavailable", "frame": "camera_optical", "units": "m",
                        "board": None, "frames": [], "warnings": ["Imported lens model has no measured board observations or poses"]}
            artifact = next((item["name"] for item in candidate["artifacts"] if item["name"] == "board-poses.json"), None)
            poses = json.loads(self.get_artifact(identifier, artifact)["body"]) if artifact else None
            if not record.get("width") or not record.get("height"):
                record.update(width=candidate["calibration"]["width"], height=candidate["calibration"]["height"])
            return build_board_diagnostics(record, candidate["calibration"], poses)

    def close(self):
        self.capture.close()
        super().close()
