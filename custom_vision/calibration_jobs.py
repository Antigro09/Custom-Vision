"""Local, camera-free calibration jobs with retained sessions and candidates.

The HTTP owner supplies its existing same-origin/token checks. This manager never
activates configuration, probes cameras, installs tools, or accepts executable
names or filesystem paths from requests. Media and worker artifacts live below a
server-configured ignored directory. One child process group owns all CPU work.
"""
from __future__ import annotations

import base64
import binascii
import copy
from datetime import datetime, timezone
import hashlib
import json
import math
import mimetypes
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
import uuid

from .calibration_session import Board, write_json

_ID = re.compile(r"[0-9a-f]{32}\Z")
_TERMINAL = {"completed", "canceled", "failed", "unavailable"}
_KINDS = {"image": (12 * 1024**2, {".png", ".jpg", ".jpeg", ".bmp", ".webp"}),
          "video": (32 * 1024**2, {".avi", ".mp4", ".mkv", ".mov", ".webm"}),
          "timestamps": (2 * 1024**2, {".jsonl"})}
_PROBE = '''import importlib,json,shutil,sys
result={"python":sys.version.split()[0],"mrcal_cli":shutil.which("mrcal-calibrate-cameras"),"mrgingham":shutil.which("mrgingham")}
for key,name in (("opencv","cv2"),("numpy","numpy"),("mrcal","mrcal"),("scipy","scipy")):
 try:
  module=importlib.import_module(name)
  result[key]=str(getattr(module,"__version__","unknown"))
  result[key+"_importable"]=True
 except Exception as exc:
  result[key]=None
  result[key+"_importable"]=False
  result[key+"_import_error"]=type(exc).__name__
print(json.dumps(result))
'''


def _now():
    return datetime.now(timezone.utc).isoformat()


def _number(value, name, low, high, integer=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not low <= value <= high
            or (integer and not isinstance(value, int))):
        raise ValueError(f"{name} must be {'an integer' if integer else 'finite'} in [{low}, {high}]")
    return value


def _object(value, name):
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be an object")
    return value


def _text(value, name, nullable=False, limit=128):
    if nullable and value is None:
        return None
    if not isinstance(value, str) or not 1 <= len(value) <= limit or any(ord(c) < 32 for c in value):
        raise ValueError(f"{name} must be a nonempty string of at most {limit} characters")
    return value


class ProcessAdapter:
    """Own one process group, never a broad process-name kill or shell command."""
    def start(self, argv, *, cwd, log, env):
        return subprocess.Popen(argv, cwd=cwd, stdout=log, stderr=subprocess.STDOUT,
                                env=env, start_new_session=True)

    def terminate(self, process, *, force=False):
        if process.poll() is not None and not (force and os.name == "posix"):
            return
        try:
            if os.name == "posix":
                os.killpg(process.pid, signal.SIGKILL if force else signal.SIGTERM)
            elif force:
                process.kill()
            else:
                process.terminate()
        except ProcessLookupError:
            pass


class CalibrationJobs:
    def __init__(self, root="data/calibration-browser", *, python=None,
                 solver_python=None, process_adapter=None, total_asset_bytes=256 * 1024**2):
        self.root = Path(root).resolve()
        # Following the venv executable symlink changes sys.prefix and loses its
        # dependencies. Preserve the configured invocation path exactly.
        self.python = os.path.abspath(os.fspath(python or sys.executable))
        self.solver_python = os.path.abspath(os.fspath(solver_python or self.python))
        self.adapter = process_adapter or ProcessAdapter()
        self.total_asset_bytes = int(_number(total_asset_bytes, "asset store limit", 1, 256 * 1024**2, True))
        self.lock = threading.RLock()
        self.closed = False
        self._active = None
        self._process = None
        self._thread = None
        self._caps = None
        for folder in ("assets", "sessions", "jobs", "candidates"):
            (self.root / folder).mkdir(parents=True, exist_ok=True)
        # A restarted desktop process cannot adopt an unknown old process group.
        # Retain data and mark interrupted jobs terminal, without killing others.
        for job in self._list("jobs", "job.json"):
            if job.get("status") not in _TERMINAL:
                job.update(status="failed", stage="interrupted", error="Desktop service restarted; saved data retained")
                self._save_job(job)

    def _safe(self, path):
        path = Path(path).resolve()
        if not path.is_relative_to(self.root):
            raise ValueError("Artifact path is outside the local calibration store")
        return path

    def _directory(self, kind, identifier):
        if not isinstance(identifier, str) or not _ID.fullmatch(identifier):
            raise ValueError(f"Invalid {kind} ID")
        return self._safe(self.root / kind / identifier)

    def _read(self, kind, identifier, filename):
        path = self._safe(self._directory(kind, identifier) / filename)
        if not path.is_file():
            raise ValueError(f"Unknown {kind} ID")
        return json.loads(path.read_text(encoding="utf-8"))

    def _list(self, kind, filename):
        result = []
        for path in sorted((self.root / kind).glob(f"*/{filename}")):
            if _ID.fullmatch(path.parent.name):
                result.append(json.loads(self._safe(path).read_text(encoding="utf-8")))
        return result

    def capabilities(self):
        with self.lock:
            if self._caps is None:
                probes = []
                env = self._environment()
                for executable in (self.python, self.solver_python):
                    try:
                        completed = subprocess.run([executable, "-c", _PROBE], capture_output=True,
                                                   text=True, timeout=5, env=env, check=False)
                        probes.append(json.loads(completed.stdout) if completed.returncode == 0 else {})
                    except (OSError, ValueError, subprocess.TimeoutExpired):
                        probes.append({})
                local, solver = probes
                mrcal = bool(solver.get("mrcal_importable") and solver.get("scipy_importable")
                             and solver.get("opencv_importable") and solver.get("numpy_importable") and solver.get("mrcal_cli"))
                opencv = bool(local.get("opencv_importable") and local.get("numpy_importable"))
                self._caps = {"schema_version": 1,
                    "versions": {key: local.get(key) for key in ("python", "opencv", "numpy", "mrcal", "scipy")},
                    "solver_versions": {key: solver.get(key) for key in ("python", "opencv", "numpy", "mrcal", "scipy")},
                    "tools": {"mrcal_calibrate_cameras_available": bool(solver.get("mrcal_cli")),
                              "mrgingham_available": bool(solver.get("mrgingham"))},
                    "solvers": {"opencv": {"available": opencv, "kind": "baseline", "holdout": False,
                                            "uncertainty": False},
                                "mrcal": {"available": mrcal,
                                          "reason": None if mrcal else "Configured isolated solver lacks mrcal, SciPy, OpenCV or its CLI",
                                          "opencv8": mrcal, "spline": mrcal, "holdout": mrcal, "uncertainty": mrcal}},
                    "inputs": ["images", "video"], "live_capture": False,
                    "limits": {"image_bytes": _KINDS["image"][0], "video_bytes": _KINDS["video"][0],
                               "timestamps_bytes": _KINDS["timestamps"][0], "max_images": 100,
                               "max_views": 180, "total_asset_bytes": self.total_asset_bytes}}
            return copy.deepcopy(self._caps)

    @staticmethod
    def _environment():
        env = dict(os.environ)
        for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
            env[key] = "1"
        env["PYTHONUNBUFFERED"] = "1"
        env["CUDA_VISIBLE_DEVICES"] = ""
        return env

    def add_asset(self, spec):
        spec = _object(spec, "asset")
        kind = spec.get("kind")
        if kind not in _KINDS:
            raise ValueError("Asset kind must be image, video or timestamps")
        name = _text(spec.get("file_name"), "file_name")
        if "/" in name or "\\" in name or Path(name).suffix.lower() not in _KINDS[kind][1]:
            raise ValueError("Use a basename with a supported file extension")
        encoded = spec.get("data_base64")
        limit = _KINDS[kind][0]
        if not isinstance(encoded, str) or not 0 < len(encoded) <= 4 * ((limit + 2) // 3):
            raise ValueError("Asset exceeds its encoded byte limit or has no data")
        try:
            body = base64.b64decode(encoded, validate=True)
        except (ValueError, binascii.Error) as exc:
            raise ValueError("Asset data must be valid base64") from exc
        if not 0 < len(body) <= limit:
            raise ValueError("Asset exceeds its decoded byte limit or has no data")
        with self.lock:
            if self.closed:
                raise RuntimeError("Calibration workspace is closed")
            assets = self.list_assets()
            if (sum(a["size_bytes"] for a in assets) + len(body) > self.total_asset_bytes
                    or len(assets) >= 1000 or (kind == "image" and sum(a["kind"] == "image" for a in assets) >= 100)):
                raise ValueError("Local asset store limit reached")
            identifier = uuid.uuid4().hex
            directory = self._directory("assets", identifier)
            directory.mkdir()
            filename = "content" + Path(name).suffix.lower()
            (directory / filename).write_bytes(body)
            record = {"asset_id": identifier, "file_name": name, "kind": kind,
                      "size_bytes": len(body), "sha256": hashlib.sha256(body).hexdigest(),
                      "stored_file": filename}
            write_json(directory / "asset.json", record)
            return {key: value for key, value in record.items() if key != "stored_file"}

    def list_assets(self):
        with self.lock:
            return [{key: value for key, value in record.items() if key != "stored_file"}
                    for record in self._list("assets", "asset.json")]

    def create_session(self, spec):
        spec = _object(spec, "session")
        board = _object(spec.get("board"), "board")
        Board(board.get("cols"), board.get("rows"), _number(board.get("square_size_m"), "square size", 1e-6, .999999))
        board = {key: board[key] for key in ("cols", "rows", "square_size_m")}
        camera = _object(spec.get("camera", {}), "camera")
        checked_camera = {"physical_id": _text(camera.get("physical_id"), "physical_id", True),
                          "user_label": _text(camera.get("user_label", "Offline camera"), "user_label"),
                          "identity_source": camera.get("identity_source", "manual"),
                          "capture_history": camera.get("capture_history")}
        if checked_camera["identity_source"] not in ("manual", "offline_metadata"):
            raise ValueError("Camera identity_source must be manual or offline_metadata")
        if checked_camera["capture_history"] is not None:
            checked_camera["capture_history"] = _text(checked_camera["capture_history"], "capture_history", limit=2048)
        mode = copy.deepcopy(_object(spec.get("mode", {}), "mode"))
        checked_mode = {"width": mode.get("width"), "height": mode.get("height"),
                        "crop": mode.get("crop"), "binning": mode.get("binning"),
                        "focus": mode.get("focus", {"kind": "unknown", "value": None, "locked": False})}
        if (checked_mode["width"] is None) != (checked_mode["height"] is None):
            raise ValueError("Mode width and height must both be supplied or both null")
        if checked_mode["width"] is not None:
            for key in ("width", "height"):
                _number(checked_mode[key], key, 1, 8192, True)
        if checked_mode["crop"] is not None:
            crop = _object(checked_mode["crop"], "crop")
            if set(crop) != {"x", "y", "width", "height"}:
                raise ValueError("Crop needs x, y, width, height")
            for key in crop:
                _number(crop[key], "crop." + key, 0 if key in ("x", "y") else 1, 8192, True)
        if checked_mode["binning"] is not None:
            values = checked_mode["binning"]
            if not isinstance(values, list) or len(values) != 2:
                raise ValueError("Binning needs [x, y]")
            for value in values:
                _number(value, "binning", 1, 16, True)
        focus = _object(checked_mode["focus"], "focus")
        if focus.get("kind") not in ("fixed", "manual", "unknown") or not isinstance(focus.get("locked", False), bool):
            raise ValueError("Focus kind must be fixed, manual or unknown and locked must be boolean")
        if focus.get("value") is not None:
            if focus.get("kind") != "manual":
                raise ValueError("Only manual focus may declare a numeric focus value")
            _number(focus["value"], "focus.value", -1e6, 1e6)
        checked_mode["focus"] = {"kind": focus["kind"], "value": focus.get("value"),
                                 "locked": focus.get("locked", False)}
        source = copy.deepcopy(_object(spec.get("input"), "input"))
        kind, identifiers = source.get("kind"), source.get("asset_ids")
        if kind not in ("images", "video") or not isinstance(identifiers, list) or not 1 <= len(identifiers) <= (100 if kind == "images" else 1):
            raise ValueError("Input requires 1..100 image assets or one video asset")
        for identifier in identifiers:
            asset = self._read("assets", identifier, "asset.json")
            if asset["kind"] != ("image" if kind == "images" else "video"):
                raise ValueError("Asset kind does not match session input")
        if len(set(identifiers)) != len(identifiers):
            raise ValueError("Input asset IDs must be unique")
        timestamps = source.get("timestamps_asset_id")
        if timestamps is not None and self._read("assets", timestamps, "asset.json")["kind"] != "timestamps":
            raise ValueError("Timestamp input must be a timestamps asset")
        selection = _object(spec.get("selection", {}), "selection")
        defaults = {"max_views": 180, "interval_s": .7, "novelty": .025,
                    "max_sharpness_px": 3., "min_contrast": 40., "max_frames": 10000}
        if set(selection) - set(defaults):
            raise ValueError("Unknown selection setting")
        selection = dict(defaults, **selection)
        bounds = {"max_views": (1, 180, True), "interval_s": (.001, 60, False),
                  "novelty": (0, 1, False), "max_sharpness_px": (.01, 100, False),
                  "min_contrast": (0, 255, False), "max_frames": (1, 10000, True)}
        for key, (low, high, integer) in bounds.items():
            _number(selection[key], key, low, high, integer)
        warnings = ["Camera identity, crop, binning and focus are user declarations; physical matching is unverified",
                    "Intrinsic selection does not measure robot mount or capture timing correction"]
        if not checked_camera["capture_history"]:
            warnings.append("No OBS/recording capture history supplied; scaling, cropping and re-encoding history is unknown")
        if timestamps is None:
            warnings.append("No measured timestamp sidecar; temporal holdout is unavailable")
        with self.lock:
            if self.closed:
                raise RuntimeError("Calibration workspace is closed")
            identifier = uuid.uuid4().hex
            directory = self._directory("sessions", identifier)
            directory.mkdir()
            record = {"session_id": identifier, "created_utc": _now(), "status": "new", "board": copy.deepcopy(board),
                      "camera": checked_camera, "mode": checked_mode,
                      "declared_mode": copy.deepcopy(checked_mode),
                      "mode_dimensions_source": "unknown" if checked_mode["width"] is None else "declared",
                      "input": {"kind": kind, "asset_ids": identifiers, "timestamps_asset_id": timestamps},
                      "selection": selection, "width": None, "height": None, "warnings": warnings,
                      "summary": {"accepted_views": 0, "processed_frames": 0, "rejected_views": 0,
                                  "coverage_fraction": 0., "coverage_grid": [[0] * 8 for _ in range(6)],
                                  "rejections_by_reason": {}, "timestamp_source": None, "preview_artifact": None},
                      "views": []}
            write_json(directory / "record.json", record)
            return copy.deepcopy(record)

    def _session_record(self, identifier):
        record = self._read("sessions", identifier, "record.json")
        # Older desktop records retained only an effective mode. Its dimensions
        # cannot retrospectively be called the original user declaration.
        if "declared_mode" not in record:
            record["declared_mode"] = copy.deepcopy(record["mode"])
            record["declared_mode"].update(width=None, height=None)
        record.setdefault("mode_dimensions_source", "unknown")
        return record

    def get_session(self, identifier):
        with self.lock:
            record = self._session_record(identifier)
            result = {key: value for key, value in record.items() if key != "selected_directory"}
            jobs = [job for job in self._list("jobs", "job.json") if job["session_id"] == identifier]
            jobs.sort(key=lambda job: job["created_utc"])
            result["jobs"] = [{key: job.get(key) for key in ("job_id", "status", "operation", "solver", "created_utc", "stage", "error")}
                              for job in jobs]
            candidates = [candidate for candidate in self._list("candidates", "candidate.json")
                          if candidate["session_id"] == identifier
                          and any(job["job_id"] == candidate["job_id"] and job["status"] == "completed" for job in jobs)]
            candidates.sort(key=lambda candidate: candidate["created_utc"])
            result["candidate_ids"] = [candidate["candidate_id"] for candidate in candidates]
            result["latest_job_id"] = jobs[-1]["job_id"] if jobs else None
            result["latest_candidate_id"] = candidates[-1]["candidate_id"] if candidates else None
            return result

    def list_sessions(self):
        with self.lock:
            return [self.get_session(record["session_id"]) for record in self._list("sessions", "record.json")]

    def _save_job(self, job):
        job["revision"] = job.get("revision", 0) + 1
        write_json(self._directory("jobs", job["job_id"]) / "job.json", job)

    def create_job(self, spec):
        spec = _object(spec, "job")
        operation = spec.get("operation")
        if operation not in ("select", "solve"):
            raise ValueError("Job operation must be select or solve")
        solver = spec.get("solver", "opencv")
        if solver not in ("opencv", "mrcal"):
            raise ValueError("Solver must be opencv or mrcal")
        options = dict(_object(spec.get("options", {}), "options"))
        defaults = {"min_views": 10 if solver == "opencv" else 40, "fov_deg": 81.,
                    "corner_detector": "auto", "no_spline": False,
                    "max_validation_rms": .6, "max_rms_px": 1., "timeout_s": 120.}
        if set(options) - set(defaults):
            raise ValueError("Unknown job option; executables and paths are server configuration only")
        options = dict(defaults, **options)
        for key, low, high, integer in (("min_views", 3 if solver == "opencv" else 10, 180, True),
                                      ("fov_deg", 20, 170, False), ("max_validation_rms", .001, 10, False),
                                      ("max_rms_px", .001, 10, False), ("timeout_s", .1, 600, False)):
            _number(options[key], key, low, high, integer)
        if options["corner_detector"] not in ("auto", "opencv-sb", "mrgingham") or not isinstance(options["no_spline"], bool):
            raise ValueError("Invalid corner detector or no_spline option")
        with self.lock:
            if self.closed:
                raise RuntimeError("Calibration workspace is closed")
            if self._active is not None:
                raise RuntimeError("One calibration job is already active")
            session = self._session_record(spec.get("session_id"))
            if operation == "solve" and session["status"] != "selected":
                raise ValueError("Select a complete saved session before solving")
            caps = self.capabilities()
            reason = None
            required = "opencv" if operation == "select" else solver
            if not caps["solvers"][required]["available"]:
                reason = caps["solvers"][required].get("reason", "Configured OpenCV worker is unavailable")
            if operation == "solve" and solver == "mrcal" and session["summary"]["timestamp_source"] != "recorded_host_read_complete":
                reason = "mrcal temporal validation requires an aligned recorded host timestamp sidecar; image order/FPS is not measured capture time"
            if operation == "solve" and solver == "mrcal" and options["corner_detector"] == "mrgingham" and not caps["tools"]["mrgingham_available"]:
                reason = "Configured mrgingham command is unavailable"
            identifier = uuid.uuid4().hex
            directory = self._directory("jobs", identifier)
            directory.mkdir()
            job = {"job_id": identifier, "session_id": session["session_id"], "operation": operation,
                   "solver": solver, "options": options, "created_utc": _now(), "status": "unavailable" if reason else "queued",
                   "stage": "unavailable" if reason else "queued", "revision": 0, "cancel_requested": False,
                   "progress": {"processed_frames": 0, "total_frames": None, "accepted_views": 0,
                                "rejected_views": 0, "coverage_fraction": 0., "last_reason": None},
                   "capability_snapshot": caps, "result": None, "error": reason}
            self._save_job(job)
            if reason:
                return copy.deepcopy(job)
            request = {"root": str(self.root), "job_id": identifier, "session": session,
                       "operation": operation, "solver": solver, "options": options}
            for asset_id in session["input"]["asset_ids"] + ([session["input"]["timestamps_asset_id"]] if session["input"]["timestamps_asset_id"] else []):
                asset = self._read("assets", asset_id, "asset.json")
                request.setdefault("assets", {})[asset_id] = str(self._safe(self._directory("assets", asset_id) / asset["stored_file"]))
            write_json(directory / "request.json", request)
            self._active = identifier
            self._thread = threading.Thread(target=self._run, args=(identifier,), name="calibration-job", daemon=True)
            self._thread.start()
            return copy.deepcopy(job)

    def get_job(self, identifier):
        with self.lock:
            return copy.deepcopy(self._read("jobs", identifier, "job.json"))

    def list_jobs(self):
        with self.lock:
            return self._list("jobs", "job.json")

    def cancel_job(self, identifier):
        with self.lock:
            job = self._read("jobs", identifier, "job.json")
            if job["status"] not in _TERMINAL:
                job["cancel_requested"] = True
                self._save_job(job)
            return copy.deepcopy(job)

    def _run(self, identifier):
        directory = self._directory("jobs", identifier)
        process = None
        try:
            with self.lock:
                job = self._read("jobs", identifier, "job.json")
                if job["cancel_requested"]:
                    job.update(status="canceled", stage="canceled")
                    self._save_job(job)
                    return
                job.update(status="running", stage=job["operation"])
                self._save_job(job)
            interpreter = self.solver_python if job["operation"] == "solve" and job["solver"] == "mrcal" else self.python
            argv = [interpreter, "-m", "custom_vision.calibration_browser_worker", "--request", str(directory / "request.json")]
            with (directory / "worker.log").open("w") as log:
                process = self.adapter.start(argv, cwd=str(Path(__file__).resolve().parents[1]), log=log, env=self._environment())
                with self.lock:
                    self._process = process
                deadline = time.monotonic() + job["options"]["timeout_s"]
                termination_at = None
                timeout = False
                while process.poll() is None:
                    with self.lock:
                        job = self._read("jobs", identifier, "job.json")
                        canceled = job["cancel_requested"]
                        progress_path = directory / "progress.json"
                        if progress_path.is_file():
                            progress = json.loads(progress_path.read_text())
                            if progress.get("progress") != job["progress"] or progress.get("stage") != job["stage"]:
                                job.update(progress=progress.get("progress", job["progress"]), stage=progress.get("stage", job["stage"]))
                                self._save_job(job)
                    timeout = timeout or time.monotonic() >= deadline
                    if (canceled or timeout) and termination_at is None:
                        self.adapter.terminate(process)
                        termination_at = time.monotonic()
                    elif termination_at is not None and time.monotonic() - termination_at >= 1.:
                        self.adapter.terminate(process, force=True)
                    time.sleep(.05)
                if termination_at is not None:
                    # The leader can exit before a native grandchild. Its owned
                    # group still receives a final kill, even after leader exit.
                    self.adapter.terminate(process, force=True)
                with self.lock:
                    job = self._read("jobs", identifier, "job.json")
                    if job["cancel_requested"]:
                        job.update(status="canceled", stage="canceled", error=None)
                    elif timeout:
                        job.update(status="failed", stage="timeout", error="Job exceeded its wall-time budget; saved data retained")
                    elif process.returncode != 0:
                        outcome = json.loads((directory / "result.json").read_text()) if (directory / "result.json").is_file() else {}
                        job.update(status="failed", stage="failed", error=outcome.get("error", f"Worker exited {process.returncode}; inspect local worker log"))
                    else:
                        outcome = json.loads((directory / "result.json").read_text())
                        job.update(status="completed", stage="completed", progress=outcome["progress"], result=outcome["result"])
                    self._active = None
                    self._process = None
                    self._save_job(job)
        except Exception as exc:
            if process is not None and process.poll() is None:
                self.adapter.terminate(process, force=True)
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    pass
            with self.lock:
                job = self._read("jobs", identifier, "job.json")
                job.update(status="canceled" if job["cancel_requested"] else "failed", stage="failed", error=str(exc))
                self._active = None
                self._process = None
                self._save_job(job)
        finally:
            with self.lock:
                if self._active == identifier:
                    self._active = None
                    self._process = None

    def list_candidates(self):
        with self.lock:
            result = []
            for record in self._list("candidates", "candidate.json"):
                if self._read("jobs", record["job_id"], "job.json")["status"] == "completed":
                    result.append(self.get_candidate(record["candidate_id"]))
            return result

    def get_candidate(self, identifier):
        with self.lock:
            candidate = self._read("candidates", identifier, "candidate.json")
            job = self._read("jobs", candidate["job_id"], "job.json")
            if job["status"] != "completed":
                raise ValueError("Candidate belongs to a canceled or incomplete job")
            return {key: value for key, value in candidate.items() if key != "directory"}

    def candidate_data(self, identifier):
        return copy.deepcopy(self.get_candidate(identifier)["calibration"])

    def _artifact(self, directory, allowed, name):
        if not isinstance(name, str) or name not in allowed:
            raise ValueError("Unknown artifact")
        path = self._safe(directory / name)
        if not path.is_file():
            raise ValueError("Missing artifact")
        return {"body": path.read_bytes(), "content_type": mimetypes.guess_type(path.name)[0] or "application/octet-stream",
                "file_name": path.name}

    def get_artifact(self, identifier, name):
        with self.lock:
            self.get_candidate(identifier)
            candidate = self._read("candidates", identifier, "candidate.json")
            directory = self._safe(self.root / candidate["directory"])
            return self._artifact(directory, {item["name"] for item in candidate["artifacts"]}, name)

    def get_session_artifact(self, identifier, name):
        with self.lock:
            session = self._session_record(identifier)
            if "selected_directory" not in session:
                raise ValueError("Session has no saved selection yet")
            directory = self._safe(self.root / session["selected_directory"])
            allowed = {view["image"] for view in session["views"]}
            if session["summary"].get("preview_artifact"):
                allowed.add(session["summary"]["preview_artifact"])
            return self._artifact(directory, allowed, name)

    def close(self):
        with self.lock:
            self.closed = True
            if self._active:
                self.cancel_job(self._active)
            thread = self._thread
        if thread and thread is not threading.current_thread():
            thread.join(3.)
