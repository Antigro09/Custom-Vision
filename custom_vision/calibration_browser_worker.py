"""One camera-free CPU worker for prerecorded selection or candidate fitting.

Only the local manager writes the request file. Raw images remain lossless; an
annotated selection preview is a separate artifact. Image-order indices may drive
selection but are explicitly not measured timestamps or temporal validation.
"""
from __future__ import annotations

import argparse
from collections import Counter
import copy
from datetime import datetime, timezone
import json
import math
import mimetypes
from pathlib import Path
import signal
import threading
import time
from types import SimpleNamespace
import uuid

import cv2
import numpy as np

from .calibration import calibrate_images
from .calibration_session import Board, Selector, Session, load_session, write_json
from .calibration_video import video_clock

_CANCELED = threading.Event()


def _check_cancel():
    if _CANCELED.is_set():
        raise InterruptedError("Calibration job canceled; saved data retained")


def _safe(root, path):
    path = Path(path).resolve()
    if not path.is_relative_to(root):
        raise ValueError("Worker artifact is outside the calibration store")
    return path


def _progress(job_dir, stage, processed=0, total=None, accepted=0, rejected=0, coverage=0., reason=None):
    data = {"processed_frames": processed, "total_frames": total, "accepted_views": accepted,
            "rejected_views": rejected, "coverage_fraction": coverage, "last_reason": reason}
    write_json(job_dir / "progress.json", {"stage": stage, "progress": data})
    return data


def _preview(directory, frame, corners, board):
    marked = (cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR) if frame.ndim == 2 else
              cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR) if frame.shape[2] == 4 else frame.copy())
    if corners is not None:
        cv2.drawChessboardCorners(marked, (board.cols, board.rows),
                                 corners.astype(np.float32).reshape(-1, 1, 2), True)
    if marked.shape[1] > 640:
        marked = cv2.resize(marked, (640, round(marked.shape[0] * 640 / marked.shape[1])), interpolation=cv2.INTER_AREA)
    if not cv2.imwrite(str(directory / "selection-preview.jpg"), marked, [cv2.IMWRITE_JPEG_QUALITY, 85]):
        raise OSError("Could not save selection preview")


def select(request, root, job_dir):
    record = copy.deepcopy(request["session"])
    board = Board(**record["board"])
    settings = record["selection"]
    selector = Selector(board, max_views=settings["max_views"], min_interval=settings["interval_s"],
                        novelty=settings["novelty"], max_sharpness=settings["max_sharpness_px"],
                        min_contrast=settings["min_contrast"])
    directory = _safe(root, root / "sessions" / record["session_id"] / ("selection-" + request["job_id"]))
    session = Session(directory, board, selector, {"kind": record["input"]["kind"],
                                                  "asset_ids": record["input"]["asset_ids"]})
    session.data.update(camera_identity=record["camera"], capture_mode=record["mode"],
                        declared_capture_mode=record["declared_mode"], physical_validation=False)
    timestamp_id = record["input"]["timestamps_asset_id"]
    explicit = _safe(root, request["assets"][timestamp_id]) if timestamp_id else None
    image_set = record["input"]["kind"] == "images"
    assets = [_safe(root, request["assets"][identifier]) for identifier in record["input"]["asset_ids"]]
    processed = rejected = 0
    reasons = Counter()
    total = len(assets) if image_set else None
    source = "recorded_host_read_complete" if explicit else ("image_sequence_index_not_capture_time" if image_set else "video_frame_index_over_reported_fps")
    last_reason = None
    cap = None
    preview = None
    finished = False
    last_checkpoint = -math.inf
    saved_bytes = 0

    def checkpoint(status="incomplete"):
        nonlocal last_checkpoint
        record.update(status=status, width=session.data["width"], height=session.data["height"],
                      selected_directory=str(directory.relative_to(root)), views=session.data["views"],
                      summary={"processed_frames": processed, "accepted_views": len(session.data["views"]),
                               "rejected_views": rejected, "coverage_fraction": float(selector.coverage.mean()),
                               "coverage_grid": selector.coverage.astype(int).tolist(), "rejections_by_reason": dict(reasons),
                               "timestamp_source": source, "preview_artifact": preview})
        if record["width"] is not None:
            record["mode"].update(width=record["width"], height=record["height"])
            record["mode_dimensions_source"] = "decoded"
        write_json(root / "sessions" / record["session_id"] / "record.json", record)
        last_checkpoint = time.monotonic()

    checkpoint()
    try:
        fps = 1.
        if not image_set:
            cap = cv2.VideoCapture(str(assets[0]))  # local file only, never device or URL
            if not cap.isOpened():
                raise ValueError("Could not decode the saved video asset")
            fps = float(cap.get(cv2.CAP_PROP_FPS))
            if not math.isfinite(fps) or fps <= 0:
                raise ValueError("Video has no usable reported FPS/timebase")
            count = float(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            total = int(count) if math.isfinite(count) and count > 0 else None
        with video_clock(assets[0], explicit) as (timestamp_at, _):
            next_sample = 0.
            for frame_id in range(min(len(assets), settings["max_frames"]) if image_set else settings["max_frames"]):
                _check_cancel()
                if image_set:
                    frame = cv2.imread(str(assets[frame_id]), cv2.IMREAD_UNCHANGED)
                    if frame is None:
                        raise ValueError("Could not decode saved image asset")
                else:
                    ok, frame = cap.read()
                    if not ok:
                        break
                if frame.ndim not in (2, 3) or (frame.ndim == 3 and frame.shape[2] not in (3, 4)):
                    raise ValueError("Unsupported calibration image channel format")
                if frame.dtype != np.uint8:
                    raise ValueError("Calibration selection accepts 8-bit images; retain original media separately")
                h, w = frame.shape[:2]
                if w * h > 16 * 1024**2 or max(w, h) > 8192:
                    raise ValueError("Decoded image exceeds the bounded pixel budget")
                if session.data["width"] is None:
                    session.data.update(width=w, height=h)
                declared = record["declared_mode"]
                if declared["width"] is not None and (w, h) != (declared["width"], declared["height"]):
                    raise ValueError("Decoded dimensions do not match the declared mode; start a matching session")
                stamp = timestamp_at(frame_id, fps) if explicit or not image_set else float(frame_id) * settings["interval_s"]
                processed += 1
                if image_set or stamp >= next_sample:
                    if saved_bytes + frame.nbytes + 4096 > 256 * 1024**2:
                        raise ValueError("Selected lossless images reached the bounded 256 MiB output budget")
                    corners, last_reason, accepted = session.process(frame, stamp, frame_id, force=image_set)
                    next_sample = stamp + settings["interval_s"]
                    if not accepted:
                        rejected += 1
                        reasons[last_reason] += 1
                    _preview(directory, frame, corners, board)
                    preview = "selection-preview.jpg"
                    if accepted:
                        saved_bytes += (directory / session.data["views"][-1]["image"]).stat().st_size
                    if accepted or time.monotonic() - last_checkpoint >= 1.:
                        checkpoint()
                progress = _progress(job_dir, "selecting", processed, total, len(session.data["views"]),
                                     rejected, float(selector.coverage.mean()), last_reason)
                if len(session.data["views"]) >= settings["max_views"]:
                    record["warnings"].append("Selection stopped at its configured view budget")
                    break
            else:
                if not image_set and (total is None or processed < total):
                    record["warnings"].append("Selection stopped at its configured frame budget")
        _check_cancel()
        if processed == 0:
            raise ValueError("Saved media contained no decodable frames")
        session.data.update(complete=True, acquisition={"frames_read": processed, "timestamp_source": source,
                                                       "reported_fps": None if image_set else fps,
                                                       "measured_capture_time_verified": False})
        session.save()
        # Never relabel requested dimensions as delivered dimensions. Null modes
        # are filled only from decoded data, with user crop/focus metadata retained.
        checkpoint("selected")
        finished = True
        progress = _progress(job_dir, "selected", processed, total, len(session.data["views"]),
                             rejected, float(selector.coverage.mean()), last_reason)
        return {"candidate_id": None, "quality_status": "selection_requires_review", "summary": record["summary"]}, progress
    finally:
        if cap is not None:
            cap.release()
        session.save()
        checkpoint("selected" if finished else "incomplete")


def solve(request, root, job_dir):
    record = request["session"]
    session_path = _safe(root, root / record["selected_directory"])
    _, data = load_session(session_path)
    count = len(data["views"])
    _progress(job_dir, "fitting_" + request["solver"], accepted=count)
    options = request["options"]
    _check_cancel()
    if request["solver"] == "opencv":
        output = job_dir / "candidate"
        output.mkdir()
        calibration = calibrate_images([session_path / view["image"] for view in data["views"]],
            data["board"]["cols"], data["board"]["rows"], data["board"]["square_size_m"],
            min_views=options["min_views"], max_rms_px=options["max_rms_px"])
        for item in calibration["per_view_errors"] + calibration["skipped_images"]:
            item["image"] = str(Path(item["image"]).relative_to(session_path))
        calibration.update(quality_status="baseline_not_hardware_validated", robot_mount_calibrated=False,
                           calibration_verified=False)
        report = {"status": "baseline_not_hardware_validated", "solver": "opencv",
                  "training_rms_px": calibration["rms_error_px"], "accepted_views": calibration["accepted_views"],
                  "holdout": {"status": "not_computed"}, "uncertainty": {"status": "not_computed"},
                  "native_models": {"status": "not_computed"}, "issues": [],
                  "warnings": ["Training residual RMS is not an independent accuracy measurement",
                               "OpenCV baseline does not provide the mrcal spline comparison or projection uncertainty",
                               "Robot mount, timing correction and physical calibration trust remain unverified"],
                  "timestamp_source": record["summary"]["timestamp_source"],
                  "validation_basis": "image_group_not_computed" if record["input"]["kind"] == "images" else "not_computed"}
        write_json(output / "intrinsics.candidate.json", calibration)
        write_json(output / "report.json", report)
    else:
        basis = request.get("validation_basis", "time_blocks")
        if basis not in ("time_blocks", "image_groups"):
            raise ValueError("Unsupported native validation basis")
        if basis == "time_blocks" and record["summary"]["timestamp_source"] not in ("recorded_host_read_complete", "host_frame_read_complete"):
            raise ValueError("Native temporal validation requires recorded host-time provenance")
        from .calibration_solver import calibrate
        args = SimpleNamespace(session=session_path, min_views=options["min_views"], fov_deg=options["fov_deg"],
                               corner_detector=options["corner_detector"], no_spline=options["no_spline"],
                               max_validation_rms=options["max_validation_rms"], timeout=options["timeout_s"],
                               validation_basis=basis)
        code = calibrate(args)
        if code not in (0, 2):
            raise RuntimeError("Native solver did not produce a reviewable candidate")
        latest = json.loads((session_path / "latest-solve.json").read_text())
        output = _safe(root, session_path / latest["directory"])
        report = json.loads((output / "report.json").read_text())
        calibration = json.loads((output / report["intrinsics_file"]).read_text())
        report.pop("source_session", None)
    provenance = {"source_kind": record.get("source_kind", record["input"]["kind"]),
                  "synthetic": record.get("source_kind") == "synthetic", "camera": record["camera"],
                  "mode": record["mode"], "capture_revision": record.get("capture_revision")}
    calibration.update(source_kind=provenance["source_kind"], synthetic=provenance["synthetic"],
                       calibration_provenance=provenance)
    report.update(source_kind=provenance["source_kind"], synthetic=provenance["synthetic"])
    write_json(output / report.get("intrinsics_file", "intrinsics.candidate.json"), calibration)
    write_json(output / "report.json", report)
    _check_cancel()
    candidate_id = uuid.uuid4().hex
    candidate_dir = root / "candidates" / candidate_id
    candidate_dir.mkdir()
    artifacts = []
    for path in sorted(output.rglob("*")):
        if path.is_file() and not path.is_symlink() and "images" not in path.relative_to(output).parts:
            artifacts.append({"name": str(path.relative_to(output)), "size_bytes": path.stat().st_size,
                              "content_type": mimetypes.guess_type(path.name)[0] or "application/octet-stream"})
    candidate = {"candidate_id": candidate_id, "session_id": record["session_id"], "job_id": request["job_id"],
                 "created_utc": datetime.now(timezone.utc).isoformat(), "solver": request["solver"],
                 "quality_status": report["status"], "camera": record["camera"], "mode": record["mode"],
                 "declared_mode": record["declared_mode"], "mode_dimensions_source": record["mode_dimensions_source"],
                 "calibration": calibration, "report": report, "artifacts": artifacts,
                 "capture_revision": record.get("capture_revision"),
                 "source_kind": record.get("source_kind", record["input"]["kind"]),
                 "synthetic": record.get("source_kind") == "synthetic",
                 "directory": str(output.relative_to(root))}
    write_json(candidate_dir / "candidate.json", candidate)
    progress = _progress(job_dir, "candidate_ready", accepted=count)
    return {"candidate_id": candidate_id, "quality_status": report["status"],
            "issues": report.get("issues", []), "warnings": report.get("warnings", [])}, progress


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request", type=Path, required=True)
    args = parser.parse_args(argv)
    cv2.setNumThreads(0)
    _CANCELED.clear()
    signal.signal(signal.SIGTERM, lambda *_: _CANCELED.set())
    signal.signal(signal.SIGINT, lambda *_: _CANCELED.set())
    job_dir = args.request.resolve().parent
    try:
        request = json.loads(args.request.read_text())
        root = Path(request["root"]).resolve()
        _safe(root, job_dir)
        operation = select if request["operation"] == "select" else solve
        result, progress = operation(request, root, job_dir)
        write_json(job_dir / "result.json", {"result": result, "progress": progress})
        return 0
    except Exception as exc:
        write_json(job_dir / "result.json", {"error": str(exc)})
        return 130 if isinstance(exc, InterruptedError) else 1


if __name__ == "__main__":
    raise SystemExit(main())
