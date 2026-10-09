#!/usr/bin/env python3
"""Separate localhost calibration desktop application with optional guided preview.

No source is opened automatically. Guided sources require an explicit selection
and start action; synthetic preview requires a separate flag. Starts no vision
pipeline, NetworkTables, or runtime controller. Native mrcal uses the explicitly
configured interpreter. Ctrl-C stops owned preview and jobs and retains results.
"""
from __future__ import annotations
import argparse
import math
from pathlib import Path
import sys
import threading

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from custom_vision.calibration_jobs import CalibrationJobs
from custom_vision.dashboard import Dashboard


class OfflineConfig:
    writable = False
    def get_config(self):
        return {"pipelines": [], "dashboard": {"stream_fps": 1},
                "networktables": {"enabled": False}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=5845)
    parser.add_argument("--data", type=Path, default=ROOT / "data/calibration-browser")
    parser.add_argument("--solver-python", help="Existing isolated interpreter with mrcal; nothing is installed")
    parser.add_argument("--guided-capture", action="store_true", help="Enable explicit session-bound chessboard preview controls")
    parser.add_argument("--synthetic-preview", action="store_true", help="Register a synthetic chessboard source (requires --guided-capture; no camera)")
    parser.add_argument("--camera-device", help="Explicit Linux /dev/videoN device or stable symlink; registered without opening it")
    parser.add_argument("--camera-physical-id", help="Public opaque identity of the selected physical camera; required with --camera-device")
    parser.add_argument("--camera-width", type=int, help="Explicit requested V4L2 mode width; required with --camera-device")
    parser.add_argument("--camera-height", type=int, help="Explicit requested V4L2 mode height; required with --camera-device")
    parser.add_argument("--camera-fps", type=float, help="Optional requested V4L2 frame rate, 1..240; registration does not verify hardware support")
    args = parser.parse_args()
    if not 0 <= args.port <= 65535: parser.error("Port must be 0..65535")
    if args.synthetic_preview and not args.guided_capture:
        parser.error("--synthetic-preview requires --guided-capture")
    camera_options = (args.camera_device, args.camera_physical_id, args.camera_width,
                      args.camera_height, args.camera_fps)
    camera_settings = None
    if any(value is not None for value in camera_options):
        if not args.camera_device:
            parser.error("Camera options require an explicit --camera-device")
        if not args.guided_capture:
            parser.error("--camera-device requires --guided-capture")
        if sys.platform != "linux":
            parser.error("Physical calibration preview requires Linux V4L2; Mac camera registration is unsupported")
        identity = (args.camera_physical_id or "").strip()
        if not identity or len(identity) > 128 or any(ord(character) < 32 for character in identity):
            parser.error("--camera-device requires a nonempty public --camera-physical-id (at most 128 characters)")
        if (args.camera_width is None or args.camera_height is None
                or not 1 <= args.camera_width <= 8192 or not 1 <= args.camera_height <= 8192
                or args.camera_width * args.camera_height > 1024 * 1024):
            parser.error("--camera-width and --camera-height are required, 1..8192, with at most one megapixel")
        if args.camera_fps is not None and (not math.isfinite(args.camera_fps) or not 1 <= args.camera_fps <= 240):
            parser.error("--camera-fps must be finite in 1..240")
        from custom_vision.device_controls import device_path
        try:
            device = device_path(args.camera_device)
        except ValueError as exc:
            parser.error(str(exc))
        camera_settings = {"source": device, "backend": "v4l2", "width": args.camera_width,
                           "height": args.camera_height, "crop": None, "binning": None,
                           "focus": {"kind": "unknown", "value": None, "locked": False}}
        if args.camera_fps is not None:
            camera_settings["fps"] = args.camera_fps
    if args.guided_capture:
        import cv2
        cv2.setNumThreads(0)
        cv2.ocl.setUseOpenCL(False)
        from custom_vision.calibration_capture import synthetic_source, v4l2_source
        from custom_vision.calibration_guided import GuidedCalibrationJobs
        sources = [synthetic_source()] if args.synthetic_preview else []
        if camera_settings is not None:
            sources.append(v4l2_source("configured_v4l2", "Configured Linux V4L2 camera", camera_settings,
                                       camera={"physical_id": identity, "user_label": "Configured Linux V4L2 camera",
                                               "identity_source": "manual", "capture_history": None}))
        jobs = GuidedCalibrationJobs(args.data, solver_python=args.solver_python, sources=sources)
    else:
        jobs = CalibrationJobs(args.data, solver_python=args.solver_python)
    dashboard = Dashboard({"host": "127.0.0.1", "port": args.port,
                           "calibration_only": True}, OfflineConfig(), calibration_jobs=jobs)
    dashboard._device_cache = {"devices": [], "note": "Offline imports only; no camera enumeration or acquisition."}
    dashboard._device_cache_until = float("inf")
    mode = "GUIDED CALIBRATION" if args.guided_capture else "OFFLINE CALIBRATION"
    print(f"{mode}: http://127.0.0.1:{dashboard.server.server_port} (no source open; runtime activation unavailable)", flush=True)
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        pass
    finally:
        dashboard.close()


if __name__ == "__main__": main()
