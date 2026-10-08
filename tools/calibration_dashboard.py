#!/usr/bin/env python3
"""Separate localhost calibration desktop application; imported media only.

Starts no cameras, vision pipelines, NetworkTables, or runtime controller. Results
are saved candidates for review/export. Native mrcal is used only when available
in the explicitly configured interpreter. Ctrl-C cancels owned jobs and exits.
"""
from __future__ import annotations
import argparse
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
    args = parser.parse_args()
    if not 0 <= args.port <= 65535: parser.error("Port must be 0..65535")
    jobs = CalibrationJobs(args.data, solver_python=args.solver_python)
    dashboard = Dashboard({"host": "127.0.0.1", "port": args.port,
                           "calibration_only": True}, OfflineConfig(), calibration_jobs=jobs)
    dashboard._device_cache = {"devices": [], "note": "Offline imports only; no camera enumeration or acquisition."}
    dashboard._device_cache_until = float("inf")
    print(f"OFFLINE CALIBRATION: http://127.0.0.1:{dashboard.server.server_port} (no cameras or runtime activation)", flush=True)
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        pass
    finally:
        dashboard.close()


if __name__ == "__main__": main()
