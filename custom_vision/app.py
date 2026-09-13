"""Headless capture, dual pipelines, preview, and NT4 robot results."""
import argparse
import logging
import signal
import threading
import time

import cv2
import numpy as np

from .config import load_config
from .camera import LatestFrameCapture
from .dashboard import Dashboard
from .publisher import Publisher

LOG = logging.getLogger("custom_vision")


def open_camera(settings):
    backend = settings.get("backend", "auto")
    source = settings["source"]
    api = {"auto": cv2.CAP_ANY, "v4l2": cv2.CAP_V4L2, "gstreamer": cv2.CAP_GSTREAMER}.get(backend)
    if api is None:
        raise ValueError(f"Unknown camera backend: {backend}")
    if backend == "auto" and (isinstance(source, int) or str(source).startswith("/dev/")):
        api = cv2.CAP_V4L2
    cap = cv2.VideoCapture(source, api)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f"Camera unavailable: {source}. Check connection and PhotonVision ownership.")
    if settings.get("fourcc"):
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*settings["fourcc"]))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, settings["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, settings["height"])
    cap.set(cv2.CAP_PROP_FPS, settings["fps"])
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if isinstance(source, int) or str(source).startswith("/dev/") or backend == "gstreamer":
        return LatestFrameCapture(cap)
    return cap


def draw_detections(frame, detections):
    result = frame.copy()
    for detection in detections:
        if "corners" in detection:
            cv2.polylines(result, [np.asarray(detection["corners"], np.int32)], True, (0, 255, 0), 2)
        elif "bbox_xyxy" in detection:
            x1, y1, x2, y2 = map(int, detection["bbox_xyxy"])
            cv2.rectangle(result, (x1, y1), (x2, y2), (0, 255, 0), 2)
        x, y = map(int, detection["center"])
        label = f"tag {detection['id']}" if "id" in detection else detection["label"]
        cv2.putText(result, label, (x, max(20, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1)
    return result


class Runtime:
    def __init__(self, config, *, stdout=False, max_frames=0):
        from .apriltags import AprilTagPipeline
        from .objects import ObjectPipeline
        self.config = config
        self.stop = threading.Event()
        self.closed = False
        self.had_error = False
        self.max_frames = max_frames
        self.threads = []
        self.lock = threading.RLock()
        self.states = {}
        self.groups = {}
        # Validate and load all backends before starting publication or capture.
        for cfg in config["pipelines"]:
            cls = AprilTagPipeline if cfg["type"] == "apriltag" else ObjectPipeline
            detector = cls(cfg["settings"], cfg.get("calibration_data"))
            source_key = str(cfg["camera"]["source"])
            if source_key in self.groups and self.groups[source_key][0][0]["camera"] != cfg["camera"]:
                raise ValueError("Pipelines sharing a camera must use identical camera settings")
            self.groups.setdefault(source_key, []).append((cfg, detector))
            self.states[cfg["name"]] = {"last_frame": time.monotonic(), "frame_id": 0, "failed": False}
        self.publisher = Publisher(config["networktables"], stdout=stdout)
        self.dashboard = Dashboard(config["dashboard"]) if config["dashboard"].get("enabled") else None

    def emit(self, cfg, frame_id, captured, detections, *, frame=None, error=None):
        now = time.monotonic()
        payload = {
            "schema_version": 1, "pipeline": cfg["name"], "type": cfg["type"],
            "connected": error is None, "frame_id": frame_id,
            "capture_monotonic_us": int(captured * 1e6),
            "publish_unix_us": time.time_ns() // 1000,
            "latency_ms": max(0.0, (now - captured) * 1000),
            "detections": detections,
            "error": error,
        }
        # Capture timestamp is host read completion, NOT sensor exposure or roboRIO time.
        with self.lock:
            if self.closed or (self.stop.is_set() and error is None):
                return
            self.publisher.publish(payload)
            if self.dashboard:
                self.dashboard.update(payload, draw_detections(frame, detections) if frame is not None else None)

    def camera_worker(self, group):
        cap = None
        count = 0
        frame_id = 0
        try:
            while not self.stop.is_set() and (not self.max_frames or count < self.max_frames):
                try:
                    if cap is None:
                        cap = open_camera(group[0][0]["camera"])
                    ok, frame = cap.read()
                    captured = getattr(cap, "last_capture_monotonic", time.monotonic())
                    if self.stop.is_set():
                        break
                    if not ok or frame is None:
                        raise RuntimeError("Camera read failed or input video reached its end")
                    if frame.ndim == 2:
                        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    if count == 0:
                        LOG.info("Camera %s delivering %dx%d", group[0][0]["camera"]["source"], frame.shape[1], frame.shape[0])
                    for cfg, detector in group:
                        start = time.monotonic()
                        try:
                            detections = detector.process(frame)
                            with self.lock:
                                self.states[cfg["name"]].update(last_frame=captured, frame_id=frame_id, failed=False)
                                # A slow backend must not revive stale targets after the watchdog expires.
                                if time.monotonic() - captured > 0.5:
                                    self.emit(cfg, frame_id, captured, [], error="Frame exceeded 500 ms age limit")
                                else:
                                    self.emit(cfg, frame_id, captured, detections, frame=frame)
                        except Exception as exc:
                            self.had_error = True
                            LOG.error("Pipeline %s failed: %s", cfg["name"], exc)
                            with self.lock:
                                self.states[cfg["name"]].update(last_frame=start, frame_id=frame_id, failed=True)
                                self.emit(cfg, frame_id, captured, [], error=str(exc))
                    frame_id += 1
                    count += 1
                except (RuntimeError, ValueError, TypeError, cv2.error) as exc:
                    self.had_error = True
                    LOG.warning("%s", exc)
                    for cfg, _ in group:
                        self.emit(cfg, frame_id, time.monotonic(), [], error=str(exc))
                    if cap is not None:
                        cap.release()
                        cap = None
                    if self.max_frames:
                        break
                    self.stop.wait(1.0)
        finally:
            if cap is not None:
                cap.release()

    def run(self):
        try:
            for group in self.groups.values():
                thread = threading.Thread(target=self.camera_worker, args=(group,), daemon=True)
                self.threads.append(thread)
                thread.start()
            while not self.stop.wait(0.1):
                if not any(thread.is_alive() for thread in self.threads):
                    break
                with self.lock:
                    for cfg in self.config["pipelines"]:
                        state = self.states[cfg["name"]]
                        if time.monotonic() - state["last_frame"] > 0.5:
                            self.emit(cfg, state["frame_id"], time.monotonic(), [], error="No fresh frame within 500 ms")
        finally:
            self.stop.set()
            for thread in self.threads:
                thread.join(timeout=2)
            for cfg in self.config["pipelines"]:
                self.emit(cfg, self.states[cfg["name"]]["frame_id"], time.monotonic(), [], error="Runtime stopped")
            with self.lock:
                self.closed = True
                if self.dashboard:
                    self.dashboard.close()
                self.publisher.close()
            if not any(thread.is_alive() for thread in self.threads):
                for group in self.groups.values():
                    for _, detector in group:
                        if hasattr(detector, "close"):
                            detector.close()
        return 1 if self.had_error and self.max_frames else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/vision.yaml")
    parser.add_argument("--check", action="store_true", help="Validate config and load detectors without opening cameras")
    parser.add_argument("--no-nt", action="store_true", help="Disable robot publication for local tests")
    parser.add_argument("--no-dashboard", action="store_true")
    parser.add_argument("--stdout-json", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after this many frames per source, 0 runs continuously")
    args = parser.parse_args(argv)
    if args.max_frames < 0:
        parser.error("--max-frames must be nonnegative")
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    try:
        config = load_config(args.config)
        if args.no_nt or args.check:
            config["networktables"]["enabled"] = False
        if args.no_dashboard or args.check:
            config["dashboard"]["enabled"] = False
        runtime = Runtime(config, stdout=args.stdout_json, max_frames=args.max_frames)
        if args.check:
            runtime.publisher.close()
            print("Configuration and detector initialization OK")
            return 0
        for signum in (signal.SIGINT, signal.SIGTERM):
            signal.signal(signum, lambda *_: runtime.stop.set())
        return runtime.run()
    except (ValueError, OSError, RuntimeError, ImportError) as exc:
        LOG.error("%s", exc)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
