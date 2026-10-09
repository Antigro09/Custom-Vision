"""LAN setup UI with bounded, asynchronous, demand-driven camera previews.

Inference calls update() only to replace references. Overlay drawing, resizing,
rotation and JPEG compression happen on a separate worker, and stop when there
are no viewers. Preview orientation never changes detector/calibration geometry.
"""
from collections import defaultdict
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import secrets
import threading
import time
from urllib.parse import parse_qs, unquote, urlsplit

from .device_controls import discover_devices

_STATIC = Path(__file__).with_name("static")
_STATIC_FILES = {"/": ("index.html", "text/html; charset=utf-8"),
                 "/static/app.js": ("app.js", "text/javascript; charset=utf-8"),
                 "/static/style.css": ("style.css", "text/css; charset=utf-8"),
                 "/static/field_model.js": ("field_model.js", "text/javascript; charset=utf-8"),
                 "/static/field_renderer.js": ("field_renderer.js", "text/javascript; charset=utf-8"),
                 "/static/field_scene.js": ("field_scene.js", "text/javascript; charset=utf-8"),
                 "/static/field_dashboard.js": ("field_dashboard.js", "text/javascript; charset=utf-8"),
                 "/static/field.css": ("field.css", "text/css; charset=utf-8"),
                 "/static/calibration_dashboard.js": ("calibration_dashboard.js", "text/javascript; charset=utf-8"),
                 "/static/calibration_capture_ui.js": ("calibration_capture_ui.js", "text/javascript; charset=utf-8"),
                 "/static/calibration_diagnostics.js": ("calibration_diagnostics.js", "text/javascript; charset=utf-8"),
                 "/static/calibration.css": ("calibration.css", "text/css; charset=utf-8")}
_MAX_BODY = 1024 * 1024
_NAME = re.compile(r"[\w-]+$")


class Dashboard:
    def __init__(self, config, controller=None, calibration_jobs=None):
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.results = {}
        self._received_at = {}
        self._field_layout = None
        self._mounts = {}
        self.images = {}
        self.preview_stats = {}
        self.config = dict(config)
        self.controller = controller
        # Only the separate calibration desktop application supplies this manager.
        # Normal vision runtime starts no calibration processes or dependency probes.
        self.calibration_jobs = calibration_jobs
        self._pending = {}
        self._epochs = defaultdict(int)
        self._viewers = {}
        self._next_encode = {}
        self._renderer = None
        self._stopped = False
        self._csrf = secrets.token_urlsafe(32)
        self._writes = threading.Lock()
        self._calibration_upload = threading.BoundedSemaphore(1)
        self._device_cache = None
        self._device_cache_until = 0.0
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def setup(self):
                super().setup()
                self.connection.settimeout(5)

            def send_body(self, status, body, kind="application/json", file_name=None):
                self.send_response(status)
                self.send_header("Content-Type", kind)
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.send_header("Referrer-Policy", "same-origin")
                self.send_header("Content-Security-Policy", "default-src 'self'; img-src 'self' blob:; frame-ancestors 'none'; base-uri 'none'; form-action 'self'")
                self.send_header("Content-Length", str(len(body)))
                if file_name is not None:
                    name = Path(file_name).name.replace('"', '').replace('\r', '').replace('\n', '')
                    disposition = "inline" if kind.startswith("image/") else "attachment"
                    self.send_header("Content-Disposition", f'{disposition}; filename="{name}"')
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    pass

            def send_json(self, value, status=200):
                self.send_body(status, json.dumps(value, allow_nan=False).encode())

            def do_GET(self):
                target = urlsplit(self.path)
                path = target.path
                try:
                    if path == "/api/status":
                        with owner.lock:
                            data = dict(owner.results)
                        self.send_json(data)
                    elif path == "/api/field-view":
                        self.send_json(owner.field_view())
                    elif path.startswith("/api/calibration/"):
                        result = owner.calibration_get(path, target.query)
                        if isinstance(result, dict) and "body" in result:
                            self.send_body(200, result["body"], result["content_type"], result["file_name"])
                        else:
                            self.send_json(result)
                    elif path == "/api/config":
                        data = owner.controller.get_config() if owner.controller else {"dashboard": owner.config}
                        self.send_json({"config": data, "csrf_token": owner._csrf,
                                        "writable": owner.controller is not None and getattr(owner.controller, "writable", True)})
                    elif path == "/api/devices":
                        # UVC enumeration is slow; cache for two seconds and keep it
                        # away from the inference thread and preview lock.
                        with owner._writes:
                            if owner.config.get("calibration_only") or owner.config.get("offline_preview"):
                                owner._device_cache = {"devices": [], "note": "Offline imports only; camera discovery is disabled."}
                            elif time.monotonic() >= owner._device_cache_until:
                                owner._device_cache = discover_devices()
                                owner._device_cache_until = time.monotonic() + 2
                            data = owner._device_cache
                        self.send_json(data)
                    elif path == "/api/preview":
                        with owner.lock:
                            data = dict(owner.preview_stats)
                        self.send_json(data)
                    elif path.startswith("/frame/"):
                        name = unquote(path[7:])
                        if not _NAME.fullmatch(name):
                            self.send_json({"error": "Invalid pipeline name"}, 404)
                            return
                        body = owner._get_image(name)
                        if body:
                            self.send_body(200, body, "image/jpeg")
                        else:
                            self.send_json({"error": "No current frame available"}, 404)
                    elif path in _STATIC_FILES:
                        filename, kind = _STATIC_FILES[path]
                        if path == "/" and owner.config.get("calibration_only"):
                            filename = "calibration.html"
                        self.send_body(200, (_STATIC / filename).read_bytes(), kind)
                    else:
                        self.send_json({"error": "Not found"}, 404)
                except (ValueError, TypeError, KeyError) as exc:
                    self.send_json({"error": str(exc)}, 400)
                except RuntimeError as exc:
                    self.send_json({"error": str(exc)}, 409)
                except Exception as exc:
                    self.send_json({"error": str(exc)}, 500)

            def do_POST(self):
                path = urlsplit(self.path).path
                calibration_request = path.startswith("/api/calibration/")
                if path not in {"/api/config", "/api/calibration", "/api/field-layout"} and not calibration_request:
                    self.send_json({"error": "Not found"}, 404)
                    return
                if calibration_request:
                    writable = owner.calibration_jobs is not None
                else:
                    writable = owner.controller is not None and getattr(owner.controller, "writable", True)
                if not writable:
                    self.send_json({"error": "Dashboard is read-only without a runtime controller"}, 405)
                    return
                origin = self.headers.get("Origin")
                host = self.headers.get("Host", "").lower()
                if origin:
                    try:
                        parsed = urlsplit(origin)
                        same_origin = parsed.scheme in {"http", "https"} and parsed.netloc.lower() == host
                    except ValueError:
                        same_origin = False
                    if not same_origin:
                        self.send_json({"error": "Cross-origin writes are not allowed"}, 403)
                        return
                if self.headers.get("Sec-Fetch-Site") == "cross-site":
                    self.send_json({"error": "Cross-site writes are not allowed"}, 403)
                    return
                if not hmac.compare_digest(self.headers.get("X-Custom-Vision-CSRF", "").encode(), owner._csrf.encode()):
                    self.send_json({"error": "Missing or invalid setup token; reload the page"}, 403)
                    return
                if self.headers.get("Content-Type", "").split(";", 1)[0].strip() != "application/json":
                    self.send_json({"error": "Content-Type must be application/json"}, 415)
                    return
                upload_slot = False
                try:
                    if self.headers.get("Transfer-Encoding"):
                        raise ValueError("Chunked request bodies are not accepted")
                    size = int(self.headers.get("Content-Length", "0"))
                    limit = 45 * 1024 * 1024 if path == "/api/calibration/assets" else _MAX_BODY
                    if not 0 < size <= limit:
                        self.send_json({"error": f"JSON body must be between 1 byte and {limit} bytes"}, 413)
                        return
                    if path == "/api/calibration/assets":
                        upload_slot = owner._calibration_upload.acquire(blocking=False)
                        if not upload_slot:
                            self.send_json({"error": "Another calibration upload is in progress"}, 409)
                            return
                    data = json.loads(self.rfile.read(size), parse_constant=lambda value: (_ for _ in ()).throw(ValueError("JSON numbers must be finite")))
                    if not isinstance(data, dict):
                        raise ValueError("Expected a JSON object")
                    with owner._writes:
                        if calibration_request:
                            result = owner.calibration_post(path, data)
                        elif path == "/api/config":
                            result = owner.controller.apply_config(data)
                        elif path == "/api/calibration":
                            if not isinstance(data.get("pipeline"), str) or not _NAME.fullmatch(data["pipeline"]):
                                raise ValueError("A valid pipeline name is required")
                            if not isinstance(data.get("data"), dict):
                                raise ValueError("Calibration data must be a JSON object")
                            result = owner.controller.upload_calibration(data["pipeline"], data["data"])
                        else:
                            if not isinstance(data.get("data"), dict):
                                raise ValueError("Field layout data must be a JSON object")
                            result = owner.controller.upload_field_layout(data["data"])
                        if not calibration_request:
                            owner._device_cache_until = 0
                    self.send_json(result if result is not None else {"ok": True})
                except (ValueError, TypeError, KeyError) as exc:
                    self.send_json({"error": str(exc)}, 400)
                except RuntimeError as exc:
                    self.send_json({"error": str(exc)}, 409)
                except Exception as exc:
                    self.send_json({"error": str(exc)}, 500)
                finally:
                    if upload_slot:
                        owner._calibration_upload.release()

            def log_message(self, *args):
                pass

        self.server = ThreadingHTTPServer((config.get("host", "127.0.0.1"), config.get("port", 5801)), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, name="vision-http", daemon=True)
        self.encoder_thread = threading.Thread(target=self._encode_loop, name="vision-preview", daemon=True)
        self.thread.start()
        self.encoder_thread.start()

    def calibration_get(self, path, query=""):
        manager = self.calibration_jobs
        active = self.controller is not None and getattr(self.controller, "writable", True)
        if path == "/api/calibration/capabilities":
            data = manager.capabilities() if manager else {
                "available": False, "supported_inputs": [],
                "reason": "Start the separate offline calibration desktop application."}
            return {**data, "runtime_activation": bool(manager and active),
                    "desktop_mode": bool(self.config.get("calibration_only"))}
        if path == "/api/calibration/status":
            status_method = getattr(self.controller, "calibration_status", None)
            pipelines = status_method() if callable(status_method) else []
            return {"runtime_activation_available": bool(manager and active),
                    "pipelines": pipelines,
                    "latest_candidates": manager.list_candidates() if manager else [],
                    "default_state": "runtime_camera_states" if pipelines else "offline_no_runtime"}
        if manager is None:
            raise ValueError("Offline calibration application is not running")
        if path == "/api/calibration/capture/sources": return manager.capture_sources()
        if path == "/api/calibration/capture/status": return manager.capture_status()
        match = re.fullmatch(r"/api/calibration/capture/([^/]+)/frame", path)
        if match:
            values = parse_qs(query, keep_blank_values=True)
            if set(values) - {"frame_id"} or len(values.get("frame_id", [])) > 1:
                raise ValueError("Preview accepts one optional exact frame_id")
            frame_id = None
            if "frame_id" in values:
                value = values["frame_id"][0]
                if not re.fullmatch(r"0|[1-9][0-9]{0,15}", value):
                    raise ValueError("Preview frame_id must be a nonnegative integer")
                frame_id = int(value)
                if frame_id > 2**53 - 1:
                    raise ValueError("Preview frame_id exceeds the exact integer bound")
            return manager.capture_frame(unquote(match[1]), frame_id=frame_id)
        match = re.fullmatch(r"/api/calibration/sessions/([^/]+)/(snapshots|mosaic)", path)
        if match:
            method = manager.snapshots if match[2] == "snapshots" else manager.mosaic
            return method(unquote(match[1]))
        match = re.fullmatch(r"/api/calibration/candidates/([^/]+)/diagnostics", path)
        if match: return manager.diagnostics(unquote(match[1]))
        if path == "/api/calibration/sessions": return manager.list_sessions()
        if path == "/api/calibration/jobs": return manager.list_jobs()
        if path == "/api/calibration/candidates": return manager.list_candidates()
        if path == "/api/calibration/assets": return manager.list_assets()
        if path == "/api/calibration/activations":
            return self.controller.list_calibration_activations() if active else []
        match = re.fullmatch(r"/api/calibration/(sessions|candidates)/([^/]+)/artifacts/(.+)", path)
        if match:
            method = manager.get_session_artifact if match[1] == "sessions" else manager.get_artifact
            return method(unquote(match[2]), unquote(match[3]))
        match = re.fullmatch(r"/api/calibration/(sessions|jobs|candidates)/([^/]+)", path)
        if match:
            method = {"sessions": manager.get_session, "jobs": manager.get_job,
                      "candidates": manager.get_candidate}[match[1]]
            return method(unquote(match[2]))
        raise ValueError("Unknown calibration route")

    def calibration_post(self, path, data):
        manager = self.calibration_jobs
        if path == "/api/calibration/assets": return manager.add_asset(data)
        if path == "/api/calibration/sessions": return manager.create_session(data)
        if path == "/api/calibration/jobs": return manager.create_job(data)
        if path == "/api/calibration/capture/start": return manager.capture_start(data)
        if path == "/api/calibration/capture/snapshot": return manager.capture_snapshot(data)
        if path == "/api/calibration/capture/stop": return manager.capture_stop(data)
        if path == "/api/calibration/candidates/import": return manager.import_candidate(data)
        match = re.fullmatch(r"/api/calibration/candidates/([^/]+)/review", path)
        if match: return manager.review(unquote(match[1]), data)
        match = re.fullmatch(r"/api/calibration/jobs/([^/]+)/cancel", path)
        if match: return manager.cancel_job(unquote(match[1]))
        active = self.controller is not None and getattr(self.controller, "writable", True)
        if not active: raise ValueError("Candidate export is available; runtime activation requires a writable controller")
        if data.get("confirmed") is not True:
            raise ValueError("Explicit confirmation is required for activation or restoration")
        match = re.fullmatch(r"/api/calibration/candidates/([^/]+)/activate", path)
        if match:
            if not isinstance(data.get("pipeline"), str) or not _NAME.fullmatch(data["pipeline"]):
                raise ValueError("A valid pipeline name is required")
            candidate_id = unquote(match[1])
            metadata_method = getattr(manager, "candidate_metadata", None)
            if callable(metadata_method):
                metadata = metadata_method(candidate_id)
            else:
                # Legacy offline adapters retain their metadata format; the
                # runtime controller independently enforces its review gates.
                candidate = manager.get_candidate(candidate_id)
                session = manager.get_session(candidate["session_id"])
                metadata = {"camera": session.get("camera", session.get("spec", {}).get("camera")),
                            "mode": session.get("mode", session.get("spec", {}).get("mode"))}
            return self.controller.activate_calibration_candidate(data["pipeline"], manager.candidate_data(candidate_id), metadata)
        match = re.fullmatch(r"/api/calibration/activations/([^/]+)/restore", path)
        if match: return self.controller.restore_calibration_activation(unquote(match[1]))
        raise ValueError("Unknown calibration route")

    def set_renderer(self, callback):
        """Register callback(payload, frame) -> annotated frame on preview thread.

        Frame references are shared and must be treated as immutable. The callback
        must copy before drawing. Replacing a callback is safe during operation.
        """
        with self.lock:
            self._renderer = callback

    def set_field_geometry(self, layout, mounts_by_pipeline):
        """Cache public geometry once; no files or inference on field-view reads.

        Validation copies allowlisted WPILib layout/mount fields, preserving the
        supplied fixed origin. A missing mount remains null, never identity.
        """
        from .localization import validate_field_layout, validate_robot_to_camera
        checked_layout = None if layout is None else validate_field_layout(layout)
        if not isinstance(mounts_by_pipeline, dict):
            raise ValueError("Field-view mounts must be a pipeline mapping")
        checked_mounts = {}
        for name, mount in mounts_by_pipeline.items():
            if not isinstance(name, str) or not _NAME.fullmatch(name):
                raise ValueError("Field-view mount needs a valid pipeline name")
            checked_mounts[name] = validate_robot_to_camera(mount)
        with self.lock:
            self._field_layout = checked_layout
            self._mounts = checked_mounts

    def field_view(self):
        """Return coherent results and host receipt ages without altering packets.

        Receipt age describes dashboard liveness, not synchronized capture age.
        Repeated HTTP polls do not reset it; invalidations are fresh receipts too.
        """
        with self.lock:
            now = time.monotonic()
            results = dict(self.results)
            ages = {name: (max(0., (now - self._received_at[name]) * 1000)
                           if name in self._received_at else None)
                    for name in results}
            return {"version": 1, "results": results, "receipt_age_ms": ages,
                    "field_layout": self._field_layout, "mounts": self._mounts}

    def update(self, payload, frame=None):
        """O(1) handoff; caller must never mutate payload/frame after this call."""
        name = payload["pipeline"]
        with self.condition:
            if self._stopped:
                return
            self.results[name] = payload
            self._received_at[name] = time.monotonic()
            if not payload.get("connected", False):
                self._epochs[name] += 1
                self._pending.pop(name, None)
                self.images.pop(name, None)
                self.preview_stats.pop(name, None)
            elif frame is not None:
                self._pending[name] = (payload, frame, self._epochs[name])
            self.condition.notify_all()

    def _get_image(self, name):
        with self.condition:
            if not self.results.get(name, {}).get("connected"):
                return None
            self._viewers[name] = time.monotonic()
            self.condition.notify_all()
            # Waiting on first preview happens only in this HTTP request. This
            # preserves immediate /frame requests without stalling inference.
            deadline = time.monotonic() + 1
            while name not in self.images and not self._stopped and self.results.get(name, {}).get("connected"):
                left = deadline - time.monotonic()
                if left <= 0:
                    break
                self.condition.wait(left)
            return self.images.get(name)

    def _encode_loop(self):
        import cv2
        while True:
            with self.condition:
                if self._stopped:
                    return
                now = time.monotonic()
                active = [name for name in self._pending if now - self._viewers.get(name, -100) < 2]
                eligible = [name for name in active if now >= self._next_encode.get(name, 0)]
                if not eligible:
                    wait = min([max(.001, self._next_encode.get(name, now) - now) for name in active] or [1])
                    self.condition.wait(wait)
                    continue
                name = min(eligible, key=lambda key: self._next_encode.get(key, 0))
                payload, frame, epoch = self._pending.pop(name)
                settings = {**self.config, **payload.get("preview_settings", {})}
                fps = min(30, max(1, float(settings.get("stream_fps", 10))))
                self._next_encode[name] = now + 1 / fps
                renderer = self._renderer
            started = time.monotonic()
            try:
                rendered = renderer(payload, frame) if renderer else frame
                angle = int(settings.get("rotation_deg", 0))
                rotations = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}
                if angle in rotations:
                    rendered = cv2.rotate(rendered, rotations[angle])
                width = min(1920, max(160, int(settings.get("stream_width", 640))))
                if rendered.shape[1] > width:
                    height = max(1, round(rendered.shape[0] * width / rendered.shape[1]))
                    rendered = cv2.resize(rendered, (width, height), interpolation=cv2.INTER_AREA)
                quality = min(95, max(20, int(settings.get("jpeg_quality", 70))))
                ok, jpeg = cv2.imencode(".jpg", rendered, [cv2.IMWRITE_JPEG_QUALITY, quality])
                if not ok:
                    raise RuntimeError("JPEG encoding failed")
                body = jpeg.tobytes()
                stats = {"encode_ms": round((time.monotonic() - started) * 1000, 3),
                         "frame_id": payload.get("frame_id"), "bytes": len(body),
                         "width": rendered.shape[1], "height": rendered.shape[0],
                         "rate_limit_fps": fps, "rotation_deg": angle}
                with self.condition:
                    # An in-flight encode may complete after a failure. Never
                    # republish it after disconnect or reconnect to a new epoch.
                    if epoch == self._epochs[name] and self.results.get(name, {}).get("connected"):
                        self.images[name] = body
                        self.preview_stats[name] = stats
                    self.condition.notify_all()
            except Exception as exc:
                with self.condition:
                    if epoch == self._epochs[name]:
                        self.images.pop(name, None)
                        self.preview_stats[name] = {"error": str(exc)}
                    self.condition.notify_all()

    def close(self):
        with self.condition:
            self._stopped = True
            self.condition.notify_all()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.encoder_thread.join(timeout=2)
        if self.calibration_jobs is not None:
            self.calibration_jobs.close()
