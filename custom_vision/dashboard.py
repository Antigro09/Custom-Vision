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
from urllib.parse import unquote, urlsplit

from .device_controls import discover_devices

_STATIC = Path(__file__).with_name("static")
_STATIC_FILES = {"/": ("index.html", "text/html; charset=utf-8"),
                 "/static/app.js": ("app.js", "text/javascript; charset=utf-8"),
                 "/static/style.css": ("style.css", "text/css; charset=utf-8")}
_MAX_BODY = 1024 * 1024
_NAME = re.compile(r"[\w-]+$")


class Dashboard:
    def __init__(self, config, controller=None):
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.results = {}
        self.images = {}
        self.preview_stats = {}
        self.config = dict(config)
        self.controller = controller
        self._pending = {}
        self._epochs = defaultdict(int)
        self._viewers = {}
        self._next_encode = {}
        self._renderer = None
        self._stopped = False
        self._csrf = secrets.token_urlsafe(32)
        self._writes = threading.Lock()
        self._device_cache = None
        self._device_cache_until = 0.0
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def setup(self):
                super().setup()
                self.connection.settimeout(5)

            def send_body(self, status, body, kind="application/json"):
                self.send_response(status)
                self.send_header("Content-Type", kind)
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.send_header("Referrer-Policy", "same-origin")
                self.send_header("Content-Security-Policy", "default-src 'self'; img-src 'self' blob:; frame-ancestors 'none'; base-uri 'none'; form-action 'self'")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    pass

            def send_json(self, value, status=200):
                self.send_body(status, json.dumps(value, allow_nan=False).encode())

            def do_GET(self):
                path = urlsplit(self.path).path
                try:
                    if path == "/api/status":
                        with owner.lock:
                            data = dict(owner.results)
                        self.send_json(data)
                    elif path == "/api/config":
                        data = owner.controller.get_config() if owner.controller else {"dashboard": owner.config}
                        self.send_json({"config": data, "csrf_token": owner._csrf,
                                        "writable": owner.controller is not None})
                    elif path == "/api/devices":
                        # UVC enumeration is slow; cache for two seconds and keep it
                        # away from the inference thread and preview lock.
                        with owner._writes:
                            if time.monotonic() >= owner._device_cache_until:
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
                        self.send_body(200, (_STATIC / filename).read_bytes(), kind)
                    else:
                        self.send_json({"error": "Not found"}, 404)
                except Exception as exc:
                    self.send_json({"error": str(exc)}, 500)

            def do_POST(self):
                path = urlsplit(self.path).path
                if path not in {"/api/config", "/api/calibration", "/api/field-layout"}:
                    self.send_json({"error": "Not found"}, 404)
                    return
                if not owner.controller:
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
                try:
                    if self.headers.get("Transfer-Encoding"):
                        raise ValueError("Chunked request bodies are not accepted")
                    size = int(self.headers.get("Content-Length", "0"))
                    if not 0 < size <= _MAX_BODY:
                        self.send_json({"error": "JSON body must be between 1 byte and 1 MiB"}, 413)
                        return
                    data = json.loads(self.rfile.read(size), parse_constant=lambda value: (_ for _ in ()).throw(ValueError("JSON numbers must be finite")))
                    if not isinstance(data, dict):
                        raise ValueError("Expected a JSON object")
                    with owner._writes:
                        if path == "/api/config":
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
                        owner._device_cache_until = 0
                    self.send_json(result if result is not None else {"ok": True})
                except (ValueError, TypeError, KeyError) as exc:
                    self.send_json({"error": str(exc)}, 400)
                except Exception as exc:
                    self.send_json({"error": str(exc)}, 500)

            def log_message(self, *args):
                pass

        self.server = ThreadingHTTPServer((config.get("host", "127.0.0.1"), config.get("port", 5801)), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, name="vision-http", daemon=True)
        self.encoder_thread = threading.Thread(target=self._encode_loop, name="vision-preview", daemon=True)
        self.thread.start()
        self.encoder_thread.start()

    def set_renderer(self, callback):
        """Register callback(payload, frame) -> annotated frame on preview thread.

        Frame references are shared and must be treated as immutable. The callback
        must copy before drawing. Replacing a callback is safe during operation.
        """
        with self.lock:
            self._renderer = callback

    def update(self, payload, frame=None):
        """O(1) handoff; caller must never mutate payload/frame after this call."""
        name = payload["pipeline"]
        with self.condition:
            if self._stopped:
                return
            self.results[name] = payload
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
