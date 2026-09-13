"""Small read-only preview and health endpoint, no external web dependencies."""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class Dashboard:
    def __init__(self, config):
        self.lock = threading.Lock()
        self.results = {}
        self.images = {}
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                path = self.path.split("?", 1)[0]
                with owner.lock:
                    if path == "/api/status":
                        body = json.dumps(owner.results, allow_nan=False).encode()
                        kind = "application/json"
                    elif path.startswith("/frame/"):
                        body = owner.images.get(path[7:], b"")
                        kind = "image/jpeg"
                    elif path == "/":
                        body = ("<!doctype html><meta charset=utf-8><title>Custom Vision</title>"
                                "<style>body{font:16px system-ui;background:#101923;color:#eee;max-width:1000px;margin:32px auto}"
                                "img{max-width:100%}pre{white-space:pre-wrap}section{background:#1d2a38;padding:20px;margin:20px 0}</style>"
                                "<h1>Custom Vision</h1><p>AprilTags and game-piece detection · camera-relative results</p><main></main>"
                                "<script>async function update(){let data=await(await fetch('/api/status')).json();"
                                "let root=document.querySelector('main');root.replaceChildren();"
                                "for(let [name,result] of Object.entries(data)){let s=document.createElement('section');"
                                "let h=document.createElement('h2');h.textContent=name+' — '+(result.connected?'Receiving frames':'Disconnected');s.append(h);"
                                "let img=document.createElement('img');img.src='/frame/'+name+'?t='+Date.now();s.append(img);"
                                "let pre=document.createElement('pre');pre.textContent=JSON.stringify(result,null,2);s.append(pre);root.append(s)}"
                                "}setInterval(()=>update().catch(()=>{}),500);update();</script>").encode()
                        kind = "text/html; charset=utf-8"
                    else:
                        self.send_error(404)
                        return
                if not body:
                    self.send_error(404, "No frame available")
                    return
                self.send_response(200)
                self.send_header("Content-Type", kind)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):
                pass

        self.server = ThreadingHTTPServer((config.get("host", "127.0.0.1"), config.get("port", 5800)), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def update(self, payload, frame=None):
        import cv2
        with self.lock:
            self.results[payload["pipeline"]] = payload
            if frame is not None:
                ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    self.images[payload["pipeline"]] = jpeg.tobytes()
            elif not payload["connected"]:
                self.images.pop(payload["pipeline"], None)

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
