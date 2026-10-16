"""Per-camera MJPEG server with two streams:

  /stream.mjpg   annotated (tag outlines, IDs, reject reasons, HUD)
  /raw.mjpg      untouched camera frames (focus tools, calibration, sanity checks)
  /              settings page: stream + live settings panel (like PhotonVision's)
  /api/state     GET  current settings/stats as JSON
  /api/set       POST {"exposure": 20, ...} -> applied live (same as editing NT)
  /api/save      POST -> write current settings to config.json
  /api/calib     POST {"op": "start" | "capture" | "calibrate" | "save" | ...} calibration
  /api/network   POST NetworkTables target, hostname and IP from the coprocessor page
  /api/admin     POST {"op": "restart" | "reboot" | "factory_reset" | "rename", "name": ...}
  /api/import    POST settings zip (or config.json) as the raw body
  /api/pipeline  POST {"op": "select" | "add" | "duplicate" | "rename" | "delete" | "default", "name": ...}
  /api/board.png GET  printable calibration board for the current board settings

Frames are encoded ONLY while someone is watching that stream -- an unwatched
stream costs nothing. Encoding happens on a separate thread (cv2.imencode releases
the GIL), and each frame is encoded once no matter how many viewers there are.
Full resolution; max_fps caps how often a new frame is encoded while watched.
"""

from __future__ import annotations

import json
import queue
import os
import threading
import time
from urllib.parse import urlsplit
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

IMPORT_LIMIT = 20 * 1024 * 1024  # settings zip: config.json + calibrations
_PAGE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web_ui.html")


class BoundedHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    def __init__(self, *args, **kwargs):
        self.slots = threading.BoundedSemaphore(32)
        super().__init__(*args, **kwargs)

    def get_request(self):
        connection, address = super().get_request()
        connection.settimeout(5)
        return connection, address

    def process_request(self, request, address):
        if not self.slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, address)
        except BaseException:
            self.slots.release()
            raise

    def process_request_thread(self, request, address):
        try:
            super().process_request_thread(request, address)
        finally:
            self.slots.release()


def _reason(error) -> str:
    """An error as an HTTP reason phrase (the page shows it): one latin-1 line."""
    text = (str(error).splitlines() or [type(error).__name__])[0]
    return text.encode("latin-1", "replace").decode("latin-1")


def allowed_origin(origin, host):
    if not origin:
        return True  # non-browser clients; this is not authentication
    try:
        parsed = urlsplit(origin)
        return parsed.scheme in ("http", "https") and parsed.hostname == urlsplit("//" + host).hostname
    except ValueError:
        return False


class _Channel:
    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg: bytes | None = None
        self.seq = 0
        self.clients = 0
        self.pending = None  # newest un-encoded frame


class MjpegStreamServer:
    def __init__(self, name: str, port: int, jpeg_quality: int = 80,
                 get_state=None, on_action=None, extra_get=None):
        """get_state() -> dict for /api/state; on_action(kind, data) handles the settings
        page's POSTs (kind is the path after /api/; data a dict, or bytes for /api/import).
        extra_get maps other GET paths to a function returning (content type, bytes)
        or (content type, bytes, download file name)."""
        self.name = name
        self.get_state = get_state
        self.on_action = on_action
        self.extra_get = extra_get or {}
        self.port = port
        self.quality = jpeg_quality
        self.channels = {"stream": _Channel(), "raw": _Channel()}
        self._work = threading.Condition()
        self.encoded_count = 0

    # ---- camera side -------------------------------------------------------

    def wants(self, channel: str) -> bool:
        return self.channels[channel].clients > 0

    def submit(self, channel: str, frame) -> None:
        """Hand over a frame to encode. Drops the previous one if the encoder is behind."""
        ch = self.channels[channel]
        with self._work:
            ch.pending = frame
            self._work.notify()

    # ---- threads -----------------------------------------------------------

    def start(self) -> None:
        threading.Thread(target=self._encoder, daemon=True, name=f"{self.name}-encoder").start()
        server = BoundedHTTPServer(("", self.port), self._make_handler())
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True, name=f"{self.name}-http").start()

    def _encoder(self) -> None:
        params = [cv2.IMWRITE_JPEG_QUALITY, self.quality]
        while True:
            with self._work:
                while not any(ch.pending is not None for ch in self.channels.values()):
                    self._work.wait()
                jobs = [(ch, ch.pending) for ch in self.channels.values() if ch.pending is not None]
                for ch, _ in jobs:
                    ch.pending = None
            for ch, frame in jobs:
                ok, buf = cv2.imencode(".jpg", frame, params)
                if not ok:
                    continue
                self.encoded_count += 1
                with ch.cond:
                    ch.jpeg = buf.tobytes()
                    ch.seq += 1
                    ch.cond.notify_all()

    def _make_handler(self):
        srv = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                path = self.path.split("?")[0]
                if path in ("/", "/index.html"):
                    with open(_PAGE_PATH, "rb") as f:  # read each time so edits show on reload
                        self._send(200, "text/html; charset=utf-8", f.read())
                    return
                if path == "/web_helpers.js":
                    with open(os.path.join(os.path.dirname(_PAGE_PATH), "web_helpers.js"), "rb") as f:
                        self._send(200, "text/javascript", f.read())
                    return
                if path == "/api/state":
                    state = srv.get_state() if srv.get_state else {"name": srv.name}
                    # default=: numpy scalars (np.bool_, np.float64) -> plain Python values
                    body = json.dumps(state, default=lambda o: o.item() if hasattr(o, "item") else str(o))
                    self._send(200, "application/json", body.encode())
                    return
                if path in srv.extra_get:
                    try:
                        ctype, body, *filename = srv.extra_get[path]()
                    except Exception as e:
                        self.send_error(500, _reason(e))
                        return
                    self._send(200, ctype, body, filename[0] if filename else None)
                    return
                if path in ("/stream.mjpg", "/stream"):
                    self._serve(srv.channels["stream"])
                elif path in ("/raw.mjpg", "/raw"):
                    self._serve(srv.channels["raw"])
                else:
                    self.send_error(404)

            def do_POST(self):
                if not allowed_origin(self.headers.get("Origin"), self.headers.get("Host", "")) or self.headers.get("Sec-Fetch-Site") == "cross-site":
                    self.send_error(403, "request origin is not this coprocessor")
                    return
                path = self.path.split("?")[0]
                kind = path[len("/api/"):]
                if srv.on_action is None or kind not in ("set", "save", "calib", "network", "admin", "import", "pipeline"):
                    self.send_error(404)
                    return
                limit = IMPORT_LIMIT if kind == "import" else 65536
                try:
                    n = int(self.headers.get("Content-Length") or 0)
                    if not 0 <= n <= limit or self.headers.get("Transfer-Encoding"):
                        self.send_error(413, "request body too large or unsupported encoding")
                        return
                    body = self.rfile.read(n)
                    if len(body) != n:
                        raise ValueError("request body was cut short")
                    if kind == "import":
                        data = body
                    else:
                        data = json.loads(body or b"{}")
                        if not isinstance(data, dict):
                            raise ValueError("expected a JSON object")
                except (ValueError, OSError) as e:
                    self.send_error(400, _reason(e))
                    return
                try:
                    status = srv.on_action(kind, None if kind == "save" else data)
                except ValueError as error:
                    self.send_error(400, _reason(error))
                    return
                except RuntimeError as error:
                    self.send_error(500, _reason(error))
                    return
                except (TimeoutError, queue.Full):
                    self.send_error(503, "settings owner is busy or unavailable")
                    return
                self._send(200, "application/json", json.dumps({"ok": True, "status": status}).encode())

            def _send(self, code, ctype, body: bytes, filename=None):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                if filename:
                    self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(body)

            def _serve(self, ch: _Channel):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                with ch.cond:
                    ch.clients += 1
                last = -1
                try:
                    while True:
                        with ch.cond:
                            if not ch.cond.wait_for(lambda: ch.jpeg is not None and ch.seq != last, timeout=10):
                                return  # stalled camera: release viewer/thread instead of spinning forever
                            data, last = ch.jpeg, ch.seq
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(data)}\r\n\r\n".encode())
                        self.wfile.write(data)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                finally:
                    with ch.cond:
                        ch.clients -= 1

            def end_headers(self):
                # One page switches between cameras, and each camera is on its own port.
                # Errors too, so that page can read why another camera refused a request.
                origin = self.headers.get("Origin") if hasattr(self, "headers") else None
                if origin and allowed_origin(origin, self.headers.get("Host", "")):
                    self.send_header("Access-Control-Allow-Origin", origin)
                    self.send_header("Vary", "Origin")
                    self.send_header("Access-Control-Expose-Headers", "Content-Disposition")
                super().end_headers()

            def log_message(self, *args):
                pass

        return Handler


def tag_box(d, camera_matrix, dist_coeffs, tag_size_m: float):
    """Image points of a cube standing on one tag (tag face first, then the face toward
    the camera), from that tag's own pose. None if the pose can't be solved."""
    h = tag_size_m / 2.0
    # SOLVEPNP_IPPE_SQUARE's required object order (TL, TR, BR, BL); the detector
    # reports corners BL, BR, TR, TL as seen looking at the tag.
    face = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])
    img = np.array([[d.getCorner(k).x, d.getCorner(k).y] for k in (3, 2, 1, 0)], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(face, img, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    # Extrude toward the camera, whichever way the tag's z axis points.
    toward = -1.0 if cv2.Rodrigues(rvec)[0][2, 2] > 0 else 1.0
    cube = np.vstack([face, face + [0.0, 0.0, toward * tag_size_m]])
    pts, _ = cv2.projectPoints(cube, rvec, tvec, camera_matrix, dist_coeffs)
    pts = pts.reshape(-1, 2)
    if not np.all(np.isfinite(pts)):
        return None
    return pts


def annotate(frame, detections, used_ids: set[int], result, reject: str | None, hud: list[str],
             camera_matrix=None, dist_coeffs=None, tag_size_m: float | None = None):
    """Draws tag outlines + IDs and a status HUD onto a copy of the frame. With a
    calibration (camera_matrix, dist_coeffs, tag_size_m), also a 3D box on each tag."""
    img = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR) if frame.ndim == 2 else frame.copy()
    h, w = img.shape[:2]
    scale = max(0.5, w / 1280.0)
    thick = max(1, int(round(2 * scale)))
    font = cv2.FONT_HERSHEY_SIMPLEX
    boxes = camera_matrix is not None and dist_coeffs is not None and tag_size_m

    for d in detections:
        pts = [(int(d.getCorner(k).x), int(d.getCorner(k).y)) for k in range(4)]
        color = (0, 220, 0) if d.getId() in used_ids else (0, 165, 255)  # green used, orange filtered
        for k in range(4):
            cv2.line(img, pts[k], pts[(k + 1) % 4], color, thick, cv2.LINE_AA)
        cube = tag_box(d, camera_matrix, dist_coeffs, tag_size_m) if boxes else None
        if cube is not None:
            c = [(int(x), int(y)) for x, y in cube]
            for k in range(4):
                cv2.line(img, c[k], c[k + 4], color, max(1, thick - 1), cv2.LINE_AA)  # pillars
                cv2.line(img, c[k + 4], c[(k + 1) % 4 + 4], color, thick, cv2.LINE_AA)  # top face
        cv2.circle(img, pts[0], thick * 2, (0, 0, 255), -1)  # corner 0 (bottom-left)
        label = f"id {d.getId()}"
        fs = 0.6 * scale
        (tw, th), base = cv2.getTextSize(label, font, fs, max(1, thick - 1))
        x = int(min(p[0] for p in pts))
        y = int(min(p[1] for p in pts)) - int(6 * scale)
        y = max(y, th + base + 2)
        cv2.rectangle(img, (x, y - th - base), (x + tw + 6, y + base), (0, 0, 0), -1)
        cv2.putText(img, label, (x + 3, y), font, fs, color, max(1, thick - 1), cv2.LINE_AA)

    lines = list(hud)
    if result is not None:
        p = result.robot_pose
        lines.append(f"pose x={p.X():.2f} y={p.Y():.2f} th={p.rotation().Z() * 57.2958:.1f}deg "
                     f"| tags {len(result.tag_ids)} | {result.avg_tag_distance:.1f} m")
    elif reject:
        lines.append(f"rejected: {reject}")

    line_h = int(26 * scale)
    box_h = line_h * len(lines) + int(10 * scale)
    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (w, box_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, img, 0.45, 0, img)
    for i, text in enumerate(lines):
        color = (80, 80, 255) if text.startswith("rejected") else (240, 240, 240)
        cv2.putText(img, text, (int(8 * scale), line_h * (i + 1)), font, 0.6 * scale, color,
                    max(1, thick - 1), cv2.LINE_AA)
    return img
