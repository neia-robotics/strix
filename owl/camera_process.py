"""One process per camera: capture -> detect -> solve -> send result to the publisher.

A process (not a thread) per camera guarantees the cameras run in parallel on
separate CPU cores, independent of Python's GIL.

Inside each process:
  - a grabber thread keeps only the newest frame (no stale backlog) and applies
    exposure/gain and resolution changes,
  - an MJPEG server (stream_server.py) serves annotated + raw streams, encoding
    only while someone is watching, plus the settings/calibration page,
  - live settings arrive from the main process over a control queue, including the
    robot's throttle request (detect at ~1 fps after it's been disabled a while;
    paused while someone has this camera's page open or is calibrating).

A camera with no calibration for its current resolution keeps running: it streams
and detects tags but publishes no poses until it's calibrated from its page.
"""

from __future__ import annotations

import dataclasses
import json
import math
import os
import queue
import sys
import threading
import time
import traceback
import uuid

import cv2
import numpy as np

from calibration_session import CalibrationSession, board_png
from camera_controls import apply_exposure_gain, control_ranges, list_modes
from config import (CameraConfig, calibration_path, find_calibration, load_field_layout,
                    robot_to_camera_transform)
from messages import PoseObservation, pose_to_tuple
from pose_solver import DetectorSettings, PoseSolver, SolverSettings, make_detector
import sysinfo
from quality_recording import QualityRecorder
from recovery import CAPTURE_TIMEOUT_S
from stream_server import MjpegStreamServer, annotate

STATS_PERIOD_S = 1.0
DETECTOR_KEYS = {f.name for f in dataclasses.fields(DetectorSettings)}
SOLVER_KEYS = {f.name for f in dataclasses.fields(SolverSettings)}


def is_video_file(device) -> bool:
    """Camera = an index (0, "1") or a /dev path. Anything else is treated as a video file."""
    if isinstance(device, int):
        return False
    return not device.startswith("/dev/") and not device.isdigit()


def open_camera(device) -> cv2.VideoCapture:
    """Open a camera with the right backend for this OS."""
    if isinstance(device, str) and device.isdigit():
        device = int(device)
    if sys.platform == "darwin":
        return cv2.VideoCapture(device, cv2.CAP_AVFOUNDATION)  # macOS: use indices (0, 1, ...)
    if sys.platform.startswith("linux"):
        return cv2.VideoCapture(device, cv2.CAP_V4L2)
    return cv2.VideoCapture(device)  # Windows: let OpenCV choose


class LatestFrameGrabber(threading.Thread):
    def __init__(self, cam: CameraConfig):
        super().__init__(daemon=True)
        self.cam = cam
        self.cond = threading.Condition()
        self.frame = None
        self.capture_time = 0.0
        self.seq = 0  # frames decoded and handed over
        self.grabbed = 0  # frames the camera delivered (decoded or not)
        self.connected = False
        self.last_progress = time.monotonic()
        self.running = True
        # Decode at most one frame per this many seconds (0 = every frame). Set while
        # throttled: grab() still drains the driver so frames stay fresh, but skipping
        # the MJPEG decode saves most of the capture CPU.
        self.min_decode_period = 0.0
        self._last_decode = 0.0
        self._controls = (cam.exposure, cam.gain)
        self._controls_dirty = True
        self._reopen = False
        self._ctl_lock = threading.Lock()

    def set_controls(self, exposure, gain):
        with self._ctl_lock:
            self._controls = (exposure, gain)
            self._controls_dirty = True

    def set_resolution(self, width: int, height: int):
        with self._ctl_lock:
            self.cam = dataclasses.replace(self.cam, width=width, height=height)
            self._reopen = True

    def _log(self, msg):
        print(f"[{self.cam.name}] {msg}", flush=True)

    def _open(self) -> cv2.VideoCapture | None:
        c = self.cam
        is_file = is_video_file(c.device)
        cap = cv2.VideoCapture(c.device) if is_file else open_camera(c.device)
        if not cap.isOpened():
            return None
        if not is_file:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*c.fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, c.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, c.height)
            cap.set(cv2.CAP_PROP_FPS, c.fps)
            # Two buffers: the camera fills one while we decode the other. With one, the
            # driver has nowhere to put the next frame during decode and every other frame
            # is lost (50 fps -> 25). The grabber reads continuously, so at most one frame
            # waits, and capture time comes from the driver's timestamp anyway (run()).
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if (w, h) != (c.width, c.height):
                self._log(f"WARNING: asked for {c.width}x{c.height}, camera gave {w}x{h}. "
                          f"Pick a resolution the camera lists on its settings page.")
        with self._ctl_lock:
            self._controls_dirty = True  # re-apply controls after every (re)open
        return cap

    def run(self):
        c = self.cam
        is_file = is_video_file(c.device)
        frame_period = 1.0 / c.fps if is_file else 0.0
        cap = None
        try:
            while self.running:
                self.last_progress = time.monotonic()
                with self._ctl_lock:
                    reopen, self._reopen = self._reopen, False
                if reopen and cap is not None:
                    cap.release()
                    cap = None
                if cap is None:
                    cap = self._open()
                    if cap is None:
                        self.connected = False
                        time.sleep(1.0)
                        continue
                    self.connected = True
                if not is_file:
                    with self._ctl_lock:
                        dirty, (exposure, gain) = self._controls_dirty, self._controls
                        self._controls_dirty = False
                    if dirty:
                        apply_exposure_gain(cap, c.device, exposure, gain, self._log)
                ok = cap.grab()
                t = time.monotonic()  # capture time; shared clock with the publisher process
                if ok and not is_file:
                    # Prefer the driver's timestamp of when the frame was captured (on Linux
                    # V4L2 it's CLOCK_MONOTONIC, the same clock as time.monotonic()), so a frame
                    # that waited in the buffer still gets its true capture time. Only trusted
                    # if it's plausible; otherwise keep the grab time.
                    ts = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
                    if 0.0 <= t - ts < 0.25:
                        t = ts
                if ok:
                    self.grabbed += 1
                    if not is_file and time.monotonic() - self._last_decode < self.min_decode_period:
                        continue
                    self._last_decode = time.monotonic()
                    ok, frame = cap.retrieve()
                if not ok:
                    if is_file and c.loop_video:
                        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        continue
                    self._log("frame grab failed, reconnecting")
                    cap.release()
                    cap = None
                    self.connected = False
                    time.sleep(0.5)
                    continue
                if is_file and (frame.shape[1], frame.shape[0]) != (self.cam.width, self.cam.height):
                    frame = cv2.resize(frame, (self.cam.width, self.cam.height))  # test videos follow resolution
                with self.cond:
                    self.frame, self.capture_time, self.seq = frame, t, self.seq + 1
                    self.cond.notify()
                if frame_period:
                    time.sleep(frame_period)  # play test videos at real-time speed
        except Exception as e:
            self._log(f"capture thread failed: {e}")
        finally:
            self.connected = False
            self.running = False
            if cap is not None:
                cap.release()
            with self.cond:
                self.cond.notify_all()

    def healthy(self, now=None):
        now = time.monotonic() if now is None else now
        return self.connected and self.is_alive() and now - self.last_progress < 5.0

    def wait_for_frame(self, last_seq: int, timeout: float = 0.5):
        with self.cond:
            if self.seq == last_seq:
                self.cond.wait(timeout)
            if self.seq == last_seq:
                return None
            return self.frame, self.capture_time, self.seq


def run_camera(cam: CameraConfig, layout_name: str, det_settings: DetectorSettings,
               solver_settings: SolverSettings, stream_cfg: dict, out_queue, ctrl_queue,
               peers: list | None = None) -> None:
    try:
        CameraWorker(cam, layout_name, det_settings, solver_settings, stream_cfg, out_queue,
                     ctrl_queue, peers or []).run()
    except Exception:
        print(f"[{cam.name}] crashed:\n{traceback.format_exc()}", flush=True)
        raise


def _board_from_values(values: dict) -> dict:
    return {
        "squares_x": values["board_squares_x"],
        "squares_y": values["board_squares_y"],
        "square_length_m": values["board_square_length_m"],
        "marker_length_m": values["board_marker_length_m"],
        "dict": values["board_dict"],
        "legacy": values["board_legacy"],
    }


class CameraWorker:
    """Everything one camera process does. Each loop: apply queued settings and
    calibration commands, wait for the newest frame, then either feed the calibration
    session or run detect -> solve -> publish, and stream if someone is watching.

    The web page's HTTP threads call web_state / web_action / board_image; they only
    read attributes that are replaced whole (never mutated), so no lock is needed.
    """

    def __init__(self, cam: CameraConfig, layout_name: str, det_settings: DetectorSettings,
                 solver_settings: SolverSettings, stream_cfg: dict, out_queue, ctrl_queue,
                 peers: list):
        self.cam = cam
        self.out_queue = out_queue
        self.ctrl_queue = ctrl_queue
        self.peers = peers
        self.web_requests = {}
        self.web_lock = threading.Lock()

        self.recorder = QualityRecorder(cam.id)
        self.solver = PoseSolver(load_field_layout(layout_name), solver_settings)
        self.det_settings = det_settings
        self.detector = make_detector(det_settings, self.log)
        self.r2c = robot_to_camera_transform(cam.robot_to_camera)
        self.std_dev_factor = cam.std_dev_factor
        self.exposure, self.gain = cam.exposure, cam.gain
        self.stream_max_fps = float(stream_cfg.get("max_fps", 30))
        self.throttle_fps = -1.0  # the robot lowers this after it's been disabled a while

        # Calibration for the resolution frames actually arrive at (None = uncalibrated).
        self.K = self.D = None
        self.calib_info = None
        self.active_res = None

        self.calib: CalibrationSession | None = None
        self.calib_msg = ""
        self.calib_cmds: queue.Queue = queue.Queue()  # from the web page's HTTP threads

        # What the web settings page shows.
        self.ui = {"values": {}, "stats": {}, "ranges": None, "modes": None}

        # Counters for the current stats period, and what the last period measured.
        self.frames = self.tagged = 0
        self.initialized = False
        self.proc_total = 0.0
        self.last_stats = time.monotonic()
        self.grabbed_at_stats = self.encoded_at_stats = 0
        self.fps_shown = self.proc_shown = 0.0
        self.last_reject = ""
        self.last_res = None  # last accepted pose this stats period (for the page)
        self.last_targets = None  # (detections, used tag ids, (w, h)) from the latest detection
        self.last_stream = 0.0
        self.last_detect = 0.0

        self.stream = None
        if stream_cfg.get("enabled", True):
            self.stream = MjpegStreamServer(
                cam.name, cam.stream_port, int(stream_cfg.get("jpeg_quality", 80)),
                self.web_state, self.web_action,
                {"/api/board.png": self.board_image, "/api/system": self.system_info})
        self.grabber = LatestFrameGrabber(cam)

    def log(self, msg: str) -> None:
        print(f"[{self.cam.name}] {msg}", flush=True)

    # ---- web page (called from HTTP threads) ---------------------------------

    def web_state(self) -> dict:
        ui = self.ui
        if ui["ranges"] is None:
            ui["ranges"] = control_ranges(self.cam.device)
        if ui["modes"] is None:
            ui["modes"] = list_modes(self.cam.device, self.cam.fourcc)
        session = self.calib
        try:
            cal_state = session.status() if session else None
        except Exception:  # raced with the camera loop; next poll gets it
            cal_state = None
        return {"name": self.cam.name, "cameras": self.peers, "values": ui["values"],
                "stats": ui["stats"], "ranges": ui["ranges"], "modes": ui["modes"],
                "calibration": cal_state, "calib_info": self.calib_info,
                "calib_msg": self.calib_msg, "detector": self.detector_text()}

    def system_info(self):
        """/api/system: this coprocessor's stats, plus what the main process reports
        (NetworkTables, every camera process) and this camera's detector."""
        info = sysinfo.snapshot()
        info["owl"] = self.ui["values"].get("_system", {})
        info["camera"] = {"name": self.cam.name, "detector": self.detector_text()}
        return "application/json", json.dumps(info).encode()

    def detector_text(self) -> str:
        if self.det_settings.backend == "cpu":
            return "cpu (wpilib)"
        det = self.detector
        return f"{det.backend} on gpu" if det.on_gpu else "cuda requested, running on cpu"

    def targets(self) -> list[dict]:
        """Per-tag rows for the page's targets table (PhotonVision-style)."""
        if not self.last_targets:
            return []
        detections, used, (w, h) = self.last_targets
        s = self.solver.s
        rows = []
        for d in detections:
            tid = d.getId()
            pts = np.array([[d.getCorner(k).x, d.getCorner(k).y] for k in range(4)], dtype=np.float64)
            area = abs(cv2.contourArea(pts.astype(np.float32))) / (w * h) * 100.0
            bl, br = pts[0], pts[1]
            skew = math.degrees(math.atan2(br[1] - bl[1], br[0] - bl[0]))
            row = {"id": tid, "area": round(area, 3), "skew": round(skew, 1),
                   "margin": round(d.getDecisionMargin(), 1), "yaw": None, "pitch": None, "dist": None}
            if self.K is not None:
                c = cv2.undistortPoints(np.array([[[d.getCenter().x, d.getCenter().y]]]), self.K, self.D).ravel()
                row["yaw"] = round(-math.degrees(math.atan(c[0])), 2)  # + left (WPILib, CCW)
                row["pitch"] = round(-math.degrees(math.atan2(c[1], math.hypot(1.0, c[0]))), 2)  # + up
                half = s.tag_size_m / 2.0
                obj = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]])
                ok, _, tvec = cv2.solvePnP(obj, pts[[3, 2, 1, 0]], self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if ok:
                    row["dist"] = round(float(np.linalg.norm(tvec)), 2)
            if tid in used:
                row["status"] = "used"
            elif tid not in self.solver.tag_corners:
                row["status"] = "not on field"
            elif d.getHamming() > s.max_hamming:
                row["status"] = f"hamming {d.getHamming()}"
            elif d.getDecisionMargin() < s.min_decision_margin:
                row["status"] = "low margin"
            else:
                row["status"] = "frame rejected"
            rows.append(row)
        rows.sort(key=lambda r: r["id"])
        return rows

    def web_action(self, kind, data=None) -> None:
        if kind == "calib":
            self.calib_cmds.put(data or {})
            return
        # The main process owns settings (NT + save); it sends the result back via ctrl_queue.
        request_id = uuid.uuid4().hex
        event = threading.Event()
        reply = {}
        with self.web_lock:
            self.web_requests[request_id] = (event, reply)
        try:
            self.out_queue.put((kind, self.cam.id, data, request_id), timeout=1)
            if not event.wait(3):
                raise TimeoutError("settings owner did not acknowledge the request")
            if not reply["ok"]:
                raise ValueError(reply["status"])
        finally:
            with self.web_lock:
                self.web_requests.pop(request_id, None)

    def board_image(self):
        return "image/png", board_png(_board_from_values(self.ui["values"]))

    # ---- main loop -----------------------------------------------------------

    def run(self) -> None:
        if self.stream:
            self.stream.start()
        self.grabber.start()
        self.log(f"started on {self.cam.device}"
                 + (f", stream at http://<this-ip>:{self.cam.stream_port}/" if self.stream else ""))
        last_seq = 0
        while True:
            if not self.grabber.is_alive() or time.monotonic() - self.grabber.last_progress > CAPTURE_TIMEOUT_S:
                raise RuntimeError("capture thread stopped or stalled; restarting camera process")
            self.apply_settings()
            self.handle_calib_cmds()
            self.update_throttle()
            got = self.grabber.wait_for_frame(last_seq)
            now = time.monotonic()
            if now - self.last_stats >= STATS_PERIOD_S:
                self.report_stats(now)
            if got is None:
                continue
            frame, capture_time, last_seq = got
            self.process_frame(frame, capture_time, now)
            self.initialized = True  # includes lazy GPU detector creation on the first frame

    def apply_settings(self) -> None:
        """Live settings from the main process (NT / web page edits, robot throttle)."""
        try:
            while True:
                changes = self.ctrl_queue.get_nowait()
                for ack in changes.pop("_acks", []):
                    with self.web_lock:
                        pending = self.web_requests.get(ack["id"])
                        if pending:
                            event, reply = pending
                            reply.update(ack)
                            event.set()
                self.ui["values"] = {**self.ui["values"], **changes}
                det_changes = {k: v for k, v in changes.items() if k in DETECTOR_KEYS}
                if det_changes:
                    self.det_settings = dataclasses.replace(self.det_settings, **det_changes)
                    self.detector = make_detector(self.det_settings, self.log)
                for k, v in changes.items():
                    if k in SOLVER_KEYS and k != "tag_size_m":
                        setattr(self.solver.s, k, v)
                if "std_dev_factor" in changes:
                    self.std_dev_factor = changes["std_dev_factor"]
                if "stream_max_fps" in changes:
                    self.stream_max_fps = changes["stream_max_fps"]
                if "throttle_fps" in changes and float(changes["throttle_fps"]) != self.throttle_fps:
                    self.throttle_fps = float(changes["throttle_fps"])
                    self.log(f"throttled to {self.throttle_fps:g} fps (robot disabled)"
                             if self.throttle_fps > 0 else "throttle off")
                if "exposure" in changes or "gain" in changes:
                    self.exposure = changes.get("exposure", self.exposure)
                    self.gain = changes.get("gain", self.gain)
                    self.grabber.set_controls(self.exposure, self.gain)
                if "resolution" in changes:
                    self.set_resolution(*(int(x) for x in changes["resolution"].split("x")))
        except queue.Empty:
            pass

    def set_resolution(self, w: int, h: int) -> None:
        if (w, h) == (self.cam.width, self.cam.height):
            return
        self.cam = dataclasses.replace(self.cam, width=w, height=h)
        self.grabber.set_resolution(w, h)
        if self.calib is not None:
            self.calib = None
            self.calib_msg = "calibration cancelled: resolution changed"
        self.log(f"switching to {w}x{h}")

    def handle_calib_cmds(self) -> None:
        """Calibration commands from the web page."""
        try:
            while True:
                cmd = self.calib_cmds.get_nowait()
                op = cmd.get("op")
                try:
                    self._calib_op(op, cmd)
                except Exception as e:
                    self.calib_msg = f"{op} failed: {e}"
                    self.log(f"calibration {self.calib_msg}")
        except queue.Empty:
            pass

    def _calib_op(self, op, cmd: dict) -> None:
        calib = self.calib
        if op == "start":
            size = self.active_res or (self.cam.width, self.cam.height)
            self.calib = CalibrationSession(_board_from_values(self.ui["values"]), size,
                                            bool(cmd.get("wide_lens")))
            self.calib_msg = ""
        elif op == "stop":
            self.calib = None
            self.calib_msg = "calibration cancelled"
        elif calib is None:
            pass
        elif op == "capture":
            calib.request_capture()
        elif op == "undo":
            calib.undo()
        elif op == "clear":
            calib.clear()
        elif op == "auto":
            calib.auto = bool(cmd.get("on"))
        elif op == "calibrate":
            calib.start_calibration()
        elif op == "discard":
            calib.discard_result()
        elif op == "preview":
            calib.preview = bool(cmd.get("on"))
        elif op == "save":
            path = calibration_path(self.cam, calib.w, calib.h)
            saved = calib.save(path)
            self.calib = None
            self.calib_msg = f"saved {os.path.basename(path)} (rms {saved['rms']:.3f} px)"
            self.log(self.calib_msg)
            self.active_res = None  # reload on the next frame

    def _watched(self) -> bool:
        return self.stream is not None and (self.stream.wants("stream") or self.stream.wants("raw"))

    def throttle_active(self) -> bool:
        """The robot's throttle request applies only while the camera is idle: it's paused
        while someone has the page/stream open (aiming, focusing, tuning) or a calibration
        session is running, so those always get full frame rate."""
        return self.throttle_fps > 0 and self.calib is None and not self._watched()

    def update_throttle(self) -> None:
        """While throttled, don't even decode the frames we skip."""
        self.grabber.min_decode_period = 1.0 / self.throttle_fps if self.throttle_active() else 0.0

    def load_calibration_for(self, w: int, h: int) -> None:
        self.last_reject = ""  # drop a stale "not calibrated" (or other) reason
        found = find_calibration(self.cam, w, h)
        if found:
            self.K = np.array(found["K"], dtype=np.float64)
            self.D = np.array(found["D"], dtype=np.float64)
            self.calib_info = {"file": os.path.basename(found["path"]), "rms": found["rms"],
                               "calibrated_at": found["calibrated_at"]}
            self.log(f"{w}x{h}: using calibration {found['path']}")
        else:
            self.K = self.D = None
            self.calib_info = None
            self.log(f"{w}x{h}: NOT CALIBRATED (no poses until you calibrate "
                     f"from http://<this-ip>:{self.cam.stream_port}/)")

    def report_stats(self, now: float) -> None:
        dt = now - self.last_stats
        grabber, stream = self.grabber, self.stream
        self.fps_shown = self.frames / dt
        grabbed = grabber.grabbed  # frames the camera delivered (processed or not)
        cam_fps = (grabbed - self.grabbed_at_stats) / dt
        self.grabbed_at_stats = grabbed
        self.proc_shown = (self.proc_total / self.frames * 1000) if self.frames else 0.0
        enc = stream.encoded_count if stream else 0
        viewers = sum(ch.clients for ch in stream.channels.values()) if stream else 0
        self.out_queue.put(("stats", self.cam.id, grabber.healthy(now), self.fps_shown, self.proc_shown,
                            self.tagged / dt, (enc - self.encoded_at_stats) / dt, viewers,
                            self.last_reject, self.initialized))
        self.encoded_at_stats = enc
        pose_txt = ""
        if self.last_res is not None:
            p = self.last_res.robot_pose
            pose_txt = (f"x={p.X():.2f} y={p.Y():.2f} th={p.rotation().Z() * 57.2958:.1f}deg, "
                        f"tags {sorted(self.last_res.tag_ids)}")
        res = self.active_res
        self.ui["stats"] = {"connected": grabber.healthy(now), "fps": self.fps_shown,
                            "processMs": self.proc_shown, "tagFps": self.tagged / dt,
                            "lastReject": self.last_reject, "pose": pose_txt, "cameraFps": cam_fps,
                            "resolution": f"{res[0]}x{res[1]}" if res else None,
                            "calibrated": self.K is not None, "throttled": self.throttle_active(),
                            "targets": self.targets()}
        self.last_res = None  # only show a pose accepted during this stats period
        self.frames = self.tagged = 0
        self.proc_total = 0.0
        self.last_stats = now

    def process_frame(self, frame, capture_time: float, now: float) -> None:
        res_now = (frame.shape[1], frame.shape[0])
        if res_now != self.active_res:
            self.active_res = res_now
            self.load_calibration_for(*res_now)

        stream = self.stream
        want_stream = stream and (stream.wants("stream") or stream.wants("raw")) and (
            self.stream_max_fps <= 0 or now - self.last_stream >= 1.0 / self.stream_max_fps)
        if want_stream:
            self.last_stream = now
            if stream.wants("raw"):
                stream.submit("raw", frame)
        want_annotated = want_stream and stream.wants("stream")

        # ---- calibration mode: no pose pipeline, just the calibration session ----
        calib = self.calib
        if calib is not None and (calib.w, calib.h) == res_now:
            t0 = time.monotonic()
            det = calib.process(self._gray(frame), now)
            self._count_frame(t0)
            if want_annotated:
                stream.submit("stream", calib.draw(frame, det, now))
            return

        # ---- throttled (robot disabled a while, nobody watching): skip detection ----
        if self.throttle_active() and now - self.last_detect < 1.0 / self.throttle_fps:
            return
        self.last_detect = now

        t0 = time.monotonic()
        detections = self.detector.detect(self._gray(frame))
        res = None
        if self.K is None:
            reject = f"not calibrated at {res_now[0]}x{res_now[1]}: calibrate it on the calibration tab"
            self.last_reject = reject
        elif detections:
            res = self.solver.solve(detections, self.K, self.D, self.r2c, self.std_dev_factor)
            self.recorder.record(detections, self.solver, res, capture_time, self.K, self.D, self.r2c)
            reject = None if res else self.solver.last_reject
        else:
            reject = None
        self._count_frame(t0)
        self.last_targets = (detections, set(res.tag_ids) if res else set(), res_now)
        if res is None:
            if self.K is not None and self.solver.last_reject:
                self.last_reject = self.solver.last_reject
        else:
            self.tagged += 1
            self.last_res = res
            self.out_queue.put(("obs", PoseObservation(
                camera_id=self.cam.id, capture_time=capture_time,
                robot_pose=pose_to_tuple(res.robot_pose),
                alt_robot_pose=pose_to_tuple(res.alt_robot_pose),
                std_dev_factor=res.std_dev_factor, tag_count=len(res.tag_ids),
                avg_tag_distance=res.avg_tag_distance, ambiguity=res.ambiguity,
                tag_mask=res.tag_mask,
            )))

        # ---- stream: only does work while someone is watching ----
        if want_annotated:
            used = set(res.tag_ids) if res else set()
            hud = self._hud(res_now)
            if self.throttle_fps > 0:
                hud.append(f"robot disabled: {self.throttle_fps:g} fps throttle paused while this page is open")
            stream.submit("stream", annotate(frame, detections, used, res, reject, hud,
                                            self.K, self.D, self.solver.s.tag_size_m))

    @staticmethod
    def _gray(frame):
        return frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def _count_frame(self, t0: float) -> None:
        self.proc_total += time.monotonic() - t0
        self.frames += 1

    def _hud(self, res_now) -> list[str]:
        exp_txt = "auto" if self.exposure is None else f"{self.exposure:g}"
        return [f"{self.cam.name}  {res_now[0]}x{res_now[1]}  {self.fps_shown:.0f} fps  "
                f"detect {self.proc_shown:.1f} ms  exposure {exp_txt}  "
                + self._detector_hud()]

    def _detector_hud(self) -> str:
        if self.det_settings.backend == "cpu":
            return f"decimate {self.det_settings.quad_decimate:g}"
        det = self.detector
        return f"{det.backend} detector" if det.on_gpu else "cuda detector off (on cpu)"
