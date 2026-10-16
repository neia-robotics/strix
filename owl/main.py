"""Multi-camera AprilTag coprocessor.

Spawns one process per camera, collects their pose observations, and publishes
them as ONE NT4 StructArray topic. Publishing happens as soon as any camera has a
result (anything that arrived meanwhile is batched in) -- no fixed publish rate,
so no added latency and no waiting on the slowest camera.

Also: live-tunable settings over NT with a save button (settings.py), and an MJPEG
stream per camera registered under /CameraPublisher so dashboards find it.

    python3 main.py --config config.json
    python3 main.py --config config.json --standalone   # bench test with no robot
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import multiprocessing as mp
import os
import queue
import signal
import socket
import threading
import time

import admin
from config import load_config, write_config
from network import apply_system, ethernet_interfaces, nt_target_text, parse_nt_target, system_support, validate_network
from recovery import needs_restart, needs_cpu_fallback


def local_ips() -> list[str]:
    """Best-effort list of this machine's IPv4 addresses (for dashboard stream URLs).

    No hostname/DNS lookup: on a robot network with no DNS server that can hang for
    seconds. A UDP connect sends nothing and only asks the kernel which local address
    routes to the target, so it returns immediately.
    """
    ips = set()
    for target in ("10.255.255.255", "8.8.8.8"):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect((target, 1))
            ips.add(s.getsockname()[0])
            s.close()
        except OSError:
            pass
    real = sorted(ip for ip in ips if not ip.startswith("127."))
    return real or ["127.0.0.1"]


class IpWatcher:
    """Refreshes local_ips() (and the wired interfaces for the networking page) in a
    background thread so the publishing loop never waits on it."""

    def __init__(self, period_s: float = 5.0):
        self._refresh()
        self._period = period_s
        threading.Thread(target=self._run, name="ip-watcher", daemon=True).start()

    def _refresh(self):
        self.ips = local_ips()
        self.interfaces = ethernet_interfaces()
        self.support = system_support()

    def _run(self):
        while True:
            time.sleep(self._period)
            try:
                self._refresh()
            except Exception:
                pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.json")
    ap.add_argument("--standalone", action="store_true",
                    help="run our own NT server (bench testing without a roboRIO)")
    ap.add_argument("--server", help="NT server IP override (e.g. 127.0.0.1 for sim)")
    ap.add_argument("--nt-port", type=int, default=5810, help="NT4 port for isolated bench tests")
    args = ap.parse_args()
    if not 1024 <= args.nt_port <= 65535:
        ap.error("--nt-port must be between 1024 and 65535")

    cfg = load_config(args.config)
    if args.standalone:
        cfg.standalone = True
    if args.server:
        cfg.server = args.server

    # Start camera processes BEFORE initializing NetworkTables ("spawn" also keeps
    # them from inheriting any NT/thread state from this process).
    from camera_process import run_camera

    ctx = mp.get_context("spawn")
    q = ctx.Queue(maxsize=256)
    procs, ctrls, pending_controls = {}, {}, {}
    last_seen = {}
    reported = set()
    cpu_only = set()
    save_replies = []
    restarts = {c.id: 0 for c in cfg.cameras}  # for the web page's diagnostics tab
    owl_started = time.time()

    # Every camera's page lists all cameras so one page can switch between them.
    peers = [{"name": c.name, "port": c.stream_port} for c in cfg.cameras]

    settings = None  # LiveSettings, once NT is up; until then cameras start in their default pipeline

    def start(cam):
        """Starts a camera process in its active pipeline (on a restart, whatever was switched to)."""
        pipe = (settings.pipes[cam.id][settings.active[cam.id]] if settings is not None
                else next(p for p in cam.pipelines if p["name"] == cam.pipeline))
        w, h = (int(x) for x in pipe["resolution"].split("x"))
        cam = dataclasses.replace(cam, width=w, height=h, exposure=pipe["exposure"], gain=pipe["gain"],
                                  std_dev_factor=pipe["std_dev_factor"])
        old = ctrls.get(cam.id)
        if old is not None:
            old.cancel_join_thread()
            old.close()
        ctrls[cam.id] = ctx.Queue(maxsize=4)
        pending_controls[cam.id] = {}
        detector = dataclasses.replace(cfg.detector, quad_decimate=pipe["quad_decimate"],
                                       min_cluster_pixels=pipe["min_cluster_pixels"])
        if cam.id in cpu_only:
            detector = dataclasses.replace(detector, backend="cpu")
        solver = dataclasses.replace(cfg.solver, **{k: pipe[k] for k in (
            "min_decision_margin", "max_single_tag_ambiguity", "max_reprojection_error_px", "max_tag_distance_m")})
        last_seen[cam.id] = time.monotonic()
        reported.discard(cam.id)
        p = ctx.Process(target=run_camera, name=cam.name, daemon=True,
                        args=(cam, cfg.field_layout, detector, solver, cfg.stream,
                              q, ctrls[cam.id], peers))
        p.start()
        procs[cam.id] = p

    def send_control(cam_id, changes):
        if "_acks" in changes:
            changes = dict(changes)
            pending_controls[cam_id].setdefault("_acks", []).extend(changes.pop("_acks"))
        pending_controls[cam_id].update(changes)
        try:
            ctrls[cam_id].put_nowait(pending_controls[cam_id])
        except queue.Full:
            return  # merge and retry next loop; a stuck worker cannot grow the queue
        pending_controls[cam_id] = {}

    for cam in cfg.cameras:
        start(cam)

    import ntcore
    from wpimath.geometry import Pose3d

    from messages import tuple_to_pose
    from settings import LiveSettings
    from vision_types import VisionObservation

    inst = ntcore.NetworkTableInstance.getDefault()

    def nt_target():
        return cfg.server or (f"team {cfg.team}" if cfg.team is not None else None)

    def start_nt():
        """(Re)starts NT from cfg. Publishers and listeners carry over a restart."""
        inst.stopServer()
        inst.stopClient()
        if cfg.standalone:
            inst.startServer("networktables.json", "", 0, args.nt_port)
            print("NT: running standalone server", flush=True)
            return
        inst.startClient4(cfg.client_name)
        if cfg.server:
            inst.setServer(cfg.server, args.nt_port)
        elif cfg.team is not None:
            inst.setServerTeam(cfg.team)
        # No target: keep running so the target can be set from the coprocessor page.
        print(f"NT: client -> {nt_target()}" if nt_target() else
              "NT: no team number or server address; set one on the coprocessor page", flush=True)

    start_nt()

    # The Pi has no real-time clock or internet, so its own timestamps are meaningless.
    # NT keeps the offset to the robot's clock, so log lines carry the robot's time
    # (the same time base as the robot's .wpilog) to line them up with robot logs.
    def robot_time() -> str:
        offset = inst.getServerTimeOffset()
        return "robot t=?" if offset is None else f"robot t={(ntcore._now() + offset) / 1e6:.2f}s"

    time_synced = [False]

    def on_connection(event):
        info = event.data
        connected = bool(event.flags & ntcore.EventFlags.kConnected)
        time_synced[0] = False
        print(f"NT: {'CONNECTED to' if connected else 'DISCONNECTED from'} {info.remote_id or 'robot'} "
              f"({info.remote_ip}) at {robot_time()}", flush=True)

    def on_time_sync(event):
        if event.data.valid and not time_synced[0]:  # once per connection, not every resync
            time_synced[0] = True
            print(f"NT: clock synced to robot, {robot_time()}", flush=True)

    inst.addConnectionListener(True, on_connection)
    inst.addTimeSyncListener(True, on_time_sync)

    # periodic: how often NT sends if a flush() below is rate-limited away (default 100 ms).
    opts = ntcore.PubSubOptions(sendAll=True, keepDuplicates=True, periodic=0.005)
    obs_pub = inst.getStructArrayTopic(cfg.topic, VisionObservation).publish(opts)
    base = cfg.topic.rsplit("/", 1)[0] or "/Vision"
    heartbeat_pub = inst.getIntegerTopic(f"{base}/heartbeat").publish()
    # Published by the robot (batched_vision.py): max detection fps, -1 = unlimited.
    throttle_sub = inst.getDoubleTopic(f"{base}/throttleFps").subscribe(-1.0)
    throttle = -1.0
    settings = LiveSettings(inst, cfg, base)

    stat_pubs = {}
    for cam in cfg.cameras:
        t = inst.getTable(f"{base}/cameras/{cam.name}")
        t.getIntegerTopic("id").publish().set(cam.id)
        stat_pubs[cam.id] = {
            "connected": t.getBooleanTopic("connected").publish(),
            "fps": t.getDoubleTopic("fps").publish(),
            "processMs": t.getDoubleTopic("processMs").publish(),
            "tagFps": t.getDoubleTopic("tagFps").publish(),
            "streamFps": t.getDoubleTopic("streamFps").publish(),
            "streamViewers": t.getIntegerTopic("streamViewers").publish(),
            "lastReject": t.getStringTopic("lastReject").publish(),
            # Same pose as in the batched topic, as a plain Pose3d so AdvantageScope can
            # drag it straight onto a field view. Debug only; the robot reads the batch.
            "robotPose": t.getStructTopic("robotPose", Pose3d).publish(),
        }

    # Register streams where Elastic / Shuffleboard look for cameras.
    stream_pubs = {}
    if cfg.stream.get("enabled", True):
        for cam in cfg.cameras:
            for suffix, path in (("", "stream.mjpg"), ("_raw", "raw.mjpg")):
                t = inst.getTable(f"/CameraPublisher/{cam.name}{suffix}")
                stream_pubs[(cam.id, path)] = t.getStringArrayTopic("streams").publish()
                t.getStringTopic("source").publish().set(f"vision:{cam.name}")
    last_ips = None
    ip_watcher = IpWatcher()
    net_apply = {"status": "", "error": False, "busy": False}  # hostname/IP helper, off the main loop

    def apply_network_in_background(net):
        net_apply.update(status="applying hostname and IP…", error=False, busy=True)

        def run():
            time.sleep(1.0)  # let the page's request finish first: its address may be about to change
            try:
                net_apply.update(status=apply_system(net), error=False)
            except RuntimeError as e:
                net_apply.update(status=f"hostname/IP not applied: {e}", error=True)
            net_apply["busy"] = False
            print(f"network: {net_apply['status']}", flush=True)

        threading.Thread(target=run, name="network-apply", daemon=True).start()

    def set_network(data):
        """Save from the coprocessor page. The NT target applies now; hostname and IP go
        through the root helper when "manage" is on. Returns (ok, status)."""
        try:
            team, server = parse_nt_target(data.get("nt_server"))
            standalone = data.get("standalone")
            if not isinstance(standalone, bool):
                raise ValueError("standalone must be true or false")
            if not standalone and team is None and server is None:
                raise ValueError("set a team number or NetworkTables server address")
            net = validate_network(data.get("network", {}))
            if net["manage"] and net_apply["busy"]:
                raise ValueError("still applying the last hostname/IP change")
        except (TypeError, ValueError) as e:
            return False, f"not saved: {e}"
        try:
            with open(cfg.path) as f:
                raw = json.load(f)
            raw.setdefault("nt", {}).update(team=team, server=server, standalone=standalone)
            raw["network"] = net
            write_config(cfg.path, raw)
        except (OSError, ValueError) as e:
            return False, f"save failed: {e}"
        if (team, server, standalone) != (cfg.team, cfg.server, cfg.standalone):
            cfg.team, cfg.server, cfg.standalone = team, server, standalone
            start_nt()
        cfg.network = net
        print(f"network: saved -> {cfg.path}", flush=True)
        if net["manage"]:
            apply_network_in_background(net)
            return True, "saved; applying hostname and IP…"
        net_apply.update(status="", error=False)
        return True, f"saved {time.strftime('%H:%M:%S')}"

    def shutdown(*_):
        for p in procs.values():
            p.terminate()
        for p in procs.values():  # let each camera release its device (a killed one can wedge it)
            p.join(timeout=2)
        os._exit(0)

    restart_at = [None]  # "restart software": exit and let systemd's Restart=always start us again

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # Give every camera the full current settings so its web page can show them
    # (after that it only receives changes).
    def initial_settings(cam_id):
        return {**settings.full_camera_settings(cam_id), "_status": settings.status_text,
                "throttle_fps": throttle}

    for cam in cfg.cameras:
        send_control(cam.id, initial_settings(cam.id))
    last_status = settings.status_text

    heartbeat = 0
    last_health = last_settings = 0.0
    # Per-camera robotPose topics are for viewing only (the robot reads the batch), so
    # they're published at 10 Hz instead of every frame to cut NT traffic on the robot.
    POSE_DEBUG_PERIOD_S = 0.1
    last_pose_pub = {cam.id: 0.0 for cam in cfg.cameras}
    # A loop pass taking longer than this gets logged with where the time went, so a
    # freeze shows up in `journalctl -u vision`.
    STALL_WARN_S = 0.5
    last_stall_print = 0.0

    def handle(msg, batch):
        if msg[0] == "obs":
            batch.append(msg[1])
        elif msg[0] == "stats":
            _, cam_id, connected, fps, proc_ms, tag_fps, stream_fps, viewers, reject, initialized = msg
            last_seen[cam_id] = time.monotonic()
            if initialized:
                reported.add(cam_id)
            p = stat_pubs[cam_id]
            p["connected"].set(bool(connected)); p["fps"].set(fps); p["processMs"].set(proc_ms)
            p["tagFps"].set(tag_fps); p["streamFps"].set(stream_fps)
            p["streamViewers"].set(viewers); p["lastReject"].set(reject)
        elif msg[0] == "set":  # from a camera's web settings page
            ok = settings.set_from_web(msg[1], msg[2])
            send_control(msg[1], {"_acks": [{"id": msg[3], "ok": ok, "status": settings.status_text}]})
        elif msg[0] == "restart":  # from the coprocessor page
            # Exiting only restarts Owl if it is a systemd service with Restart= (admin.py).
            blocker = admin.restart_blocker()
            if blocker is None:
                restart_at[0] = time.monotonic() + 1.0  # after the page gets its answer
                print("restart requested from the coprocessor page", flush=True)
            send_control(msg[1], {"_acks": [{"id": msg[3], "ok": blocker is None,
                                             "status": blocker or "restarting Owl…"}]})
        elif msg[0] == "pipeline":  # from a camera's tuning page
            try:
                ok, status = True, settings.pipeline_op(msg[1], msg[2])
            except ValueError as e:
                ok, status = False, str(e)
            send_control(msg[1], {"_acks": [{"id": msg[3], "ok": ok, "status": status}]})
        elif msg[0] == "network":  # from the coprocessor page
            ok, status = set_network(msg[2] or {})
            send_control(msg[1], {"_acks": [{"id": msg[3], "ok": ok, "status": status}]})
        elif msg[0] == "save":
            settings.request_save()
            save_replies.append((msg[1], msg[3]))

    while True:
        t_loop = time.monotonic()
        batch = []
        try:
            handle(q.get(timeout=0.1), batch)
            while True:  # batch in anything else that's already waiting
                handle(q.get_nowait(), batch)
        except queue.Empty:
            pass

        if batch:
            now_mono = time.monotonic()
            observations = [
                VisionObservation(
                    cameraId=o.camera_id,
                    robotPose=tuple_to_pose(o.robot_pose),
                    altRobotPose=tuple_to_pose(o.alt_robot_pose),
                    ageSeconds=now_mono - o.capture_time,
                    stdDevFactor=o.std_dev_factor,
                    tagCount=o.tag_count,
                    avgTagDistance=o.avg_tag_distance,
                    ambiguity=o.ambiguity,
                    tagMask=o.tag_mask,
                )
                for o in batch
            ]
            obs_pub.set(observations, ntcore._now())
            # Send now. Otherwise NT batches changes and sends them every 100 ms, which
            # measured ~70 ms of added latency. (flush() is rate-limited by ntcore.)
            inst.flush()
            for o in observations:
                if now_mono - last_pose_pub[o.cameraId] >= POSE_DEBUG_PERIOD_S:
                    last_pose_pub[o.cameraId] = now_mono
                    stat_pubs[o.cameraId]["robotPose"].set(o.robotPose)
        t_published = time.monotonic()

        now = time.monotonic()
        if now - last_settings > 0.1:  # live settings -> camera processes
            last_settings = now
            g, per_cam = settings.poll()
            for cam_id, request_id in save_replies:
                send_control(cam_id, {"_acks": [{"id": request_id,
                    "ok": settings.status_text.startswith("saved "), "status": settings.status_text}]})
            save_replies.clear()
            for cam_id in pending_controls:
                if pending_controls[cam_id]:
                    send_control(cam_id, {})
            for cam in cfg.cameras:
                msg = {**g, **per_cam.get(cam.id, {})}
                if msg:
                    send_control(cam.id, msg)
            new_throttle = throttle_sub.get()
            if math.isfinite(new_throttle) and (new_throttle == -1 or .1 <= new_throttle <= 240) and new_throttle != throttle:
                throttle = new_throttle
                for cam in cfg.cameras:
                    send_control(cam.id, {"throttle_fps": throttle})
            if settings.status_text != last_status:  # "unsaved changes" / "saved ..." on the web pages
                last_status = settings.status_text
                for cam in cfg.cameras:
                    send_control(cam.id, {"_status": last_status})
        t_settings = time.monotonic()

        if now - last_health > 1.0:
            last_health = now
            heartbeat += 1
            heartbeat_pub.set(heartbeat)
            for cam in cfg.cameras:  # restart crashed camera processes with current settings
                if needs_restart(procs[cam.id].is_alive(), now, last_seen[cam.id], cam.id in reported):
                    stat_pubs[cam.id]["connected"].set(False)
                    stat_pubs[cam.id]["fps"].set(0)
                    old = procs[cam.id]
                    native_failure = needs_cpu_fallback(old.exitcode)
                    if old.is_alive():
                        old.terminate()
                    old.join(timeout=.5)
                    if old.is_alive():
                        old.kill()
                        old.join(timeout=.5)
                    if cfg.detector.backend == "cuda" and native_failure:
                        cpu_only.add(cam.id)
                    print(f"[{cam.name}] process died or stalled; restarting"
                          + (" on CPU" if cam.id in cpu_only else ""), flush=True)
                    restarts[cam.id] += 1
                    start(cam)
                    send_control(cam.id, initial_settings(cam.id))
            ips = ip_watcher.ips  # IP can change after boot (robot radio DHCP); refreshed in background
            # Diagnostics for the web pages: NetworkTables and every camera process.
            system = {
                "started": owl_started,
                "ips": ips,
                "nt": {
                    "mode": "standalone server" if cfg.standalone else "client",
                    "target": None if cfg.standalone else (nt_target() or "not set"),
                    "connected": bool(inst.isConnected()) or cfg.standalone,
                    "peers": [{"id": c.remote_id, "ip": c.remote_ip} for c in inst.getConnections()],
                    "time_synced": time_synced[0],
                },
                "throttle_fps": throttle,
                "network": {  # the coprocessor page's networking form
                    "nt_server": nt_target_text(cfg.team, cfg.server),
                    "standalone": cfg.standalone,
                    "settings": cfg.network,
                    "interfaces": ip_watcher.interfaces,
                    "unsupported": ip_watcher.support,
                    **net_apply,
                },
                "cameras": [{"name": c.name, "port": c.stream_port, "alive": procs[c.id].is_alive(),
                             "cpu_fallback": c.id in cpu_only, "pid": procs[c.id].pid, "restarts": restarts[c.id]} for c in cfg.cameras],
            }
            for cam in cfg.cameras:
                send_control(cam.id, {"_system": system})
            if ips != last_ips:
                last_ips = ips
                for (cam_id, path), pub in stream_pubs.items():
                    port = cfg.cameras[cam_id].stream_port
                    pub.set([f"mjpg:http://{ip}:{port}/{path}" for ip in ips])

        if restart_at[0] is not None and time.monotonic() >= restart_at[0]:
            shutdown()
        t_end = time.monotonic()
        if t_end - t_loop > STALL_WARN_S and t_end - last_stall_print > 5.0:
            last_stall_print = t_end
            print(f"WARNING: main loop stalled {t_end - t_loop:.2f} s at {robot_time()} (queue+publish {t_published - t_loop:.2f}, "
                  f"settings {t_settings - t_published:.2f}, health/restarts/IPs {t_end - t_settings:.2f})", flush=True)


if __name__ == "__main__":
    main()
