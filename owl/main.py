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
import multiprocessing as mp
import os
import queue
import signal
import socket
import threading
import time

from config import load_config


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
    """Refreshes local_ips() in a background thread so the publishing loop never waits on it."""

    def __init__(self, period_s: float = 5.0):
        self.ips = local_ips()
        self._period = period_s
        threading.Thread(target=self._run, name="ip-watcher", daemon=True).start()

    def _run(self):
        while True:
            time.sleep(self._period)
            try:
                self.ips = local_ips()
            except Exception:
                pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.json")
    ap.add_argument("--standalone", action="store_true",
                    help="run our own NT server (bench testing without a roboRIO)")
    ap.add_argument("--server", help="NT server IP override (e.g. 127.0.0.1 for sim)")
    args = ap.parse_args()

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
    procs, ctrls = {}, {}
    restarts = {c.id: 0 for c in cfg.cameras}  # for the web page's diagnostics tab
    owl_started = time.time()

    # Every camera's page lists all cameras so one page can switch between them.
    peers = [{"name": c.name, "port": c.stream_port} for c in cfg.cameras]

    def start(cam):
        ctrls[cam.id] = ctx.Queue()
        p = ctx.Process(target=run_camera, name=cam.name, daemon=True,
                        args=(cam, cfg.field_layout, cfg.detector, cfg.solver, cfg.stream,
                              q, ctrls[cam.id], peers))
        p.start()
        procs[cam.id] = p

    for cam in cfg.cameras:
        start(cam)

    import ntcore
    from wpimath.geometry import Pose3d

    from messages import tuple_to_pose
    from settings import LiveSettings
    from vision_types import VisionObservation

    inst = ntcore.NetworkTableInstance.getDefault()
    if cfg.standalone:
        inst.startServer()
        print("NT: running standalone server", flush=True)
    else:
        inst.startClient4(cfg.client_name)
        if cfg.server:
            inst.setServer(cfg.server)
        elif cfg.team is not None:
            inst.setServerTeam(cfg.team)
        else:
            raise SystemExit("config needs nt.team or nt.server (or use --standalone)")
        print(f"NT: client -> {cfg.server or f'team {cfg.team}'}", flush=True)

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

    def shutdown(*_):
        for p in procs.values():
            p.terminate()
        os._exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # Give every camera the full current settings so its web page can show them
    # (after that it only receives changes).
    def initial_settings(cam_id):
        return {**settings.full_camera_settings(cam_id), "_status": settings.status_text,
                "throttle_fps": throttle}

    for cam in cfg.cameras:
        ctrls[cam.id].put(initial_settings(cam.id))
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
            _, cam_id, connected, fps, proc_ms, tag_fps, stream_fps, viewers, reject = msg
            p = stat_pubs[cam_id]
            p["connected"].set(bool(connected)); p["fps"].set(fps); p["processMs"].set(proc_ms)
            p["tagFps"].set(tag_fps); p["streamFps"].set(stream_fps)
            p["streamViewers"].set(viewers); p["lastReject"].set(reject)
        elif msg[0] == "set":  # from a camera's web settings page
            settings.set_from_web(msg[1], msg[2])
        elif msg[0] == "save":
            settings.request_save()

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
            for cam in cfg.cameras:
                msg = {**g, **per_cam.get(cam.id, {})}
                if msg:
                    ctrls[cam.id].put(msg)
            new_throttle = throttle_sub.get()
            if new_throttle != throttle:
                throttle = new_throttle
                for cam in cfg.cameras:
                    ctrls[cam.id].put({"throttle_fps": throttle})
            if settings.status_text != last_status:  # "unsaved changes" / "saved ..." on the web pages
                last_status = settings.status_text
                for cam in cfg.cameras:
                    ctrls[cam.id].put({"_status": last_status})
        t_settings = time.monotonic()

        if now - last_health > 1.0:
            last_health = now
            heartbeat += 1
            heartbeat_pub.set(heartbeat)
            for cam in cfg.cameras:  # restart crashed camera processes with current settings
                if not procs[cam.id].is_alive():
                    print(f"[{cam.name}] process died, restarting", flush=True)
                    restarts[cam.id] += 1
                    start(cam)
                    ctrls[cam.id].put(initial_settings(cam.id))
            ips = ip_watcher.ips  # IP can change after boot (robot radio DHCP); refreshed in background
            # Diagnostics for the web pages: NetworkTables and every camera process.
            system = {
                "started": owl_started,
                "ips": ips,
                "nt": {
                    "mode": "standalone server" if cfg.standalone else "client",
                    "target": None if cfg.standalone else (cfg.server or f"team {cfg.team}"),
                    "connected": bool(inst.isConnected()) or cfg.standalone,
                    "peers": [{"id": c.remote_id, "ip": c.remote_ip} for c in inst.getConnections()],
                    "time_synced": time_synced[0],
                },
                "throttle_fps": throttle,
                "cameras": [{"name": c.name, "port": c.stream_port, "alive": procs[c.id].is_alive(),
                             "pid": procs[c.id].pid, "restarts": restarts[c.id]} for c in cfg.cameras],
            }
            for cam in cfg.cameras:
                ctrls[cam.id].put({"_system": system})
            if ips != last_ips:
                last_ips = ips
                for (cam_id, path), pub in stream_pubs.items():
                    port = cfg.cameras[cam_id].stream_port
                    pub.set([f"mjpg:http://{ip}:{port}/{path}" for ip in ips])

        t_end = time.monotonic()
        if t_end - t_loop > STALL_WARN_S and t_end - last_stall_print > 5.0:
            last_stall_print = t_end
            print(f"WARNING: main loop stalled {t_end - t_loop:.2f} s at {robot_time()} (queue+publish {t_published - t_loop:.2f}, "
                  f"settings {t_settings - t_published:.2f}, health/restarts/IPs {t_end - t_settings:.2f})", flush=True)


if __name__ == "__main__":
    main()
