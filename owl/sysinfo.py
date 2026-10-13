"""Coprocessor stats for the web page's diagnostics tab.

Reads Linux's /proc and /sys directly (Orange Pi, Jetson), so it needs no extra
packages there. Uses psutil instead when it's installed (e.g. on a Mac for
development); without either, it reports what the standard library can.
"""

from __future__ import annotations

import glob
import os
import platform
import shutil
import socket
import subprocess
import threading
import time

try:
    import psutil  # optional
except ImportError:
    psutil = None

_lock = threading.Lock()
_prev_cpu: list[tuple[int, int]] | None = None  # (total, idle) jiffies per core, for deltas
_HERE = os.path.dirname(os.path.abspath(__file__))


def _read(path: str) -> str | None:
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def _cpu_times_linux() -> list[tuple[int, int]] | None:
    stat = _read("/proc/stat")
    if not stat:
        return None
    cores = []
    for line in stat.splitlines():
        if line.startswith("cpu") and line[3:4].isdigit():
            v = [int(x) for x in line.split()[1:]]
            idle = v[3] + (v[4] if len(v) > 4 else 0)  # idle + iowait
            cores.append((sum(v[:8]), idle))
    return cores or None


def cpu_percent() -> list[float] | None:
    """Busy % per core since the previous call (the first call covers time since boot)."""
    global _prev_cpu
    now = _cpu_times_linux()
    if now is None:
        return psutil.cpu_percent(percpu=True) if psutil else None
    with _lock:
        prev, _prev_cpu = _prev_cpu, now
    if prev is None or len(prev) != len(now):
        prev = [(0, 0)] * len(now)
    out = []
    for (t1, i1), (t0, i0) in zip(now, prev):
        dt = t1 - t0
        out.append(round(100.0 * (1 - (i1 - i0) / dt), 1) if dt > 0 else 0.0)
    return out


def memory() -> dict | None:
    info = _read("/proc/meminfo")
    if info:
        kb = {}
        for line in info.splitlines():
            k, _, v = line.partition(":")
            kb[k] = int(v.split()[0]) if v.split() else 0
        total, avail = kb.get("MemTotal", 0) * 1024, kb.get("MemAvailable", 0) * 1024
        return {"total": total, "used": total - avail}
    if psutil:
        vm = psutil.virtual_memory()
        return {"total": vm.total, "used": vm.total - vm.available}
    return None


def temperatures() -> list[dict]:
    temps = []
    for zone in sorted(glob.glob("/sys/class/thermal/thermal_zone*")):
        t = _read(f"{zone}/temp")
        if t and t.lstrip("-").isdigit():
            temps.append({"name": _read(f"{zone}/type") or os.path.basename(zone), "c": int(t) / 1000.0})
    if not temps and psutil and hasattr(psutil, "sensors_temperatures"):
        for name, entries in (psutil.sensors_temperatures() or {}).items():
            for e in entries[:1]:
                temps.append({"name": name, "c": e.current})
    return temps


def gpu_load() -> float | None:
    """Jetson GPU busy % (the kernel reports it in tenths of a percent)."""
    for path in ("/sys/devices/gpu.0/load", "/sys/devices/platform/gpu.0/load",
                 "/sys/devices/platform/bus@0/17000000.gpu/load"):
        v = _read(path)
        if v and v.isdigit():
            return int(v) / 10.0
    return None


def uptime_s() -> float | None:
    up = _read("/proc/uptime")
    if up:
        return float(up.split()[0])
    if psutil:
        return time.time() - psutil.boot_time()
    return None


def _versions() -> dict:
    from importlib import metadata

    v = {"python": platform.python_version()}
    for name in ("pyntcore", "robotpy-apriltag", "robotpy-wpimath", "opencv-python-headless",
                 "opencv-python", "numpy"):
        try:
            v[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            pass
    try:
        v["strix"] = subprocess.run(["git", "-C", _HERE, "rev-parse", "--short", "HEAD"],
                                    capture_output=True, text=True, timeout=2).stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        v["strix"] = None
    jetson = _read("/etc/nv_tegra_release")
    if jetson:
        v["jetson"] = jetson.splitlines()[0]
    return v


_VERSIONS: dict | None = None


def snapshot() -> dict:
    global _VERSIONS
    if _VERSIONS is None:
        _VERSIONS = _versions()
    disk = shutil.disk_usage(_HERE)
    load = os.getloadavg() if hasattr(os, "getloadavg") else None
    model = _read("/proc/device-tree/model")
    return {
        "host": socket.gethostname(),
        "model": model.rstrip("\x00") if model else None,
        "os": f"{platform.system()} {platform.release()}",
        "arch": platform.machine(),
        "uptime_s": uptime_s(),
        "cpu": cpu_percent(),
        "cpu_count": os.cpu_count(),
        "load": list(load) if load else None,
        "memory": memory(),
        "disk": {"total": disk.total, "used": disk.used},
        "temps": temperatures(),
        "gpu": gpu_load(),
        "versions": _VERSIONS,
        "time": time.time(),
    }
