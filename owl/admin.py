"""The coprocessor page's maintenance buttons: logs, settings export/import and
factory reset (restart and reboot are in main.py / deploy/owl-admin).

Everything that replaces files first copies what it replaces into
backups/<time>-<what>/ next to config.json, so any of it can be undone by copying
that folder back. Owl restarts afterwards (systemd's Restart=always brings it back).
"""

from __future__ import annotations

import fcntl
import io
import json
import os
import re
import shutil
import subprocess
import tempfile
import time
import zipfile
from contextlib import contextmanager

from config import load_config

OWL_DIR = os.path.dirname(os.path.abspath(__file__))


def _unit() -> str | None:
    """The systemd unit Owl runs in (owl.service), from its cgroup; None outside systemd."""
    try:
        with open("/proc/self/cgroup") as f:
            m = re.search(r"/([^/]+\.service)\s*$", f.read(), re.M)
        return m.group(1) if m else None
    except OSError:
        return None


def logs(lines: int = 500) -> str:
    unit = _unit()
    if not unit or not shutil.which("journalctl"):
        raise RuntimeError("logs are only available when Owl runs as a systemd service")
    r = subprocess.run(["journalctl", "-u", unit, "-n", str(lines), "--no-pager", "-o", "short-iso"],
                       capture_output=True, text=True, timeout=15)
    if r.returncode != 0:
        raise RuntimeError(r.stderr.strip() or "journalctl failed (is this user in the adm group?)")
    return r.stdout


@contextmanager
def _locked(config_path: str):
    """One maintenance operation at a time (each camera's page can start one)."""
    with open(os.path.join(os.path.dirname(config_path), ".owl-admin.lock"), "w") as f:
        try:
            fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError("another maintenance operation is running") from None
        yield


def _calib_dir(config_path: str) -> str:
    with open(config_path) as f:
        raw = json.load(f)
    return os.path.join(os.path.dirname(config_path), raw.get("calibration_dir", "calibrations"))


def backup(config_path: str, what: str) -> str:
    """Copies config.json and calibrations into backups/."""
    base = os.path.dirname(config_path)
    dest = os.path.join(base, "backups", f"{time.strftime('%Y%m%d-%H%M%S')}-{what}")
    os.makedirs(dest)
    shutil.copy2(config_path, dest)
    calib = _calib_dir(config_path)
    if os.path.isdir(calib):
        shutil.copytree(calib, os.path.join(dest, "calibrations"))
    return dest


# ---- settings export / import ----------------------------------------------------

def export_settings(config_path: str) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        z.write(config_path, "config.json")
        calib = _calib_dir(config_path)
        if os.path.isdir(calib):
            for name in sorted(os.listdir(calib)):
                if name.endswith(".json"):
                    z.write(os.path.join(calib, name), f"calibrations/{name}")
    return buf.getvalue()


def import_settings(config_path: str, data: bytes) -> str:
    """Accepts an export zip, or a bare config.json. Checks the config loads before
    replacing anything. Calibrations in the zip are added (same names overwritten)."""
    if data[:2] == b"PK":
        try:
            z = zipfile.ZipFile(io.BytesIO(data))
            members = {n: z.read(n) for n in z.namelist()
                       if n == "config.json" or re.fullmatch(r"calibrations/[^/]+\.json", n)}
        except (zipfile.BadZipFile, KeyError) as e:
            raise ValueError(f"not a settings export: {e}") from None
    else:
        members = {"config.json": data}
    if "config.json" not in members:
        raise ValueError("the zip has no config.json")
    with _locked(config_path), tempfile.TemporaryDirectory() as tmp:
        try:
            raw = json.loads(members["config.json"])
            raw["calibration_dir"] = "calibrations"
            with open(os.path.join(tmp, "config.json"), "w") as f:
                json.dump(raw, f)
            load_config(os.path.join(tmp, "config.json"))
        except Exception as e:
            raise ValueError(f"config.json doesn't load: {e}") from None
        dest = backup(config_path, "before-import")
        calib = _calib_dir(config_path)
        for name, body in members.items():
            if name.startswith("calibrations/"):
                os.makedirs(calib, exist_ok=True)
                with open(os.path.join(calib, os.path.basename(name)), "wb") as f:
                    f.write(body)
        _atomic_write(config_path, members["config.json"])
    n = len(members) - 1
    return f"imported config.json and {n} calibration{'s' * (n != 1)} (old files in {os.path.relpath(dest, OWL_DIR)})"


def _atomic_write(path: str, body: bytes) -> None:
    fd, tmp = tempfile.mkstemp(prefix=".owl-", dir=os.path.dirname(path))
    with os.fdopen(fd, "wb") as f:
        f.write(body)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


# ---- factory reset ---------------------------------------------------------------

def factory_reset(config_path: str) -> str:
    """Defaults for every setting (config.example.json), keeping only what Owl needs to
    start: each camera's name, device, mode and mount position, and the NT target.
    Calibrations are moved into the backup."""
    with _locked(config_path):
        with open(config_path) as f:
            old = json.load(f)
        with open(os.path.join(OWL_DIR, "config.example.json")) as f:
            new = json.load(f)
        new["nt"] = {**new["nt"], **{k: old.get("nt", {}).get(k, new["nt"].get(k))
                                       for k in ("team", "server", "client_name")}, "standalone": False}
        template = new["cameras"][0]
        new["cameras"] = [{**template, **{k: c[k] for k in ("name", "device", "width", "height", "fps",
                                                             "fourcc", "robot_to_camera") if k in c}}
                          for c in old["cameras"]]
        dest = backup(config_path, "before-factory-reset")
        calib = _calib_dir(config_path)
        if os.path.isdir(calib):
            shutil.rmtree(calib)
        _atomic_write(config_path, (json.dumps(new, indent=2) + "\n").encode())
    return f"settings reset; old config and calibrations in {os.path.relpath(dest, OWL_DIR)}"


# ---- camera rename ---------------------------------------------------------------

CAMERA_NAME = re.compile(r"[A-Za-z0-9_-]{1,32}")


def rename_camera(config_path: str, index: int, new: str) -> str:
    """Renames config.json's camera <index> and its calibration files
    (<name>_<W>x<H>.json and <name>.json). The NT topics and stream names follow the
    name; the robot reads cameras by index, so it isn't affected."""
    new = str(new).strip()
    if not CAMERA_NAME.fullmatch(new):
        raise ValueError("camera name must be 1-32 letters, digits, - or _")
    with _locked(config_path):
        with open(config_path) as f:
            raw = json.load(f)
        cams = raw["cameras"]
        if not 0 <= index < len(cams):
            raise ValueError("no such camera")
        old = cams[index].get("name", f"camera{index}")
        if new == old:
            return f"camera is already named {new}"
        if any(c.get("name", f"camera{i}") == new for i, c in enumerate(cams)):
            raise ValueError(f"another camera is already named {new}")
        calib = _calib_dir(config_path)
        pattern = re.compile(re.escape(old) + r"(_\d+x\d+)?\.json")
        moves = [(os.path.join(calib, f), os.path.join(calib, new + f[len(old):]))
                 for f in (sorted(os.listdir(calib)) if os.path.isdir(calib) else []) if pattern.fullmatch(f)]
        if any(os.path.exists(dst) for _, dst in moves):
            raise ValueError(f"calibration files named {new}_... already exist; move them first")
        backup(config_path, "before-rename")
        for src, dst in moves:
            os.rename(src, dst)
        explicit = cams[index].get("calibration")  # older configs name their file
        if explicit and os.path.basename(explicit) in {os.path.basename(src) for src, _ in moves}:
            moved = dict((os.path.basename(a), os.path.basename(b)) for a, b in moves)
            cams[index]["calibration"] = os.path.join(os.path.dirname(explicit), moved[os.path.basename(explicit)])
        cams[index]["name"] = new
        _atomic_write(config_path, (json.dumps(raw, indent=2) + "\n").encode())
    n = len(moves)
    return f"renamed {old} to {new} ({n} calibration file{'s' * (n != 1)})"
