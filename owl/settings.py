"""Live-tunable settings over NetworkTables, with a save button.

  /Vision/settings/<name>                 shared settings (detector, filters, stream,
                                          calibration board)
  /Vision/cameras/<camera>/settings/<name> per-camera settings (exposure, gain, std_dev_factor,
                                          resolution as "1600x1300")
  /Vision/settings/save                   set true -> current values written to config.json
  /Vision/settings/status                 "saved 14:02:11" / "unsaved changes" / errors

config.json is the source of truth: on startup every entry is set FROM the file.
Changes apply immediately but only persist when you press save, so experimenting
never overwrites a good config, and the SD card isn't rewritten on every nudge.

NT has no null, so exposure/gain use -1 to mean "auto" / "leave alone"
(saved to config.json as null).
"""

from __future__ import annotations

import json
import os
import tempfile
import time

import ntcore
import cv2

from config import AppConfig
from validation import validate_settings, setting

# name -> (config.json section, key in that section, NT type)
GLOBAL_SETTINGS = {
    "quad_decimate": ("detector", "quad_decimate", float),
    "min_cluster_pixels": ("detector", "min_cluster_pixels", int),
    "min_decision_margin": ("filters", "min_decision_margin", float),
    "max_single_tag_ambiguity": ("filters", "max_single_tag_ambiguity", float),
    "max_reprojection_error_px": ("filters", "max_reprojection_error_px", float),
    "max_tag_distance_m": ("filters", "max_tag_distance_m", float),
    "stream_max_fps": ("stream", "max_fps", float),
    "board_squares_x": ("calibration_board", "squares_x", int),
    "board_squares_y": ("calibration_board", "squares_y", int),
    "board_square_length_m": ("calibration_board", "square_length_m", float),
    "board_marker_length_m": ("calibration_board", "marker_length_m", float),
    "board_dict": ("calibration_board", "dict", str),
    "board_legacy": ("calibration_board", "legacy", bool),
}
CAMERA_SETTINGS = {"exposure": float, "gain": float, "std_dev_factor": float, "resolution": str}

def parse_resolution(text: str) -> tuple[int, int]:
    w, h = text.split("x")
    return int(w), int(h)


def _entry(inst, path, typ, value):
    if typ is bool:
        e = inst.getBooleanTopic(path).getEntry(bool(value))
        e.set(bool(value))
    elif typ is int:
        e = inst.getIntegerTopic(path).getEntry(int(value))
        e.set(int(value))
    elif typ is str:
        e = inst.getStringTopic(path).getEntry(str(value))
        e.set(str(value))
    else:
        e = inst.getDoubleTopic(path).getEntry(float(value))
        e.set(float(value))
    return e


class LiveSettings:
    def __init__(self, inst: ntcore.NetworkTableInstance, cfg: AppConfig, base: str = "/Vision"):
        self.cfg = cfg
        self.values: dict[str, object] = {}
        self.cam_values: dict[int, dict[str, object]] = {}
        self.entries: dict[str, object] = {}
        self.cam_entries: dict[int, dict[str, object]] = {}

        for name, (_, _, typ) in GLOBAL_SETTINGS.items():
            v = typ(self._initial_global(name))
            self.values[name] = v
            self.entries[name] = _entry(inst, f"{base}/settings/{name}", typ, v)

        for cam in cfg.cameras:
            vals = {
                "exposure": -1.0 if cam.exposure is None else float(cam.exposure),
                "gain": -1.0 if cam.gain is None else float(cam.gain),
                "std_dev_factor": float(cam.std_dev_factor),
                "resolution": f"{cam.width}x{cam.height}",
            }
            self.cam_values[cam.id] = vals
            self.cam_entries[cam.id] = {
                k: _entry(inst, f"{base}/cameras/{cam.name}/settings/{k}", CAMERA_SETTINGS[k], v)
                for k, v in vals.items()
            }

        self.save_entry = inst.getBooleanTopic(f"{base}/settings/save").getEntry(False)
        self.save_entry.set(False)
        self._status_pub = inst.getStringTopic(f"{base}/settings/status").publish()
        self.status_text = ""
        self._set_status("loaded from config.json")
        self.dirty = False

    def _initial_global(self, name):
        section, key, _ = GLOBAL_SETTINGS[name]
        if section == "stream":
            return self.cfg.stream.get(key, 30)
        if section == "calibration_board":
            return self.cfg.calibration_board[key]
        if hasattr(self.cfg.detector, key):
            return getattr(self.cfg.detector, key)
        return getattr(self.cfg.solver, key)

    def _set_status(self, text: str):
        self.status_text = text
        self._status_pub.set(text)

    def set_from_web(self, cam_id: int, changes: dict):
        """Apply changes sent from a camera's web page by writing them to the NT entries,
        so dashboards see them too and the next poll() handles them like any NT edit."""
        try:
            if not isinstance(changes, dict) or cam_id not in self.cam_values:
                raise ValueError("invalid camera/settings request")
            global_values = {k: e.get() for k, e in self.entries.items()}
            camera_values = {k: e.get() for k, e in self.cam_entries[cam_id].items()}
            for name, value in changes.items():
                if name in GLOBAL_SETTINGS:
                    global_values[name] = value
                elif name in CAMERA_SETTINGS:
                    camera_values[name] = value
                else:
                    raise ValueError(f"unknown setting: {name}")
            global_values = validate_settings(global_values)
            camera_values = validate_settings(camera_values)
            for name in changes:
                if name in GLOBAL_SETTINGS:
                    self.entries[name].set(global_values[name])
                else:
                    self.cam_entries[cam_id][name].set(camera_values[name])
            return True
        except (TypeError, ValueError, OverflowError, cv2.error) as e:
            self._set_status(f"settings rejected: {e}")
            return False

    def request_save(self):
        self.save_entry.set(True)

    @staticmethod
    def camera_message(vals: dict) -> dict:
        """Convert NT-side camera values into what the camera process expects."""
        out = {}
        for k, v in vals.items():
            out[k] = None if k in ("exposure", "gain") and v < 0 else v
        return out

    def full_camera_settings(self, cam_id: int) -> dict:
        """Everything a (re)started camera process needs to match current live values."""
        return {**self.values, **self.camera_message(self.cam_values[cam_id])}

    def poll(self) -> tuple[dict, dict[int, dict]]:
        """Returns (global changes, per-camera changes) since the last poll."""
        rejected = []
        def checked(entries, previous):
            candidate = dict(previous)
            for key, entry in entries.items():
                try:
                    candidate[key] = setting(key, entry.get())
                except (ValueError, TypeError, OverflowError, cv2.error) as error:
                    entry.set(previous[key])
                    rejected.append(str(error))
            try:
                validate_settings(candidate)
            except (ValueError, TypeError, OverflowError, cv2.error) as error:
                # Board dimensions/dictionary form one dependent group. Other edits survive.
                for key, entry in entries.items():
                    if key.startswith("board_"):
                        candidate[key] = previous[key]
                        entry.set(previous[key])
                rejected.append(str(error))
            changes = {k: v for k, v in candidate.items() if v != previous[k]}
            previous.update(changes)
            return changes

        g = checked(self.entries, self.values)
        per_cam = {}
        for cam_id, entries in self.cam_entries.items():
            changes = checked(entries, self.cam_values[cam_id])
            if changes:
                per_cam[cam_id] = self.camera_message(changes)
        if g or per_cam:
            self.dirty = True
            self._set_status("unsaved changes")

        if rejected:
            self._set_status("settings rejected: " + "; ".join(rejected))

        if self.save_entry.get():
            self.save_entry.set(False)
            self._save()
        return g, per_cam

    def _save(self):
        try:
            with open(self.cfg.path) as f:
                raw = json.load(f)
            for name, (section, key, _) in GLOBAL_SETTINGS.items():
                raw.setdefault(section, {})[key] = self.values[name]
            for cam in self.cfg.cameras:
                entry = raw["cameras"][cam.id]
                vals = self.camera_message(self.cam_values[cam.id])
                entry["exposure"] = vals["exposure"]
                entry["gain"] = vals["gain"]
                entry["std_dev_factor"] = vals["std_dev_factor"]
                entry["width"], entry["height"] = parse_resolution(vals["resolution"])

            # Atomic write: a power cut mid-save leaves either the old or the new file, never half.
            d = os.path.dirname(self.cfg.path)
            fd, tmp = tempfile.mkstemp(prefix=".config-", suffix=".json", dir=d)
            with os.fdopen(fd, "w") as f:
                json.dump(raw, f, indent=2)
                f.write("\n")
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.cfg.path)
            self.dirty = False
            msg = f"saved {time.strftime('%H:%M:%S')}"
            print(f"settings: {msg} -> {self.cfg.path}", flush=True)
            self._set_status(msg)
        except Exception as e:  # keep running; report on NT
            print(f"settings: save failed: {e}", flush=True)
            self._set_status(f"save failed: {e}")
