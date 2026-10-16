"""Live-tunable settings over NetworkTables, with a save button.

  /Vision/settings/<name>                  shared settings (stream fps, calibration board)
  /Vision/cameras/<camera>/settings/<name> the camera's ACTIVE pipeline (pipelines.py):
                                           resolution as "1600x1300", exposure, gain,
                                           std_dev_factor, detector and filter values.
                                           Editing one edits that pipeline.
  /Vision/cameras/<camera>/pipelineRequest the robot writes a pipeline name to switch to
  /Vision/cameras/<camera>/pipelineState   the active pipeline's name
  /Vision/cameras/<camera>/pipelines       every pipeline's name, in order
  /Vision/settings/save                    set true -> current values written to config.json
  /Vision/settings/status                  "saved 14:02:11" / "unsaved changes" / errors

config.json is the source of truth: on startup every entry is set FROM the file.
Changes apply immediately but only persist when you press save, so experimenting
never overwrites a good config, and the SD card isn't rewritten on every nudge.
Saving keeps each camera's default (boot) pipeline; switching pipelines doesn't
change it, only "set as default" on the page does.

NT has no null, so exposure/gain use -1 to mean "auto" / "leave alone"
(saved to config.json as null).
"""

from __future__ import annotations

import json
import time

import ntcore
import cv2

import pipelines as pl
from config import AppConfig, write_config
from validation import validate_settings, setting

# name -> (config.json section, key in that section, NT type)
GLOBAL_SETTINGS = {
    "stream_max_fps": ("stream", "max_fps", float),
    "board_squares_x": ("calibration_board", "squares_x", int),
    "board_squares_y": ("calibration_board", "squares_y", int),
    "board_square_length_m": ("calibration_board", "square_length_m", float),
    "board_marker_length_m": ("calibration_board", "marker_length_m", float),
    "board_dict": ("calibration_board", "dict", str),
    "board_legacy": ("calibration_board", "legacy", bool),
}
# Per camera, the active pipeline's values.
CAMERA_SETTINGS = {"resolution": str, "exposure": float, "gain": float, "std_dev_factor": float,
                   "quad_decimate": float, "min_cluster_pixels": int, "min_decision_margin": float,
                   "max_single_tag_ambiguity": float, "max_reprojection_error_px": float,
                   "max_tag_distance_m": float}
assert set(CAMERA_SETTINGS) == set(pl.PIPELINE_KEYS)


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
        self.cam_values: dict[int, dict[str, object]] = {}  # active pipeline, NT form
        self.entries: dict[str, object] = {}
        self.cam_entries: dict[int, dict[str, object]] = {}
        self.pipes: dict[int, list[dict]] = {}  # config form (None = auto)
        self.active: dict[int, int] = {}
        self.default: dict[int, str] = {}
        self._pipe_pubs: dict[int, tuple] = {}
        self._requests: dict[int, tuple] = {}  # cam_id -> (subscriber, last value seen)
        self._pipes_changed: set[int] = set()

        for name, (_, _, typ) in GLOBAL_SETTINGS.items():
            v = typ(self._initial_global(name))
            self.values[name] = v
            self.entries[name] = _entry(inst, f"{base}/settings/{name}", typ, v)

        for cam in cfg.cameras:
            self.pipes[cam.id] = [dict(p) for p in cam.pipelines]
            self.default[cam.id] = cam.pipeline
            self.active[cam.id] = [p["name"] for p in cam.pipelines].index(cam.pipeline)
            vals = pl.to_nt(self.pipes[cam.id][self.active[cam.id]])
            self.cam_values[cam.id] = vals
            t = f"{base}/cameras/{cam.name}"
            self.cam_entries[cam.id] = {k: _entry(inst, f"{t}/settings/{k}", CAMERA_SETTINGS[k], v)
                                        for k, v in vals.items()}
            self._pipe_pubs[cam.id] = (inst.getStringTopic(f"{t}/pipelineState").publish(),
                                       inst.getStringArrayTopic(f"{t}/pipelines").publish())
            sub = inst.getStringTopic(f"{t}/pipelineRequest").subscribe("")
            self._requests[cam.id] = (sub, sub.get())
            self._publish_pipes(cam.id)

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
        return self.cfg.calibration_board[key]

    def _set_status(self, text: str):
        self.status_text = text
        self._status_pub.set(text)

    # ---- pipelines ------------------------------------------------------------

    def pipeline_info(self, cam_id: int) -> dict:
        """What the page shows: names in order, the active one and the boot default."""
        return {"names": [p["name"] for p in self.pipes[cam_id]],
                "active": self.pipes[cam_id][self.active[cam_id]]["name"], "default": self.default[cam_id]}

    def _publish_pipes(self, cam_id: int):
        state, names = self._pipe_pubs[cam_id]
        info = self.pipeline_info(cam_id)
        state.set(info["active"])
        names.set(info["names"])
        self._pipes_changed.add(cam_id)

    def _index(self, cam_id: int, name) -> int:
        names = [p["name"].lower() for p in self.pipes[cam_id]]
        key = str(name or "").strip().lower()
        if key not in names:
            raise ValueError(f"no pipeline named {name!r}")
        return names.index(key)

    def _switch(self, cam_id: int, index: int):
        """Make a pipeline active by writing its values into the camera's NT entries;
        the next poll() sends whatever differs to the camera process."""
        self.active[cam_id] = index
        for k, v in pl.to_nt(self.pipes[cam_id][index]).items():
            self.cam_entries[cam_id][k].set(v)
        self._publish_pipes(cam_id)

    def pipeline_op(self, cam_id: int, data: dict) -> str:
        """From the page: select, add, duplicate, rename, delete, default."""
        if cam_id not in self.pipes or not isinstance(data, dict):
            raise ValueError("invalid pipeline request")
        op, pipes = data.get("op"), self.pipes[cam_id]
        current = pipes[self.active[cam_id]]
        names = [p["name"] for p in pipes]
        if op == "select":
            self._switch(cam_id, self._index(cam_id, data.get("name")))
            return f"switched to {pipes[self.active[cam_id]]['name']}"
        if op in ("add", "duplicate"):
            if len(pipes) >= pl.MAX_PIPELINES:
                raise ValueError(f"at most {pl.MAX_PIPELINES} pipelines per camera")
            if op == "add":
                w, h = parse_resolution(current["resolution"])
                values = pl.defaults(w, h, self.cfg.detector, self.cfg.solver)
                name = pl.unique_name(names, pl.check_name(data.get("name") or f"pipeline {len(pipes) + 1}"))
            else:
                values = {k: current[k] for k in pl.PIPELINE_KEYS}
                name = pl.unique_name(names, current["name"])
            pipes.append({"name": name, **values})
            self._switch(cam_id, len(pipes) - 1)
            msg = f"added {name}"
        elif op == "rename":
            name = pl.check_name(data.get("name"))
            if name.lower() in [n.lower() for n in names if n != current["name"]]:
                raise ValueError(f"another pipeline is already named {name}")
            if self.default[cam_id] == current["name"]:
                self.default[cam_id] = name
            msg = f"renamed {current['name']} to {name}"
            current["name"] = name
            self._publish_pipes(cam_id)
        elif op == "delete":
            if len(pipes) == 1:
                raise ValueError("a camera needs at least one pipeline")
            i = self.active[cam_id]
            del pipes[i]
            if self.default[cam_id] == current["name"]:
                self.default[cam_id] = pipes[0]["name"]
            self._switch(cam_id, max(0, i - 1))
            msg = f"deleted {current['name']}"
        elif op == "default":
            self.default[cam_id] = current["name"]
            self._publish_pipes(cam_id)
            msg = f"{current['name']} is now the default"
        else:
            raise ValueError(f"unknown pipeline operation: {op}")
        self.dirty = True
        self._set_status("unsaved changes")
        return msg

    def _robot_requests(self):
        """The robot's pipelineRequest: acted on when it changes, so a page switch
        isn't undone by a request the robot made earlier. Unknown names are ignored
        (pipelineState keeps saying what's active)."""
        for cam_id, (sub, last) in self._requests.items():
            want = sub.get()
            if want == last:
                continue
            self._requests[cam_id] = (sub, want)
            if not want:  # unpublished / reset (e.g. NT reconnect)
                continue
            try:
                i = self._index(cam_id, want)
            except ValueError:
                print(f"settings: robot asked for unknown pipeline {want!r}", flush=True)
                continue
            if i != self.active[cam_id]:
                self._switch(cam_id, i)

    # ---- edits ------------------------------------------------------------------

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
        return pl.from_nt(vals)

    def full_camera_settings(self, cam_id: int) -> dict:
        """Everything a (re)started camera process needs to match current live values."""
        return {**self.values, **self.camera_message(self.cam_values[cam_id]),
                "_pipelines": self.pipeline_info(cam_id)}

    def poll(self) -> tuple[dict, dict[int, dict]]:
        """Returns (global changes, per-camera changes) since the last poll."""
        self._robot_requests()
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
        edited = bool(g)
        for cam_id, entries in self.cam_entries.items():
            changes = checked(entries, self.cam_values[cam_id])
            if changes:
                pipe = self.pipes[cam_id][self.active[cam_id]]
                msg = self.camera_message(changes)
                # A switch writes the new pipeline's own values, so they match it already;
                # anything else is an edit to the active pipeline.
                edited |= any(pipe[k] != v for k, v in msg.items())
                pipe.update(msg)
                per_cam[cam_id] = msg
            if cam_id in self._pipes_changed:
                per_cam.setdefault(cam_id, {})["_pipelines"] = self.pipeline_info(cam_id)
        self._pipes_changed.clear()
        if edited:
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
                pipes = self.pipes[cam.id]
                entry["pipelines"] = [dict(p) for p in pipes]
                entry["pipeline"] = self.default[cam.id]
                # The camera's own fields mirror its default pipeline, for tools that read them.
                boot = next(p for p in pipes if p["name"] == self.default[cam.id])
                entry["width"], entry["height"] = parse_resolution(boot["resolution"])
                for k in ("exposure", "gain", "std_dev_factor"):
                    entry[k] = boot[k]

            write_config(self.cfg.path, raw)
            self.dirty = False
            msg = f"saved {time.strftime('%H:%M:%S')}"
            print(f"settings: {msg} -> {self.cfg.path}", flush=True)
            self._set_status(msg)
        except Exception as e:  # keep running; report on NT
            print(f"settings: save failed: {e}", flush=True)
            self._set_status(f"save failed: {e}")
