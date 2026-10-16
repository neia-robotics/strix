"""Per-camera pipelines: named sets of the settings that change how a camera detects.

Like PhotonVision, each camera has an ordered list of pipelines and one is active.
A pipeline holds the camera mode and every tuning value for that camera:

  resolution "WxH", exposure, gain (None = auto), std_dev_factor,
  quad_decimate, min_cluster_pixels                       detector
  min_decision_margin, max_single_tag_ambiguity,
  max_reprojection_error_px, max_tag_distance_m           filters

Switching only changes what differs, so pipelines at the same resolution switch
without reopening the camera. Calibrations stay per camera and per resolution.

Unlike PhotonVision, pipelines are addressed by name (a deleted pipeline can't
shift which one an index means), the camera always keeps at least one, and a
switch requested by the robot lasts until Owl restarts: the camera boots into its
default pipeline (config.json's "pipeline"), which only changes from the page.

In config.json:  "cameras": [{..., "pipeline": "main", "pipelines": [{"name": "main", ...}]}]
A camera without "pipelines" (older configs) gets one, "default", built from its
own width/height/exposure/gain/std_dev_factor and the shared detector/filters.
"""

from __future__ import annotations

import re

from validation import validate_settings

# The tuning values a pipeline holds, in the order the page shows them.
TUNING_KEYS = ("quad_decimate", "min_cluster_pixels", "min_decision_margin", "max_single_tag_ambiguity",
               "max_reprojection_error_px", "max_tag_distance_m")
PIPELINE_KEYS = ("resolution", "exposure", "gain", "std_dev_factor") + TUNING_KEYS
NAME = re.compile(r"[A-Za-z0-9 _().-]{1,32}")
MAX_PIPELINES = 16


def check_name(name) -> str:
    name = str(name if name is not None else "").strip()
    if not NAME.fullmatch(name):
        raise ValueError("pipeline name must be 1-32 letters, digits, spaces or _ - . ( )")
    return name


def unique_name(names, base: str) -> str:
    """base, or "base (1)", "base (2)", ... whichever isn't taken (case-insensitive)."""
    taken = {n.lower() for n in names}
    if base.lower() not in taken:
        return base
    stem = re.sub(r" \(\d+\)$", "", base)[:27]
    i = 1
    while f"{stem} ({i})".lower() in taken:
        i += 1
    return f"{stem} ({i})"


def validate_pipeline(raw: dict, fallback: dict) -> dict:
    """A normalized pipeline: name plus every PIPELINE_KEYS value, filling gaps from fallback."""
    if not isinstance(raw, dict):
        raise ValueError("each pipeline must be an object")
    unknown = set(raw) - set(PIPELINE_KEYS) - {"name"}
    if unknown:
        raise ValueError(f"unknown pipeline setting: {sorted(unknown)[0]}")
    p = {"name": check_name(raw.get("name"))}
    values = {k: raw.get(k, fallback[k]) for k in PIPELINE_KEYS}
    checked = validate_settings({k: -1 if v is None and k in ("exposure", "gain") else v for k, v in values.items()})
    for k in PIPELINE_KEYS:
        v = checked[k]
        if k in ("exposure", "gain"):
            v = None if v < 0 else v  # -1 on NT means auto; config.json uses null
        p[k] = v
    return p


def defaults(width: int, height: int, det, solver, *, exposure=None, gain=None, std_dev_factor=1.0) -> dict:
    """Values for a new pipeline: this resolution, auto exposure, the shared tuning."""
    v = {"resolution": f"{width}x{height}", "exposure": exposure, "gain": gain, "std_dev_factor": std_dev_factor}
    for k in TUNING_KEYS:
        v[k] = getattr(det, k) if hasattr(det, k) else getattr(solver, k)
    return v


def load_camera_pipelines(cam_raw: dict, base: dict) -> tuple[list[dict], str]:
    """(pipelines, default name) for one config.json camera entry."""
    raw_list = cam_raw.get("pipelines")
    if raw_list is None:
        raw_list = [{"name": "default"}]
    if not isinstance(raw_list, list) or not raw_list:
        raise ValueError("pipelines must be a non-empty list")
    if len(raw_list) > MAX_PIPELINES:
        raise ValueError(f"at most {MAX_PIPELINES} pipelines per camera")
    pipes = [validate_pipeline(p, base) for p in raw_list]
    names = [p["name"].lower() for p in pipes]
    if len(set(names)) != len(names):
        raise ValueError("pipeline names must be unique per camera")
    default = cam_raw.get("pipeline", pipes[0]["name"])
    if str(default).lower() not in names:
        raise ValueError(f"default pipeline {default!r} isn't in the camera's pipelines")
    return pipes, pipes[names.index(str(default).lower())]["name"]


def to_nt(p: dict) -> dict:
    """A pipeline's values as NT entries hold them (auto exposure/gain = -1)."""
    return {k: (-1.0 if p[k] is None else float(p[k])) if k in ("exposure", "gain") else p[k] for k in PIPELINE_KEYS}


def from_nt(values: dict) -> dict:
    return {k: (None if k in ("exposure", "gain") and v < 0 else v) for k, v in values.items()}

