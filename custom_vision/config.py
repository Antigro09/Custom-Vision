"""Load a validated runtime configuration; relative paths are relative to the YAML."""
import json
import math
from pathlib import Path

import yaml


def load_config(path):
    path = Path(path).resolve()
    with path.open() as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("Configuration must be a YAML mapping")
    pipelines = config.get("pipelines")
    if not isinstance(pipelines, list) or not pipelines:
        raise ValueError("At least one pipeline is required")
    names = set()
    for pipeline in pipelines:
        if not isinstance(pipeline, dict):
            raise ValueError("Each pipeline must be a mapping")
        name = pipeline.get("name", "")
        if not isinstance(name, str) or not name or not name.replace("_", "").replace("-", "").isalnum() or name in names:
            raise ValueError("Pipeline names must be unique letters, numbers, underscores or hyphens")
        names.add(name)
        if pipeline.get("type") not in ("apriltag", "object"):
            raise ValueError(f"Unsupported pipeline type for {name}")
        camera = pipeline.setdefault("camera", {})
        if not isinstance(camera, dict):
            raise ValueError("camera must be a mapping")
        camera.setdefault("source", 0)
        for key, default in (("width", 640), ("height", 480), ("fps", 30)):
            camera.setdefault(key, default)
            if isinstance(camera[key], bool) or not isinstance(camera[key], (int, float)) or not math.isfinite(camera[key]) or camera[key] <= 0:
                raise ValueError(f"camera.{key} must be positive")
            if key in ("width", "height") and not isinstance(camera[key], int):
                raise ValueError(f"camera.{key} must be an integer")
        source = camera["source"]
        if not isinstance(source, (str, int)) or isinstance(source, bool):
            raise ValueError("camera.source must be a device index, path, or GStreamer pipeline")
        if (isinstance(source, int) and source < 0) or (isinstance(source, str) and not source):
            raise ValueError("camera.source must be nonnegative and nonempty")
        if camera.get("backend", "auto") not in ("auto", "v4l2", "gstreamer"):
            raise ValueError("camera.backend must be auto, v4l2, or gstreamer")
        fourcc = camera.get("fourcc")
        if fourcc is not None and (not isinstance(fourcc, str) or len(fourcc) != 4 or not fourcc.isascii()):
            raise ValueError("camera.fourcc must be exactly four ASCII characters")
        if isinstance(source, str) and camera.get("backend") != "gstreamer" and not source.startswith(("/", "rtsp://", "http://", "https://")):
            camera["source"] = str(path.parent / source)
        pipeline["calibration_data"] = None
        if pipeline.get("calibration"):
            calibration_path = (path.parent / pipeline["calibration"]).resolve()
            with calibration_path.open() as stream:
                pipeline["calibration_data"] = json.load(stream)
        settings = pipeline.setdefault("settings", {})
        if not isinstance(settings, dict):
            raise ValueError("settings must be a mapping")
        if settings.get("model_path"):
            settings["model_path"] = str((path.parent / settings["model_path"]).resolve())
    nt = config.setdefault("networktables", {})
    if not isinstance(nt, dict):
        raise ValueError("networktables must be a mapping")
    nt.setdefault("enabled", False)
    if not isinstance(nt["enabled"], bool):
        raise ValueError("networktables.enabled must be boolean")
    if nt.get("server") is not None and not isinstance(nt["server"], str):
        raise ValueError("networktables.server must be a hostname string")
    if nt["enabled"] and not nt.get("server"):
        team = nt.get("team")
        if not isinstance(team, int) or isinstance(team, bool) or not 1 <= team <= 99999:
            raise ValueError("Enabled NetworkTables requires a team number or explicit server")
    config.setdefault("dashboard", {"enabled": True, "host": "127.0.0.1", "port": 5800})
    dashboard = config["dashboard"]
    if not isinstance(dashboard, dict):
        raise ValueError("dashboard must be a mapping")
    if not isinstance(dashboard.get("enabled", False), bool):
        raise ValueError("dashboard.enabled must be boolean")
    port = dashboard.get("port", 5800)
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise ValueError("dashboard.port must be an integer from 1 through 65535")
    if not isinstance(dashboard.get("host", "127.0.0.1"), str):
        raise ValueError("dashboard.host must be a string")
    return config
