#!/usr/bin/env python3
"""
PhotonVision-style camera dashboard for apriltag_vision.

- Auto-detects V4L2 camera modes: pixel format + resolution + FPS
- Saves up to 4 camera slots persistently across reboots
- Exports a YAML snapshot for the selected slot
- Writes a profile that apriltag_vision can auto-load with --camera-slot
"""

from __future__ import annotations

import argparse
import copy
import glob
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import ttk
from typing import Any

try:
    import cv2
except ImportError as exc:  # pragma: no cover
    print("ERROR: OpenCV Python bindings are required.", file=sys.stderr)
    raise SystemExit(1) from exc

try:
    import numpy as np
except ImportError as exc:  # pragma: no cover
    print("ERROR: numpy is required.", file=sys.stderr)
    raise SystemExit(1) from exc


WINDOW_TITLE = "Custom Vision Camera Dashboard"
MAX_CAMERA_SLOTS = 4
MAX_TEAM_NUMBER = 9999
PIPELINE_TYPES = ["AprilTag"]
PROCESSING_MODES = ["2D", "3D"]
STREAM_DISPLAYS = ["Processed Stream View", "Raw Stream View"]
CAMERA_TYPES = ["Generic", "OV9281"]
BACKEND_OPTIONS = ["auto", "v4l2", "gstreamer"]
TAG_FAMILY_OPTIONS = [
    "tag16h5",
    "tag25h9",
    "tag36h10",
    "tag36h11",
    "tagCircle21h7",
    "tagCircle49h12",
    "tagCustom48h12",
    "tagStandard41h12",
    "tagStandard52h13",
]
STREAM_RES_PRESETS = [(640, 400), (960, 600), (1280, 800), (1920, 1080)]
ORIENTATION_LABEL_TO_VALUE = {
    "Normal": "normal",
    "Rotate 90 CW": "cw90",
    "Rotate 180": "180",
    "Rotate 90 CCW": "ccw90",
}
ORIENTATION_VALUE_TO_LABEL = {value: key for key, value in ORIENTATION_LABEL_TO_VALUE.items()}


@dataclass(frozen=True)
class CaptureMode:
    width: int
    height: int
    fps: float
    pixel_format: str
    description: str = ""

    @property
    def label(self) -> str:
        details = f"{self.width}x{self.height} | {format_fps(self.fps)} FPS | {self.pixel_format}"
        if self.description:
            return f"{details} | {self.description}"
        return details

    def to_dict(self) -> dict[str, Any]:
        return {
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "pixel_format": self.pixel_format,
            "description": self.description,
            "label": self.label,
        }

    @staticmethod
    def from_dict(data: dict[str, Any] | None) -> "CaptureMode | None":
        if not isinstance(data, dict):
            return None
        try:
            width = int(data["width"])
            height = int(data["height"])
            fps = float(data["fps"])
            pixel_format = str(data.get("pixel_format", "")).strip().upper()
        except (KeyError, TypeError, ValueError):
            return None
        if width <= 0 or height <= 0 or fps <= 0.0 or not pixel_format:
            return None
        return CaptureMode(
            width=width,
            height=height,
            fps=fps,
            pixel_format=normalize_pixel_format(pixel_format),
            description=str(data.get("description", "")).strip(),
        )


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def format_fps(value: float) -> str:
    rounded = round(value)
    if abs(value - rounded) < 0.05:
        return str(int(rounded))
    return f"{value:.1f}"


def normalize_pixel_format(value: str) -> str:
    value = value.strip().upper()
    if value == "MJPEG":
        return "MJPG"
    if value == "YUY2":
        return "YUYV"
    return value


def fourcc_to_string(raw_value: float) -> str:
    if raw_value is None or math.isnan(raw_value):
        return "unknown"
    raw = int(round(raw_value))
    chars = [
        chr(raw & 0xFF),
        chr((raw >> 8) & 0xFF),
        chr((raw >> 16) & 0xFF),
        chr((raw >> 24) & 0xFF),
    ]
    text = "".join(chars).strip("\x00")
    return text or "unknown"


def pixel_format_fourcc(pixel_format: str) -> int:
    normalized = normalize_pixel_format(pixel_format)
    if len(normalized) != 4:
        raise ValueError(f"Expected fourcc-like pixel format, got '{pixel_format}'")
    return cv2.VideoWriter_fourcc(*normalized)


def default_profile_path() -> Path:
    if os.name == "nt":
        return Path.home() / "custom_vision" / "camera_dashboard.json"
    return Path.home() / ".config" / "custom_vision" / "camera_dashboard.json"


def default_export_dir(profile_path: Path) -> Path:
    return profile_path.parent / "exports"


def stream_resolution_label(width: int, height: int) -> str:
    return f"{width}x{height}"


def parse_stream_resolution(label: str) -> tuple[int, int]:
    match = re.fullmatch(r"\s*(\d+)x(\d+)\s*", label)
    if not match:
        return STREAM_RES_PRESETS[0]
    return int(match.group(1)), int(match.group(2))


def run_command(args: list[str]) -> str | None:
    try:
        completed = subprocess.run(
            args,
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError:
        return None
    if completed.returncode != 0:
        return None
    return completed.stdout


def extract_camera_index(device_label: str) -> int | None:
    match = re.search(r"(\d+)$", device_label.strip())
    if not match:
        return None
    return int(match.group(1))


def sort_device_path(path: str) -> tuple[int, str]:
    index = extract_camera_index(path)
    return (index if index is not None else 9999, path)


def discover_video_devices() -> list[str]:
    device_paths = sorted(set(glob.glob("/dev/video*")), key=sort_device_path)
    if device_paths:
        return device_paths
    return [str(i) for i in range(MAX_CAMERA_SLOTS)]


def parse_v4l2_modes(output: str) -> list[CaptureMode]:
    modes: dict[tuple[str, int, int, str], CaptureMode] = {}
    current_format = ""
    current_description = ""
    current_size: tuple[int, int] | None = None

    for raw_line in output.splitlines():
        line = raw_line.strip()
        format_match = re.match(r"\[\d+\]: '([^']+)' \((.+)\)", line)
        if format_match:
            current_format = normalize_pixel_format(format_match.group(1))
            current_description = format_match.group(2).strip()
            current_size = None
            continue

        size_match = re.match(r"Size: Discrete (\d+)x(\d+)", line)
        if size_match:
            current_size = (int(size_match.group(1)), int(size_match.group(2)))
            continue

        interval_match = re.match(r"Interval: Discrete [0-9.]+s \(([0-9.]+) fps\)", line)
        if interval_match and current_format and current_size:
            fps = float(interval_match.group(1))
            key = (current_format, current_size[0], current_size[1], format_fps(fps))
            modes[key] = CaptureMode(
                width=current_size[0],
                height=current_size[1],
                fps=fps,
                pixel_format=current_format,
                description=current_description,
            )

    return sorted(
        modes.values(),
        key=lambda mode: (mode.width, mode.height, mode.fps, mode.pixel_format),
    )


def open_capture(device_label: str, backend: str) -> cv2.VideoCapture:
    backend = backend.strip().lower()
    device_arg: int | str = int(device_label) if device_label.isdigit() else device_label

    if backend == "v4l2" and hasattr(cv2, "CAP_V4L2"):
        cap = cv2.VideoCapture(device_arg, cv2.CAP_V4L2)
        if cap.isOpened():
            return cap
    elif backend == "gstreamer" and hasattr(cv2, "CAP_GSTREAMER"):
        cap = cv2.VideoCapture(device_arg, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            return cap

    return cv2.VideoCapture(device_arg)


def detect_capture_modes(device_label: str, backend: str) -> list[CaptureMode]:
    if shutil.which("v4l2-ctl") and device_label.startswith("/dev/video") and backend in {"auto", "v4l2"}:
        output = run_command(["v4l2-ctl", "--list-formats-ext", "--device", device_label])
        if output:
            modes = parse_v4l2_modes(output)
            if modes:
                return modes

    cap = open_capture(device_label, backend)
    try:
        if not cap.isOpened():
            return []
        width = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH))) or 640
        height = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))) or 480
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        pixel_format = fourcc_to_string(cap.get(cv2.CAP_PROP_FOURCC))
        return [CaptureMode(width=width, height=height, fps=fps, pixel_format=pixel_format)]
    finally:
        cap.release()


def apply_capture_mode(cap: cv2.VideoCapture, mode: CaptureMode | None) -> None:
    if mode is None:
        return
    cap.set(cv2.CAP_PROP_FOURCC, float(pixel_format_fourcc(mode.pixel_format)))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(mode.width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(mode.height))
    cap.set(cv2.CAP_PROP_FPS, float(mode.fps))


def apply_live_controls(cap: cv2.VideoCapture, slot: dict[str, Any]) -> None:
    controls = slot.get("controls", {})
    low_latency = bool(controls.get("low_latency_mode", True))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1.0 if low_latency else 4.0)
    red_awb_prop = getattr(cv2, "CAP_PROP_WHITE_BALANCE_RED_V", None)
    blue_awb_prop = getattr(cv2, "CAP_PROP_WHITE_BALANCE_BLUE_U", None)

    auto_exposure = bool(controls.get("auto_exposure", False))
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75 if auto_exposure else 0.25)
    if "exposure" in controls:
        cap.set(cv2.CAP_PROP_EXPOSURE, float(controls["exposure"]))
    if "brightness" in controls:
        cap.set(cv2.CAP_PROP_BRIGHTNESS, float(controls["brightness"]))
    if "gain" in controls:
        cap.set(cv2.CAP_PROP_GAIN, float(controls["gain"]))
    if red_awb_prop is not None and "red_awb_gain" in controls:
        cap.set(red_awb_prop, float(controls["red_awb_gain"]) * 100.0)
    if blue_awb_prop is not None and "blue_awb_gain" in controls:
        cap.set(blue_awb_prop, float(controls["blue_awb_gain"]) * 100.0)

    auto_white_balance = bool(controls.get("auto_white_balance", True))
    cap.set(cv2.CAP_PROP_AUTO_WB, 1.0 if auto_white_balance else 0.0)
    if "white_balance_temperature" in controls:
        cap.set(cv2.CAP_PROP_WB_TEMPERATURE, float(controls["white_balance_temperature"]))


def apply_output_orientation(frame: np.ndarray, orientation: str) -> np.ndarray:
    if orientation == "cw90":
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if orientation == "180":
        return cv2.rotate(frame, cv2.ROTATE_180)
    if orientation == "ccw90":
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return frame


def make_placeholder_frame(width: int, height: int, text: str) -> np.ndarray:
    frame = np.zeros((max(1, height), max(1, width), 3), dtype=np.uint8)
    frame[:, :] = (28, 33, 40)
    cv2.putText(frame, text, (20, max(40, height // 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 230, 240), 2)
    return frame


def default_slot(slot_index: int, device_label: str = "") -> dict[str, Any]:
    camera_index = extract_camera_index(device_label) if device_label else None
    stream_width, stream_height = STREAM_RES_PRESETS[0]
    nickname_suffix = camera_index if camera_index is not None else slot_index
    return {
        "slot": slot_index,
        "camera_index": camera_index,
        "device_label": device_label,
        "camera_nickname": f"camera{nickname_suffix}",
        "team_number": 0,
        "camera_backend": "v4l2",
        "pipeline_index": 0,
        "pipeline_type": PIPELINE_TYPES[0],
        "processing_mode": PROCESSING_MODES[1],
        "decimate": 2,
        "multitag_enabled": True,
        "camera_type": CAMERA_TYPES[0],
        "orientation": "normal",
        "stream_display": STREAM_DISPLAYS[0],
        "stream_width": stream_width,
        "stream_height": stream_height,
        "capture_mode": {},
        "controls": {
            "auto_exposure": False,
            "exposure": -6,
            "brightness": 128,
            "gain": 75,
            "red_awb_gain": 11,
            "blue_awb_gain": 20,
            "auto_white_balance": True,
            "white_balance_temperature": 4000,
            "low_latency_mode": True,
        },
        "paths": {
            "calibration": "",
            "field_layout": "",
        },
        "apriltag": {
            "tag_family": "tag36h11",
            "threads": 4,
            "quad_decimate": 2.0,
            "quad_sigma": 0.0,
            "refine_edges": True,
            "pose_iterations": 0,
            "max_error_bits": 3,
            "decision_margin": 30.0,
        },
    }


def normalize_slot_data(slot: dict[str, Any] | None, slot_index: int, devices: list[str]) -> dict[str, Any]:
    fallback_device = devices[slot_index] if slot_index < len(devices) else ""
    base = default_slot(slot_index, fallback_device)
    if not isinstance(slot, dict):
        return base

    normalized = copy.deepcopy(base)
    for key in (
        "slot",
        "device_label",
        "camera_index",
        "camera_nickname",
        "team_number",
        "camera_backend",
        "pipeline_index",
        "pipeline_type",
        "processing_mode",
        "decimate",
        "multitag_enabled",
        "camera_type",
        "orientation",
        "stream_display",
        "stream_width",
        "stream_height",
        "capture_width",
        "capture_height",
        "capture_fps",
        "pixel_format",
    ):
        if key in slot:
            normalized[key] = slot[key]

    if isinstance(slot.get("controls"), dict):
        normalized["controls"].update(slot["controls"])
    if isinstance(slot.get("paths"), dict):
        normalized["paths"].update(slot["paths"])
    if isinstance(slot.get("apriltag"), dict):
        normalized["apriltag"].update(slot["apriltag"])

    capture_mode = CaptureMode.from_dict(slot.get("capture_mode"))
    if capture_mode is None:
        legacy_width = slot.get("capture_width")
        legacy_height = slot.get("capture_height")
        legacy_fps = slot.get("capture_fps")
        legacy_pixel_format = slot.get("pixel_format")
        try:
            if legacy_width and legacy_height and legacy_fps and legacy_pixel_format:
                capture_mode = CaptureMode(
                    width=int(legacy_width),
                    height=int(legacy_height),
                    fps=float(legacy_fps),
                    pixel_format=normalize_pixel_format(str(legacy_pixel_format)),
                )
        except (TypeError, ValueError):
            capture_mode = None
    normalized["capture_mode"] = capture_mode.to_dict() if capture_mode else {}

    if not normalized["device_label"]:
        normalized["device_label"] = fallback_device
    if normalized["camera_index"] is None:
        normalized["camera_index"] = extract_camera_index(str(normalized["device_label"]))
    if not normalized["camera_nickname"]:
        nickname_suffix = normalized["camera_index"] if normalized["camera_index"] is not None else slot_index
        normalized["camera_nickname"] = f"camera{nickname_suffix}"
    normalized["slot"] = slot_index
    return normalized


def load_profile(path: Path, devices: list[str]) -> dict[str, Any]:
    raw_data: dict[str, Any] = {}
    if path.exists():
        try:
            raw_data = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            raw_data = {}

    slots = raw_data.get("camera_slots", [])
    normalized_slots = [
        normalize_slot_data(slots[index] if index < len(slots) else None, index, devices)
        for index in range(MAX_CAMERA_SLOTS)
    ]
    return {
        "version": 1,
        "camera_slots": normalized_slots,
    }


def save_profile(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


class DashboardApp:
    def __init__(self, root: tk.Tk, profile_path: Path, initial_slot: int) -> None:
        self.root = root
        self.profile_path = profile_path.expanduser().resolve()
        self.export_dir = default_export_dir(self.profile_path)
        self.devices = discover_video_devices()
        self.profile_data = load_profile(self.profile_path, self.devices)
        self.mode_cache: dict[tuple[str, str], list[CaptureMode]] = {}
        self.capture_mode_lookup: dict[str, CaptureMode] = {}
        self.current_slot_index = clamp(initial_slot, 0, MAX_CAMERA_SLOTS - 1)
        self.cap: cv2.VideoCapture | None = None
        self.last_applied_signature: tuple[Any, ...] | None = None
        self._loading = False

        self.slot_var = tk.StringVar(value=str(self.current_slot_index + 1))
        self.device_var = tk.StringVar()
        self.backend_var = tk.StringVar()
        self.nickname_var = tk.StringVar()
        self.team_number_var = tk.StringVar()
        self.pipeline_index_var = tk.StringVar()
        self.processing_mode_var = tk.StringVar()
        self.decimate_var = tk.StringVar()
        self.multitag_var = tk.BooleanVar()
        self.camera_type_var = tk.StringVar()
        self.orientation_var = tk.StringVar()
        self.stream_display_var = tk.StringVar()
        self.capture_mode_var = tk.StringVar()
        self.stream_resolution_var = tk.StringVar()
        self.auto_exposure_var = tk.BooleanVar()
        self.exposure_var = tk.StringVar()
        self.brightness_var = tk.StringVar()
        self.gain_var = tk.StringVar()
        self.red_awb_gain_var = tk.StringVar()
        self.blue_awb_gain_var = tk.StringVar()
        self.auto_white_balance_var = tk.BooleanVar()
        self.white_balance_var = tk.StringVar()
        self.low_latency_var = tk.BooleanVar()
        self.calibration_path_var = tk.StringVar()
        self.field_layout_path_var = tk.StringVar()
        self.tag_family_var = tk.StringVar()
        self.threads_var = tk.StringVar()
        self.blur_sigma_var = tk.StringVar()
        self.refine_edges_var = tk.BooleanVar()
        self.pose_iterations_var = tk.StringVar()
        self.max_error_bits_var = tk.StringVar()
        self.decision_margin_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Ready")
        self.actual_mode_var = tk.StringVar(value="No camera open")
        self.launch_command_var = tk.StringVar()

        self.preview_label: ttk.Label | None = None
        self.capture_mode_combo: ttk.Combobox | None = None
        self.device_combo: ttk.Combobox | None = None

        self.build_ui()
        self.install_traces()
        self.load_slot(self.current_slot_index)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(30, self.update_preview)

    def build_ui(self) -> None:
        self.root.title(WINDOW_TITLE)
        self.root.geometry("1600x940")

        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=3)
        outer.columnconfigure(1, weight=2)
        outer.rowconfigure(0, weight=1)

        preview_frame = ttk.LabelFrame(outer, text="Preview")
        preview_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        preview_frame.columnconfigure(0, weight=1)
        preview_frame.rowconfigure(0, weight=1)

        self.preview_label = ttk.Label(preview_frame, anchor="center")
        self.preview_label.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)

        preview_info = ttk.Frame(preview_frame)
        preview_info.grid(row=1, column=0, sticky="ew", padx=8, pady=(0, 8))
        preview_info.columnconfigure(0, weight=1)
        ttk.Label(preview_info, textvariable=self.actual_mode_var).grid(row=0, column=0, sticky="w")
        ttk.Entry(preview_info, textvariable=self.launch_command_var).grid(row=1, column=0, sticky="ew", pady=(6, 0))

        controls_container = ttk.Frame(outer)
        controls_container.grid(row=0, column=1, sticky="nsew")
        controls_container.columnconfigure(0, weight=1)
        controls_container.rowconfigure(0, weight=1)

        canvas = tk.Canvas(controls_container, highlightthickness=0)
        scrollbar = ttk.Scrollbar(controls_container, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

        scroll_frame = ttk.Frame(canvas)
        scroll_frame.columnconfigure(0, weight=1)
        window_id = canvas.create_window((0, 0), window=scroll_frame, anchor="nw")

        def on_scroll_frame_configure(_event: tk.Event[Any]) -> None:
            canvas.configure(scrollregion=canvas.bbox("all"))

        def on_canvas_configure(event: tk.Event[Any]) -> None:
            canvas.itemconfigure(window_id, width=event.width)

        scroll_frame.bind("<Configure>", on_scroll_frame_configure)
        canvas.bind("<Configure>", on_canvas_configure)

        row = 0
        row = self.build_camera_frame(scroll_frame, row)
        row = self.build_pipeline_frame(scroll_frame, row)
        row = self.build_controls_frame(scroll_frame, row)
        row = self.build_runtime_frame(scroll_frame, row)

        action_frame = ttk.LabelFrame(scroll_frame, text="Actions")
        action_frame.grid(row=row, column=0, sticky="ew", pady=(10, 0))
        action_frame.columnconfigure(0, weight=1)
        ttk.Button(action_frame, text="Refresh Devices", command=self.refresh_devices).grid(
            row=0, column=0, sticky="ew", padx=8, pady=(8, 4)
        )
        ttk.Button(action_frame, text="Refresh Modes", command=self.refresh_modes_and_reopen).grid(
            row=1, column=0, sticky="ew", padx=8, pady=4
        )
        ttk.Button(action_frame, text="Export YAML Snapshot", command=self.export_yaml_snapshot).grid(
            row=2, column=0, sticky="ew", padx=8, pady=4
        )
        ttk.Label(action_frame, textvariable=self.status_var, wraplength=420).grid(
            row=3, column=0, sticky="w", padx=8, pady=(8, 8)
        )

    def build_camera_frame(self, parent: ttk.Frame, row: int) -> int:
        frame = ttk.LabelFrame(parent, text="Camera")
        frame.grid(row=row, column=0, sticky="ew")
        frame.columnconfigure(1, weight=1)

        ttk.Label(frame, text="Camera Slot").grid(row=0, column=0, sticky="w", padx=8, pady=(8, 4))
        slot_combo = ttk.Combobox(frame, textvariable=self.slot_var, values=["1", "2", "3", "4"], state="readonly")
        slot_combo.grid(row=0, column=1, sticky="ew", padx=8, pady=(8, 4))
        slot_combo.bind("<<ComboboxSelected>>", self.on_slot_selected)

        ttk.Label(frame, text="Device").grid(row=1, column=0, sticky="w", padx=8, pady=4)
        self.device_combo = ttk.Combobox(frame, textvariable=self.device_var, state="readonly")
        self.device_combo.grid(row=1, column=1, sticky="ew", padx=8, pady=4)
        self.device_combo.bind("<<ComboboxSelected>>", self.on_device_changed)

        ttk.Label(frame, text="Backend").grid(row=2, column=0, sticky="w", padx=8, pady=4)
        backend_combo = ttk.Combobox(
            frame,
            textvariable=self.backend_var,
            values=BACKEND_OPTIONS,
            state="readonly",
        )
        backend_combo.grid(row=2, column=1, sticky="ew", padx=8, pady=4)
        backend_combo.bind("<<ComboboxSelected>>", self.on_backend_changed)

        ttk.Label(frame, text="Nickname").grid(row=3, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(frame, textvariable=self.nickname_var).grid(row=3, column=1, sticky="ew", padx=8, pady=4)

        ttk.Label(frame, text="Team Number").grid(row=4, column=0, sticky="w", padx=8, pady=4)
        ttk.Spinbox(frame, from_=0, to=MAX_TEAM_NUMBER, textvariable=self.team_number_var).grid(
            row=4, column=1, sticky="ew", padx=8, pady=4
        )

        ttk.Label(frame, text="Capture Mode").grid(row=5, column=0, sticky="w", padx=8, pady=(4, 8))
        self.capture_mode_combo = ttk.Combobox(frame, textvariable=self.capture_mode_var, state="readonly")
        self.capture_mode_combo.grid(row=5, column=1, sticky="ew", padx=8, pady=(4, 8))
        self.capture_mode_combo.bind("<<ComboboxSelected>>", self.on_capture_mode_changed)
        return row + 1

    def build_pipeline_frame(self, parent: ttk.Frame, row: int) -> int:
        frame = ttk.LabelFrame(parent, text="Pipeline")
        frame.grid(row=row, column=0, sticky="ew", pady=(10, 0))
        frame.columnconfigure(1, weight=1)

        ttk.Label(frame, text="Pipeline Index").grid(row=0, column=0, sticky="w", padx=8, pady=(8, 4))
        ttk.Spinbox(frame, from_=0, to=9, textvariable=self.pipeline_index_var).grid(
            row=0, column=1, sticky="ew", padx=8, pady=(8, 4)
        )

        ttk.Label(frame, text="Pipeline Type").grid(row=1, column=0, sticky="w", padx=8, pady=4)
        pipeline_type = ttk.Combobox(frame, values=PIPELINE_TYPES, state="readonly")
        pipeline_type.grid(row=1, column=1, sticky="ew", padx=8, pady=4)
        pipeline_type.set(PIPELINE_TYPES[0])

        ttk.Label(frame, text="Processing Mode").grid(row=2, column=0, sticky="w", padx=8, pady=4)
        ttk.Combobox(frame, textvariable=self.processing_mode_var, values=PROCESSING_MODES, state="readonly").grid(
            row=2, column=1, sticky="ew", padx=8, pady=4
        )

        ttk.Label(frame, text="Decimate").grid(row=3, column=0, sticky="w", padx=8, pady=4)
        ttk.Spinbox(frame, from_=1, to=4, textvariable=self.decimate_var).grid(
            row=3, column=1, sticky="ew", padx=8, pady=4
        )

        ttk.Checkbutton(frame, text="Enable MultiTag", variable=self.multitag_var).grid(
            row=4, column=0, columnspan=2, sticky="w", padx=8, pady=4
        )

        ttk.Label(frame, text="Camera Type").grid(row=5, column=0, sticky="w", padx=8, pady=4)
        ttk.Combobox(frame, textvariable=self.camera_type_var, values=CAMERA_TYPES, state="readonly").grid(
            row=5, column=1, sticky="ew", padx=8, pady=4
        )

        ttk.Label(frame, text="Orientation").grid(row=6, column=0, sticky="w", padx=8, pady=4)
        ttk.Combobox(
            frame,
            textvariable=self.orientation_var,
            values=list(ORIENTATION_LABEL_TO_VALUE.keys()),
            state="readonly",
        ).grid(row=6, column=1, sticky="ew", padx=8, pady=4)

        ttk.Label(frame, text="Stream Display").grid(row=7, column=0, sticky="w", padx=8, pady=4)
        ttk.Combobox(
            frame,
            textvariable=self.stream_display_var,
            values=STREAM_DISPLAYS,
            state="readonly",
        ).grid(row=7, column=1, sticky="ew", padx=8, pady=4)

        ttk.Label(frame, text="Stream Resolution").grid(row=8, column=0, sticky="w", padx=8, pady=(4, 8))
        ttk.Combobox(
            frame,
            textvariable=self.stream_resolution_var,
            values=[stream_resolution_label(w, h) for w, h in STREAM_RES_PRESETS],
            state="readonly",
        ).grid(row=8, column=1, sticky="ew", padx=8, pady=(4, 8))
        return row + 1

    def build_controls_frame(self, parent: ttk.Frame, row: int) -> int:
        frame = ttk.LabelFrame(parent, text="Camera Controls")
        frame.grid(row=row, column=0, sticky="ew", pady=(10, 0))
        frame.columnconfigure(1, weight=1)

        ttk.Checkbutton(frame, text="Auto Exposure", variable=self.auto_exposure_var).grid(
            row=0, column=0, columnspan=2, sticky="w", padx=8, pady=(8, 4)
        )

        self.add_spinbox_row(frame, 1, "Exposure", self.exposure_var, -13, 1)
        self.add_spinbox_row(frame, 2, "Brightness", self.brightness_var, 0, 255)
        self.add_spinbox_row(frame, 3, "Gain", self.gain_var, 0, 255)
        self.add_spinbox_row(frame, 4, "Red AWB Gain", self.red_awb_gain_var, 0, 40)
        self.add_spinbox_row(frame, 5, "Blue AWB Gain", self.blue_awb_gain_var, 0, 40)

        ttk.Checkbutton(frame, text="Auto White Balance", variable=self.auto_white_balance_var).grid(
            row=6, column=0, columnspan=2, sticky="w", padx=8, pady=4
        )

        self.add_spinbox_row(frame, 7, "White Balance Temp", self.white_balance_var, 2500, 7500)
        ttk.Checkbutton(frame, text="Low Latency Mode", variable=self.low_latency_var).grid(
            row=8, column=0, columnspan=2, sticky="w", padx=8, pady=(4, 8)
        )
        return row + 1

    def build_runtime_frame(self, parent: ttk.Frame, row: int) -> int:
        frame = ttk.LabelFrame(parent, text="Runtime")
        frame.grid(row=row, column=0, sticky="ew", pady=(10, 0))
        frame.columnconfigure(1, weight=1)

        ttk.Label(frame, text="Calibration Path").grid(row=0, column=0, sticky="w", padx=8, pady=(8, 4))
        ttk.Entry(frame, textvariable=self.calibration_path_var).grid(row=0, column=1, sticky="ew", padx=8, pady=(8, 4))

        ttk.Label(frame, text="Field Layout Path").grid(row=1, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(frame, textvariable=self.field_layout_path_var).grid(row=1, column=1, sticky="ew", padx=8, pady=4)

        ttk.Label(frame, text="Tag Family").grid(row=2, column=0, sticky="w", padx=8, pady=4)
        ttk.Combobox(frame, textvariable=self.tag_family_var, values=TAG_FAMILY_OPTIONS, state="readonly").grid(
            row=2, column=1, sticky="ew", padx=8, pady=4
        )

        self.add_spinbox_row(frame, 3, "Threads", self.threads_var, 1, 16)
        self.add_spinbox_row(frame, 4, "Blur Sigma", self.blur_sigma_var, 0, 10, increment=0.1)

        ttk.Checkbutton(frame, text="Refine Edges", variable=self.refine_edges_var).grid(
            row=5, column=0, columnspan=2, sticky="w", padx=8, pady=4
        )

        self.add_spinbox_row(frame, 6, "Pose Iterations", self.pose_iterations_var, 0, 100)
        self.add_spinbox_row(frame, 7, "Max Error Bits", self.max_error_bits_var, 0, 5)
        self.add_spinbox_row(frame, 8, "Decision Margin", self.decision_margin_var, 0, 100, increment=0.5, pady=(4, 8))
        return row + 1

    def add_spinbox_row(
        self,
        parent: ttk.LabelFrame,
        row: int,
        label: str,
        variable: tk.Variable,
        low: float,
        high: float,
        increment: float = 1.0,
        pady: tuple[int, int] | int = 4,
    ) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=8, pady=pady)
        ttk.Spinbox(parent, from_=low, to=high, increment=increment, textvariable=variable).grid(
            row=row,
            column=1,
            sticky="ew",
            padx=8,
            pady=pady,
        )

    def install_traces(self) -> None:
        watched_vars = [
            self.device_var,
            self.backend_var,
            self.nickname_var,
            self.team_number_var,
            self.pipeline_index_var,
            self.processing_mode_var,
            self.decimate_var,
            self.multitag_var,
            self.camera_type_var,
            self.orientation_var,
            self.stream_display_var,
            self.capture_mode_var,
            self.stream_resolution_var,
            self.auto_exposure_var,
            self.exposure_var,
            self.brightness_var,
            self.gain_var,
            self.red_awb_gain_var,
            self.blue_awb_gain_var,
            self.auto_white_balance_var,
            self.white_balance_var,
            self.low_latency_var,
            self.calibration_path_var,
            self.field_layout_path_var,
            self.tag_family_var,
            self.threads_var,
            self.blur_sigma_var,
            self.refine_edges_var,
            self.pose_iterations_var,
            self.max_error_bits_var,
            self.decision_margin_var,
        ]
        for variable in watched_vars:
            variable.trace_add("write", self.on_state_trace)

    def current_slot(self) -> dict[str, Any]:
        return self.profile_data["camera_slots"][self.current_slot_index]

    def on_state_trace(self, *_args: Any) -> None:
        if self._loading:
            return
        self.save_ui_to_current_slot()
        self.persist_profile()
        self.update_launch_command()

    def load_slot(self, slot_index: int) -> None:
        self.current_slot_index = clamp(slot_index, 0, MAX_CAMERA_SLOTS - 1)
        slot = self.current_slot()
        self._loading = True
        try:
            self.slot_var.set(str(self.current_slot_index + 1))
            self.device_var.set(str(slot.get("device_label", "")))
            self.backend_var.set(str(slot.get("camera_backend", "v4l2")))
            self.nickname_var.set(str(slot.get("camera_nickname", "")))
            self.team_number_var.set(str(slot.get("team_number", 0)))
            self.pipeline_index_var.set(str(slot.get("pipeline_index", 0)))
            self.processing_mode_var.set(str(slot.get("processing_mode", PROCESSING_MODES[1])))
            self.decimate_var.set(str(slot.get("decimate", 2)))
            self.multitag_var.set(bool(slot.get("multitag_enabled", True)))
            self.camera_type_var.set(str(slot.get("camera_type", CAMERA_TYPES[0])))
            self.orientation_var.set(
                ORIENTATION_VALUE_TO_LABEL.get(str(slot.get("orientation", "normal")), "Normal")
            )
            self.stream_display_var.set(str(slot.get("stream_display", STREAM_DISPLAYS[0])))
            self.stream_resolution_var.set(
                stream_resolution_label(
                    int(slot.get("stream_width", STREAM_RES_PRESETS[0][0])),
                    int(slot.get("stream_height", STREAM_RES_PRESETS[0][1])),
                )
            )

            controls = slot.get("controls", {})
            self.auto_exposure_var.set(bool(controls.get("auto_exposure", False)))
            self.exposure_var.set(str(controls.get("exposure", -6)))
            self.brightness_var.set(str(controls.get("brightness", 128)))
            self.gain_var.set(str(controls.get("gain", 75)))
            self.red_awb_gain_var.set(str(controls.get("red_awb_gain", 11)))
            self.blue_awb_gain_var.set(str(controls.get("blue_awb_gain", 20)))
            self.auto_white_balance_var.set(bool(controls.get("auto_white_balance", True)))
            self.white_balance_var.set(str(controls.get("white_balance_temperature", 4000)))
            self.low_latency_var.set(bool(controls.get("low_latency_mode", True)))

            paths = slot.get("paths", {})
            self.calibration_path_var.set(str(paths.get("calibration", "")))
            self.field_layout_path_var.set(str(paths.get("field_layout", "")))

            apriltag = slot.get("apriltag", {})
            self.tag_family_var.set(str(apriltag.get("tag_family", "tag36h11")))
            self.threads_var.set(str(apriltag.get("threads", 4)))
            self.blur_sigma_var.set(str(apriltag.get("quad_sigma", 0.0)))
            self.refine_edges_var.set(bool(apriltag.get("refine_edges", True)))
            self.pose_iterations_var.set(str(apriltag.get("pose_iterations", 0)))
            self.max_error_bits_var.set(str(apriltag.get("max_error_bits", 3)))
            self.decision_margin_var.set(str(apriltag.get("decision_margin", 30.0)))
        finally:
            self._loading = False

        self.refresh_devices(update_status=False)
        self.refresh_modes(update_status=False)
        saved_mode = CaptureMode.from_dict(slot.get("capture_mode"))
        self.capture_mode_var.set(saved_mode.label if saved_mode else "")
        self.update_launch_command()
        self.reopen_capture()

    def save_ui_to_current_slot(self) -> None:
        slot = self.current_slot()
        slot["slot"] = self.current_slot_index
        slot["device_label"] = self.device_var.get().strip()
        slot["camera_index"] = extract_camera_index(slot["device_label"])
        slot["camera_nickname"] = self.nickname_var.get().strip() or f"camera{self.current_slot_index}"
        slot["team_number"] = clamp(self.safe_int(self.team_number_var.get(), 0), 0, MAX_TEAM_NUMBER)
        slot["camera_backend"] = self.backend_var.get().strip() or "v4l2"
        slot["pipeline_index"] = clamp(self.safe_int(self.pipeline_index_var.get(), 0), 0, 9)
        slot["pipeline_type"] = PIPELINE_TYPES[0]
        slot["processing_mode"] = self.processing_mode_var.get().strip() or PROCESSING_MODES[1]
        slot["decimate"] = clamp(self.safe_int(self.decimate_var.get(), 2), 1, 4)
        slot["multitag_enabled"] = bool(self.multitag_var.get())
        slot["camera_type"] = self.camera_type_var.get().strip() or CAMERA_TYPES[0]
        slot["orientation"] = ORIENTATION_LABEL_TO_VALUE.get(self.orientation_var.get(), "normal")
        slot["stream_display"] = self.stream_display_var.get().strip() or STREAM_DISPLAYS[0]
        stream_width, stream_height = parse_stream_resolution(self.stream_resolution_var.get())
        slot["stream_width"] = stream_width
        slot["stream_height"] = stream_height

        capture_mode = self.selected_capture_mode()
        slot["capture_mode"] = capture_mode.to_dict() if capture_mode else {}
        if capture_mode:
            slot["capture_width"] = capture_mode.width
            slot["capture_height"] = capture_mode.height
            slot["capture_fps"] = capture_mode.fps
            slot["pixel_format"] = capture_mode.pixel_format

        slot["controls"] = {
            "auto_exposure": bool(self.auto_exposure_var.get()),
            "exposure": self.safe_int(self.exposure_var.get(), -6),
            "brightness": self.safe_int(self.brightness_var.get(), 128),
            "gain": self.safe_int(self.gain_var.get(), 75),
            "red_awb_gain": self.safe_int(self.red_awb_gain_var.get(), 11),
            "blue_awb_gain": self.safe_int(self.blue_awb_gain_var.get(), 20),
            "auto_white_balance": bool(self.auto_white_balance_var.get()),
            "white_balance_temperature": self.safe_int(self.white_balance_var.get(), 4000),
            "low_latency_mode": bool(self.low_latency_var.get()),
        }

        slot["paths"] = {
            "calibration": self.calibration_path_var.get().strip(),
            "field_layout": self.field_layout_path_var.get().strip(),
        }

        slot["apriltag"] = {
            "tag_family": self.tag_family_var.get().strip() or "tag36h11",
            "threads": self.safe_int(self.threads_var.get(), 4),
            "quad_decimate": float(slot["decimate"]),
            "quad_sigma": self.safe_float(self.blur_sigma_var.get(), 0.0),
            "refine_edges": bool(self.refine_edges_var.get()),
            "pose_iterations": self.safe_int(self.pose_iterations_var.get(), 0),
            "max_error_bits": self.safe_int(self.max_error_bits_var.get(), 3),
            "decision_margin": self.safe_float(self.decision_margin_var.get(), 30.0),
        }

    def persist_profile(self) -> None:
        save_profile(self.profile_path, self.profile_data)

    def refresh_devices(self, update_status: bool = True) -> None:
        self.devices = discover_video_devices()
        values = tuple(self.devices)
        if self.device_combo is not None:
            self.device_combo["values"] = values
        current_device = self.device_var.get().strip()
        if current_device not in values:
            self._loading = True
            try:
                self.device_var.set(values[0] if values else "")
            finally:
                self._loading = False
        if update_status:
            self.status_var.set(f"Detected {len(values)} camera device(s)")
            self.save_ui_to_current_slot()
            self.persist_profile()
            self.refresh_modes(update_status=False)
            self.reopen_capture()

    def refresh_modes_and_reopen(self) -> None:
        self.refresh_modes(force_refresh=True)
        self.reopen_capture()

    def refresh_modes(self, force_refresh: bool = False, update_status: bool = True) -> None:
        device = self.device_var.get().strip()
        backend = self.backend_var.get().strip() or "v4l2"
        key = (device, backend)
        if force_refresh or key not in self.mode_cache:
            self.mode_cache[key] = detect_capture_modes(device, backend) if device else []

        modes = self.mode_cache.get(key, [])
        self.capture_mode_lookup = {mode.label: mode for mode in modes}
        mode_labels = tuple(self.capture_mode_lookup.keys())
        if self.capture_mode_combo is not None:
            self.capture_mode_combo["values"] = mode_labels

        current_label = self.capture_mode_var.get().strip()
        saved_mode = CaptureMode.from_dict(self.current_slot().get("capture_mode"))
        desired_label = current_label
        if saved_mode is not None and saved_mode.label in self.capture_mode_lookup:
            desired_label = saved_mode.label
        if desired_label not in self.capture_mode_lookup and mode_labels:
            desired_label = mode_labels[0]

        self._loading = True
        try:
            self.capture_mode_var.set(desired_label)
        finally:
            self._loading = False

        self.save_ui_to_current_slot()
        self.persist_profile()
        self.update_launch_command()
        if update_status:
            if mode_labels:
                self.status_var.set(f"Detected {len(mode_labels)} capture mode(s) for {device}")
            else:
                self.status_var.set(f"No capture modes detected for {device or 'the selected device'}")

    def selected_capture_mode(self) -> CaptureMode | None:
        label = self.capture_mode_var.get().strip()
        if label in self.capture_mode_lookup:
            return self.capture_mode_lookup[label]
        return CaptureMode.from_dict(self.current_slot().get("capture_mode"))

    def reopen_capture(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None

        device = self.device_var.get().strip()
        if not device:
            self.actual_mode_var.set("No device selected")
            return

        backend = self.backend_var.get().strip() or "v4l2"
        self.cap = open_capture(device, backend)
        if not self.cap.isOpened():
            self.actual_mode_var.set(f"Failed to open {device}")
            self.status_var.set(f"Could not open camera device {device}")
            self.cap.release()
            self.cap = None
            return

        apply_capture_mode(self.cap, self.selected_capture_mode())
        self.save_ui_to_current_slot()
        apply_live_controls(self.cap, self.current_slot())
        self.last_applied_signature = None
        self.status_var.set(f"Opened {device} with backend {backend}")

    def current_live_signature(self) -> tuple[Any, ...]:
        slot = self.current_slot()
        mode = self.selected_capture_mode()
        controls = slot.get("controls", {})
        return (
            mode.label if mode else "",
            controls.get("auto_exposure"),
            controls.get("exposure"),
            controls.get("brightness"),
            controls.get("gain"),
            controls.get("red_awb_gain"),
            controls.get("blue_awb_gain"),
            controls.get("auto_white_balance"),
            controls.get("white_balance_temperature"),
            controls.get("low_latency_mode"),
        )

    def update_preview(self) -> None:
        if self.cap is None and self.device_var.get().strip():
            self.reopen_capture()

        frame: np.ndarray | None = None
        if self.cap is not None:
            signature = self.current_live_signature()
            if signature != self.last_applied_signature:
                apply_capture_mode(self.cap, self.selected_capture_mode())
                apply_live_controls(self.cap, self.current_slot())
                self.last_applied_signature = signature
            ok, frame = self.cap.read()
            if not ok or frame is None or frame.size == 0:
                frame = None

        if frame is None:
            mode = self.selected_capture_mode()
            width = mode.width if mode else 960
            height = mode.height if mode else 540
            frame = make_placeholder_frame(width, height, "No camera frame")

        orientation = ORIENTATION_LABEL_TO_VALUE.get(self.orientation_var.get(), "normal")
        display_frame = apply_output_orientation(frame, orientation)
        actual_width = 0
        actual_height = 0
        actual_fps = 0.0
        actual_format = "unknown"
        if self.cap is not None:
            actual_width = int(round(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
            actual_height = int(round(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            actual_format = fourcc_to_string(self.cap.get(cv2.CAP_PROP_FOURCC))
        self.actual_mode_var.set(
            f"Actual: {actual_width}x{actual_height} | {format_fps(actual_fps)} FPS | {actual_format}"
        )

        if self.preview_label is not None:
            self.render_frame_to_label(display_frame, self.preview_label)

        self.root.after(30, self.update_preview)

    def render_frame_to_label(self, frame: np.ndarray, label: ttk.Label) -> None:
        target_width = max(640, label.winfo_width() - 10)
        target_height = max(360, label.winfo_height() - 10)
        scale = min(target_width / max(1, frame.shape[1]), target_height / max(1, frame.shape[0]))
        resized = cv2.resize(frame, (max(1, int(frame.shape[1] * scale)), max(1, int(frame.shape[0] * scale))))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        ppm_header = f"P6 {rgb.shape[1]} {rgb.shape[0]} 255 ".encode("ascii")
        photo = tk.PhotoImage(data=ppm_header + rgb.tobytes(), format="PPM")
        label.configure(image=photo)
        label.image = photo

    def export_yaml_snapshot(self) -> None:
        self.save_ui_to_current_slot()
        slot = self.current_slot()
        self.export_dir.mkdir(parents=True, exist_ok=True)
        path = self.export_dir / f"camera_slot_{self.current_slot_index}.yaml"

        capture_mode = CaptureMode.from_dict(slot.get("capture_mode"))
        controls = slot.get("controls", {})
        apriltag = slot.get("apriltag", {})
        paths = slot.get("paths", {})

        lines = [
            "# Generated by camera_mode_dashboard.py",
            "camera:",
            f"  slot: {self.current_slot_index}",
            f"  device: \"{slot.get('device_label', '')}\"",
            f"  nickname: \"{slot.get('camera_nickname', '')}\"",
            f"  team_number: {slot.get('team_number', 0)}",
            f"  backend: \"{slot.get('camera_backend', 'v4l2')}\"",
            f"  pixel_format: \"{capture_mode.pixel_format if capture_mode else ''}\"",
            f"  width: {capture_mode.width if capture_mode else 0}",
            f"  height: {capture_mode.height if capture_mode else 0}",
            f"  fps: {format_fps(capture_mode.fps) if capture_mode else '0'}",
            f"  stream_width: {slot.get('stream_width', 0)}",
            f"  stream_height: {slot.get('stream_height', 0)}",
            f"  orientation: \"{slot.get('orientation', 'normal')}\"",
            f"  pipeline_index: {slot.get('pipeline_index', 0)}",
            f"  processing_mode: \"{slot.get('processing_mode', PROCESSING_MODES[1])}\"",
            f"  camera_type: \"{slot.get('camera_type', CAMERA_TYPES[0])}\"",
            f"  stream_display: \"{slot.get('stream_display', STREAM_DISPLAYS[0])}\"",
            f"  decimate: {slot.get('decimate', 2)}",
            f"  multitag_enabled: {str(bool(slot.get('multitag_enabled', True))).lower()}",
            f"  calibration_path: \"{paths.get('calibration', '')}\"",
            f"  field_layout_path: \"{paths.get('field_layout', '')}\"",
            "controls:",
            f"  auto_exposure: {str(bool(controls.get('auto_exposure', False))).lower()}",
            f"  exposure: {controls.get('exposure', -6)}",
            f"  brightness: {controls.get('brightness', 128)}",
            f"  gain: {controls.get('gain', 75)}",
            f"  red_awb_gain: {controls.get('red_awb_gain', 11)}",
            f"  blue_awb_gain: {controls.get('blue_awb_gain', 20)}",
            f"  auto_white_balance: {str(bool(controls.get('auto_white_balance', True))).lower()}",
            f"  white_balance_temperature: {controls.get('white_balance_temperature', 4000)}",
            f"  low_latency_mode: {str(bool(controls.get('low_latency_mode', True))).lower()}",
            "apriltag:",
            f"  tag_family: \"{apriltag.get('tag_family', 'tag36h11')}\"",
            f"  threads: {apriltag.get('threads', 4)}",
            f"  quad_decimate: {apriltag.get('quad_decimate', 2.0)}",
            f"  quad_sigma: {apriltag.get('quad_sigma', 0.0)}",
            f"  refine_edges: {str(bool(apriltag.get('refine_edges', True))).lower()}",
            f"  pose_iterations: {apriltag.get('pose_iterations', 0)}",
            f"  max_error_bits: {apriltag.get('max_error_bits', 3)}",
            f"  decision_margin: {apriltag.get('decision_margin', 30.0)}",
        ]
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        self.persist_profile()
        self.status_var.set(f"Saved YAML snapshot to {path}")

    def update_launch_command(self) -> None:
        self.launch_command_var.set(
            f"./build/apriltag_vision --camera-slot {self.current_slot_index} --profile-path \"{self.profile_path}\""
        )

    def on_slot_selected(self, _event: tk.Event[Any]) -> None:
        self.save_ui_to_current_slot()
        self.persist_profile()
        self.load_slot(self.safe_int(self.slot_var.get(), 1) - 1)

    def on_device_changed(self, _event: tk.Event[Any]) -> None:
        if self._loading:
            return
        self.refresh_modes(force_refresh=True)
        self.reopen_capture()

    def on_backend_changed(self, _event: tk.Event[Any]) -> None:
        if self._loading:
            return
        self.refresh_modes(force_refresh=True)
        self.reopen_capture()

    def on_capture_mode_changed(self, _event: tk.Event[Any]) -> None:
        if self._loading:
            return
        self.save_ui_to_current_slot()
        self.persist_profile()
        self.reopen_capture()

    def on_close(self) -> None:
        self.save_ui_to_current_slot()
        self.persist_profile()
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        self.root.destroy()

    @staticmethod
    def safe_int(value: str, default: int) -> int:
        try:
            return int(float(value))
        except (TypeError, ValueError):
            return default

    @staticmethod
    def safe_float(value: str, default: float) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Custom Vision camera mode dashboard")
    parser.add_argument(
        "--profile-path",
        default=str(default_profile_path()),
        help="Persistent camera dashboard profile path",
    )
    parser.add_argument(
        "--slot",
        type=int,
        default=1,
        help="Initial camera slot to open (1-4)",
    )
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    profile_path = Path(args.profile_path).expanduser()
    initial_slot = clamp(args.slot - 1, 0, MAX_CAMERA_SLOTS - 1)

    root = tk.Tk()
    DashboardApp(root, profile_path=profile_path, initial_slot=initial_slot)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
