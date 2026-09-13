"""Discover real UVC capabilities and apply only controls a device advertises.

No fixed camera capability claims and no shell execution. MJPEG capture mode
validation is based on discrete tuples reported by v4l2-ctl, not guessed rates.
"""
from pathlib import Path
import re
import shutil
import subprocess


_SUPPORTED_TYPES = {"int", "bool", "menu", "intmenu"}
_DEVICE = re.compile(r"/dev/video[0-9]+$")


def device_path(source):
    if isinstance(source, bool):
        raise ValueError("Camera source must be a UVC device")
    if isinstance(source, int) and source >= 0:
        source = f"/dev/video{source}"
    if not isinstance(source, str):
        raise ValueError("Camera source must be a UVC device")
    path = str(Path(source).resolve())
    if not _DEVICE.fullmatch(path):
        raise ValueError("Controls require /dev/videoN or a stable symlink to it")
    return path


def _run(source, *arguments):
    binary = shutil.which("v4l2-ctl")
    if not binary:
        raise RuntimeError("v4l2-ctl is unavailable; install the v4l-utils system package")
    result = subprocess.run([binary, "--device", device_path(source), *arguments],
                            capture_output=True, text=True, timeout=4, check=False)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip() or "UVC operation failed")
    return result.stdout


def parse_modes(output):
    """Return only explicit format/resolution/frame-rate tuples from the driver."""
    fourcc = None
    size = None
    modes = []
    for line in output.splitlines():
        fmt = re.search(r"\[\d+\]: '(.{4})'", line)
        if fmt:
            fourcc, size = fmt.group(1), None
        resolution = re.search(r"Size: Discrete (\d+)x(\d+)", line)
        if resolution:
            size = tuple(map(int, resolution.groups()))
        elif "Size:" in line:
            size = None
        rate = re.search(r"Interval: Discrete .*\(([0-9.]+) fps\)", line)
        if rate and size and fourcc:
            mode = {"fourcc": fourcc, "width": size[0], "height": size[1], "fps": float(rate.group(1))}
            if mode not in modes:
                modes.append(mode)
    return modes


def parse_controls(output):
    controls = []
    current = None
    for line in output.splitlines():
        match = re.match(r"\s*([a-zA-Z0-9_]+)\s+0x[0-9a-fA-F]+\s+\((\w+)\)\s*:\s*(.*)", line)
        if match:
            name, kind, attributes = match.groups()
            current = {"name": name, "type": kind, "menu": {}}
            for key in ("min", "max", "step", "default", "value"):
                value = re.search(rf"\b{key}=(-?\d+)", attributes)
                if value:
                    current[key] = int(value.group(1))
            if kind == "bool":
                current.setdefault("min", 0)
                current.setdefault("max", 1)
            current.setdefault("step", 1)
            flags = attributes.split("flags=", 1)[-1] if "flags=" in attributes else ""
            current["flags"] = [part.strip() for part in flags.split(",") if part.strip()]
            current["writable"] = kind in _SUPPORTED_TYPES and not any(
                flag in {"read-only", "disabled", "inactive"} for flag in current["flags"])
            controls.append(current)
        elif current and current["type"] in {"menu", "intmenu"}:
            entry = re.match(r"\s+(-?\d+):\s+(.+)", line)
            if entry:
                current["menu"][entry.group(1)] = entry.group(2).strip()
    return controls


def describe_device(source):
    path = device_path(source)
    name_file = Path("/sys/class/video4linux") / Path(path).name / "name"
    name = name_file.read_text().strip() if name_file.exists() else Path(path).name
    return {"source": path, "name": name,
            "modes": parse_modes(_run(path, "--list-formats-ext")),
            "controls": parse_controls(_run(path, "--list-ctrls-menus"))}


def discover_devices():
    devices = []
    for path in sorted(Path("/dev").glob("video[0-9]*")):
        try:
            devices.append(describe_device(str(path)))
        except (ValueError, OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
            devices.append({"source": str(path), "name": path.name, "modes": [], "controls": [], "error": str(exc)})
    return {"available": shutil.which("v4l2-ctl") is not None, "devices": devices,
            "note": "Only driver-advertised discrete modes and UVC controls are offered."}


def validate_controls(controls, descriptors):
    if not isinstance(controls, dict):
        raise ValueError("Camera controls must be a mapping of UVC control names to integers")
    advertised = {item["name"]: item for item in descriptors}
    for name, value in controls.items():
        spec = advertised.get(name)
        if not spec or not spec.get("writable"):
            raise ValueError(f"Control {name} is unsupported, inactive, or read-only")
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"Control {name} must be an integer")
        if "min" not in spec or "max" not in spec:
            raise ValueError(f"Control {name} has no safe advertised range")
        if not spec["min"] <= value <= spec["max"]:
            raise ValueError(f"Control {name} must be in [{spec['min']}, {spec['max']}]")
        if (value - spec["min"]) % max(1, spec.get("step", 1)):
            raise ValueError(f"Control {name} must follow the advertised step")
        if spec.get("menu") and str(value) not in spec["menu"]:
            raise ValueError(f"Control {name} must use an advertised menu option")
    return dict(controls)


def apply_controls(source, controls):
    """Validate an entire batch before applying it; return driver readback.

Automatic exposure/white balance should be disabled and saved before changing
manual controls if the driver currently marks those manual controls inactive.
"""
    if not controls:
        return {}
    path = device_path(source)
    descriptors = parse_controls(_run(path, "--list-ctrls-menus"))
    values = validate_controls(controls, descriptors)
    # Drivers commonly need auto flags first. All names came from enumeration.
    names = sorted(values, key=lambda name: ("auto" not in name, name))
    _run(path, "--set-ctrl", ",".join(f"{name}={values[name]}" for name in names))
    actual = {item["name"]: item.get("value") for item in parse_controls(_run(path, "--list-ctrls-menus"))}
    return {name: actual.get(name) for name in names}
