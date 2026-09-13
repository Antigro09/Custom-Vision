#!/usr/bin/env python3
"""Read-only environment inventory; no invented readiness or performance claims."""
import glob
import importlib
import json
import platform
import subprocess

report = {"platform": platform.platform(), "python": platform.python_version(), "cameras": glob.glob('/dev/video*'), "stable_camera_paths": glob.glob('/dev/v4l/by-id/*')}
for name in ('numpy', 'cv2', 'pupil_apriltags', 'ntcore', 'tensorrt'):
    try:
        module = importlib.import_module(name)
        report[name] = {"version": getattr(module, '__version__', 'installed'), "path": module.__file__}
        if name == 'cv2':
            report[name]['opencv_cuda_devices'] = module.cuda.getCudaEnabledDeviceCount()
            report[name]['gstreamer'] = 'GStreamer:                   YES' in module.getBuildInformation()
    except Exception as exc:
        report[name] = {"error": str(exc)}
for command in (['v4l2-ctl', '--list-devices'], ['/usr/sbin/nvpmodel', '-q'], ['systemctl', 'is-active', 'photonvision']):
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=10)
        report[' '.join(command)] = (result.stdout + result.stderr).strip()
    except (OSError, subprocess.TimeoutExpired) as exc:
        report[' '.join(command)] = str(exc)
report['live_validation_possible'] = bool(report['cameras'])
print(json.dumps(report, indent=2))
