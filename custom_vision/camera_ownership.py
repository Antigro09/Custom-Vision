"""Cooperative physical-camera ownership for runtime and calibration capture.

No device discovery or device opening happens here. POSIX advisory locks cover
cooperating processes; a Linux fuser probe catches already-open foreign readers
where available. That probe is not an exclusive kernel camera reservation.
"""
from __future__ import annotations

import hashlib
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import threading


class CameraBusyError(RuntimeError):
    pass


_LOCK = threading.RLock()
_OWNERS = {}


def physical_device(source, backend="auto"):
    """Canonicalize integer Linux UVC indices and /dev aliases; files are not cameras."""
    if backend == "gstreamer":
        # GStreamer strings may contain device paths, but guessing an arbitrary
        # pipeline's ownership would be unsafe. Such pipelines are not GUI inputs.
        return None
    if isinstance(source, int) and not isinstance(source, bool):
        if sys.platform != "linux":
            return None
        source = f"/dev/video{source}"
    if isinstance(source, (str, os.PathLike)) and os.fspath(source).startswith("/dev/"):
        return str(Path(source).resolve())
    return None


def foreign_camera_readers(device):
    """Return a checked status without killing anyone or exposing device paths."""
    if sys.platform != "linux" or shutil.which("fuser") is None:
        return {"supported": False, "busy": None, "reason": "Foreign camera reader check unavailable"}
    try:
        result = subprocess.run([shutil.which("fuser"), device], capture_output=True,
                                text=True, timeout=1.0, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return {"supported": False, "busy": None, "reason": "Foreign camera reader check failed"}
    readers = [int(value) for value in re.findall(r"\b\d+\b", result.stdout)]
    if result.returncode == 0:
        return {"supported": True, "busy": bool(readers), "reason": "Foreign camera reader present" if readers else None}
    if result.returncode == 1 and not result.stderr.strip() and not result.stdout.strip():
        return {"supported": True, "busy": False, "reason": None}
    return {"supported": False, "busy": None, "reason": "Foreign camera ownership could not be checked"}


class CameraLease:
    def __init__(self, device, owner, fd, external_check):
        self.device, self.owner, self.fd = device, owner, fd
        self.external_check = external_check
        self.released = False

    def release(self):
        with _LOCK:
            if self.released:
                return
            self.released = True
            if _OWNERS.get(self.device) is self:
                del _OWNERS[self.device]
            if self.fd is not None:
                import fcntl
                fcntl.flock(self.fd, fcntl.LOCK_UN)
                os.close(self.fd)
                self.fd = None


def acquire_camera(device, owner, *, require_foreign_check=False, checker=None, lock_root=None):
    """Reserve a selected canonical device before creating its reader.

    Calibration live sources require the foreign-reader check. Runtime callers
    still use cooperative locks when that OS facility is unavailable.
    """
    if not isinstance(device, str) or not device.startswith("/dev/"):
        raise ValueError("A canonical selected physical device is required")
    device = str(Path(device).resolve())
    with _LOCK:
        if device in _OWNERS:
            raise CameraBusyError("Camera is already owned by this process")
        fd = None
        try:
            if os.name == "posix":
                import fcntl
                directory = Path(lock_root or (Path(tempfile.gettempdir()) / f"custom-vision-camera-leases-{os.getuid()}"))
                directory.mkdir(mode=0o700, parents=True, exist_ok=True)
                if directory.is_symlink() or directory.stat().st_uid != os.getuid() or directory.stat().st_mode & 0o077:
                    raise CameraBusyError("Camera lease directory has unsafe ownership or permissions")
                path = directory / (hashlib.sha256(device.encode()).hexdigest() + ".lock")
                fd = os.open(path, os.O_CREAT | os.O_RDWR | getattr(os, "O_NOFOLLOW", 0), 0o600)
                if os.fstat(fd).st_uid != os.getuid() or os.fstat(fd).st_mode & 0o077:
                    raise CameraBusyError("Camera lease file has unsafe ownership or permissions")
                try:
                    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError as exc:
                    raise CameraBusyError("Camera is reserved by another cooperating process") from exc
            elif require_foreign_check:
                raise CameraBusyError("Cross-process camera ownership is unsupported on this platform")
            checked = (checker or foreign_camera_readers)(device)
            if checked.get("busy") or (require_foreign_check and not checked.get("supported")):
                raise CameraBusyError(checked.get("reason") or "Camera ownership unavailable")
            lease = CameraLease(device, owner, fd, checked)
            _OWNERS[device] = lease
            return lease
        except Exception:
            if fd is not None:
                os.close(fd)
            raise


class LeasedCapture:
    """Release the lease only after the underlying reader releases its device."""
    def __init__(self, capture, lease):
        self.capture, self.lease = capture, lease

    def __getattr__(self, name):
        return getattr(self.capture, name)

    def release(self, *args, **kwargs):
        result = self.capture.release(*args, **kwargs)
        if result is not False:
            self.lease.release()
        return result
