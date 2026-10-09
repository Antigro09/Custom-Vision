"""Opaque revisions of the public geometry inputs actually used by a pipeline.

``public-geometry-v1`` hashes UTF-8 JSON (sorted object keys, compact separators,
no NaN) in an envelope containing ``format``, ``kind`` and ``parameters``. Input
normalizers supply numeric types/defaults; negative zero becomes positive zero.
Field tags are ordered by ID and equivalent quaternion signs are canonicalized.
The SHA-256 digest is prefixed with ``sha256:``. No filenames, arbitrary metadata,
credentials, source addresses, or physical verification acknowledgements enter
the hash. A missing geometry input has a null revision, never a guessed value.

Changing intrinsics, mount or field coordinates requires dropping pending old
observations. The runtime applies configuration changes by invalidating the old
runtime and starting a new boot_id, including source/camera, timing correction,
target-plane/anchor/covariance and POI changes not covered by these three hashes.
Revisions identify configuration, not calibration accuracy or a measured mount.
"""

from __future__ import annotations

import hashlib
import json


def _canonical_numbers(value):
    if isinstance(value, float) and value == 0:
        return 0.0
    if isinstance(value, dict):
        return {key: _canonical_numbers(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_canonical_numbers(item) for item in value]
    return value


def _revision(kind, parameters):
    envelope = {"format": "public-geometry-v1", "kind": kind,
                "parameters": _canonical_numbers(parameters)}
    encoded = json.dumps(envelope, sort_keys=True, separators=(",", ":"),
                         ensure_ascii=True, allow_nan=False).encode("utf-8")
    return "sha256:" + hashlib.sha256(encoded).hexdigest()


def geometry_revisions(pipeline, config):
    """Return the three nullable revision fields, using strict public allowlists."""
    from .calibration import validate_calibration
    from .localization import validate_field_layout, validate_robot_to_camera

    calibration = pipeline.get("calibration_data")
    if calibration is not None:
        normalized = validate_calibration(calibration)
        calibration = {key: normalized[key] for key in
                       ("width", "height", "distortion_model", "camera_matrix", "dist_coeffs")}
    mount = validate_robot_to_camera(pipeline.get("robot_to_camera"))
    layout = config.get("field_layout_data")
    if layout is not None:
        layout = validate_field_layout(layout)
        layout["tags"].sort(key=lambda tag: tag["ID"])
        for tag in layout["tags"]:
            quaternion = tag["pose"]["rotation"]["quaternion"]
            first = next((quaternion[key] for key in ("W", "X", "Y", "Z")
                          if quaternion[key] != 0), 1.)
            if first < 0:
                quaternion.update({key: -value for key, value in quaternion.items()})
    return {"calibration_revision": None if calibration is None else _revision("calibration", calibration),
            "mount_revision": None if mount is None else _revision("mount", mount),
            "field_layout_revision": None if layout is None else _revision("field_layout", layout)}
