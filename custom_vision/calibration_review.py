"""Software review gates; a review never certifies a physical camera or mount.

Public status values contain no file references, camera identifiers or labels.
Synthetic/recorded evidence may be reviewed and exported without making it
eligible for physical activation. Exact camera/mode matching is a separate gate.
"""
from __future__ import annotations

import copy
import math

from .calibration import validate_calibration
from .revisions import geometry_revisions


ACCEPTED_QUALITY = frozenset(("heuristics_passed_not_hardware_validated",
                              "baseline_not_hardware_validated"))


def calibration_revision(data):
    return geometry_revisions({"calibration_data": data}, {})["calibration_revision"]


def _quality(data, metadata):
    if metadata.get("stale") is True:
        return data.get("quality_status", metadata.get("quality_status")), "Candidate capture evidence is stale; solve and review the current session"
    report = metadata.get("report", {})
    if not isinstance(report, dict):
        return None, "Candidate report must be an object"
    statuses = [item for item in (data.get("quality_status"), metadata.get("quality_status"),
                                  report.get("status")) if item is not None]
    if not statuses or any(not isinstance(item, str) for item in statuses):
        return None, "Candidate quality status is unknown"
    if len(set(statuses)) != 1:
        return None, "Candidate quality statuses disagree"
    status = statuses[0]
    if status not in ACCEPTED_QUALITY:
        return status, "Candidate quality requires investigation before review or activation"
    if report.get("issues"):
        return status, "Candidate report contains unresolved quality issues"
    return status, None


def _synthetic(metadata):
    return (metadata.get("synthetic") is True
            or metadata.get("source_kind") == "synthetic"
            or metadata.get("input_kind") == "synthetic"
            or (isinstance(metadata.get("source"), dict)
                and metadata["source"].get("kind") == "synthetic"))


def _identity(value):
    if not isinstance(value, str) or not value.strip() or value.strip().lower() == "unknown":
        raise ValueError("physical camera identity provenance is unknown")
    return value


def _crop(value):
    if isinstance(value, list) and len(value) == 4:
        value = dict(zip(("x", "y", "width", "height"), value))
    if not isinstance(value, dict) or set(value) != {"x", "y", "width", "height"}:
        raise ValueError("crop provenance is unknown or invalid")
    for key, minimum in (("x", 0), ("y", 0), ("width", 1), ("height", 1)):
        if type(value[key]) is not int or value[key] < minimum:
            raise ValueError("crop provenance is unknown or invalid")
    return value


def _binning(value):
    if (not isinstance(value, list) or len(value) != 2
            or any(type(item) is not int or item < 1 for item in value)):
        raise ValueError("binning provenance is unknown or invalid")
    return value


def _focus(value):
    if not isinstance(value, dict) or value.get("kind") not in ("fixed", "manual"):
        raise ValueError("focus kind provenance is unknown or invalid")
    if value.get("locked") is not True:
        raise ValueError("focus locking is unverified")
    setting = value.get("value")
    if value["kind"] == "manual" and (isinstance(setting, bool)
            or not isinstance(setting, (int, float)) or not math.isfinite(setting)):
        raise ValueError("focus value provenance is unknown or invalid")
    if setting is not None and (isinstance(setting, bool)
            or not isinstance(setting, (int, float)) or not math.isfinite(setting)):
        raise ValueError("focus value provenance is unknown or invalid")
    # Fixed optics have no programmable focus setting. Null is meaningful here,
    # unlike an unknown focus kind, and is checked against the exact camera.
    return {"kind": value["kind"], "value": setting, "locked": True}


def candidate_provenance(metadata):
    """Return exact declared camera/mode provenance, or fail closed."""
    if not isinstance(metadata, dict):
        raise ValueError("Candidate metadata must be an object")
    if _synthetic(metadata):
        raise ValueError("Synthetic calibration evidence cannot activate a physical camera")
    camera, mode = metadata.get("camera"), metadata.get("mode")
    if not isinstance(camera, dict) or not isinstance(mode, dict):
        raise ValueError("Candidate camera and mode metadata must be objects")
    for field in ("width", "height"):
        if type(mode.get(field)) is not int or mode[field] <= 0:
            raise ValueError(f"Candidate mode metadata {field} is unknown or invalid")
    return {"physical_id": _identity(camera.get("physical_id")),
            "width": mode["width"], "height": mode["height"],
            "crop": _crop(mode.get("crop")), "binning": _binning(mode.get("binning")),
            "focus": _focus(mode.get("focus"))}


def candidate_review_status(data, metadata=None):
    """Describe software review independently of physical activation eligibility."""
    data = validate_calibration(data)
    metadata = {} if metadata is None else metadata
    if not isinstance(metadata, dict):
        raise ValueError("Candidate metadata must be an object")
    quality, reason = _quality(data, metadata)
    review = metadata.get("review") or {}
    reviewed = (isinstance(review, dict) and review.get("confirmed") is True
                and review.get("quality_status") == quality
                and review.get("calibration_revision") == calibration_revision(data)
                and reason is None)
    try:
        candidate_provenance(metadata)
        provenance = True
    except ValueError:
        provenance = False
    return {"quality_status": quality, "review_allowed": reason is None,
            "reviewed": reviewed, "review_reason": reason,
            "activation_provenance_available": provenance,
            "physical_verification": False,
            "diagnostic_baseline": quality == "baseline_not_hardware_validated"}


def review_candidate(data, metadata, *, confirmed):
    """Create a review marker bound to geometry and quality after explicit review."""
    if confirmed is not True:
        raise ValueError("Candidate review requires explicit confirmation")
    status = candidate_review_status(data, metadata)
    if not status["review_allowed"]:
        raise ValueError(status["review_reason"])
    checked = copy.deepcopy(metadata)
    checked["review"] = {"confirmed": True, "quality_status": status["quality_status"],
                         "calibration_revision": calibration_revision(data)}
    return checked


def require_candidate_review(data, metadata):
    """Require accepted software quality and a current review before activation."""
    status = candidate_review_status(data, metadata)
    if not status["review_allowed"]:
        raise ValueError(status["review_reason"])
    if not status["reviewed"]:
        raise ValueError("Candidate requires explicit review of its current calibration and quality")
    candidate_provenance(metadata)
    return status
