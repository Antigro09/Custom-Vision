"""Small Python boundary around the GIL-free C++ AprilTag/PnP frame path.

The default detector is CPU AprilTag. An optional NVIDIA cuAprilTags build adds
full-frame CUDA detection, followed by exact AprilTag CPU quality verification
and calibrated PnP. ``preprocess`` independently selects grayscale conversion.
CPU preprocessing is usually preferable for monochrome USB cameras because gray
input is borrowed without a copy; GPU detection uploads/expands it separately.
"""

from __future__ import annotations

from .calibration import validate_calibration


def native_capabilities() -> dict:
    try:
        from . import _native
    except ImportError as exc:
        return {"available": False, "error": str(exc), "apriltag_device": "cpu",
                "cuda_preprocess": False}
    return {"available": True, **_native.capabilities()}


class NativeAprilTagPipeline:
    """One native detector per camera, with independent buffers/worker threads.

    Inputs are uint8 gray or BGR images with contiguous pixels and positive row
    stride (padded image rows/ROIs are supported). Concurrent callers on a single
    detector serialize safely; use separate detectors for two camera pipelines.
    """

    def __init__(self, config: dict, calibration: dict | None = None):
        if not isinstance(config, dict):
            raise ValueError("apriltags configuration must be an object")
        self.calibration = validate_calibration(calibration) if calibration is not None else None
        self.mode = config.get("mode", "3d")
        self.detector_device = config.get("detector_device", "cpu")
        self.pose_device = config.get("pose_device", "cpu")
        self.tag_size_m = float(config.get("tag_size_m", 0.1651))
        try:
            from . import _native
        except ImportError as exc:
            raise RuntimeError("Native AprilTag core is unavailable; run scripts/build_native.sh "
                               "or explicitly select backend: pupil") from exc
        self.detector = _native.Detector(config, self.calibration)

    def process(self, frame) -> list[dict]:
        return self._get_detector().process(frame)

    def _estimate_pose(self, corners) -> dict:
        """Pose-only entry point for synthetic geometric verification."""
        return self._get_detector().estimate_pose(corners)

    def estimate_poses(self, corners):
        """Pose-only (N,4,2) batch, including transfers and result conversion."""
        return self._get_detector().estimate_poses(corners)

    def estimate_multitag(self, corners, field_corners):
        """Joint CUDA field-to-optical-camera solve; no CPU seed or fallback."""
        return self._get_detector().estimate_multitag(corners,field_corners)

    def _get_detector(self):
        detector = self.detector
        if detector is None:
            raise RuntimeError("Native AprilTag pipeline is closed")
        return detector

    def close(self) -> None:
        """Release workers/GPU allocations after the runtime joins camera workers.

        Dropping the final native reference immediately destroys its RAII-owned
        detector, buffers, and stream. Repeated calls are safe.
        """
        self.detector = None

    @property
    def last_timings(self) -> dict:
        return self._get_detector().last_timings

    @property
    def last_profile(self) -> dict:
        """On-demand AprilTag phase timings and number of candidate quads."""
        return self._get_detector().last_profile
