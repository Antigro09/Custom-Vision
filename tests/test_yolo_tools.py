"""Small CPU-only checks for export contracts and benchmark failure cleanup."""
import subprocess
import sys
from pathlib import Path

import pytest

from scripts.export_yolo import validate_export_shapes
from scripts.benchmark_objects import measure_detectors


INPUT = {"images": [1, 3, 640, 640]}


@pytest.mark.parametrize("task,outputs", [
    ("detect", {"boxes": [1, 64, 6]}),
    ("segment", {"proto": [1, 32, 160, 160], "boxes": [1, 64, 38]}),
    ("segment", {"boxes": [1, 300, 7], "proto": [1, 1, 80, 80]}),
])
def test_static_yolo26_export_shapes(task, outputs):
    validate_export_shapes(INPUT, outputs, task, [640, 640])


@pytest.mark.parametrize("task,outputs", [
    ("segment", {"raw": [1, 116, 8400], "proto": [1, 32, 160, 160]}),
    ("segment", {"boxes": [1, 64, 37], "proto": [1, 32, 160, 160]}),
    ("segment", {"boxes": [1, 64, 38]}),
    ("segment", {"boxes": [1, 64, 38], "proto": [2, 32, 160, 160]}),
    ("segment", {"boxes": [1, 64, 38], "proto": [1, 32, 160, 160], "extra": [1, 3]}),
    ("detect", {"boxes": [1, 84, 8400]}),
    ("detect", {"boxes": [1, 64, 6], "extra": [1, 3]}),
    ("detect", {"boxes": [1, 0, 6]}),
    ("detect", {"boxes": [1, "N", 6]}),
    ("detect", {"boxes": [1, True, 6]}),
    ("detect", {"boxes": [1, 301, 6]}),
    ("segment", {"boxes": [1, 64, 135], "proto": [1, 129, 80, 80]}),
    ("segment", {"boxes": [1, 64, 134], "proto": [1, 128, 640, 640]}),
])
def test_incompatible_exports_fail_before_manifest(task, outputs):
    with pytest.raises(ValueError):
        validate_export_shapes(INPUT, outputs, task, [640, 640])


@pytest.mark.parametrize("inputs", [
    {}, {"images": [0, 3, 640, 640]}, {"images": [1, 3, 320, 640]},
    {"images": [1, 3, 640, 640], "extra": [1]},
])
def test_export_requires_exact_static_input(inputs):
    with pytest.raises(ValueError, match="static RGB input"):
        validate_export_shapes(inputs, {"boxes": [1, 64, 6]}, "detect", [640, 640])


def test_benchmark_collects_each_camera_after_warmup():
    class Detector:
        def __init__(self):
            self.calls = 0
            self.last_timings = {"inference_ms": 1.0}
        def process(self, _image):
            self.calls += 1
            return [{"segmentation": {"contour_px": []}}]
    detectors = [Detector(), Detector()]
    results = measure_detectors(detectors, None, frames=3, warmup=2, barrier_timeout_s=1)
    assert [detector.calls for detector in detectors] == [5, 5]
    assert len(results) == 2
    for result in results:
        assert len(result["samples_ms"]) == 3
        assert result["counts"] == [1, 1, 1]
        assert result["masks"] == [1, 1, 1]
        assert result["stages"] == [{"inference_ms": 1.0}] * 3


@pytest.mark.parametrize("failing_index", [0, 1])
def test_benchmark_warmup_failure_releases_peer_and_preserves_error(failing_index):
    # A subprocess gives the regression a hard deadline: without barrier abort,
    # executor shutdown would otherwise hang pytest itself for the 60s timeout.
    code = '''
import threading
from scripts.benchmark_objects import measure_detectors
ready = threading.Event()
class Good:
    last_timings = {}
    def process(self, _image):
        ready.set()
        return []
class Bad:
    def process(self, _image):
        assert ready.wait(2)
        raise ValueError("original warmup failure")
detectors = [Good(), Good()]
detectors[FAIL_INDEX] = Bad()
try:
    measure_detectors(detectors, None, frames=1, warmup=1)
except ValueError as exc:
    assert str(exc) == "original warmup failure"
else:
    raise AssertionError("The failing detector was not reported")
'''.replace("FAIL_INDEX", str(failing_index))
    completed = subprocess.run([sys.executable, "-c", code],
                               cwd=Path(__file__).resolve().parents[1],
                               capture_output=True, text=True, timeout=10)
    assert completed.returncode == 0, completed.stdout + completed.stderr
