#!/usr/bin/env python3
"""Compare real CPU geometry/serialization against a supplied, trusted checkout.

This is a synthetic host microbenchmark, NOT camera, GPU, NT-network, or robot
latency. Run on the target Jetson for target-host numbers. Baseline Python code
is imported and executed, so supply only a checkout you trust.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import platform
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from custom_vision.object_geometry import ObjectGeometry
from custom_vision.publisher import wire_packet

CALIBRATION = {"width": 1280, "height": 800,
               "camera_matrix": [[800., 0., 640.], [0., 800., 400.], [0., 0., 1.]],
               "dist_coeffs": [0.] * 5}
MOUNT = {"translation_m": [0., 0., 1.], "rotation_rpy_deg": [0., 45., 0.]}
SHAPE = (800, 1280, 3)


def load_baseline(root: Path, module: str):
    path = root / "custom_vision" / f"{module}.py"
    if not path.is_file():
        raise ValueError(f"Missing baseline source: {path}")
    name = f"custom_vision._host_benchmark_baseline_{module}"
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    sys.modules[name] = value
    spec.loader.exec_module(value)
    return value


def fixture(count: int):
    return [{"class_id": 0, "label": "synthetic_piece", "confidence": .9,
             "bbox_xyxy": [400. + 55. * (i % 8), 290. + 55. * (i // 8),
                           424. + 55. * (i % 8), 314. + 55. * (i // 8)]}
            for i in range(count)]


def geometry(cls):
    return cls({"target_height_m": 0.}, CALIBRATION, MOUNT)


def payload_for(count: int):
    result = geometry(ObjectGeometry).enrich(fixture(count), SHAPE, 100.)
    assert len(result["objects"]["targets"]) == count
    return dict(schema_version=2, pipeline="synthetic", connected=True, frame_id=1,
                capture_monotonic_us=100000000, latency_ms=1.23456789, **result)


def verify_stream(reference_class, reference_wire):
    """Compare outputs through reordered/missed/expired/reset/class-changing data."""
    rng = np.random.default_rng(1086)
    pipes = (geometry(reference_class), geometry(ObjectGeometry))
    stamp = 100.
    for step in range(300):
        count = int(rng.integers(0, 33))
        detections = fixture(count)
        for detection in detections:
            detection["class_id"] = int(rng.integers(0, 3))
            detection["label"] = f"class_{detection['class_id']}"
            dx, dy = rng.uniform(-5, 5, size=2)
            box = detection["bbox_xyxy"]
            detection["bbox_xyxy"] = [box[0] + dx, box[1] + dy, box[2] + dx, box[3] + dy]
        rng.shuffle(detections)
        stamp += .2 if step % 13 == 0 else .01
        if step % 37 == 0:
            for pipe in pipes:
                pipe.reset()
        outputs = [pipe.enrich(copy.deepcopy(detections), SHAPE, stamp) for pipe in pipes]
        assert outputs[0] == outputs[1], f"Geometry/tracking mismatch at frame {step}"
        assert reference_wire(outputs[0]) == wire_packet(outputs[1]), f"Wire mismatch at frame {step}"
    return 300


def measure_pair(before, after, *, samples, warmup, batch):
    # Alternate order each batch to reduce drift bias; warm both implementations.
    for _ in range(warmup):
        before()
        after()
    timings = [[], []]
    functions = (before, after)
    for sample in range(samples):
        for index in ((0, 1) if sample % 2 == 0 else (1, 0)):
            start = time.perf_counter_ns()
            for _ in range(batch):
                functions[index]()
            timings[index].append((time.perf_counter_ns() - start) / batch / 1e6)
    summaries = [{"median_ms": float(np.median(values)),
                  "p95_batch_mean_ms": float(np.percentile(values, 95)),
                  "samples_batch_mean_ms": values} for values in timings]
    old, new = [summary["median_ms"] for summary in summaries]
    return {"before": summaries[0], "after": summaries[1], "speedup": old / new,
            "median_time_reduction_percent": 100. * (1. - new / old)}


def identity(root):
    paths = ("custom_vision/publisher.py", "custom_vision/object_geometry.py")
    result = {"sha256": {path: hashlib.sha256((root / path).read_bytes()).hexdigest() for path in paths}}
    try:
        result["commit"] = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"],
                                                    stderr=subprocess.DEVNULL, text=True).strip()
        result["dirty"] = bool(subprocess.check_output(["git", "-C", str(root), "status", "--porcelain"], text=True))
    except (OSError, subprocess.CalledProcessError):
        result["commit"] = None
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-repo", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--samples", type=int, default=61)
    parser.add_argument("--warmup", type=int, default=20)
    parser.add_argument("--batch", type=int, default=10)
    args = parser.parse_args(argv)
    if args.samples < 5 or args.warmup < 1 or args.batch < 1:
        parser.error("Use samples >= 5, warmup >= 1 and batch >= 1")
    baseline_root = args.baseline_repo.resolve()
    if baseline_root == ROOT:
        parser.error("Baseline and current source directories must differ")
    reference_class = load_baseline(baseline_root, "object_geometry").ObjectGeometry
    reference_wire = load_baseline(baseline_root, "publisher").wire_packet
    cv2.setNumThreads(1)
    verified = verify_stream(reference_class, reference_wire)
    result = {"synthetic_only": True, "includes_gpu_inference": False,
              "includes_camera_transport_or_nt_network": False,
              "environment": {"platform": platform.platform(), "machine": platform.machine(),
                              "python": platform.python_version(), "numpy": np.__version__,
                              "opencv": cv2.__version__, "opencv_threads": cv2.getNumThreads()},
              "baseline": identity(baseline_root), "current": identity(ROOT),
              "equivalent_stream_frames": verified,
              "method": {"samples": args.samples, "warmup": args.warmup, "batch": args.batch,
                         "order": "alternating paired batches", "tail_metric": "p95 of batch means, NOT per-frame p95"},
              "measurements": []}
    for count in (0, 1, 5, 16, 32):
        data = payload_for(count)
        before_json = json.dumps(reference_wire(data), allow_nan=False, separators=(",", ":"))
        assert before_json == json.dumps(wire_packet(data), allow_nan=False, separators=(",", ":"))
        funcs = {"wire_packet": (lambda: reference_wire(data), lambda: wire_packet(data)),
                 "wire_packet_plus_json": (
                     lambda: json.dumps(reference_wire(data), allow_nan=False, separators=(",", ":")),
                     lambda: json.dumps(wire_packet(data), allow_nan=False, separators=(",", ":")))}
        pipes = [geometry(reference_class), geometry(ObjectGeometry)]
        detections = fixture(count)
        stamps = [100., 100.]

        def enrich(index):
            stamps[index] += .01
            return pipes[index].enrich(detections, SHAPE, stamps[index])

        funcs["geometry_enrich"] = (lambda: enrich(0), lambda: enrich(1))
        for stage, (before, after) in funcs.items():
            stats = measure_pair(before, after, samples=args.samples, warmup=args.warmup, batch=args.batch)
            stats.update(stage=stage, detections=count)
            result["measurements"].append(stats)
            print(f"{stage:24s} n={count:2d}: {stats['before']['median_ms']:.4f} -> "
                  f"{stats['after']['median_ms']:.4f} ms ({stats['speedup']:.2f}x)", file=sys.stderr)
    text = json.dumps(result, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)
    else:
        print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
