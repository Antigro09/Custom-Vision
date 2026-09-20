#!/usr/bin/env python3
"""Repeated rendered-image compute benchmark; no camera or robot latency is measured."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import platform
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
# Import by package name so imports work both as a script and in pytest.
from scripts.demo_apriltags import fixture


def scene(width=1280, height=800, tag_count=4, noise=False):
    rng = np.random.default_rng(1086)
    gray = np.clip(rng.normal(155, 20 if noise else 1, (height, width)), 0, 255).astype(np.uint8)
    # Add distractor edges, then isolated markers; no claim this replaces field scenes.
    for i in range(25):
        x, y = rng.integers([0, 0], [width, height])
        cv2.rectangle(gray, (int(x), int(y)), (min(int(x)+40, width-1), min(int(y)+25,height-1)), int(rng.integers(0,255)), 2)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    ids = [7, 12, 20, 24][:tag_count]
    for i, ident in enumerate(ids):
        size = 112 if i % 2 else 160
        x = 110 + (i % 2) * (width // 2)
        y = 70 + (i // 2) * (height // 2)
        gray[y-20:y+size+20, x-20:x+size+20] = 245
        marker = cv2.aruco.generateImageMarker(dictionary, ident, size)
        gray[y:y+size, x:x+size] = marker
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), ids


def describe(values):
    values = np.asarray(values, dtype=float)
    return {"count": len(values), "p50_ms": float(np.percentile(values, 50)),
            "p95_ms": float(np.percentile(values, 95)), "p99_ms": float(np.percentile(values, 99)),
            "max_ms": float(values.max())}


def build_pipeline(settings, calibration, layout, mount, poi_enabled):
    """Use the runtime factory so CPU/CUDA single/joint solver wiring is identical."""
    from custom_vision.app import make_detector
    cfg = {'name': 'benchmark', 'type': 'apriltag', 'settings': dict(settings),
           'calibration_data': calibration, 'robot_to_camera': mount,
           'poi': {'enabled': poi_enabled, 'calibration_verified': poi_enabled,
                   'max_ambiguity': .3,
                   'targets': [{'name': 'benchmark_aim', 'tag_id': 7, 'offset_m': [0, 0, .1]}]
                   if poi_enabled else []}}
    return make_detector(cfg, {'field_layout_data': layout})


def check_frame(found, ids, *, mode, require_single, localization_result=None, poi_result=None,
                expected_pose_device=None):
    """Reject fast-but-wrong results after every peer leaves its timed interval."""
    problems = []
    actual = [d.get('id') for d in found]
    if len(actual) != len(ids) or set(actual) != set(ids):
        problems.append('detected tag IDs differ from rendered IDs')
    if mode == '3d' and require_single:
        if any(not d.get('pose_valid') for d in found):
            problems.append('single-tag pose invalid')
        if expected_pose_device is not None and any(d.get('pose_valid') and d.get('pose_device', 'cpu') != expected_pose_device for d in found):
            problems.append('single-tag pose ran on a different pose device')
    if localization_result is not None:
        loc = localization_result.get('localization', {})
        if not loc.get('valid') or loc.get('method') != 'multitag_pnp':
            problems.append('joint localization invalid')
        else:
            if expected_pose_device is not None and loc.get('pose_device') != expected_pose_device:
                problems.append('joint localization ran on a different pose device')
            # This fixture's camera is at (1, 2, .6) in field NWU. Rasterization
            # changes detected corners slightly, so this is a gross-error guard.
            position = (loc.get('field_to_camera') or {}).get('translation_m')
            if position is None or not np.isfinite(position).all() or np.linalg.norm(np.asarray(position) - [1., 2., .6]) > .1:
                problems.append('field camera position differs from fixture by over 0.1 m')
    if poi_result is not None:
        target = next((t for t in poi_result.get('targets', []) if t.get('name') == 'benchmark_aim'), {})
        if not poi_result.get('valid') or poi_result.get('selected_name') != 'benchmark_aim' or not target.get('valid'):
            problems.append('POI invalid: ' + str(target.get('invalid_reason', poi_result.get('invalid_reason'))))
        else:
            # Tag 7 at (3, 1.6, 1.0), offset +0.1 m along tag-up, observed by
            # camera (1,2,.6): optical right/down/forward point (.4,-.5,2).
            point = target.get('camera_translation_m')
            angles = [target.get('tx_deg'), target.get('ty_deg')]
            if point is None or not np.isfinite(point).all() or np.linalg.norm(np.asarray(point) - [.4, -.5, 2.]) > .1:
                problems.append('POI camera position differs from fixture by over 0.1 m')
            if any(not isinstance(v, (int, float)) or not np.isfinite(v) for v in angles):
                problems.append('POI bearing is not finite')
    return problems


def measure(backend, mode, decimate, cameras, frames, warmup, preprocess='cpu', noise=False,
            device='cpu', contrast=5, localization=False, pose_device='cpu', poi=False,
            round_index=0, barrier_timeout=120.):
    if frames < 1 or warmup < 0 or cameras not in (1, 2) or not np.isfinite(barrier_timeout) or barrier_timeout <= 0:
        raise ValueError('positive frames/timeout, nonnegative warmup and 1 or 2 cameras required')
    if pose_device not in ('cpu', 'cuda') or backend not in ('native', 'pupil'):
        raise ValueError('unsupported pose device or detector backend')
    if (pose_device == 'cuda' or device == 'cuda' or preprocess == 'cuda') and backend != 'native':
        raise ValueError('CUDA detection, preprocessing and pose require native backend')
    if pose_device == 'cuda' and mode != '3d':
        raise ValueError('CUDA pose requires 3d mode')
    if (localization or poi) and (noise or mode != '3d'):
        raise ValueError('localization and POI use their own calibrated 3d scene')
    image, ids = scene(noise=noise)
    calibration = dict(width=1280, height=800, camera_matrix=[[950,0,640],[0,950,400],[0,0,1]], dist_coeffs=[0]*5)
    settings = dict(backend=backend, mode=mode, quad_decimate=decimate, threads=2, preprocess=preprocess,
                    min_decision_margin=30, detector_device=device, pose_device=pose_device,
                    min_white_black_diff=contrast)
    layout = None
    mount = {'translation_m': [.25, .1, .45], 'rotation_rpy_deg': [0, 0, 0]}
    if localization or poi:
        image, calibration, layout = fixture()
        ids = [7, 12, 20]
        settings.update(max_ambiguity=.3)
    # Runtime factory owns the optimization and native solver callbacks. The
    # requested mapped/POI modes determine which correctness outputs are required.
    skip_single_when_multi = localization and not poi
    require_single = not skip_single_when_multi
    start_barrier = threading.Barrier(cameras)
    finish_barrier = threading.Barrier(cameras)
    cancel = threading.Event()

    def worker(index):
        detector = None
        try:
            # Construct and run each detector on its own worker, like the runtime.
            detector = build_pipeline(settings, calibration, layout if localization else None,
                                      mount if localization or poi else None, poi)
            localizer = detector.localization if localization else None
            tracker = detector.poi if poi else None
            stages = {key: [] for key in ('detector_ms', 'poi_ms', 'localization_ms')}
            samples, measured_records, validation_samples = [], [], []
            correct = 0
            failures = []

            def call():
                before = time.perf_counter_ns()
                found = detector.process(image)
                detected = time.perf_counter_ns()
                # Snapshot detector timings before localization's optional fallback
                # solves can change the detector's last_timings property.
                native = dict(getattr(detector, 'last_timings', {}))
                timings_read = time.perf_counter_ns()
                poi_result = tracker.process(found, image.shape, before / 1e9) if tracker else None
                aimed = time.perf_counter_ns()
                localized = localizer.enrich(found, image.shape) if localizer else None
                finished = time.perf_counter_ns()
                timing = dict(detector_ms=(detected-before)/1e6, native_timing_read_ms=(timings_read-detected)/1e6,
                              poi_ms=(aimed-timings_read)/1e6, localization_ms=(finished-aimed)/1e6)
                if not tracker: timing['poi_ms'] = 0.
                if not localizer: timing['localization_ms'] = 0.
                # Retain fresh per-call result dictionaries for validation after
                # every peer finishes timing. No NumPy correctness checks or
                # summary calculations run alongside a peer's measured work.
                return (finished-before)/1e6, timing, native, found, localized, poi_result

            startup_ms = None
            for _ in range(warmup):
                if cancel.is_set(): raise RuntimeError('Peer benchmark worker failed')
                elapsed, _, _, found, localized, poi_result = call()
                if startup_ms is None: startup_ms = elapsed
                problems = check_frame(found, ids, mode=mode, require_single=require_single,
                                       localization_result=localized, poi_result=poi_result,
                                       expected_pose_device=pose_device)
                if problems: raise RuntimeError('Warmup correctness failed: ' + '; '.join(problems))
            start_barrier.wait(timeout=barrier_timeout)
            start = time.perf_counter()
            for i in range(frames):
                if cancel.is_set(): raise RuntimeError('Peer benchmark worker failed')
                record = call()
                samples.append(record[0])
                measured_records.append(record)
            elapsed = time.perf_counter() - start
            # Both correctness checks (CPU/GIL) and CUDA teardown can interfere
            # with peers. Every worker must leave its measured interval first.
            finish_barrier.wait(timeout=barrier_timeout)
            for i, (_, timing, native, found, localized, poi_result) in enumerate(measured_records):
                before_check = time.perf_counter_ns()
                problems = check_frame(found, ids, mode=mode, require_single=require_single,
                                       localization_result=localized, poi_result=poi_result,
                                       expected_pose_device=pose_device)
                validation_samples.append((time.perf_counter_ns()-before_check)/1e6)
                correct += not problems
                if problems and len(failures) < 5: failures.append({'frame': i, 'problems': problems})
                for key, val in native.items():
                    if key.endswith('_ms') and isinstance(val, (float, int)) and np.isfinite(val) and val >= 0:
                        timing['native_' + key] = float(val)
                joint_timings = (localized or {}).get('localization', {}).get('gpu_timings') or {}
                for key, val in joint_timings.items():
                    if key.endswith('_ms') and isinstance(val, (float, int)) and np.isfinite(val) and val >= 0:
                        timing['joint_cuda_' + key] = float(val)
                for key, val in timing.items(): stages.setdefault(key, []).append(val)
            return dict(worker=index, samples_ms=samples, stages_ms=stages, elapsed_s=elapsed,
                        validation_samples_ms=validation_samples, validation_excluded_from_latency_and_fps=True,
                        correct_frames=correct, failure_examples=failures, first_warmup_ms=startup_ms)
        except BaseException:
            cancel.set()
            start_barrier.abort()
            finish_barrier.abort()
            raise
        finally:
            if detector is not None and hasattr(detector, 'close'):
                detector.close()

    with ThreadPoolExecutor(max_workers=cameras) as pool:
        futures = [pool.submit(worker, i) for i in range(cameras)]
        results = [f.result() for f in futures]
    timings = np.array([x for r in results for x in r['samples_ms']])
    stage_names = sorted(set().union(*(r['stages_ms'] for r in results)))
    pooled_stages = {name: describe([v for r in results for v in r['stages_ms'].get(name, [])])
                     for name in stage_names}
    summary = describe(timings)
    return dict(scene='mapped_three_tags' if localization or poi else ('pixel_noise_stress' if noise else 'structured'),
                backend=backend, mode=mode, preprocess=preprocess, detector_device=device, pose_device=pose_device,
                min_white_black_diff=contrast, includes_localization=localization, includes_poi=poi,
                skip_single_when_multi=skip_single_when_multi,
                quad_decimate=decimate, threads_per_camera=2, cameras=cameras, round=round_index,
                frames_per_camera=frames, warmup_frames=warmup, image_size=[image.shape[1], image.shape[0]], tag_ids=ids,
                fixture_sha256=hashlib.sha256(image.tobytes()).hexdigest(), calibration=calibration,
                effective_pipeline_settings=settings, robot_to_camera=mount if localization or poi else None,
                poi_fixture={'tag_id': 7, 'offset_m': [0, 0, .1], 'max_ambiguity': .3,
                             'max_reprojection_error_px': 3., 'calibration_verified_for_synthetic_fixture_only': True} if poi else None,
                **{k: round(v, 3) for k, v in summary.items() if k != 'count'},
                per_camera_fps=[round(frames/r['elapsed_s'], 2) for r in results],
                first_warmup_ms=[round(r['first_warmup_ms'], 3) if r['first_warmup_ms'] is not None else None for r in results],
                correct_frames=sum(r['correct_frames'] for r in results), total_frames=cameras*frames,
                over_24_ms=int(sum(timings > 24)), stages_ms=pooled_stages,
                validation_ms=describe([v for r in results for v in r['validation_samples_ms']]),
                validation_excluded_from_latency_and_fps=True, workers_results=results)


def metadata():
    paths = ['scripts/benchmark_apriltags.py', 'scripts/demo_apriltags.py', 'native/apriltags.cpp',
             'native/cuda_pose.cu', 'native/cuda_pose.hpp', 'native/cuda_multitag.cu',
             'native/cuda_multitag.hpp', 'native/cuda_multitag_math.hpp', 'native/pnp_math.hpp',
             'custom_vision/native_apriltags.py', 'custom_vision/apriltags.py',
             'custom_vision/localization.py', 'custom_vision/poi.py', 'custom_vision/app.py']
    sources = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths if (ROOT/p).exists()}
    spec = importlib.util.find_spec('custom_vision._native')
    binary = Path(spec.origin) if spec and spec.origin else None
    from custom_vision.native_apriltags import native_capabilities
    try:
        revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
        dirty = bool(subprocess.check_output(['git', 'status', '--porcelain', '--untracked-files=no'], cwd=ROOT, text=True))
    except (OSError, subprocess.CalledProcessError):
        revision, dirty = None, None
    return dict(platform=platform.platform(), python=platform.python_version(), opencv=cv2.__version__, numpy=np.__version__,
                source_revision=revision, tracked_tree_dirty=dirty, source_sha256=sources,
                native_binary_path=str(binary) if binary else None,
                native_binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest() if binary else None,
                capabilities=native_capabilities())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['pupil','native'], default='native')
    parser.add_argument('--mode', choices=['2d','3d'], default='3d')
    parser.add_argument('--decimate', type=float, default=2)
    parser.add_argument('--cameras', type=int, choices=[1,2], default=2)
    parser.add_argument('--frames', type=int, default=100)
    parser.add_argument('--warmup', type=int, default=10)
    parser.add_argument('--preprocess', choices=['cpu','cuda'], default='cpu')
    parser.add_argument('--detector-device', choices=['cpu','cuda'], default='cpu')
    parser.add_argument('--pose-device', choices=['cpu','cuda'], default='cpu')
    parser.add_argument('--min-white-black-diff', type=int, default=5)
    parser.add_argument('--localization', action='store_true', help='Include joint field localization on a mapped three-tag scene')
    parser.add_argument('--poi', action='store_true', help='Include validated tag-relative POI before localization; single-tag poses remain enabled')
    parser.add_argument('--noise-stress', action='store_true')
    parser.add_argument('--rounds', type=int, default=1, help='Repeat this exact configuration; use paired separate calls to compare devices')
    parser.add_argument('--barrier-timeout', type=float, default=120., help='Worker start/finish coordination timeout in seconds')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    if args.frames < 1 or args.warmup < 0 or args.rounds < 1 or not np.isfinite(args.barrier_timeout) or args.barrier_timeout <= 0:
        parser.error('frames/rounds/timeout must be positive and warmup nonnegative')
    if args.backend != 'native' and (args.detector_device != 'cpu' or args.pose_device != 'cpu' or args.preprocess != 'cpu'):
        parser.error('CUDA detection, preprocessing and pose require native')
    if args.pose_device == 'cuda' and args.mode != '3d': parser.error('CUDA pose requires 3d mode')
    if (args.localization or args.poi) and (args.noise_stress or args.mode != '3d'):
        parser.error('localization and POI use their own calibrated 3d scene')
    if args.output and args.output.exists(): parser.error('Output already exists; use a new filename')
    cv2.setNumThreads(1)
    before = metadata()
    caps = before['capabilities']
    if args.backend == 'native' and not caps.get('available'): parser.error('Build the native extension first')
    if args.pose_device == 'cuda' and not (caps.get('cuda_pose_compiled') and caps.get('cuda_pose_devices')):
        parser.error('CUDA pose requires a compiled native CUDA solver and real GPU')
    results = [measure(args.backend, args.mode, args.decimate, args.cameras, args.frames, args.warmup,
                       args.preprocess, args.noise_stress, args.detector_device, args.min_white_black_diff,
                       args.localization, args.pose_device, args.poi, i, args.barrier_timeout) for i in range(args.rounds)]
    after = metadata()
    if before['source_sha256'] != after['source_sha256'] or before['native_binary_sha256'] != after['native_binary_sha256']:
        raise RuntimeError('Source or native binary changed during benchmark; discard results')
    excluded = ['exposure', 'USB transport', 'MJPEG decode', 'capture queue', 'preview', 'NT serialization/transport', 'robot receipt']
    if not args.localization: excluded.append('field localization')
    if not args.poi: excluded.append('POI tracking')
    report = dict(synthetic_compute_only=True, excludes=excluded, recorded_utc=datetime.now(timezone.utc).isoformat(),
                  method='Repeated rendered pixels; warmed independent camera workers; wall latency includes detector, native timing reads, POI and localization. Per-frame results are retained and correctness checks run only after every worker finishes timing. Reported per-camera FPS and latency exclude validation and teardown, which are isolated by start/finish barriers.',
                  settings=vars(args) | {'output': str(args.output) if args.output else None}, metadata=before,
                  all_frames_correct=all(r['correct_frames'] == r['total_frames'] for r in results),
                  result=results[0], results=results)
    data = json.dumps(report, indent=2, allow_nan=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x') as output: output.write(data + '\n')
    print(data)
    return 0 if report['all_frames_correct'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
