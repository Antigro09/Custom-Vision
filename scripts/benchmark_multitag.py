#!/usr/bin/env python3
"""Real CPU/CUDA joint localization on synthetic known-world corners, not camera latency."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
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
from custom_vision.app import make_detector
from custom_vision.native_apriltags import native_capabilities

TAG_SIZE = .1651
CALIBRATION = {'width': 1280, 'height': 800,
               'camera_matrix': [[870., 0., 640.], [0., 865., 400.], [0., 0., 1.]],
               'dist_coeffs': [-.13, .04, .001, -.002, .003, .005, -.002, 0.]}


def transform(translation, rpy):
    """Independent field NWU rigid transform, without production pose helpers."""
    roll, pitch, yaw = np.deg2rad(rpy)
    rx = np.array([[1., 0., 0.], [0., math.cos(roll), -math.sin(roll)], [0., math.sin(roll), math.cos(roll)]])
    ry = np.array([[math.cos(pitch), 0., math.sin(pitch)], [0., 1., 0.], [-math.sin(pitch), 0., math.cos(pitch)]])
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0.], [math.sin(yaw), math.cos(yaw), 0.], [0., 0., 1.]])
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = rz @ ry @ rx, translation
    return out


def quaternion(rotation):
    vector = cv2.Rodrigues(rotation)[0].reshape(3)
    angle = float(np.linalg.norm(vector))
    return np.r_[math.cos(angle/2), vector * math.sin(angle/2)/angle] if angle else np.array([1., 0., 0., 0.])


def pose_matrix(pose):
    q = np.asarray(pose['rotation_quaternion_wxyz'], dtype=float)
    t = np.asarray(pose['translation_m'], dtype=float)
    if q.shape != (4,) or t.shape != (3,) or not np.isfinite(q).all() or not np.isfinite(t).all() or abs(np.linalg.norm(q)-1.) > 1e-6:
        raise ValueError('Malformed published pose')
    w, x, y, z = q
    axis = q[1:]
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    out = np.eye(4)
    out[:3, :3] = (w*w-axis@axis)*np.eye(3) + 2*np.outer(axis, axis) + 2*w*cross
    out[:3, 3] = t
    return out


def project(world, camera):
    relative = (world-camera[:3, 3]) @ camera[:3, :3]
    optical = np.stack((-relative[..., 1], -relative[..., 2], relative[..., 0]), axis=-1)
    if np.min(optical[..., 2]) <= 0: raise ValueError('Fixture contains corners behind camera')
    pixels = cv2.projectPoints(optical.reshape(-1, 3), np.zeros(3), np.zeros(3),
                               np.array(CALIBRATION['camera_matrix']),
                               np.array(CALIBRATION['dist_coeffs']))[0].reshape(world.shape[:-1]+(2,))
    return pixels


def make_scene(count, layout_kind, noise_px=0., views=12):
    """Moderately oblique views of a physical-scale wall arrangement, computed before timing."""
    if count not in (2, 4, 8, 16) or layout_kind not in ('planar', 'nonplanar'):
        raise ValueError('Expected 2/4/8/16 tags and planar/nonplanar layout')
    columns = 2 if count <= 4 else 4
    rows = count // columns
    ys = np.linspace(2.75, 4.15, columns)
    zs = np.linspace(.6, 1.5, rows) if rows > 1 else np.array([1.05])
    h = TAG_SIZE/2
    # Decoded printed TR/TL/BL/BR in TAG WPILib axes; independent of runtime constants.
    corners = np.array([[0., h, h, 1.], [0., -h, h, 1.], [0., -h, -h, 1.], [0., h, -h, 1.]])
    tags, world, ids = [], [], []
    for i in range(count):
        depth = 0. if layout_kind == 'planar' else [-.2, .2, .1, -.1][i % 4]
        yaw = 180. if layout_kind == 'planar' else [165., 195., 170., 190.][i % 4]
        pose = transform([5.+depth, float(ys[i % columns]), float(zs[i // columns])], [0., 0., yaw])
        q = quaternion(pose[:3, :3])
        tag_id = i+1
        tags.append({'ID': tag_id, 'pose': {'translation': dict(zip('xyz', pose[:3, 3].tolist())),
                                          'rotation': {'quaternion': dict(zip(('W', 'X', 'Y', 'Z'), q.tolist()))}}})
        world.append((pose @ corners.T).T[:, :3])
        ids.append(tag_id)
    world = np.array(world)
    observations = []
    for i in range(views):
        phase = 2*math.pi*i/views
        camera = transform([2.+.06*math.sin(phase), 3.+.04*math.cos(phase), .7+.025*math.sin(phase)],
                           [1.+.3*math.sin(phase), -4.+.4*math.cos(phase), 17.+math.sin(phase)])
        pixels = project(world, camera)
        if not ((pixels[..., 0] > 10).all() and (pixels[..., 0] < 1270).all()
                and (pixels[..., 1] > 10).all() and (pixels[..., 1] < 790).all()):
            raise ValueError('Fixture corners must be inside the calibrated image')
        if noise_px: pixels += np.random.default_rng(1086+i).normal(0., noise_px, pixels.shape)
        detections = [{'id': tag_id, 'corners': uv.tolist(), 'center': uv.mean(axis=0).tolist(),
                       'decision_margin': 100., 'hamming': 0, 'pose_valid': False,
                       'pose_attempted': False, 'pose_device': 'none', 'pose_invalid_reason': 'deferred_multitag'}
                      for tag_id, uv in zip(ids, pixels)]
        observations.append({'camera': camera, 'pixels': pixels, 'detections': detections})
    digest = hashlib.sha256(world.tobytes())
    for obs in observations:
        digest.update(obs['camera'].tobytes())
        digest.update(obs['pixels'].tobytes())
    return {'layout': {'field': {'length': 16.5, 'width': 8.2}, 'tags': tags},
            'world': world, 'ids': ids, 'views': observations, 'sha256': digest.hexdigest()}


def accuracy_limits(noise_px):
    return {'translation_m': .03 if noise_px else 5e-4,
            'rotation_deg': .75 if noise_px else .02,
            'independent_reprojection_rms_px': .35 if noise_px else 1e-3,
            'reported_reprojection_agreement_px': 1e-4, 'ambiguity': .5}


def validate_output(output, scene, view, device, limits):
    """Check truth and pixel residuals independently, only after peers finish timing."""
    loc = output.get('localization', {})
    if not loc.get('valid') or loc.get('method') != 'multitag_pnp' or loc.get('pose_device') != device:
        raise ValueError(f'Invalid joint {device} pose: {loc.get("invalid_reason", loc)}')
    if (loc.get('used_tag_ids') != scene['ids'] or loc.get('rejected_tag_ids')
            or loc.get('single_tag_fallback', {}).get('calls', 0)):
        raise ValueError('Clean fixture must accept every tag and perform no single-tag fallback')
    if len(output.get('detections', [])) != len(scene['ids']) or any(
            d.get('pose_source') != 'field_layout_multitag' or d.get('pose_device') != device
            for d in output['detections']):
        raise ValueError('Per-tag poses must derive from the joint requested-device solve')
    estimated = pose_matrix(loc['field_to_camera'])
    truth = view['camera']
    translation = float(np.linalg.norm(estimated[:3, 3]-truth[:3, 3]))
    cosine = float((np.trace(estimated[:3, :3].T @ truth[:3, :3])-1)/2)
    rotation = math.degrees(math.acos(float(np.clip(cosine, -1., 1.))))
    residual = project(scene['world'], estimated)-view['pixels']
    reprojection = float(np.sqrt(np.mean(np.sum(residual**2, axis=-1))))
    agreement = abs(reprojection-float(loc['reprojection_error_px']))
    ambiguity = float(loc['ambiguity'])
    metrics = {'translation_m': translation, 'rotation_deg': rotation,
               'independent_reprojection_rms_px': reprojection,
               'reported_reprojection_agreement_px': agreement, 'ambiguity': ambiguity}
    if any(not math.isfinite(v) or v < 0 or v > limits[k] for k, v in metrics.items()):
        raise ValueError(f'Known-camera accuracy failed: {metrics}; limits={limits}')
    return metrics


def describe(values):
    values = [float(x) for x in values if x is not None]
    if not values: return None
    return {'count': len(values), 'p50': float(np.percentile(values, 50)),
            'p95': float(np.percentile(values, 95)), 'p99': float(np.percentile(values, 99)),
            'maximum': max(values)}


def run(device, count, layout_kind, workers, frames, warmup, round_index, noise_px=0., barrier_timeout=120.):
    scene = make_scene(count, layout_kind, noise_px)
    limits = accuracy_limits(noise_px)
    start_barrier, finish_barrier = threading.Barrier(workers), threading.Barrier(workers)
    cancel = threading.Event()
    settings = {'backend': 'native', 'mode': '3d', 'detector_device': 'cpu', 'pose_device': device,
                'multitag': True, 'tag_size_m': TAG_SIZE, 'max_ambiguity': limits['ambiguity'],
                'max_reprojection_error_px': 3.}

    def worker(index):
        detector = None
        try:
            detector = make_detector({'type': 'apriltag', 'settings': settings, 'calibration_data': CALIBRATION,
                                      'robot_to_camera': None, 'poi': {'enabled': False}},
                                     {'field_layout_data': scene['layout']})
            localizer = detector.localization
            view_indices = [(i+index) % len(scene['views']) for i in range(frames)]
            # Preallocate fresh input dictionaries outside every peer's timed
            # interval. No projection, deepcopy, or accuracy work competes with it.
            inputs = [copy.deepcopy(scene['views'][i]['detections']) for i in view_indices]
            first_call_ms = None
            for i in range(warmup):
                view = scene['views'][(i+index) % len(scene['views'])]
                copied = copy.deepcopy(view['detections'])
                started = time.perf_counter_ns()
                output = localizer.enrich(copied, (800, 1280))
                duration = (time.perf_counter_ns()-started)/1e6
                if first_call_ms is None: first_call_ms = duration
                validate_output(output, scene, view, device, limits)
            start_barrier.wait(timeout=barrier_timeout)
            outputs, wall = [], []
            started_all = time.perf_counter()
            for source in inputs:
                if cancel.is_set(): raise RuntimeError('Peer benchmark worker failed')
                before = time.perf_counter_ns()
                output = localizer.enrich(source, (800, 1280))
                wall.append((time.perf_counter_ns()-before)/1e6)
                outputs.append(output)
            elapsed = time.perf_counter()-started_all
            finish_barrier.wait(timeout=barrier_timeout)
            accuracies = [validate_output(output, scene, scene['views'][view_index], device, limits)
                          for output, view_index in zip(outputs, view_indices)]
            raw = {'wall_ms': wall}
            for key, field in [('native_call_ms', 'call_ms'), ('native_pose_ms', 'pose_ms'), ('kernel_ms', 'pose_kernel_ms')]:
                raw[key] = [output['localization'].get('gpu_timings', {}).get(field) for output in outputs]
            if device == 'cuda' and any(value is None or not math.isfinite(value) or value < 0
                                         for key in ('native_call_ms', 'native_pose_ms', 'kernel_ms') for value in raw[key]):
                raise ValueError('Actual CUDA native call/kernel timing is missing or invalid')
            return {'worker': index, 'first_call_ms': first_call_ms, 'elapsed_s': elapsed,
                    'frames_per_second': frames/elapsed, 'raw': raw, 'view_indices': view_indices,
                    'raw_accuracy': accuracies,
                    'raw_field_to_camera': [output['localization']['field_to_camera'] for output in outputs],
                    'accepted_tag_ids': scene['ids'], 'all_frames_verified': True}
        except BaseException:
            cancel.set()
            start_barrier.abort()
            finish_barrier.abort()
            raise
        finally:
            if detector is not None: detector.close()

    with ThreadPoolExecutor(max_workers=workers) as pool:
        futures = [pool.submit(worker, i) for i in range(workers)]
        results = [future.result() for future in futures]
    stages = {key: describe([value for worker in results for value in worker['raw'][key]])
              for key in ('wall_ms', 'native_call_ms', 'native_pose_ms', 'kernel_ms')}
    return {'pose_device': device, 'tags_per_frame': count, 'layout': layout_kind,
            'workers': workers, 'frames_per_worker': frames, 'warmup_frames': warmup,
            'round': round_index, 'noise_px': noise_px, 'fixture_sha256': scene['sha256'],
            'field_layout': scene['layout'], 'pipeline_settings': settings,
            'accuracy_limits': limits, 'all_frames_verified': True,
            'max_errors': {key: max(a[key] for result in results for a in result['raw_accuracy']) for key in limits},
            **stages, 'workers_results': results}


def provenance():
    sources = ['scripts/benchmark_multitag.py', 'custom_vision/app.py', 'custom_vision/localization.py',
               'custom_vision/native_apriltags.py', 'native/apriltags.cpp', 'native/cuda_multitag.cu',
               'native/cuda_multitag.hpp', 'native/cuda_multitag_math.hpp', 'native/pnp_math.hpp',
               'native/cuda_pose.cu', 'native/cuda_pose.hpp']
    binary = Path(importlib.util.find_spec('custom_vision._native').origin)
    try:
        revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    except (OSError, subprocess.CalledProcessError): revision = None
    return {'source_revision': revision,
            'source_sha256': {source: hashlib.sha256((ROOT/source).read_bytes()).hexdigest() for source in sources},
            'native_binary_path': str(binary), 'native_binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--devices', nargs='+', choices=['cpu', 'cuda'], default=['cpu', 'cuda'])
    parser.add_argument('--tags', nargs='+', type=int, choices=[2, 4, 8, 16], default=[2, 4, 8, 16])
    parser.add_argument('--layouts', nargs='+', choices=['planar', 'nonplanar'], default=['planar', 'nonplanar'])
    parser.add_argument('--cameras', type=int, choices=[1, 2], default=2)
    parser.add_argument('--frames', type=int, default=200)
    parser.add_argument('--warmup', type=int, default=20)
    parser.add_argument('--rounds', type=int, default=2)
    parser.add_argument('--noise-px', type=float, choices=[0., .1], default=0.)
    parser.add_argument('--barrier-timeout', type=float, default=120.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if min(args.frames, args.warmup, args.rounds) < 1 or not math.isfinite(args.barrier_timeout) or args.barrier_timeout <= 0:
        parser.error('frames/warmup/rounds and finite barrier timeout must be positive')
    if args.output.exists(): parser.error('Output already exists; choose a new filename')
    caps = native_capabilities()
    if not caps.get('available'): parser.error('Build the native extension first')
    if 'cuda' in args.devices and not (caps.get('cuda_multitag_compiled') and caps.get('cuda_pose_devices')):
        parser.error('CUDA joint PnP requires its compiled kernel and actual CUDA hardware')
    cv2.setNumThreads(1)
    before = provenance()
    report = {'synthetic_only': True, 'includes_camera_or_detector': False,
              'excludes': ['exposure', 'USB transfer', 'MJPEG decode', 'capture queue', 'AprilTag detection',
                           'POI', 'preview', 'NT serialization/transport', 'robot receipt'],
              'recorded_utc': datetime.now(timezone.utc).isoformat(), 'capabilities': caps,
              'platform': platform.platform(), 'python': platform.python_version(), 'opencv': cv2.__version__,
              'numpy': np.__version__, 'calibration': CALIBRATION,
              'method': 'Same runtime factory for CPU/CUDA. Independently generated field corners with 12 modestly varying oblique camera poses. POI disabled; all input single-tag poses deferred. Sequential device cases reverse order every other round. Independent correctness checks and CUDA teardown run only after every worker finishes timing.',
              'timing_scope': 'Wall time covers the entire Localization.enrich call, including coordinate changes and target derivation. CUDA native/kernel times are nested stages, not total latency. Worker FPS excludes correctness checks, input copying, initialization and teardown. CPU GPU-stage values are null, not zero.',
              'first_call_scope': 'First enrichment after pipeline construction, not cold process startup; earlier cases may initialize CUDA.',
              'settings': vars(args) | {'output': str(args.output)}, 'provenance': before, 'results': []}
    for round_index in range(args.rounds):
        order = args.devices if round_index % 2 == 0 else args.devices[::-1]
        for count in args.tags:
            for layout in args.layouts:
                for device in order:
                    result = run(device, count, layout, args.cameras, args.frames, args.warmup,
                                 round_index, args.noise_px, args.barrier_timeout)
                    report['results'].append(result)
                    print(f'{device} {layout} tags={count} round={round_index}: '
                          f'p50={result["wall_ms"]["p50"]:.3f} ms p95={result["wall_ms"]["p95"]:.3f} ms', flush=True)
    after = provenance()
    if any(before[key] != after[key] for key in ('source_sha256', 'native_binary_sha256')):
        raise RuntimeError('Source/native binary changed during benchmark; discard results')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream: stream.write(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
