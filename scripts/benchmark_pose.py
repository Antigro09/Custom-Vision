#!/usr/bin/env python3
"""Pose-only CPU/CUDA comparison on known synthetic corners, not camera latency.

Measures real native PnP execution, including transfers, synchronization, native
result checks and Python conversion. No simulated CUDA backend exists.
"""
from __future__ import annotations
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import platform
import sys
import threading
import time
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from custom_vision.native_apriltags import NativeAprilTagPipeline, native_capabilities

CAL={'width':1280,'height':800,'camera_matrix':[[760.,0.,640.],[0.,765.,400.],[0.,0.,1.]],
     'dist_coeffs':[-.12,.03,.001,-.001,.001,.01,-.004,0.]}
OBJ=np.array([[-.08255,.08255,0.],[.08255,.08255,0.],[.08255,-.08255,0.],[-.08255,-.08255,0.]])
MATRIX=np.array(CAL['camera_matrix'])
DISTORTION=np.array(CAL['dist_coeffs'])


def describe(values):
    return dict(p50=float(np.percentile(values,50)),p95=float(np.percentile(values,95)),
                p99=float(np.percentile(values,99)),maximum=float(max(values)))


def verify_poses(output, observed, truth):
    """Independently check geometry; a solver's own small residual is insufficient."""
    if len(output)!=len(observed):
        raise RuntimeError('Pose count differs from input tag count')
    errors=[]
    for pose, corners, (rotation, translation) in zip(output, observed, truth):
        if not pose.get('pose_valid'):
            raise RuntimeError('Known synthetic pose was rejected')
        rvec=np.asarray(pose.get('rvec_rad'),dtype=np.float64)
        tvec=np.asarray(pose.get('tvec_m'),dtype=np.float64)
        reported=pose.get('reprojection_error_px')
        if (rvec.shape!=(3,) or tvec.shape!=(3,) or not np.isfinite(rvec).all()
                or not np.isfinite(tvec).all() or isinstance(reported,bool)
                or not isinstance(reported,(int,float)) or not math.isfinite(reported)
                or not 0<=reported<=1e-3):
            raise RuntimeError('Known synthetic pose has invalid numeric geometry or residual')
        estimate_rotation=cv2.Rodrigues(rvec)[0]
        if np.any(((estimate_rotation@OBJ.T).T+tvec)[:,2]<=0):
            raise RuntimeError('Known synthetic pose places corners behind camera')
        projected=cv2.projectPoints(OBJ,rvec,tvec,MATRIX,DISTORTION)[0].reshape(4,2)
        error=float(np.sqrt(np.mean(np.sum((projected-corners)**2,axis=1))))
        rotation_error=math.degrees(math.acos(float(np.clip((np.trace(estimate_rotation@rotation.T)-1)/2,-1,1))))
        if (not math.isfinite(error) or error>1e-3 or np.linalg.norm(tvec-translation)>1e-3
                or rotation_error>.05):
            raise RuntimeError('Known synthetic pose failed independent residual/translation/rotation check')
        errors.append(error)
    return max(errors)


def run(device,count,workers,frames,warmup,round_index):
    barrier=threading.Barrier(workers)
    rng=np.random.default_rng(1086)
    # Varied poses, not just the same frontal square; generation is outside timing.
    batches=[]
    for _ in range(32):
        uv=[];truth=[]
        for _ in range(count):
            r=rng.uniform(-.8,.8,3);r[2]=rng.uniform(-math.pi,math.pi)
            t=np.array([rng.uniform(-.4,.4),rng.uniform(-.3,.3),rng.uniform(.6,4.)])
            uv.append(cv2.projectPoints(OBJ,r,t,MATRIX,DISTORTION)[0].reshape(4,2))
            truth.append((cv2.Rodrigues(r)[0],t))
        batches.append((np.array(uv),truth))
    def worker(index):
        p=None
        try:
            p=NativeAprilTagPipeline({'mode':'3d','tag_size_m':.1651,'pose_device':device},CAL)
            def call(i):
                corners,truth=batches[(i+index)%len(batches)]
                started=time.perf_counter_ns();out=p.estimate_poses(corners)
                elapsed=(time.perf_counter_ns()-started)/1e6
                native=p.last_timings
                return elapsed,native['pose_kernel_ms'],native['pose_ms'],out,corners,truth
            first_result=call(0)
            first=first_result[0]
            verify_poses(*first_result[3:])
            for i in range(warmup):verify_poses(*call(i)[3:])
            barrier.wait(timeout=120)
            timings=[];kernel=[];native=[];pending=[];start=time.perf_counter()
            for i in range(frames):
                a,b,c,out,corners,truth=call(i)
                timings.append(a);kernel.append(b);native.append(c);pending.append((out,corners,truth))
            elapsed=time.perf_counter()-start
            # Validate after ALL peers stop timing, so OpenCV/NumPy validation
            # cannot compete for the GIL/CPU while another camera is measured.
            # Teardown also waits until no worker is still timing CUDA.
            barrier.wait(timeout=120)
            errors=[verify_poses(*sample) for sample in pending]
            overhead=[a-b for a,b in zip(timings,native)]
            return dict(worker=index,first_call_ms=first,wall_ms=describe(timings),
                        kernel_ms=describe(kernel),native_pose_ms=describe(native),
                        wall_minus_native_ms=describe(overhead),
                        raw_wall_ms=timings,raw_kernel_ms=kernel,raw_native_pose_ms=native,
                        raw_wall_minus_native_ms=overhead,
                        max_independent_reprojection_error_px=max(errors),
                        batch_calls_per_second=frames/elapsed)
        except BaseException:
            barrier.abort()
            raise
        finally:
            if p is not None:p.close()
    with ThreadPoolExecutor(max_workers=workers) as pool:
        futures=[pool.submit(worker,i) for i in range(workers)]
        results=[f.result() for f in futures]
    pooled={key:describe([v for result in results for v in result['raw_'+key]])
            for key in ('wall_ms','kernel_ms','native_pose_ms','wall_minus_native_ms')}
    return dict(pose_device=device,tags_per_frame=count,workers=workers,round=round_index,
                **pooled,workers_results=results)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--devices',nargs='+',choices=['cpu','cuda'],default=['cpu','cuda'])
    parser.add_argument('--tags',nargs='+',type=int,default=[1,2,4,8,16,32])
    parser.add_argument('--cameras',type=int,choices=[1,2],default=2)
    parser.add_argument('--frames',type=int,default=300)
    parser.add_argument('--warmup',type=int,default=30)
    parser.add_argument('--rounds',type=int,default=2)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv)
    if args.frames<1 or args.warmup<1 or args.rounds<1 or any(not 1<=n<=256 for n in args.tags):
        parser.error('Positive frames/warmup/rounds and 1..256 tags required')
    if args.output.exists():parser.error('Output already exists; use a new filename')
    caps=native_capabilities()
    if not caps.get('available'):parser.error('Build the native extension first')
    if 'cuda' in args.devices and not (caps.get('cuda_pose_compiled') and caps.get('cuda_pose_devices')):
        parser.error('CUDA pose build and a real GPU are required; use --devices cpu for CPU only')
    cv2.setNumThreads(1)
    report=dict(synthetic_only=True,includes_camera_or_detector=False,capabilities=caps,
                recorded_utc=datetime.now(timezone.utc).isoformat(),
                platform=platform.platform(),python=platform.python_version(),opencv=cv2.__version__,numpy=np.__version__,
                method='Sequential CPU/GPU rounds, reversed device order every other round; warmed independent worker instances; call wall time includes native copies, native result checks and Python conversion. Every timed output is independently validated after all workers finish timing.',
                throughput_scope='Measured loop includes native timing retrieval and storing outputs; excludes independent benchmark validation, warmup, startup and teardown.',
                wall_minus_native_scope='Outer call wall time minus native pose stage: includes argument handling, waiting to reacquire the GIL, Python result construction and scheduling; not a pure GIL measurement.',
                first_call_scope='First estimate_poses call after detector construction, not cold process/context startup; earlier cases may already have initialized CUDA.',
                accuracy_limits={'independent_reprojection_rms_px':1e-3,'translation_error_m':1e-3,'rotation_error_deg':.05},
                settings=vars(args)|{'output':str(args.output)},
                source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in [ROOT/'native/cuda_pose.cu',ROOT/'native/cuda_pose.hpp',ROOT/'native/cuda_multitag.cu',ROOT/'native/cuda_multitag.hpp',ROOT/'native/cuda_multitag_math.hpp',ROOT/'native/pnp_math.hpp',ROOT/'native/apriltags.cpp',Path(__file__).resolve()]},results=[])
    native_path=Path(importlib.util.find_spec('custom_vision._native').origin)
    report['native_binary_sha256']=hashlib.sha256(native_path.read_bytes()).hexdigest()
    for round_index in range(args.rounds):
        for count in args.tags:
            devices=args.devices if round_index%2==0 else args.devices[::-1]
            for device in devices:
                data=run(device,count,args.cameras,args.frames,args.warmup,round_index)
                report['results'].append(data)
                print(f'{device} tags={count} round={round_index}: batch p50={data["wall_ms"]["p50"]:.3f} ms, p95={data["wall_ms"]["p95"]:.3f} ms',flush=True)
    if (any(hashlib.sha256((ROOT/path).read_bytes()).hexdigest()!=digest
            for path,digest in report['source_sha256'].items())
            or hashlib.sha256(native_path.read_bytes()).hexdigest()!=report['native_binary_sha256']):
        raise RuntimeError('Source or native binary changed during benchmark; discard results')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:
        stream.write(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return 0


if __name__=='__main__':raise SystemExit(main())
