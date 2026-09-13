#!/usr/bin/env python3
"""Synthetic compute benchmark; excludes exposure, UVC transport and robot receipt."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import threading
import time

import cv2
import numpy as np


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


def measure(backend, mode, decimate, cameras, frames, warmup, preprocess='cpu', noise=False, device='cpu', contrast=5, localization=False):
    from custom_vision.apriltags import AprilTagPipeline
    if backend == 'native':
        from custom_vision.native_apriltags import NativeAprilTagPipeline
        cls = NativeAprilTagPipeline
    else:
        cls = AprilTagPipeline
    image, ids = scene(noise=noise)
    calibration = dict(width=1280,height=800,camera_matrix=[[950,0,640],[0,950,400],[0,0,1]],dist_coeffs=[0]*5)
    settings=dict(mode=mode,quad_decimate=decimate,threads=2,preprocess=preprocess,min_decision_margin=30,
                  detector_device=device,min_white_black_diff=contrast)
    localizers=[]
    if localization:
        from demo_apriltags import fixture
        from custom_vision.localization import Localization
        image, calibration, layout=fixture()
        ids=[7,12,20]
        settings.update(known_tag_ids=ids,skip_single_when_multi=True,max_ambiguity=.3)
        mount={'translation_m':[.25,.1,.45],'rotation_rpy_deg':[0,0,0]}
        localizers=[Localization(settings,calibration,layout,mount) for _ in range(cameras)]
    detectors = [cls(settings, calibration) for _ in range(cameras)]
    barrier = threading.Barrier(cameras)
    def worker(index):
        detector=detectors[index]
        measurements = []
        correct = 0
        startup_ms=None
        for _ in range(warmup):
            before=time.perf_counter_ns()
            found=detector.process(image)
            if localization: localizers[index].enrich(found,image.shape)
            if startup_ms is None: startup_ms=(time.perf_counter_ns()-before)/1e6
        barrier.wait()
        start = time.perf_counter()
        for _ in range(frames):
            before = time.perf_counter_ns()
            found = detector.process(image)
            pose_ok=True
            if localization:
                enriched=localizers[index].enrich(found,image.shape)
                pose_ok=enriched['localization']['valid']
            measurements.append((time.perf_counter_ns()-before)/1e6)
            correct += set(d['id'] for d in found) == set(ids) and pose_ok
        elapsed = time.perf_counter()-start
        return dict(samples_ms=measurements,elapsed_s=elapsed,correct_frames=correct,first_warmup_ms=startup_ms)
    with ThreadPoolExecutor(max_workers=cameras) as pool:
        results=list(pool.map(worker,range(cameras)))
    # GPU teardown can synchronize shared device resources. Do not destroy one
    # camera while another is still being timed. Runtime also joins all workers.
    for detector in detectors:
        if hasattr(detector,'close'): detector.close()
    timings=np.array([x for r in results for x in r['samples_ms']])
    return dict(scene="mapped_three_tags" if localization else ("pixel_noise_stress" if noise else "structured"),backend=backend,mode=mode,preprocess=preprocess,detector_device=device,min_white_black_diff=contrast,includes_localization=localization,quad_decimate=decimate,threads_per_camera=2,cameras=cameras,
                frames_per_camera=frames,warmup_frames=warmup,image_size=[1280,800],tag_ids=ids,
                p50_ms=round(float(np.percentile(timings,50)),3),p95_ms=round(float(np.percentile(timings,95)),3),
                p99_ms=round(float(np.percentile(timings,99)),3),max_ms=round(float(max(timings)),3),
                per_camera_fps=[round(frames/r['elapsed_s'],2) for r in results],
                first_warmup_ms=[round(r['first_warmup_ms'],3) if r['first_warmup_ms'] is not None else None for r in results],
                correct_frames=sum(r['correct_frames'] for r in results),total_frames=cameras*frames,
                over_24_ms=int(sum(timings>24)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend',choices=['pupil','native'],default='native')
    parser.add_argument('--mode',choices=['2d','3d'],default='3d')
    parser.add_argument('--decimate',type=float,default=2)
    parser.add_argument('--cameras',type=int,choices=[1,2],default=2)
    parser.add_argument('--frames',type=int,default=100)
    parser.add_argument('--warmup',type=int,default=10)
    parser.add_argument('--preprocess',choices=['cpu','cuda'],default='cpu')
    parser.add_argument('--detector-device',choices=['cpu','cuda'],default='cpu')
    parser.add_argument('--min-white-black-diff',type=int,default=5)
    parser.add_argument('--localization',action='store_true',help='Use a separate mapped three-tag scene and include joint field localization')
    parser.add_argument('--noise-stress',action='store_true')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.frames < 1 or args.warmup < 0: parser.error('frames must be positive and warmup nonnegative')
    if args.backend != 'native' and args.detector_device != 'cpu': parser.error('CUDA detection requires native')
    if args.localization and (args.noise_stress or args.mode != '3d'): parser.error('localization uses its own 3d scene')
    cv2.setNumThreads(1)
    result=measure(args.backend,args.mode,args.decimate,args.cameras,args.frames,args.warmup,args.preprocess,args.noise_stress,args.detector_device,args.min_white_black_diff,args.localization)
    excluded=['exposure','USB transport','MJPEG decode','NT transport','robot receipt']
    if not args.localization: excluded.append('field localization')
    report=dict(synthetic_compute_only=True,excludes=excluded,result=result)
    data=json.dumps(report,indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(data+'\n')
    print(data)

if __name__ == '__main__':
    main()
