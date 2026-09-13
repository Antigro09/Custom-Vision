#!/usr/bin/env python3
"""Measure complete YOLO preprocessing/inference/decoding, excluding camera and NT."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import threading
import time

import cv2
import numpy as np

from custom_vision.objects import ObjectPipeline


def measure_detectors(detectors, image, frames, warmup, *, barrier_timeout_s=60.0):
    """Run independent workers, releasing peers if a warmup fails."""
    barrier=threading.Barrier(len(detectors),timeout=barrier_timeout_s)
    errors=[]
    error_lock=threading.Lock()
    def worker(index):
        try:
            detector=detectors[index]
            first=None
            for _ in range(warmup):
                start=time.perf_counter();detector.process(image)
                if first is None: first=(time.perf_counter()-start)*1000
            barrier.wait()
            measurements=[];counts=[];masks=[];stages=[]
            start=time.perf_counter()
            for _ in range(frames):
                before=time.perf_counter_ns();found=detector.process(image)
                measurements.append((time.perf_counter_ns()-before)/1e6)
                counts.append(len(found));masks.append(sum(d.get('segmentation') is not None for d in found))
                stages.append(dict(detector.last_timings))
            return dict(samples_ms=measurements,counts=counts,masks=masks,stages=stages,first_warmup_ms=first,elapsed_s=time.perf_counter()-start)
        except BaseException as exc:
            # Record the original detector error before waking blocked peers, so
            # a secondary BrokenBarrierError cannot obscure the useful failure.
            if not isinstance(exc,threading.BrokenBarrierError):
                with error_lock: errors.append(exc)
            barrier.abort()
            raise
    try:
        with ThreadPoolExecutor(max_workers=len(detectors)) as pool:
            return list(pool.map(worker,range(len(detectors))))
    except BaseException as exc:
        if errors: raise errors[0]
        if isinstance(exc,threading.BrokenBarrierError):
            raise RuntimeError(f'Benchmark workers did not synchronize within {barrier_timeout_s:g}s') from exc
        raise


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--engine',type=Path,required=True)
    p.add_argument('--image',type=Path,required=True,help='Representative local color image')
    p.add_argument('--cameras',type=int,choices=[1,2],default=2)
    p.add_argument('--frames',type=int,default=200)
    p.add_argument('--warmup',type=int,default=30)
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    if a.frames<1 or a.warmup<1: p.error('frames and warmup must be positive')
    meta=json.loads(a.manifest.read_text())
    cfg={key:meta[key] for key in ('labels','task','input_size','output_format')}
    cfg.update(backend='tensorrt',model_path=str(a.engine),confidence_threshold=.35,max_detections=16,max_masks=8)
    cv2.setNumThreads(1)
    image=cv2.imread(str(a.image))
    if image is None: p.error('Could not read benchmark image')
    image=cv2.resize(image,(1280,800))
    detectors=[ObjectPipeline(cfg) for _ in range(a.cameras)]
    try:
        results=measure_detectors(detectors,image,a.frames,a.warmup)
    finally:
        for detector in detectors: detector.close()
    times=np.array([v for r in results for v in r['samples_ms']])
    stages={key:round(float(np.median([s[key] for r in results for s in r['stages']])),3) for key in results[0]['stages'][0]}
    counts=[v for r in results for v in r['counts']];masks=[v for r in results for v in r['masks']]
    report={'compute_only':True,'excludes':['exposure','camera transport','camera decode','metric geometry','NT transport','robot receipt'],
        'model':a.engine.name,'model_sha256':hashlib.sha256(a.engine.read_bytes()).hexdigest(),'task':meta['task'],
        'model_scope':'COCO pretrained benchmark if using bundled benchmark weights; never proof of game-piece accuracy',
        'image_sha256':hashlib.sha256(a.image.read_bytes()).hexdigest(),'image_size':[1280,800],'network_size':meta['input_size'],
        'cameras':a.cameras,'frames_per_camera':a.frames,'warmup_per_camera':a.warmup,
        'p50_ms':round(float(np.percentile(times,50)),3),'p95_ms':round(float(np.percentile(times,95)),3),
        'p99_ms':round(float(np.percentile(times,99)),3),'max_ms':round(float(times.max()),3),
        'per_camera_fps':[round(a.frames/r['elapsed_s'],2) for r in results],
        'detections_per_frame_range':[min(counts),max(counts)],'masks_per_frame_range':[min(masks),max(masks)],
        'median_stages_ms':stages,'first_warmup_ms':[round(r['first_warmup_ms'],3) for r in results]}
    text=json.dumps(report,indent=2)
    if a.output:a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(text+'\n')
    print(text)


if __name__=='__main__': main()
