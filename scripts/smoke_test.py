#!/usr/bin/env python3
"""Generate synthetic tag/ball input and exercise both actual pipelines end to end."""
import json
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml


def main():
    root = Path(__file__).resolve().parents[1]
    output = root / 'data' / 'smoke'
    output.mkdir(parents=True, exist_ok=True)
    frame = np.full((480, 640, 3), 35, np.uint8)
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 7, 160)
    frame[50:250, 50:250] = 255
    frame[70:230, 70:230] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    cv2.circle(frame, (470, 330), 35, (245, 245, 245), -1)
    video = output / 'synthetic.avi'
    writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'MJPG'), 30, (640, 480))
    if not writer.isOpened():
        raise RuntimeError('MJPEG test video encoder unavailable')
    for _ in range(12):
        writer.write(frame)
    writer.release()
    calibration = dict(width=640, height=480, camera_matrix=[[600, 0, 320], [0, 600, 240], [0, 0, 1]], dist_coeffs=[0]*5)
    (output / 'synthetic_calibration.json').write_text(json.dumps(calibration))
    config = yaml.safe_load((root / 'config' / 'vision.yaml').read_text())
    config['networktables']['enabled'] = False
    config['dashboard']['enabled'] = False
    for pipeline in config['pipelines']:
        pipeline['camera'] = dict(source=str(video), width=640, height=480, fps=30)
        pipeline['calibration'] = 'synthetic_calibration.json'
    config_path = output / 'vision.yaml'
    config_path.write_text(yaml.safe_dump(config))
    start = time.perf_counter()
    proc = subprocess.run([sys.executable, '-m', 'custom_vision.app', '--config', str(config_path), '--max-frames', '10', '--stdout-json'], cwd=root, text=True, capture_output=True, timeout=60)
    (output / 'results.jsonl').write_text(proc.stdout)
    (output / 'runtime.log').write_text(proc.stderr)
    if proc.returncode:
        raise RuntimeError(proc.stderr)
    packets = [json.loads(line) for line in proc.stdout.splitlines()]
    tags = [p for p in packets if p['type'] == 'apriltag' and p['connected']]
    objects = [p for p in packets if p['type'] == 'object' and p['connected']]
    assert len(tags) == len(objects) == 10, 'Expected 10 processed frames per pipeline'
    assert all(any(d['id'] == 7 and d['pose_valid'] for d in p['detections']) for p in tags)
    assert all(any(abs(d['center'][0] - 470) < 3 and abs(d['center'][1] - 330) < 3 for d in p['detections']) for p in objects)
    assert all(not p['connected'] and not p['detections'] for p in packets[-2:]), 'Shutdown did not clear targets'
    summary = {'synthetic_only': True, 'frames_per_pipeline': 10, 'tag_id': 7, 'ball_candidate_detected': True, 'shutdown_cleared': True, 'wall_seconds': round(time.perf_counter()-start, 3)}
    (output / 'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
