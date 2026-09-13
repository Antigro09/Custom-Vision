#!/usr/bin/env python3
"""Exercise 2D, single-tag PnP and MultiTag through decoded synthetic MJPEG video."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import time

import cv2
import yaml
from demo_apriltags import fixture


def main():
    root = Path(__file__).resolve().parents[1]
    output = root / 'data' / 'smoke'
    output.mkdir(parents=True, exist_ok=True)
    frame, calibration, layout = fixture()
    video = output / 'synthetic.avi'
    writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'MJPG'), 30, (1280, 800))
    if not writer.isOpened():
        raise RuntimeError('MJPEG test video encoder unavailable')
    for _ in range(12):
        writer.write(frame)
    writer.release()
    (output / 'calibration.json').write_text(json.dumps(calibration))
    (output / 'field.json').write_text(json.dumps(layout))
    config = yaml.safe_load((root / 'config' / 'vision.yaml').read_text())
    config['networktables']['enabled'] = False
    config['dashboard']['enabled'] = False
    config['field_layout'] = 'field.json'
    config['max_frame_age_ms'] = 1000  # CI verifies correctness, not hardware timing.
    template = config['pipelines'][0]
    try:
        from custom_vision import _native
        backend = 'native'
    except ImportError:
        backend = 'pupil'
    config['pipelines'] = []
    for name, mode, multitag in [('aim', '2d', False), ('single', '3d', False), ('multi', '3d', True)]:
        pipeline = copy.deepcopy(template)
        pipeline.update(name=name, enabled=True, input_kind='synthetic', calibration='calibration.json',
                        robot_to_camera={'translation_m': [.25, .1, .45], 'rotation_rpy_deg': [0, 0, 0]})
        pipeline['camera'] = dict(source=str(video), width=1280, height=800, fps=30)
        pipeline['settings'].update(backend=backend, detector_device='cpu', mode=mode, multitag=multitag, max_ambiguity=.3)
        config['pipelines'].append(pipeline)
    config_path = output / 'vision.yaml'
    config_path.write_text(yaml.safe_dump(config))
    start = time.perf_counter()
    proc = subprocess.run([sys.executable, '-m', 'custom_vision.app', '--config', str(config_path),
                           '--max-frames', '10', '--stdout-json'], cwd=root, text=True, capture_output=True, timeout=90)
    (output / 'results.jsonl').write_text(proc.stdout)
    (output / 'runtime.log').write_text(proc.stderr)
    if proc.returncode:
        raise RuntimeError(proc.stderr)
    packets = [json.loads(line) for line in proc.stdout.splitlines()]
    groups = {name: [p for p in packets if p['pipeline'] == name and p['connected']] for name in ('aim', 'single', 'multi')}
    assert all(len(group) == 10 for group in groups.values()), 'Expected ten processed frames per pipeline'
    assert all({d['id'] for d in p['detections']} == {7, 12, 20} for group in groups.values() for p in group)
    assert all(not d['pose_valid'] for p in groups['aim'] for d in p['detections'])
    assert all(all(d['camera_to_target'] for d in p['detections']) for p in groups['single'])
    for p in groups['multi']:
        loc = p['localization']
        assert loc['valid'] and len(loc['used_tag_ids']) == 3, loc
        actual = loc['field_to_robot']['translation_m']
        assert all(abs(a-b) < .04 for a,b in zip(actual, [.75, 1.9, .15])), actual
    assert all(not p['connected'] and not p['detections'] for p in packets[-3:]), 'Shutdown did not clear targets'
    summary = dict(synthetic_only=True, backend=backend, frames_per_pipeline=10,
                   tag_ids=[7, 12, 20], modes=['2d', 'single_pnp', 'multitag'], shutdown_cleared=True,
                   wall_seconds=round(time.perf_counter()-start, 3))
    (output / 'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
