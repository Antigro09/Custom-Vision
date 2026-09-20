"""Offline statistics retain failures and never mislabel warmup or dropped frames."""
import importlib.util
import json
from pathlib import Path

import cv2
import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]


def module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts' / (name + '.py'))
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


summarizer = module('summarize_vision_log')


def summary(tmp_path, records, warmup=0):
    path = tmp_path / 'observations.jsonl'
    path.write_text('\n'.join(x if isinstance(x, str) else json.dumps(x) for x in records) + '\n')
    return summarizer.summarize(path, warmup)


def packet(**values):
    return dict(pipeline='front', boot_id='boot-1', connected=True, detections=[]) | values


def test_warmup_is_per_camera_and_boot_but_never_hides_failures(tmp_path):
    records = [packet(latency_ms=100, detections=[{'id': 7, 'pose_valid': False,
                                                   'pose_invalid_reason': 'reprojection_error'}])]
    records += [dict(packet(), connected=False, error='No fresh frame within 100 ms'),
                packet(latency_ms=2, detections=[{'id': 7, 'pose_valid': True}]),
                dict(packet(latency_ms=200), pipeline='rear'),
                dict(packet(latency_ms=6), pipeline='rear'),
                dict(packet(latency_ms=300), boot_id='boot-2'),
                dict(packet(), boot_id='boot-2', connected=False, error='shutdown')]
    result = summary(tmp_path, records, warmup=1)
    groups = {(g['pipeline'], g['boot_id']): g for g in result['pipelines']}
    first = groups['front', 'boot-1']
    assert first['records'] == 3 and first['connected'] == 2
    assert first['warmup_connected_frames_excluded'] == 1
    assert first['measured_connected_frames'] == 1
    assert first['measurements']['latency_ms']['p50'] == 2
    assert first['tags'] == 2 and first['valid_poses'] == 1
    assert first['invalid_poses'] == {'reprojection_error': 1}
    assert first['errors'] == {'No fresh frame within 100 ms': 1}
    assert groups['rear', 'boot-1']['measurements']['latency_ms']['p50'] == 6
    assert groups['front', 'boot-2']['measurements'] == {}
    assert groups['front', 'boot-2']['errors'] == {'shutdown': 1}


def test_nonfinite_negative_boolean_and_missing_metrics_do_not_become_fast_samples(tmp_path):
    records = [packet(latency_ms=value, native_timings={'pose_ms': value})
               for value in (0., 2., float('nan'), float('inf'), -1., True, None, '4')]
    records += [packet(fps=50.), packet(native_timings='bad')]
    result = summary(tmp_path, records)['pipelines'][0]
    assert result['measurements']['latency_ms'] == {
        'samples': 2, 'p50': 1., 'p95': 1.9, 'p99': 1.98, 'maximum': 2.}
    assert result['measurements']['native_pose_ms']['samples'] == 2
    assert result['invalid_measurements'] == {'latency_ms': 6, 'native_pose_ms': 6}
    assert result['invalid_fields'] == {'native_timings': 1}
    assert result['measurements']['fps']['samples'] == 1


def test_counter_resets_do_not_turn_maximum_into_total_dropped_frames(tmp_path):
    records = [packet(dropped_frames=value) for value in (10, 13, 2, 5)]
    records += [packet(), packet(dropped_frames=-1), packet(dropped_frames=True)]
    result = summary(tmp_path, records)['pipelines'][0]
    assert result['initial_dropped'] == 10
    assert result['last_dropped'] == 5 and result['max_dropped'] == 13
    assert result['observed_dropped_increments'] == 8
    assert result['drop_counter_resets'] == 1
    assert result['invalid_fields'] == {'dropped_frames': 2}
    unknown = summary(tmp_path, [packet()])['pipelines'][0]
    assert unknown['initial_dropped'] is None and unknown['last_dropped'] is None


def test_bad_record_envelopes_are_counted_instead_of_crashing(tmp_path):
    records = ['not JSON', 'null', '[]', '{}', dict(packet(), boot_id={}),
               dict(packet(), connected='false'), dict(packet(), pipeline=''),
               packet(detections=[None, {'id': True}, {'class_id': 0}, {'id': 7, 'pose_valid': 'yes'}]),
               packet(detections=None)]
    result = summary(tmp_path, records)
    assert result['malformed_lines'] == 7
    good = result['pipelines'][0]
    assert good['records'] == 2 and good['tags'] == 1 and good['valid_poses'] == 0
    assert good['invalid_poses'] == {'unknown': 1}
    assert good['invalid_fields'] == {'detection': 1, 'tag_id': 1, 'detections': 1}
    # Fully serializable statistics even when the input contains NaN elsewhere.
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('warmup', [-1, True, 1.5])
def test_invalid_warmup_rejected_by_library_api(tmp_path, warmup):
    with pytest.raises(ValueError, match='warmup'):
        summary(tmp_path, [packet()], warmup)


def test_pose_benchmark_recomputes_accuracy_instead_of_trusting_reported_residual():
    benchmark = module('benchmark_pose')
    rvec = np.array([.3, -.25, 2.])
    tvec = np.array([.1, -.2, 2.5])
    uv = cv2.projectPoints(benchmark.OBJ, rvec, tvec, benchmark.MATRIX,
                           benchmark.DISTORTION)[0].reshape(4, 2)
    pose = {'pose_valid': True, 'rvec_rad': rvec.tolist(), 'tvec_m': tvec.tolist(),
            'reprojection_error_px': 0.}
    truth = [(cv2.Rodrigues(rvec)[0], tvec)]
    assert benchmark.verify_poses([pose], [uv], truth) < 1e-10
    wrong = dict(pose, tvec_m=[.2, -.2, 2.5])  # Keeps a fictitious zero residual.
    with pytest.raises(RuntimeError, match='independent'):
        benchmark.verify_poses([wrong], [uv], truth)
    for residual in (float('nan'), -1., True):
        with pytest.raises(RuntimeError, match='numeric'):
            benchmark.verify_poses([dict(pose, reprojection_error_px=residual)], [uv], truth)
