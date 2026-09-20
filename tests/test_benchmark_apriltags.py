"""Benchmark coordination/contracts; mock timings do not establish CUDA performance."""
import json
import threading
import time

import pytest

from scripts import benchmark_apriltags as bench


def fake_components(monkeypatch, *, fail_worker=None):
    from custom_vision import localization, poi, native_apriltags
    instances = []
    lock = threading.Lock()
    finished = set()

    class Detector:
        def __init__(self, settings, calibration):
            with lock:
                self.index = len(instances)
                instances.append(self)
            self.settings = settings
            self.closed = False
            self.calls = 0
            self.last_timings = {'detect_ms': 2., 'pose_ms': .2}

        def process(self, image):
            if self.index == fail_worker:
                raise RuntimeError('test worker failure')
            self.calls += 1
            # Permit interleaving; tests assert cleanup only after all timing ends.
            time.sleep(.001)
            return [{'id': n, 'pose_valid': True} for n in [7, 12, 20]]

        def _estimate_pose(self, corners):
            return {'pose_valid': True}

        def close(self):
            self.closed = True
            finished.add(self.index)

    class Localizer:
        def __init__(self, settings, calibration, layout, mount):
            self.field_tags = {n: {} for n in [7, 12, 20]} if layout else {}
            self.pose_device = settings['pose_device']

        def enrich(self, detections, shape):
            assert hasattr(self.single_pose_solver, '__self__')
            # POI must already have seen the original detections when enabled.
            for detection in detections: detection['enriched'] = True
            return {'detections': detections, 'localization': {
                'valid': True, 'method': 'multitag_pnp', 'pose_device': self.pose_device,
                'field_to_camera': {'translation_m': [1., 2., .6]}}}

    class Tracker:
        def __init__(self, settings, calibration, mount):
            self.settings = settings
            assert not settings['enabled'] or settings['calibration_verified']

        def process(self, detections, shape, captured):
            assert all('enriched' not in d for d in detections)
            return {'valid': True, 'selected_name': 'benchmark_aim', 'targets': [
                {'name': 'benchmark_aim', 'valid': True, 'camera_translation_m': [.4, -.5, 2.],
                 'tx_deg': 11.31, 'ty_deg': 14.036}]}

    monkeypatch.setattr(native_apriltags, 'NativeAprilTagPipeline', Detector)
    monkeypatch.setattr(localization, 'Localization', Localizer)
    monkeypatch.setattr(poi, 'PointOfInterestTracker', Tracker)
    return instances, finished, Detector


def test_poi_before_localization_records_raw_and_pooled_stages(monkeypatch):
    instances, finished, _ = fake_components(monkeypatch)
    result = bench.measure('native', '3d', 2, 2, 3, 1, localization=True, poi=True)
    assert result['correct_frames'] == result['total_frames'] == 6
    assert result['includes_poi'] and result['includes_localization']
    assert result['skip_single_when_multi'] is False
    assert len(result['fixture_sha256']) == 64
    assert len(result['workers_results']) == 2
    assert finished == {0, 1}
    assert all(i.closed and i.calls == 4 for i in instances)
    assert all(i.settings['pose_device'] == 'cpu' and not i.settings.get('skip_single_when_multi') for i in instances)
    for worker in result['workers_results']:
        assert len(worker['samples_ms']) == 3
        assert len(worker['stages_ms']['native_pose_ms']) == 3
        assert worker['failure_examples'] == []
    assert result['stages_ms']['native_pose_ms']['count'] == 6
    assert result['stages_ms']['native_pose_ms']['p50_ms'] == .2
    assert 'validation_ms' not in result['stages_ms']
    assert result['validation_excluded_from_latency_and_fps']
    assert result['validation_ms']['count'] == 6
    assert all(len(worker['validation_samples_ms']) == 3 for worker in result['workers_results'])


def test_localization_only_preserves_skip_optimization(monkeypatch):
    instances, _, _ = fake_components(monkeypatch)
    result = bench.measure('native', '3d', 2, 1, 2, 0, localization=True)
    assert result['skip_single_when_multi']
    assert instances[0].settings['known_tag_ids'] == [7, 12, 20]
    assert result['first_warmup_ms'] == [None]
    assert result['stages_ms']['poi_ms']['max_ms'] == 0.


def test_worker_failure_aborts_barriers_and_closes_instances(monkeypatch):
    instances, _, _ = fake_components(monkeypatch, fail_worker=1)
    started = time.monotonic()
    with pytest.raises((RuntimeError, threading.BrokenBarrierError)):
        bench.measure('native', '3d', 2, 2, 3, 1, localization=True, barrier_timeout=.5)
    assert time.monotonic() - started < 2.
    assert len(instances) == 2 and all(i.closed for i in instances)


def test_teardown_waits_for_peer_measured_calls(monkeypatch):
    instances, _, cls = fake_components(monkeypatch)
    original_process = cls.process
    original_close = cls.close

    def process(self, image):
        if self.index == 1: time.sleep(.003)
        return original_process(self, image)

    def close(self):
        assert len(instances) == 2 and all(i.calls == 3 for i in instances)
        original_close(self)

    monkeypatch.setattr(cls, 'process', process)
    monkeypatch.setattr(cls, 'close', close)
    bench.measure('native', '3d', 2, 2, 3, 0, localization=True)


def test_correctness_rejects_wrong_ids_invalid_poses_and_geometry():
    found = [{'id': 7, 'pose_valid': False}]
    errors = bench.check_frame(found, [7, 12, 20], mode='3d', require_single=True,
                              localization_result={'localization': {'valid': False}},
                              poi_result={'valid': False, 'invalid_reason': 'unverified'})
    assert len(errors) == 4
    loc = {'localization': {'valid': True, 'method': 'multitag_pnp',
                           'field_to_camera': {'translation_m': [5, 0, 0]}}}
    poi = {'valid': True, 'selected_name': 'benchmark_aim', 'targets': [
        {'name': 'benchmark_aim', 'valid': True, 'camera_translation_m': [0, 0, 9],
         'tx_deg': float('nan'), 'ty_deg': 1}]}
    errors = bench.check_frame([{'id': 7}], [7], mode='3d', require_single=False,
                              localization_result=loc, poi_result=poi)
    assert len(errors) == 3


def test_output_refuses_overwrite_before_starting_work(tmp_path, monkeypatch):
    output = tmp_path / 'existing.json'
    output.write_text('original')
    monkeypatch.setattr(bench, 'metadata', lambda: pytest.fail('must reject before hardware inspection'))
    with pytest.raises(SystemExit) as exc:
        bench.main(['--output', str(output)])
    assert exc.value.code == 2
    assert output.read_text() == 'original'


def test_rounds_raw_reporting_and_incorrect_exit(tmp_path, monkeypatch, capsys):
    seen_rounds = []
    monkeypatch.setattr(bench, 'metadata', lambda: {'source_sha256': {'fixture': 'abc'},
                                                 'native_binary_sha256': 'binary', 'capabilities': {'available': True}})
    def measure(*args):
        seen_rounds.append(args[-2])
        return {'correct_frames': 2 if args[-2] == 0 else 1, 'total_frames': 2,
                'workers_results': [{'samples_ms': [.1, .2]}]}
    monkeypatch.setattr(bench, 'measure', measure)
    output = tmp_path / 'rounds.json'
    assert bench.main(['--rounds', '2', '--output', str(output)]) == 1
    report = json.loads(output.read_text())
    assert seen_rounds == [0, 1]
    assert report['all_frames_correct'] is False
    assert report['result'] == report['results'][0]
    assert report['results'][0]['workers_results'][0]['samples_ms'] == [.1, .2]
    assert report['metadata']['native_binary_sha256'] == 'binary'
    capsys.readouterr()


def test_source_changed_during_run_rejects_report(tmp_path, monkeypatch):
    calls = iter(['before', 'after'])
    monkeypatch.setattr(bench, 'metadata', lambda: {'source_sha256': {'fixture': next(calls)},
                                                 'native_binary_sha256': 'binary', 'capabilities': {'available': True}})
    monkeypatch.setattr(bench, 'measure', lambda *args: {'correct_frames': 1, 'total_frames': 1})
    output = tmp_path / 'changed.json'
    with pytest.raises(RuntimeError, match='changed during benchmark'):
        bench.main(['--output', str(output)])
    assert not output.exists()


@pytest.mark.parametrize('args', [
    ['--mode', '2d', '--pose-device', 'cuda'],
    ['--backend', 'pupil', '--pose-device', 'cuda'],
    ['--poi', '--noise-stress'],
    ['--poi', '--mode', '2d'],
    ['--rounds', '0'],
])
def test_invalid_comparisons_rejected_before_hardware(args, monkeypatch):
    monkeypatch.setattr(bench, 'metadata', lambda: pytest.fail('configuration must fail first'))
    with pytest.raises(SystemExit) as exc: bench.main(args)
    assert exc.value.code == 2


def test_real_cpu_detector_poi_and_localization_on_rendered_fixture():
    pytest.importorskip('pupil_apriltags')
    result = bench.measure('pupil', '3d', 2, 1, 2, 1, localization=True, poi=True)
    assert result['correct_frames'] == result['total_frames'] == 2
    assert result['workers_results'][0]['failure_examples'] == []
    assert result['stages_ms']['poi_ms']['max_ms'] > 0
    assert result['stages_ms']['localization_ms']['max_ms'] > 0


def test_correctness_waits_for_all_peers_to_finish_timed_processing(monkeypatch):
    instances, _, cls = fake_components(monkeypatch)
    original_process = cls.process
    original_check = bench.check_frame
    completed = [0, 0]
    checks = []

    def process(self, image):
        if self.index == 1: time.sleep(.003)
        result = original_process(self, image)
        completed[self.index] += 1
        return result

    def check(*args, **kwargs):
        # warmup=0: every validation must be behind the shared finish barrier.
        assert completed == [3, 3]
        checks.append(1)
        return original_check(*args, **kwargs)

    monkeypatch.setattr(cls, 'process', process)
    monkeypatch.setattr(bench, 'check_frame', check)
    result = bench.measure('native', '3d', 2, 2, 3, 0, localization=True, poi=True)
    assert len(checks) == 6
    assert result['correct_frames'] == 6
    assert all(instance.closed for instance in instances)


def test_correctness_rejects_hidden_cpu_joint_or_single_pose_fallback():
    loc = {'localization': {'valid': True, 'method': 'multitag_pnp', 'pose_device': 'cpu',
                           'field_to_camera': {'translation_m': [1., 2., .6]}}}
    errors = bench.check_frame([{'id': 7, 'pose_valid': True, 'pose_device': 'cpu'}], [7],
                              mode='3d', require_single=True, localization_result=loc,
                              expected_pose_device='cuda')
    assert errors == ['single-tag pose ran on a different pose device',
                      'joint localization ran on a different pose device']


def test_pipeline_build_uses_runtime_factory_and_explicit_backend(monkeypatch):
    from custom_vision import app
    calls = []
    expected = object()
    def factory(cfg, global_config):
        calls.append((cfg, global_config))
        return expected
    monkeypatch.setattr(app, 'make_detector', factory)
    settings = {'backend': 'native', 'pose_device': 'cuda'}
    assert bench.build_pipeline(settings, {'intrinsics': 'fixture'}, {'field': 'fixture'},
                                {'mount': 'fixture'}, True) is expected
    cfg, global_config = calls[0]
    assert cfg['type'] == 'apriltag' and cfg['settings'] == settings
    assert cfg['settings'] is not settings
    assert cfg['poi']['enabled'] and cfg['poi']['targets'][0]['offset_m'] == [0, 0, .1]
    assert global_config['field_layout_data'] == {'field': 'fixture'}
