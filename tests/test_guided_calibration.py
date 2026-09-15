"""Guided calibration regressions plus real mrcal integration when installed.

Synthetic images exercise software contracts; they are not Arducam accuracy data.
"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from custom_vision.calibration_session import (Board, Selector, Session, LiveReader, Analyzer,
                                               detect_board, grid_coverage, load_session, overlay, main, write_json)
from custom_vision.calibration_mount import estimate_mount, validate_transform, rpy_degrees
from custom_vision.calibration_solver import split_views, export_opencv, calibrate
from custom_vision.localization import CV_TO_NWU, _rpy_rotation


@pytest.fixture(scope='module')
def rendered_boards(tmp_path_factory):
    directory = tmp_path_factory.mktemp('guided-boards')
    board = Board(7, 5, .03)
    rng = np.random.default_rng(20260915)
    matrix = np.array([[600., 0, 320.], [0, 605., 240.], [0, 0, 1.]])
    distortion = np.array([-.06, .015, .0005, -.0008, 0.])
    outer = np.array([[-1, -1, 0], [7, -1, 0], [7, 5, 0], [-1, 5, 0]], float) * .03
    observations = []
    for index in range(60):
        for _ in range(1000):
            r = np.r_[rng.uniform(-.65, .65, 2), rng.uniform(-.15, .15)]
            rotation = cv2.Rodrigues(r)[0]
            z = rng.uniform(.28, .65)
            center = np.array([(rng.uniform(70, 570) - 320) / 600 * z,
                               (rng.uniform(65, 415) - 240) / 605 * z, z])
            t = center - rotation @ (np.array([3, 2, 0]) * .03)
            bounds = cv2.projectPoints(outer, r, t, matrix, distortion)[0].reshape(-1, 2)
            if np.all(bounds > 8) and np.all(bounds < [632, 472]):
                break
        frame = np.full((1440, 1920), 190, np.uint8)
        big_matrix = matrix.copy()
        big_matrix[:2] *= 3
        # Raster boundaries and resize both use pixel centers, not pixel edges.
        big_matrix[:2, 2] += 1.5
        for y in range(-1, board.rows):
            for x in range(-1, board.cols):
                # Subdivide distorted square edges rather than treating curves as straight.
                line = np.linspace(0, 1, 9, endpoint=False)
                polygon = np.concatenate((np.c_[x + line, np.full(9, y)],
                                          np.c_[np.full(9, x + 1), y + line],
                                          np.c_[x + 1 - line, np.full(9, y + 1)],
                                          np.c_[np.full(9, x), y + 1 - line]))
                points = np.c_[polygon, np.zeros(len(polygon))] * .03
                q = cv2.projectPoints(points, r, t, big_matrix, distortion)[0].reshape(-1, 2)
                cv2.fillPoly(frame, [np.rint(q).astype(np.int32)], 15 if (x + y) % 2 == 0 else 245)
        frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)
        path = directory / f'frame{index:06d}.png'
        assert cv2.imwrite(str(path), frame)
        q = cv2.projectPoints(board.points(), r, t, matrix, distortion)[0].reshape(-1, 2)
        observations.append((path, q))
    return board, matrix, distortion, observations


@pytest.mark.parametrize('args', [(True, 5, .03), (2, 5, .03), (7, 41, .03), (7, 5, float('nan')),
                                  (7, 5, 0), (7, 5, -.1), (7, 5, 25)])
def test_board_validation(args):
    with pytest.raises(ValueError):
        Board(*args)


def test_points_use_inner_corner_grid_and_meters():
    b = Board(7, 5, .03)
    np.testing.assert_allclose(b.points()[6], [.18, 0, 0])
    np.testing.assert_allclose(b.points()[7], [0, .03, 0])


def regular_corners():
    x, y = np.meshgrid(np.arange(7) * 30 + 100, np.arange(5) * 30 + 100)
    return np.stack((x, y), axis=-1).reshape(-1, 2).astype(float)


def test_selector_rejects_duplicate_reversed_blurred_and_small_views():
    s = Selector(Board(7, 5, .03))
    q = regular_corners()
    good = {'sharpness_px': 1., 'contrast': 180.}
    assert s.consider(q, good, (480, 640), 0.)[0]
    assert not s.consider(q, good, (480, 640), 1., force=True)[0]
    assert not s.consider(q[::-1], good, (480, 640), 2.)[0]
    assert not s.consider(q + 90, dict(good, sharpness_px=5), (480, 640), 3.)[0]
    assert not s.consider(q + 90, dict(good, contrast=5), (480, 640), 3.)[0]
    assert not s.consider(q * .1, good, (480, 640), 3.)[0]
    assert s.consider(q + [180, 120], good, (480, 640), 3.)[0]
    assert len(s.accepted) == 2


def test_square_rotations_are_duplicates():
    b = Board(5, 5, .03)
    q = regular_corners().reshape(5, 7, 2)[:, :5].reshape(-1, 2)
    s = Selector(b)
    metrics = {'sharpness_px': 1., 'contrast': 180.}
    assert s.consider(q, metrics, (480, 640), 0.)[0]
    for k in (1, 2, 3):
        rotated = np.rot90(q.reshape(5, 5, 2), k, axes=(0, 1)).reshape(-1, 2)
        assert not s.consider(rotated, metrics, (480, 640), 2. * k)[0]


def test_holdout_separates_time_blocks():
    views = [{'source_time_s': i * .7} for i in range(90)]
    train, held = split_views(views)
    assert not set(train) & set(held)
    assert len(train) + len(held) == 90
    assert not {int(views[i]['source_time_s'] // 3) for i in train} & {
        int(views[i]['source_time_s'] // 3) for i in held}
    assert (train, held) == split_views(views)
    with pytest.raises(ValueError, match='five'):
        split_views([{'source_time_s': i / 30} for i in range(80)])


def test_real_detection_lossless_save_and_overlay(rendered_boards, tmp_path):
    board, _, _, observations = rendered_boards
    path, expected = observations[0]
    frame = cv2.imread(str(path))
    before = frame.copy()
    session = Session(tmp_path / 'session', board, Selector(board), {'synthetic': True})
    q, reason, accepted = session.process(frame, 0., 0)
    assert accepted, reason
    assert min(np.mean(np.linalg.norm(q - expected, axis=1)),
               np.mean(np.linalg.norm(q[::-1] - expected, axis=1))) < .65
    np.testing.assert_array_equal(cv2.imread(str(session.path / session.data['views'][0]['image'])), before)
    preview = overlay(frame, (frame, q, reason, accepted, 0.), session)
    assert preview.shape == (490, 1280, 3)
    np.testing.assert_array_equal(frame, before)
    assert cv2.imwrite(str(tmp_path / 'synthetic-preview.png'), preview)
    if os.environ.get('VISION_TEST_GUI') == '1':
        cv2.imshow('calibration-integration-test', preview)
        cv2.waitKey(20)
        cv2.destroyAllWindows()
    with pytest.raises(ValueError, match='incomplete'):
        load_session(session.path)
    session.data['complete'] = True
    session.save()
    assert load_session(session.path)[1]['width'] == 640
    with pytest.raises(FileExistsError):
        Session(session.path, board, Selector(board), {})


def test_video_cli_extracts_unannotated_frames(rendered_boards, tmp_path):
    board, _, _, observations = rendered_boards
    video = tmp_path / 'input.avi'
    writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'FFV1'), 2., (640, 480))
    assert writer.isOpened()
    for path, _ in observations[:12]:
        writer.write(cv2.imread(str(path)))
    writer.release()
    session = tmp_path / 'session'
    result = main(['video', '--video', str(video), '--session', str(session), '--headless', '--capture-only',
                   '--board-cols', '7', '--board-rows', '5', '--square-size-m', '.03', '--interval', '.4'])
    assert result == 0
    data = load_session(session)[1]
    assert len(data['views']) >= 8
    for view in data['views']:
        original = cv2.imread(str(observations[view['source_frame_id']][0]))
        np.testing.assert_array_equal(cv2.imread(str(session / view['image'])), original)


def test_live_reader_records_losslessly_and_releases(monkeypatch, tmp_path):
    frames = [np.full((48, 64, 3), i * 30, np.uint8) for i in range(4)]
    class FakeCapture:
        def __init__(self, *args):
            self.index = 0
            self.released = False
        def isOpened(self): return True
        def set(self, *args): return True
        def get(self, *args): return 30.
        def getBackendName(self): return 'TEST'
        def read(self):
            if self.index == 4: return False, None
            value = frames[self.index]
            self.index += 1
            return True, value
        def release(self): self.released = True
    cap = FakeCapture()
    real_capture = cv2.VideoCapture
    monkeypatch.setattr(cv2, 'VideoCapture', lambda *args: cap)
    reader = LiveReader('0', (64, 48), 30., 'MJPG', tmp_path)
    reader.thread.join(5)
    reader.close()
    assert reader.frames == 4 and cap.released
    assert 'Camera read failed' in reader.error
    video = real_capture(str(tmp_path / 'raw.avi'))
    for frame in frames:
        ok, decoded = video.read()
        assert ok
        np.testing.assert_array_equal(decoded, frame)
    video.release()
    assert len((tmp_path / 'raw-timestamps.jsonl').read_text().splitlines()) == 4


def test_background_analyzer_never_mutates_frame(rendered_boards, tmp_path):
    b, _, _, observations = rendered_boards
    frame = cv2.imread(str(observations[0][0]))
    before = frame.copy()
    session = Session(tmp_path / 'session', b, Selector(b), {})
    analyzer = Analyzer(session)
    assert analyzer.submit(0, 0., frame)
    deadline = time.monotonic() + 10
    while analyzer.result is None and analyzer.error is None and time.monotonic() < deadline:
        time.sleep(.02)
    analyzer.close()
    assert analyzer.error is None and analyzer.result is not None
    np.testing.assert_array_equal(frame, before)


@pytest.mark.parametrize('transform', [np.zeros((4, 4)), np.diag([1, 1, -1, 1]), np.ones((3, 3))])
def test_invalid_surveys_rejected(transform):
    with pytest.raises(ValueError):
        validate_transform(transform)


@pytest.mark.parametrize('rpy', [[0, 0, 0], [10, 45, -20], [0, 90, 20], [0, -90, -10]])
def test_robot_rotation_round_trip(rpy):
    R = _rpy_rotation(rpy)
    np.testing.assert_allclose(_rpy_rotation(rpy_degrees(R)), R, atol=1e-9)


def test_mount_transform_directions_and_opencv_to_wpilib_conversion():
    board = Board(7, 5, .03)
    K = np.array([[600., 0, 320], [0, 600., 240], [0, 0, 1]])
    calibration = {'width': 640, 'height': 480, 'camera_matrix': K.tolist(), 'dist_coeffs': [0.] * 8}
    camera_T_board = np.eye(4)
    r = np.array([.3, -.4, .05])
    camera_T_board[:3, :3] = cv2.Rodrigues(r)[0]
    camera_T_board[:3, 3] = [-.1, -.08, .55]
    q = cv2.projectPoints(board.points(), r, camera_T_board[:3, 3], K, np.zeros(8))[0].reshape(-1, 2)
    robot_T_board = np.eye(4)
    robot_T_board[:3, :3] = _rpy_rotation([15, -10, 30])
    robot_T_board[:3, 3] = [1.2, -.3, .4]
    result = estimate_mount(calibration, board, q, robot_T_board)
    cv_from_nwu = np.eye(4)
    cv_from_nwu[:3, :3] = CV_TO_NWU.T
    expected = robot_T_board @ np.linalg.inv(camera_T_board) @ cv_from_nwu
    np.testing.assert_allclose(result['robot_T_camera_nwu'], expected, atol=1e-6)
    np.testing.assert_allclose(_rpy_rotation(result['robot_to_camera']['rotation_rpy_deg']), expected[:3, :3], atol=1e-6)
    assert result['physical_validation_required']


def test_spline_cannot_be_silently_exported_as_opencv():
    model = SimpleNamespace(intrinsics=lambda: ('LENSMODEL_SPLINED_STEREOGRAPHIC', np.zeros(40)))
    with pytest.raises(ValueError, match='exact'):
        export_opencv(None, model, {})


def mrcal_available():
    mrcal = pytest.importorskip('mrcal')
    if shutil.which('mrcal-calibrate-cameras') is None:
        pytest.skip('Actual mrcal solver command is not installed')
    return mrcal


def make_solver_session(tmp_path, rendered_boards):
    board, _, _, observations = rendered_boards
    session = Session(tmp_path / 'solver-session', board, Selector(board), {'synthetic': True})
    views = []
    for index, (path, q) in enumerate(observations):
        destination = session.path / 'frames' / path.name
        shutil.copyfile(path, destination)
        views.append({'image': 'frames/' + path.name, 'source_frame_id': index, 'source_time_s': index * 1.1,
                      'corners': q.tolist()})
    session.data.update(width=640, height=480, views=views, complete=True)
    session.save()
    return session.path


def test_real_mrcal_full_solve_validation_uncertainty_and_runtime_export(rendered_boards, tmp_path):
    mrcal = mrcal_available()
    session = make_solver_session(tmp_path, rendered_boards)
    args = SimpleNamespace(session=session, min_views=40, corner_detector='opencv-sb', timeout=300.,
                           fov_deg=60., no_spline=False, max_validation_rms=.6)
    result = calibrate(args)
    assert result in (0, 2)  # 2 is a retained candidate needing review, NOT a solver failure.
    latest = json.loads((session / 'latest-solve.json').read_text())
    directory = session / latest['directory']
    report = json.loads((directory / 'report.json').read_text())
    assert set(report['models']) == {'opencv8', 'spline'}
    for model in report['models'].values():
        assert model['holdout']['rms_px'] < .6
        assert model['uncertainty'] is not None, model.get('uncertainty_error')
    assert 'model_difference' in report, report.get('model_difference_error')
    exported = json.loads((directory / report['intrinsics_file']).read_text())
    K = np.array(exported['camera_matrix'])
    assert K[0, 0] == pytest.approx(600., rel=.035)
    assert K[1, 1] == pytest.approx(605., rel=.035)
    assert len(exported['dist_coeffs']) == 8 and exported['robot_mount_calibrated'] is False
    poses = json.loads((directory / 'board-poses.json').read_text())
    assert len(poses['poses']) == 60 and poses['robot_to_camera'] is None
    if Path('calibration-results').is_dir():
        shutil.copytree(directory, Path('calibration-results') / 'synthetic-solve', ignore=shutil.ignore_patterns('images'))
    # mrcal files themselves retain optimization data for uncertainty and analysis.
    model = mrcal.cameramodel(str(directory / report['models']['opencv8']['model']))
    assert model.optimization_inputs() is not None
