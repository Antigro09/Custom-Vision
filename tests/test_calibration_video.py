"""Timestamp replay and actual mrgingham detector regressions."""
import json
import shutil

import cv2
import numpy as np
import pytest

from custom_vision.calibration_session import Board, Selector, Session, detect_board


def test_video_clock_preserves_capture_time_and_rejects_misaligned_sidecar(tmp_path):
    from custom_vision.calibration_video import video_clock
    sidecar = tmp_path / 'raw-timestamps.jsonl'
    sidecar.write_text('\n'.join(json.dumps({'frame_id': i, 'elapsed_s': t})
                                for i, t in enumerate([.1, .7, 2.3])) + '\n')
    with video_clock(tmp_path / 'raw.avi') as (stamp, source):
        assert source == 'recorded_host_read_complete'
        assert [stamp(i, 30.) for i in range(3)] == [.1, .7, 2.3]
        with pytest.raises(ValueError, match='ended'):
            stamp(3, 30.)
    with video_clock(tmp_path / 'other.avi') as (stamp, source):
        assert source == 'video_frame_index_over_reported_fps'
        assert stamp(30, 30.) == 1.
    with video_clock(tmp_path / 'raw.avi') as (stamp, _):
        with pytest.raises(ValueError, match='aligned'):
            stamp(2, 30.)


def test_real_mrgingham_square_grid_path(tmp_path):
    from custom_vision.calibration_solver import final_observations
    if shutil.which('mrgingham') is None:
        pytest.skip('Actual mrgingham command is not installed')
    board = Board(6, 6, .03)
    frame = np.full((480, 640), 180, np.uint8)
    for y in range(7):
        for x in range(7):
            frame[100 + 40*y:140 + 40*y, 150 + 40*x:190 + 40*x] = 10 if (x+y)%2 else 240
    session = Session(tmp_path / 'square-session', board, Selector(board), {'synthetic': True})
    filename = session.path / 'frames' / 'frame000000.png'
    assert cv2.imwrite(str(filename), frame)
    corners, _ = detect_board(frame, board)
    assert corners is not None
    session.data.update(width=640, height=480, complete=True,
                        views=[{'image': 'frames/frame000000.png', 'source_time_s': 0., 'corners': corners.tolist()}])
    session.save()
    output = tmp_path / 'corners'
    output.mkdir()
    views, metadata = final_observations(session.path, session.data, 'auto', output, 30.)
    assert metadata['detector'] == 'mrgingham' and not metadata['rejected_images']
    assert len(views) == 1 and len(views[0]['corners']) == 36
    assert all(0 < w <= 1 for w in views[0]['weights'])
