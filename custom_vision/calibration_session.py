"""Guided monocular calibration acquisition; never changes a running robot config.

The existing custom_vision.calibration image calibrator remains independent.
Lossless selected images, NOT annotated preview frames, are the calibration data.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import queue
import shutil
import subprocess
import sys
import threading
import time

import cv2
import numpy as np


@dataclass(frozen=True)
class Board:
    cols: int
    rows: int
    square_size_m: float

    def __post_init__(self):
        if any(isinstance(v, bool) or not isinstance(v, int) or not 3 <= v <= 40
               for v in (self.cols, self.rows)):
            raise ValueError('Board dimensions must be 3..40 INNER corners, not squares')
        if not math.isfinite(self.square_size_m) or not 0 < self.square_size_m < 1:
            raise ValueError('Measure square size in meters, between 0 and 1')

    def points(self):
        points = np.zeros((self.rows * self.cols, 3), np.float64)
        points[:, :2] = np.mgrid[:self.cols, :self.rows].T.reshape(-1, 2) * self.square_size_m
        return points


def write_json(path, value):
    path = Path(path)
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    os.replace(tmp, path)


def detect_board(frame, board):
    gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(
        gray, (board.cols, board.rows), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None, {'reason': 'Show the whole board; vary distance and tilt'}
    corners = corners.reshape(-1, 2)
    # Edge-transition width, not an image-wide sharpness score polluted by noise.
    sharpness, _ = cv2.estimateChessboardSharpness(gray, (board.cols, board.rows), corners)
    return corners, {'sharpness_px': float(sharpness[0]),
                     'contrast': float(sharpness[2] - sharpness[1])}


def grid_coverage(corners, width, height):
    cells = (np.asarray(corners) / [width, height] * [8, 6]).astype(int)
    cells = np.clip(cells, [0, 0], [7, 5])
    grid = np.zeros((6, 8), bool)
    grid[cells[:, 1], cells[:, 0]] = True
    return grid


class Selector:
    def __init__(self, board, *, max_views=180, min_interval=.7,
                 novelty=.025, max_sharpness=3., min_contrast=40.):
        self.board = board
        self.max_views, self.min_interval = max_views, min_interval
        self.novelty, self.max_sharpness, self.min_contrast = novelty, max_sharpness, min_contrast
        self.accepted = []
        self.last_stamp = -math.inf
        self.coverage = np.zeros((6, 8), bool)

    def consider(self, corners, metrics, shape, stamp, *, force=False):
        if corners is None:
            return False, metrics.get('reason', 'Board not found')
        h, w = shape[:2]
        corners = np.asarray(corners, np.float64)
        if corners.shape != (self.board.cols * self.board.rows, 2) or not np.isfinite(corners).all():
            return False, 'Invalid corner data'
        if not np.isfinite([metrics['sharpness_px'], metrics['contrast']]).all():
            return False, 'Invalid sharpness/contrast'
        if metrics['sharpness_px'] > self.max_sharpness:
            return False, 'Blur: hold still, improve light or shorten exposure'
        if metrics['contrast'] < self.min_contrast:
            return False, 'Low board contrast: improve even lighting'
        if np.any(corners < 2) or np.any(corners > [w - 3, h - 3]):
            return False, 'Keep every inner corner inside the image'
        lattice = corners.reshape(self.board.rows, self.board.cols, 2)
        spacing = np.concatenate((np.linalg.norm(np.diff(lattice, axis=0), axis=2).ravel(),
                                  np.linalg.norm(np.diff(lattice, axis=1), axis=2).ravel()))
        if np.min(spacing) < 8:
            return False, 'Board too small or too oblique (corners <8px apart)'
        if len(self.accepted) >= self.max_views:
            return False, 'View limit reached; video recording continues'
        if not force and stamp - self.last_stamp < self.min_interval:
            return False, 'Hold briefly; waiting for the sample interval'
        normalized = corners / [w, h]
        variants = [normalized, normalized[::-1]]
        if self.board.cols == self.board.rows:
            square = normalized.reshape(self.board.rows, self.board.cols, 2)
            variants += [np.rot90(square, k, axes=(0, 1)).reshape(-1, 2) for k in (1, 3)]
        distance = min((np.sqrt(np.mean(np.sum((v - old) ** 2, axis=1)))
                        for old in self.accepted for v in variants), default=math.inf)
        new_cells = grid_coverage(corners, w, h) & ~self.coverage
        # Even manual captures cannot insert nearly-identical frames or bad images.
        if distance < self.novelty and not new_cells.any():
            return False, 'Already covered: translate, tilt, or change board distance'
        self.accepted.append(normalized.copy())
        self.last_stamp = stamp
        self.coverage |= grid_coverage(corners, w, h)
        return True, f'Accepted view {len(self.accepted)}; cover edges and add tilted close-ups'


class Session:
    def __init__(self, path, board, selector, source):
        self.path = Path(path).resolve()
        self.path.mkdir(parents=True, exist_ok=False)  # never mix old/new calibrations
        (self.path / 'frames').mkdir()
        self.board, self.selector = board, selector
        self.data = {'schema_version': 1, 'board': asdict(board), 'source': source,
                     'created_utc': datetime.now(timezone.utc).isoformat(),
                     'width': None, 'height': None, 'views': [], 'complete': False,
                     'corner_detector_preview': 'opencv_findChessboardCornersSB',
                     'quality_thresholds': {'max_sharpness_px': selector.max_sharpness,
                                            'min_contrast': selector.min_contrast,
                                            'novelty_normalized_rms': selector.novelty},
                     'extrinsics_reference': 'board poses only; robot mount not determined'}
        self.save()

    def save(self):
        self.data['coverage_grid'] = self.selector.coverage.astype(int).tolist()
        self.data['corner_cell_coverage_fraction'] = float(self.selector.coverage.mean())
        write_json(self.path / 'session.json', self.data)

    def process(self, frame, stamp, frame_id, force=False):
        h, w = frame.shape[:2]
        if self.data['width'] is not None and (w, h) != (self.data['width'], self.data['height']):
            raise ValueError('Capture resolution changed mid-session. Start a new session.')
        self.data.update(width=w, height=h)
        corners, metrics = detect_board(frame, self.board)
        accepted, reason = self.selector.consider(corners, metrics, frame.shape, stamp, force=force)
        if accepted:
            relative = f'frames/frame{len(self.data["views"]):06d}.png'
            if not cv2.imwrite(str(self.path / relative), frame, [cv2.IMWRITE_PNG_COMPRESSION, 1]):
                raise OSError('Could not save lossless calibration image')
            self.data['views'].append({'image': relative, 'source_frame_id': int(frame_id),
                                       'source_time_s': float(stamp), 'corners': corners.tolist(), **metrics})
            self.save()
        return corners, reason, accepted


class LiveReader:
    """Only this thread owns camera reads, recording and release. One latest slot.

    Lossless FFV1 recording happens before publishing a preview/sample. Recording
    may reduce delivered FPS; host-read timestamps make that visible, not hidden.
    No promise of capturing every hardware exposure or hardware timestamps.
    """
    def __init__(self, camera, size, fps, fourcc, session, *, record=True, max_seconds=0):
        self.camera, self.size, self.fps, self.fourcc = camera, size, fps, fourcc
        self.session, self.record, self.max_seconds = session, record, max_seconds
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.latest = None
        self.error = None
        self.info = {}
        self.frames = 0
        self.thread = threading.Thread(target=self._run, daemon=True, name='calibration-camera')
        self.thread.start()

    def _run(self):
        cap = writer = stamps = None
        start = time.monotonic()
        try:
            source = int(self.camera) if str(self.camera).isdigit() else self.camera
            backend = cv2.CAP_V4L2 if sys.platform.startswith('linux') else cv2.CAP_ANY
            cap = cv2.VideoCapture(source, backend)
            if not cap.isOpened():
                raise RuntimeError('Camera unavailable. Check USB, permissions and other vision services.')
            for key, value in ((cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc)),
                               (cv2.CAP_PROP_FRAME_WIDTH, self.size[0]),
                               (cv2.CAP_PROP_FRAME_HEIGHT, self.size[1]), (cv2.CAP_PROP_FPS, self.fps),
                               (cv2.CAP_PROP_BUFFERSIZE, 1)):
                cap.set(key, value)
            self.info = {'backend': cap.getBackendName(), 'requested_fps': self.fps,
                         'reported_fps': cap.get(cv2.CAP_PROP_FPS),
                         'exposure': cap.get(cv2.CAP_PROP_EXPOSURE), 'gain': cap.get(cv2.CAP_PROP_GAIN),
                         'auto_exposure': cap.get(cv2.CAP_PROP_AUTO_EXPOSURE),
                         'timestamp_source': 'host_read_complete_not_exposure'}
            if self.record:
                stamps = (self.session / 'raw-timestamps.jsonl').open('w')
            while not self.stop.is_set():
                ok, frame = cap.read()
                now = time.monotonic()
                if not ok or frame is None:
                    raise RuntimeError('Camera read failed; recorded data were retained')
                if (frame.shape[1], frame.shape[0]) != tuple(self.size):
                    raise ValueError(f'Camera delivered {frame.shape[1]}x{frame.shape[0]}, requested {self.size}')
                if self.record:
                    if writer is None:
                        writer = cv2.VideoWriter(str(self.session / 'raw.avi'),
                                                 cv2.VideoWriter_fourcc(*'FFV1'), self.fps,
                                                 self.size, frame.ndim == 3)
                        if not writer.isOpened():
                            raise RuntimeError('FFV1 recording unavailable; retry --no-record (PNG samples still saved)')
                    writer.write(frame)
                    stamps.write(json.dumps({'frame_id': self.frames, 'host_monotonic_s': now,
                                              'elapsed_s': now - start}) + '\n')
                    if self.frames % 30 == 0:
                        stamps.flush()
                with self.lock:
                    self.latest = (self.frames, now - start, frame)
                self.frames += 1
                if self.max_seconds and now - start >= self.max_seconds:
                    break
        except Exception as exc:
            self.error = str(exc)
        finally:
            if writer is not None:
                writer.release()
            if stamps is not None:
                stamps.close()
            if cap is not None:
                cap.release()
            self.stop.set()

    def close(self):
        self.stop.set()
        self.thread.join(3.)
        if self.thread.is_alive():
            raise RuntimeError('Camera read is blocked; exit this process before reopening the camera')


class Analyzer:
    """Expensive corner detection never runs in the preview or capture thread."""
    def __init__(self, session):
        self.session = session
        self.queue = queue.Queue(maxsize=1)
        self.stop = threading.Event()
        self.result = None
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True, name='calibration-corners')
        self.thread.start()

    def submit(self, frame_id, stamp, frame, force=False):
        try:
            self.queue.put_nowait((frame_id, stamp, frame, force))
            return True
        except queue.Full:
            return False

    def _run(self):
        try:
            while not self.stop.is_set():
                try:
                    frame_id, stamp, frame, force = self.queue.get(timeout=.1)
                except queue.Empty:
                    continue
                try:
                    corners, reason, accepted = self.session.process(frame, stamp, frame_id, force)
                    self.result = (frame.copy(), corners, reason, accepted, stamp)
                finally:
                    self.queue.task_done()
        except Exception as exc:
            self.error = str(exc)
            self.stop.set()

    def close(self):
        self.stop.set()
        self.thread.join(30.)
        if self.thread.is_alive():
            raise RuntimeError('Corner detector did not stop; session remains incomplete')


def overlay(live, result, session, *, auto=True):
    """Two panes prevent old detected corners being drawn on a newer live image."""
    def pane(frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR) if frame.ndim == 2 else frame.copy()
        return cv2.resize(frame, (640, 400), interpolation=cv2.INTER_AREA)
    left = pane(live)
    right = np.zeros_like(left)
    reason = 'Finding a complete board...'
    if result is not None:
        frame, corners, reason, accepted, _ = result
        marked = frame.copy()
        if marked.ndim == 2:
            marked = cv2.cvtColor(marked, cv2.COLOR_GRAY2BGR)
        if corners is not None:
            cv2.drawChessboardCorners(marked, (session.board.cols, session.board.rows),
                                      corners.astype(np.float32).reshape(-1, 1, 2), True)
        right = pane(marked)
    for row, col in np.argwhere(session.selector.coverage):
        cv2.rectangle(left, (col * 80, row * 400 // 6), ((col + 1) * 80 - 1, (row + 1) * 400 // 6 - 1),
                      (50, 180, 70), 1)
    for image, label in ((left, 'LIVE / occupied corner cells'), (right, 'LAST ANALYZED FRAME / exact corner overlay')):
        cv2.rectangle(image, (0, 0), (640, 26), (18, 18, 18), -1)
        cv2.putText(image, label, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, .47, (255, 255, 255), 1)
    canvas = np.zeros((490, 1280, 3), np.uint8)
    canvas[:400] = np.concatenate((left, right), axis=1)
    lines = [f'Views: {len(session.data["views"])} | coverage: {session.selector.coverage.mean():.0%} '
             f'| AUTO {"ON" if auto else "OFF"} | quality heuristic, not calibration accuracy',
             reason,
             'A auto/manual | SPACE request sample | Q finish | ESC save without solving. '
             'Keep focus fixed; include tilted close-ups, edges, corners.']
    for y, line in zip((422, 448, 476), lines):
        cv2.putText(canvas, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, .51, (240, 240, 240), 1)
    return canvas


def display_available():
    return not sys.platform.startswith('linux') or bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))


def capture_session(args):
    board = Board(args.board_cols, args.board_rows, args.square_size_m)
    if not args.headless and not display_available():
        raise RuntimeError('No graphical desktop detected. Attach a monitor or pass --headless --seconds N.')
    if not args.headless:
        probe = subprocess.run([sys.executable, '-c',
                                'import cv2; cv2.namedWindow("Calibration check"); cv2.destroyAllWindows()'],
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, timeout=10)
        if probe.returncode:
            raise RuntimeError('OpenCV GUI cannot open a window. Use a working desktop or --headless.')
    if args.headless and args.camera is not None and args.seconds <= 0:
        raise ValueError('Headless camera capture requires --seconds')
    selector = Selector(board, max_views=args.max_views, min_interval=args.interval,
                        novelty=args.novelty, max_sharpness=args.max_sharpness_px)
    session = Session(args.session, board, selector, {'camera': args.camera, 'video': args.video})
    reader = analyzer = cap = None
    clock_context = None
    canceled = False
    try:
        if args.video:
            cap = cv2.VideoCapture(str(Path(args.video).expanduser()))
            if not cap.isOpened():
                raise ValueError(f'Cannot open video: {args.video}')
            fps = float(cap.get(cv2.CAP_PROP_FPS))
            if not math.isfinite(fps) or fps <= 0:
                raise ValueError('Video has no usable FPS/timebase. Remux it with correct timestamps first.')
            from .calibration_video import video_clock
            clock_context = video_clock(args.video, getattr(args, 'timestamps', None))
            timestamp_at, timestamp_source = clock_context.__enter__()
            frame_id = 0
            next_stamp = 0.
            auto = True
            while True:
                ok, frame = cap.read()
                if not ok:
                    break
                stamp = timestamp_at(frame_id, fps)
                if args.seconds and stamp > args.seconds:
                    break
                if stamp >= next_stamp:
                    corners, reason, accepted = session.process(frame, stamp, frame_id)
                    next_stamp = stamp + args.interval
                    if not args.headless:
                        cv2.imshow('Custom Vision Calibration', overlay(frame, (frame, corners, reason, accepted, stamp), session))
                        key = cv2.waitKey(1) & 255
                        if key in (27, ord('q')):
                            canceled = key == 27
                            break
                frame_id += 1
            session.data['acquisition'] = {'video_frames_read': frame_id, 'reported_fps': fps,
                                          'timestamp_source': timestamp_source}
        else:
            reader = LiveReader(args.camera, (args.width, args.height), args.fps, args.fourcc,
                                session.path, record=not args.no_record, max_seconds=args.seconds)
            analyzer = Analyzer(session)
            auto, force, last_id, next_stamp = True, False, -1, 0.
            while True:
                with reader.lock:
                    latest = reader.latest
                if latest is not None:
                    frame_id, stamp, frame = latest
                    if frame_id != last_id and ((auto and stamp >= next_stamp) or force):
                        if analyzer.submit(frame_id, stamp, frame, force):
                            last_id, next_stamp, force = frame_id, stamp + args.interval, False
                    if not args.headless:
                        cv2.imshow('Custom Vision Calibration', overlay(frame, analyzer.result, session, auto=auto))
                key = (cv2.waitKey(15) & 255) if not args.headless else -1
                if args.headless:
                    time.sleep(.02)
                if key == ord('a'):
                    auto = not auto
                elif key == 32:
                    force = True
                elif key in (ord('q'), 27):
                    canceled = key == 27
                    break
                if analyzer.error:
                    raise RuntimeError(analyzer.error)
                if reader.stop.is_set():
                    break
            reader.close()
            analyzer.close()
            if reader.error or analyzer.error:
                raise RuntimeError(reader.error or analyzer.error)
            session.data['acquisition'] = dict(reader.info, frames_read=reader.frames,
                                               lossless_recording=not args.no_record)
        session.data['complete'] = True
    finally:
        if clock_context is not None:
            clock_context.__exit__(None, None, None)
        if reader is not None:
            reader.close()
        if analyzer is not None:
            analyzer.close()
        if cap is not None:
            cap.release()
        if not args.headless:
            cv2.destroyAllWindows()
        session.save()
    print(f'Saved {len(session.data["views"])} selected lossless frames to {session.path}')
    return session.path, canceled


def load_session(path):
    path = Path(path).resolve()
    data = json.loads((path / 'session.json').read_text())
    Board(**data['board'])
    if not data.get('complete'):
        raise ValueError('Session is incomplete; inspect acquisition errors before calibrating')
    for field in ('width', 'height'):
        if not isinstance(data.get(field), int) or data[field] <= 0:
            raise ValueError('Session has no valid image size')
    for view in data['views']:
        image = (path / view['image']).resolve()
        if not image.is_relative_to(path) or not image.is_file():
            raise ValueError('Missing or unsafe session image path')
        q = np.asarray(view['corners'], np.float64)
        if q.shape != (data['board']['cols'] * data['board']['rows'], 2) or not np.isfinite(q).all():
            raise ValueError('Malformed session corner observations')
    return path, data


def run_solver(args):
    session, _ = load_session(args.session)
    worker = Path(__file__).resolve().with_name('calibration_solver.py')
    command = [args.mrcal_python, str(worker), '--session', str(session),
               '--min-views', str(args.min_views), '--fov-deg', str(args.fov_deg),
               '--corner-detector', args.corner_detector, '--max-validation-rms', str(args.max_validation_rms),
               '--timeout', str(args.timeout)]
    if args.no_spline:
        command.append('--no-spline')
    completed = subprocess.run(command, check=False)
    return completed.returncode


def add_solve_arguments(parser):
    parser.add_argument('--mrcal-python', default='/usr/bin/python3', help='Isolated system Python with mrcal installed')
    parser.add_argument('--min-views', type=int, default=40)
    parser.add_argument('--fov-deg', type=float, default=81., help='Initial horizontal FOV estimate, not a calibration')
    parser.add_argument('--corner-detector', choices=('auto', 'opencv-sb', 'mrgingham'), default='auto')
    parser.add_argument('--no-spline', action='store_true', help='Skip the richer reference lens model; validation still runs')
    parser.add_argument('--max-validation-rms', type=float, default=.6, help='Heuristic holdout RMS limit in pixels')
    parser.add_argument('--timeout', type=float, default=1800., help='Timeout per external solver stage in seconds')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    for name in ('capture', 'video'):
        p = sub.add_parser(name, help='Live camera capture' if name == 'capture' else 'Extract from a video with visual playback')
        p.add_argument('--session', type=Path, default=Path('data/calibration') / datetime.now().strftime('%Y%m%d-%H%M%S'))
        for key in ('cols', 'rows'):
            p.add_argument('--board-' + key, type=int, required=True, help='INNER corner count, not square count')
        p.add_argument('--square-size-m', type=float, required=True)
        p.add_argument('--headless', action='store_true')
        p.add_argument('--seconds', type=float, default=0.)
        p.add_argument('--max-views', type=int, default=180)
        p.add_argument('--interval', type=float, default=.7)
        p.add_argument('--novelty', type=float, default=.025)
        p.add_argument('--max-sharpness-px', type=float, default=3.)
        p.add_argument('--capture-only', action='store_true', help='Keep the session without starting the mrcal solve')
        if name == 'capture':
            p.add_argument('--camera', default='0')
            p.add_argument('--width', type=int, default=1280)
            p.add_argument('--height', type=int, default=800)
            p.add_argument('--fps', type=float, default=30.)
            p.add_argument('--fourcc', default='MJPG')
            p.add_argument('--no-record', action='store_true', help='Do not write raw.avi; selected PNGs are still saved')
            p.set_defaults(video=None)
        else:
            p.add_argument('--video', required=True)
            p.add_argument('--timestamps', type=Path, help='Optional raw-timestamps.jsonl; auto-detected beside raw.avi')
            p.set_defaults(camera=None)
        add_solve_arguments(p)
    p = sub.add_parser('solve', help='Re-run mrcal on a saved session without a camera')
    p.add_argument('--session', type=Path, required=True)
    add_solve_arguments(p)
    p = sub.add_parser('mount', help='Estimate robot mount using a surveyed board pose; NOT inferred from a freehand video')
    p.add_argument('--calibration', type=Path, required=True)
    p.add_argument('--session', type=Path, required=True, help='Uses board dimensions from this session')
    p.add_argument('--image', type=Path, required=True, help='New image of the physically surveyed, stationary board')
    p.add_argument('--survey', type=Path, required=True, help='JSON containing measured robot_T_board (4x4)')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--confirm-board-order', action='store_true', help='Confirm corner 0/+X/+Y shown in the saved preview')
    p.add_argument('--reverse-corners', action='store_true')
    p.add_argument('--max-rms-px', type=float, default=.6)
    p = sub.add_parser('doctor', help='Check local dependencies; no camera changes')
    p.add_argument('--mrcal-python', default='/usr/bin/python3')
    args = parser.parse_args(argv)
    try:
        cv2.setNumThreads(1)
        if args.command == 'doctor':
            print('UI Python:', sys.executable, '| OpenCV:', cv2.__version__)
            print('mrcal-calibrate-cameras:', shutil.which('mrcal-calibrate-cameras'))
            print('mrgingham:', shutil.which('mrgingham'))
            return subprocess.run([args.mrcal_python, '-c',
                                   'import mrcal, cv2, scipy; print("mrcal:", getattr(mrcal, "__version__", "distro-package"), "OpenCV:", cv2.__version__)']).returncode
        if args.command == 'mount':
            from .calibration_mount import run_mount
            return run_mount(args)
        if not 10 <= args.min_views <= 1000 or not math.isfinite(args.fov_deg) or not 20 <= args.fov_deg <= 170:
            raise ValueError('Require min-views 10..1000 and initial FOV 20..170 degrees')
        if not math.isfinite(args.timeout) or args.timeout <= 0 or not math.isfinite(args.max_validation_rms) or args.max_validation_rms <= 0:
            raise ValueError('Timeout and validation RMS limit must be finite and positive')
        if args.command in ('capture', 'video'):
            if (not 10 <= args.max_views <= 1000 or not math.isfinite(args.interval) or args.interval <= 0
                    or not math.isfinite(args.seconds) or args.seconds < 0
                    or not math.isfinite(args.novelty) or args.novelty < 0
                    or not math.isfinite(args.max_sharpness_px) or args.max_sharpness_px <= 0):
                raise ValueError('Invalid acquisition limits')
            if args.command == 'capture' and (args.width <= 0 or args.height <= 0 or len(args.fourcc) != 4
                                              or not math.isfinite(args.fps) or args.fps <= 0):
                raise ValueError('Invalid camera mode')
            args.session, canceled = capture_session(args)
            if args.capture_only or canceled:
                return 0
        return run_solver(args)
    except (ValueError, OSError, RuntimeError, cv2.error) as exc:
        print(f'Calibration: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('Interrupted; saved session files were retained', file=sys.stderr)
        return 130
