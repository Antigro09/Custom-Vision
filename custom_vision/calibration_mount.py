"""Robot mount extrinsics from a separately surveyed board, with explicit axes.

A freehand chessboard video cannot determine where the camera sits on the robot.
The user must survey the board pose relative to the chosen robot origin. Ordinary
chessboards are symmetric: physically verify the corner numbering before export.
"""
from __future__ import annotations
import json
import math

import cv2
import numpy as np

from .calibration import load_calibration, validate_calibration
from .calibration_session import Board, detect_board, write_json
from .localization import CV_TO_NWU


def validate_transform(value):
    transform = np.asarray(value, np.float64)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError('robot_T_board must be a finite 4x4 rigid transformation in meters')
    rotation = transform[:3, :3]
    if (not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1., atol=1e-6)):
        raise ValueError('robot_T_board must contain a right-handed orthonormal rotation, not a reflection/scale')
    return transform


def rpy_degrees(rotation):
    pitch = math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0]))
    if abs(math.cos(pitch)) < 1e-8:
        roll, yaw = 0., math.atan2(-rotation[0, 1], rotation[1, 1])
    else:
        roll, yaw = math.atan2(rotation[2, 1], rotation[2, 2]), math.atan2(rotation[1, 0], rotation[0, 0])
    return np.degrees([roll, pitch, yaw]).tolist()


def estimate_mount(calibration, board, observed, robot_T_board, *, max_rms_px=.6):
    calibration = validate_calibration(calibration)
    robot_T_board = validate_transform(robot_T_board)
    observed = np.asarray(observed, np.float64)
    if observed.shape != (board.cols * board.rows, 2) or not np.isfinite(observed).all():
        raise ValueError('Need all board corners in the explicitly confirmed physical order')
    if np.any(observed < 0) or np.any(observed >= [calibration['width'], calibration['height']]):
        raise ValueError('Observed board corners lie outside the calibrated image')
    if not math.isfinite(max_rms_px) or max_rms_px <= 0:
        raise ValueError('RMS threshold must be finite and positive')
    matrix = np.asarray(calibration['camera_matrix'])
    distortion = np.asarray(calibration['dist_coeffs'])
    points = board.points()
    solved = cv2.solvePnPGeneric(points, observed, matrix, distortion, flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    for r, t in zip(solved[1], solved[2]):
        r, t = cv2.solvePnPRefineLM(points, observed, matrix, distortion, r.copy(), t.copy())
        rotation = cv2.Rodrigues(r)[0]
        camera_points = points @ rotation.T + t.reshape(3)
        if not np.isfinite(camera_points).all() or np.any(camera_points[:, 2] <= 0):
            continue
        predicted = cv2.projectPoints(points, r, t, matrix, distortion)[0].reshape(-1, 2)
        rms = float(np.sqrt(np.mean(np.sum((predicted - observed) ** 2, axis=1))))
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = rotation, t.ravel()
        candidates.append((rms, T))
    candidates.sort(key=lambda item: item[0])
    if not candidates or candidates[0][0] > max_rms_px:
        raise ValueError('Mount pose reprojection error is too high, or board lies behind camera')
    best_rms, camera_T_board = candidates[0]
    if len(candidates) > 1:
        other_rms, other = candidates[1]
        angle = np.linalg.norm(cv2.Rodrigues(camera_T_board[:3, :3].T @ other[:3, :3])[0])
        distinct = angle > math.radians(1.) or np.linalg.norm(camera_T_board[:3, 3] - other[:3, 3]) > .01
        if distinct and (other_rms - best_rms < .1 or best_rms / max(other_rms, 1e-12) > .8):
            raise ValueError('Ambiguous planar mount pose. Use a more oblique view or a better-surveyed board position.')
    robot_T_camera_cv = robot_T_board @ np.linalg.inv(camera_T_board)
    cv_from_nwu = np.eye(4)
    cv_from_nwu[:3, :3] = CV_TO_NWU.T
    robot_T_camera_nwu = robot_T_camera_cv @ cv_from_nwu
    return {'robot_to_camera': {'translation_m': robot_T_camera_nwu[:3, 3].tolist(),
                                'rotation_rpy_deg': rpy_degrees(robot_T_camera_nwu[:3, :3])},
            'robot_T_camera_nwu': robot_T_camera_nwu.tolist(),
            'robot_T_camera_cv': robot_T_camera_cv.tolist(),
            'camera_cv_T_board': camera_T_board.tolist(),
            'reprojection_rms_px': best_rms,
            'coordinate_convention': 'robot and runtime camera: +X forward,+Y left,+Z up; rotation Rz(yaw) Ry(pitch) Rx(roll)',
            'physical_validation_required': True,
            'warning': 'Survey/scale/board-order errors are not measured by reprojection RMS. Verify mount physically.'}


def run_mount(args):
    if args.output.exists():
        raise ValueError('Output already exists; choose a new filename instead of overwriting a measured mount')
    calibration = load_calibration(args.calibration)
    board = Board(**json.loads((args.session / 'session.json').read_text())['board'])
    frame = cv2.imread(str(args.image))
    if frame is None or frame.shape[:2] != (calibration['height'], calibration['width']):
        raise ValueError('Mount image is missing or its resolution differs from the calibration')
    corners, _ = detect_board(frame, board)
    if corners is None:
        raise ValueError('Complete chessboard not found in the surveyed mount image')
    if args.reverse_corners:
        corners = corners[::-1].copy()
    preview = frame.copy()
    cv2.drawChessboardCorners(preview, (board.cols, board.rows), corners.reshape(-1, 1, 2), True)
    for index, text in ((0, '0: SURVEY ORIGIN'), (board.cols - 1, '+X BOARD'),
                        ((board.rows - 1) * board.cols, '+Y BOARD')):
        p = tuple(np.rint(corners[index]).astype(int))
        cv2.circle(preview, p, 7, (0, 0, 255), 2)
        cv2.putText(preview, text, p, cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255), 2)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    preview_path = args.output.with_suffix('.corners.png')
    if not cv2.imwrite(str(preview_path), preview):
        raise OSError('Could not save mount corner-order preview')
    if not args.confirm_board_order:
        print(f'Inspect {preview_path}. Survey the SAME physical origin/+X/+Y corners.\n'
              'Then rerun with --confirm-board-order; use --reverse-corners for a 180-degree reversal.')
        return 2
    survey = json.loads(args.survey.read_text())
    result = estimate_mount(calibration, board, corners, survey['robot_T_board'], max_rms_px=args.max_rms_px)
    result.update(survey_file=str(args.survey), calibration_file=str(args.calibration), image=str(args.image))
    write_json(args.output, result)
    print(f'Saved {args.output}; copy only its robot_to_camera block into the pipeline after physical verification')
    return 0
