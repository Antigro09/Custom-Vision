"""Isolated mrcal worker. Execute with distro Python, not the Jetson GPU venv.

Actual mrcal solves, time-block held-out lens validation, projection uncertainty,
and exact OPENCV8 export. A spline model is NEVER relabeled as OpenCV distortion.
"""
from __future__ import annotations
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from custom_vision.calibration import validate_calibration
from custom_vision.calibration_session import Board, grid_coverage, load_session, write_json


def split_views(views, seed=1086):
    """Hold out whole 3-second blocks, not neighboring near-duplicate frames.

    This mitigates temporal leakage, but a second independent capture is stronger.
    """
    groups = {}
    for index, view in enumerate(views):
        stamp = float(view['source_time_s'])
        if not math.isfinite(stamp) or stamp < 0:
            raise ValueError('Invalid source timestamp')
        groups.setdefault(int(stamp // 3), []).append(index)
    if len(groups) < 5:
        raise ValueError('Need at least five separate 3-second time blocks; record more diverse views')
    order = list(groups)
    np.random.default_rng(seed).shuffle(order)
    held = []
    for key in order:
        if len(held) >= max(6, math.ceil(len(views) * .2)):
            break
        held += groups[key]
    training = sorted(set(range(len(views))) - set(held))
    if len(training) < 10:
        raise ValueError('Too few training views after time-block holdout; collect a longer sequence')
    return training, sorted(held)


def run_command(command, cwd, logfile, timeout):
    write_json(Path(logfile).with_suffix('.command.json'), {'argv': command, 'cwd': str(cwd)})
    with Path(logfile).open('w') as log:
        try:
            result = subprocess.run(command, cwd=cwd, stdout=log, stderr=subprocess.STDOUT,
                                    timeout=timeout, check=False)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f'Solver timed out; inspect {logfile}') from exc
    if result.returncode:
        raise RuntimeError(f'Command failed ({result.returncode}); inspect {logfile}')


def final_observations(session, data, detector, output, timeout):
    board = Board(**data['board'])
    if detector == 'auto':
        detector = 'mrgingham' if board.cols == board.rows and shutil.which('mrgingham') else 'opencv-sb'
    selected = []
    rejected = []
    if detector == 'mrgingham':
        if board.cols != board.rows:
            raise ValueError('mrgingham needs a square grid; use --corner-detector opencv-sb for rectangular boards')
        executable = shutil.which('mrgingham')
        if executable is None:
            raise RuntimeError('mrgingham is not installed; use the calibration installer')
        command = [executable, '--jobs', '2', '--gridn', str(board.cols), 'frames/frame*.png']
        with (output / 'mrgingham.vnl').open('w') as stdout, (output / 'mrgingham.log').open('w') as stderr:
            result = subprocess.run(command, cwd=session, stdout=stdout, stderr=stderr,
                                    timeout=timeout, check=False)
        if result.returncode:
            raise RuntimeError('mrgingham failed; inspect mrgingham.log')
        rows = {}
        for line in (output / 'mrgingham.vnl').read_text().splitlines():
            if not line.strip() or line.lstrip().startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 3 or '-' in parts[1:]:
                continue
            level = float(parts[3]) if len(parts) > 3 else 0.
            if level < 0:
                continue
            rows.setdefault(parts[0], []).append([float(parts[1]), float(parts[2]), 2. ** -level])
        for view in data['views']:
            points = rows.get(view['image'], [])
            if len(points) != board.cols * board.rows:
                rejected.append(view['image'])
                continue
            item = dict(view, corners=np.asarray(points)[:, :2].tolist(), weights=np.asarray(points)[:, 2].tolist())
            selected.append(item)
    else:
        for view in data['views']:
            image = cv2.imread(str(session / view['image']), cv2.IMREAD_GRAYSCALE)
            if image is None or image.shape != (data['height'], data['width']):
                raise ValueError('Missing image or calibration resolution mismatch')
            found, q = cv2.findChessboardCornersSB(image, (board.cols, board.rows),
                flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
            if not found:
                rejected.append(view['image'])
                continue
            selected.append(dict(view, corners=q.reshape(-1, 2).tolist(), weights=[1.] * len(q)))
    for view in selected:
        q = np.asarray(view['corners'])
        if not np.isfinite(q).all() or np.any(q < 0) or np.any(q >= [data['width'], data['height']]):
            raise ValueError('Final corner detector returned invalid pixel coordinates')
    return selected, {'detector': detector, 'rejected_images': rejected}


def solve_model(mrcal, session, data, views, indices, lensmodel, directory, focal, timeout):
    directory.mkdir(parents=True)
    (directory / 'images').mkdir()
    with (directory / 'corners.vnl').open('w') as stream:
        stream.write('# filename x y weight\n')
        for index in indices:
            view = views[index]
            name = f'images/frame{index:06d}.png'
            (directory / name).symlink_to(session / view['image'])
            for q, weight in zip(view['corners'], view['weights']):
                stream.write(f'{name} {q[0]:.12g} {q[1]:.12g} {weight:.12g}\n')
    command = [shutil.which('mrcal-calibrate-cameras'), '--corners-cache', 'corners.vnl',
               '--corners-cache-has-weights', '--lensmodel', lensmodel, '--focal', str(focal),
               '--object-width-n', str(data['board']['cols']), '--object-height-n', str(data['board']['rows']),
               '--object-spacing', str(data['board']['square_size_m']), '--imagersize', str(data['width']),
               str(data['height']), '--outdir', '.', 'images/frame*.png']
    if command[0] is None:
        raise RuntimeError('mrcal-calibrate-cameras is missing. Run scripts/install_calibration.sh')
    print(f'mrcal: fitting {lensmodel} to {len(indices)} views ({directory.name})', flush=True)
    run_command(command, directory, directory / 'solver.log', timeout)
    models = list(directory.glob('*.cameramodel'))
    if len(models) != 1:
        raise RuntimeError(f'Expected exactly one monocular .cameramodel in {directory}')
    model = mrcal.cameramodel(str(models[0]))
    return model, models[0]


def pose_residuals(mrcal, model, points, observed):
    from scipy.optimize import least_squares
    rays = mrcal.unproject(observed, *model.intrinsics(), normalize=True)
    if not np.isfinite(rays).all() or np.any(rays[:, 2] <= 0):
        raise ValueError('Lens model has invalid/backward rays within the observed field of view')
    normalized = np.ascontiguousarray(rays[:, :2] / rays[:, 2:3])
    ok, r, t = cv2.solvePnP(points, normalized, np.eye(3), None, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise ValueError('Could not initialize held-out board pose')
    def residual(rt):
        camera_points = points @ cv2.Rodrigues(rt[:3])[0].T + rt[3:]
        return (mrcal.project(camera_points, *model.intrinsics()) - observed).ravel()
    result = least_squares(residual, np.r_[r.ravel(), t.ravel()], method='lm', max_nfev=150)
    camera_points = points @ cv2.Rodrigues(result.x[:3])[0].T + result.x[3:]
    if not result.success or np.any(camera_points[:, 2] <= 0) or not np.isfinite(result.fun).all():
        raise ValueError('Held-out pose solve failed or lies behind the camera')
    return result.fun.reshape(-1, 2)


def validate_holdout(mrcal, model, views, indices):
    points = mrcal.ref_calibration_object(**model.optimization_inputs()).reshape(-1, 3)
    errors, records = [], []
    for index in indices:
        observed = np.asarray(views[index]['corners'], np.float64)
        residual = pose_residuals(mrcal, model, points, observed)
        magnitude = np.linalg.norm(residual, axis=1)
        errors.extend(magnitude.tolist())
        records.append({'image': views[index]['image'], 'rms_px': float(np.sqrt(np.mean(magnitude ** 2)))})
    return {'views': records, 'rms_px': float(np.sqrt(np.mean(np.square(errors)))),
            'p95_corner_error_px': float(np.percentile(errors, 95)),
            'note': 'Intrinsics and board warp fixed from training; six pose variables fitted per unseen view; no holdout outlier removal'}


def save_heatmap(path, values, title):
    values = np.asarray(values, float)
    good = np.isfinite(values)
    limit = max(.1, float(np.percentile(values[good], 95))) if good.any() else 1.
    mapped = np.nan_to_num(values, nan=limit, posinf=limit, neginf=limit)
    image = cv2.applyColorMap(np.uint8(np.clip(mapped / limit, 0, 1) * 255), cv2.COLORMAP_TURBO)
    image[~good] = (180, 180, 180)
    image = cv2.resize(image, (800, 500), interpolation=cv2.INTER_NEAREST)
    canvas = np.zeros((560, 800, 3), np.uint8)
    canvas[:500] = image
    cv2.putText(canvas, title, (8, 524), cv2.FONT_HERSHEY_SIMPLEX, .52, (255, 255, 255), 1)
    cv2.putText(canvas, f'Blue=0, red>={limit:.3f}px; gray=invalid. Pixel-space grid.', (8, 548),
                cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
    if not cv2.imwrite(str(path), canvas):
        raise OSError(f'Could not write {path}')


def uncertainty_report(mrcal, model, path):
    w, h = model.imagersize()
    x, y = np.meshgrid(np.linspace(0, w - 1, 24), np.linspace(0, h - 1, 16))
    q = np.stack((x, y), axis=-1)
    rays = mrcal.unproject(q, *model.intrinsics(), normalize=True)
    uncertainty = mrcal.projection_uncertainty(rays, model, atinfinity=True, what='worstdirection-stdev')
    save_heatmap(path, uncertainty, 'Projection uncertainty at infinity (1 sigma; sampling error only)')
    finite = np.isfinite(uncertainty)
    if not finite.all():
        raise ValueError('Projection uncertainty is undefined in part of the image')
    return {'median_px': float(np.median(uncertainty)), 'p95_px': float(np.percentile(uncertainty, 95)),
            'maximum_px': float(np.max(uncertainty)), 'grid_px': q.tolist(), 'values_px': uncertainty.tolist(),
            'note': 'Sampling uncertainty, not lens-model bias, board survey error, or a physical accuracy certificate'}


def export_opencv(mrcal, model, data):
    lensmodel, intrinsics = model.intrinsics()
    if lensmodel != 'LENSMODEL_OPENCV8' or len(intrinsics) != 12:
        raise ValueError('Only exact LENSMODEL_OPENCV8 export is allowed; spline conversion would be lossy')
    fx, fy, cx, cy = intrinsics[:4]
    result = validate_calibration({'width': data['width'], 'height': data['height'],
              'camera_matrix': [[float(fx), 0., float(cx)], [0., float(fy), float(cy)], [0., 0., 1.]],
              'dist_coeffs': intrinsics[4:].tolist(), 'distortion_model': 'opencv_pinhole',
              'calibrator': 'mrcal', 'lensmodel': lensmodel,
              'board': data['board'], 'robot_mount_calibrated': False})
    x, y = np.meshgrid(np.linspace(-.7, .7, 9), np.linspace(-.45, .45, 7))
    points = np.stack((x, y, np.ones_like(x)), axis=-1).reshape(-1, 3)
    expected = mrcal.project(points, *model.intrinsics())
    actual = cv2.projectPoints(points, np.zeros(3), np.zeros(3), np.asarray(result['camera_matrix']),
                               np.asarray(result['dist_coeffs']))[0].reshape(-1, 2)
    if not np.allclose(actual, expected, atol=1e-7, rtol=1e-10):
        raise ValueError('mrcal/OpenCV projection contract mismatch')
    return result


def board_poses(model, indices, views):
    inputs = model.optimization_inputs()
    frames = inputs['frames_rt_toref']
    mapping = inputs['indices_frame_camintrinsics_camextrinsics']
    records = []
    for observation_index, (frame_index, camera_index, extrinsics_index) in enumerate(mapping):
        if camera_index != 0 or extrinsics_index != -1:
            raise ValueError('Expected monocular reference camera; cannot label multi-camera poses as camera poses')
        rt = frames[frame_index]
        transform = np.eye(4)
        transform[:3, :3] = cv2.Rodrigues(rt[:3])[0]
        transform[:3, 3] = rt[3:]
        records.append({'image': views[indices[observation_index]]['image'],
                         'camera_cv_T_board': transform.tolist()})
    return {'convention': 'p_camera_cv = R @ p_board + t; camera +x right,+y down,+z forward; meters',
            'robot_to_camera': None, 'warning': 'Board-relative poses are NOT robot mount extrinsics',
            'poses': records, 'calobject_warp': np.asarray(inputs.get('calobject_warp', [0., 0.])).tolist()}


def calibrate(args):
    import mrcal
    session, data = load_session(args.session)
    output = session / ('solve-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    output.mkdir()
    report = {'status': 'in_progress', 'mrcal_version': str(getattr(mrcal, '__version__', 'distro-package-see-dpkg')),
              'source_session': str(session), 'robot_mount_calibrated': False, 'issues': [], 'warnings': [],
              'models': {}, 'synthetic_accuracy_claim': False}
    try:
        views, corner_info = final_observations(session, data, args.corner_detector, output, args.timeout)
        report.update(corner_detection=corner_info, accepted_views=len(views))
        if len(views) < args.min_views:
            raise ValueError(f'Only {len(views)} final usable views; require {args.min_views}. Capture diverse sharp views.')
        report['image_sha256'] = {v['image']: hashlib.sha256((session / v['image']).read_bytes()).hexdigest() for v in views}
        train, held = split_views(views)
        report['validation'] = {'training_images': [views[i]['image'] for i in train],
                                'held_out_images': [views[i]['image'] for i in held],
                                'method': 'deterministic 3-second-block holdout; not independent recapture'}
        coverage = np.zeros((6, 8), bool)
        for v in views:
            coverage |= grid_coverage(v['corners'], data['width'], data['height'])
        report['coverage_fraction'] = float(coverage.mean())
        if coverage.mean() < .70:
            report['issues'].append('Less than 70% corner-cell coverage; include image edges and corners')
        # The CLI initializes a stereographic model even when fitting OPENCV8.
        focal = data['width'] / (4 * math.tan(math.radians(args.fov_deg) / 4))
        lensmodels = {'opencv8': 'LENSMODEL_OPENCV8'}
        if not args.no_spline:
            # A moderate grid to compare lens-model bias; not a universal optimum.
            ny = max(6, round(12 * data['height'] / data['width']))
            lensmodels['spline'] = f'LENSMODEL_SPLINED_STEREOGRAPHIC_order=3_Nx=12_Ny={ny}_fov_x_deg={args.fov_deg:g}'
        else:
            report['warnings'].append('Rich spline reference not fitted; parametric lens bias may be hidden')
        fitted = {}
        for name, lensmodel in lensmodels.items():
            training, _ = solve_model(mrcal, session, data, views, train, lensmodel,
                                       output / name / 'training', focal, args.timeout)
            validation = validate_holdout(mrcal, training, views, held)
            full, modelpath = solve_model(mrcal, session, data, views, list(range(len(views))), lensmodel,
                                          output / name / 'full', focal, args.timeout)
            fitted[name] = full
            info = {'model': str(modelpath.relative_to(output)), 'lensmodel': lensmodel,
                    'holdout': validation, 'uncertainty': None}
            report['models'][name] = info
            if validation['rms_px'] > args.max_validation_rms:
                report['issues'].append(f'{name}: held-out RMS exceeds {args.max_validation_rms:g}px')
            observations = full.optimization_inputs()['observations_board']
            info['outlier_fraction'] = float(np.mean(observations[..., 2] <= 0))
            if info['outlier_fraction'] > .05:
                report['issues'].append(f'{name}: more than 5% rejected calibration corners')
            try:
                info['uncertainty'] = uncertainty_report(mrcal, full, output / f'{name}-uncertainty.png')
                if info['uncertainty']['p95_px'] > .5:
                    report['issues'].append(f'{name}: p95 sampling uncertainty exceeds 0.5px')
            except Exception as exc:
                info['uncertainty_error'] = str(exc)
                report['issues'].append(f'{name}: uncertainty unavailable; do not treat residual RMS as sufficient')
        if 'spline' in fitted:
            try:
                delta, _, _, _ = mrcal.projection_diff((fitted['spline'], fitted['opencv8']),
                                                       gridn_width=24, gridn_height=16, use_uncertainties=False)
                save_heatmap(output / 'spline-vs-opencv8.png', delta, 'Spline / OPENCV8 projection difference (rotation aligned)')
                if not np.isfinite(delta).all():
                    raise ValueError('Invalid model-difference grid')
                report['model_difference'] = {'p95_px': float(np.percentile(delta, 95)), 'maximum_px': float(np.max(delta)),
                                              'note': 'Model disagreement is a diagnostic, not ground-truth error'}
                if report['model_difference']['p95_px'] > .5:
                    report['issues'].append('Spline/OPENCV8 disagreement >0.5px p95; investigate lens-model bias or coverage')
            except Exception as exc:
                report['model_difference_error'] = str(exc)
                report['issues'].append('Could not compare richer and runtime lens models')
        exported = export_opencv(mrcal, fitted['opencv8'], data)
        exported.update(accepted_views=len(views), validation_rms_px=report['models']['opencv8']['holdout']['rms_px'])
        report['status'] = 'needs_review' if report['issues'] else 'heuristics_passed_not_hardware_validated'
        exported['quality_status'] = report['status']
        filename = 'intrinsics.candidate.json' if report['issues'] else 'intrinsics.json'
        write_json(output / filename, exported)
        report['intrinsics_file'] = filename
        write_json(output / 'board-poses.json', board_poses(fitted['opencv8'], list(range(len(views))), views))
        report['warnings'].append('Calibration accuracy does not establish detector FPS, exposure latency, or robot pose accuracy')
        write_json(output / 'report.json', report)
        write_json(session / 'latest-solve.json', {'directory': output.name, 'status': report['status']})
        print(f'Calibration report: {output / "report.json"}\nStatus: {report["status"]}', flush=True)
        for issue in report['issues']:
            print('REVIEW:', issue)
        return 2 if report['issues'] else 0
    except Exception as exc:
        report.update(status='failed', error=str(exc))
        write_json(output / 'report.json', report)
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    from custom_vision.calibration_session import add_solve_arguments
    add_solve_arguments(parser)
    args = parser.parse_args(argv)
    try:
        return calibrate(args)
    except Exception as exc:
        print(f'mrcal calibration failed: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
