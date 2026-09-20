"""Real CUDA joint field PnP, using independent known-world synthetic scenes.

These tests exercise GPU kernels when built on actual CUDA hardware. They do
not simulate GPU success or establish physical camera/field accuracy.
"""
import copy
from concurrent.futures import ThreadPoolExecutor
import json
import math

import cv2
import numpy as np
import pytest

from custom_vision import app
from custom_vision.localization import Localization
from custom_vision.native_apriltags import NativeAprilTagPipeline, native_capabilities


SIZE = .1651
CAL = {'width': 1280, 'height': 800,
       'camera_matrix': [[870., 0., 640.], [0., 865., 400.], [0., 0., 1.]],
       'dist_coeffs': [-.13, .04, .001, -.002, .003, .005, -.002, 0.]}


def require_gpu():
    capabilities = native_capabilities()
    if not capabilities.get('available'):
        pytest.skip('Native extension is not built; CUDA MultiTag execution unverified')
    if not capabilities.get('cuda_multitag_compiled'):
        pytest.skip('Actual CUDA MultiTag kernel is not compiled')
    if not capabilities.get('cuda_pose_devices'):
        pytest.skip('CUDA MultiTag build exists but no real CUDA device is available')
    assert capabilities.get('cuda_multitag_max_tags') == 256
    return capabilities


def transform(translation, rpy=(0., 0., 0.)):
    roll, pitch, yaw = np.deg2rad(rpy)
    rx = np.array([[1., 0., 0.], [0., math.cos(roll), -math.sin(roll)],
                   [0., math.sin(roll), math.cos(roll)]])
    ry = np.array([[math.cos(pitch), 0., math.sin(pitch)], [0., 1., 0.],
                   [-math.sin(pitch), 0., math.cos(pitch)]])
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0.],
                   [math.sin(yaw), math.cos(yaw), 0.], [0., 0., 1.]])
    result = np.eye(4)
    result[:3, :3] = rz @ ry @ rx
    result[:3, 3] = translation
    return result


def scene(planar=False):
    camera = transform([2., 3., .7], [1., -4., 17.])
    if planar:
        tags = {7: transform([5., 3.3, 1.3], [0., 0., 180.]),
                12: transform([5., 4.4, .8], [0., 0., 180.]),
                20: transform([5., 2.6, .9], [0., 0., 180.])}
    else:
        tags = {7: transform([5., 3.3, 1.3], [0., 0., 180.]),
                12: transform([5.4, 4.4, .8], [0., 0., 200.]),
                20: transform([4.8, 2.6, .9], [0., 0., 160.])}
    return camera, tags


def field_layout(tags):
    entries = []
    for tag_id, pose in tags.items():
        rvec = cv2.Rodrigues(pose[:3, :3])[0].reshape(3)
        angle = float(np.linalg.norm(rvec))
        quaternion = (np.r_[math.cos(angle / 2), rvec * math.sin(angle / 2) / angle]
                      if angle else np.array([1., 0., 0., 0.]))
        entries.append({'ID': tag_id, 'pose': {
            'translation': dict(zip('xyz', pose[:3, 3].tolist())),
            'rotation': {'quaternion': dict(zip(('W', 'X', 'Y', 'Z'), quaternion.tolist()))}}})
    return {'field': {'length': 16.5, 'width': 8.2}, 'tags': entries}


def observation(tags, camera, calibration=CAL, *, noise=0., seed=1086):
    # Decoded TR/TL/BL/BR printed corners, in tag WPILib coordinates. This does
    # not import production corner/basis helpers or derive truth from a solver.
    h = SIZE / 2
    local = np.array([[0., h, h, 1.], [0., -h, h, 1.],
                      [0., -h, -h, 1.], [0., h, -h, 1.]])
    points = np.array([(pose @ local.T).T[:, :3] for pose in tags.values()])
    relative = (points - camera[:3, 3]) @ camera[:3, :3]
    optical = np.stack((-relative[:, :, 1], -relative[:, :, 2], relative[:, :, 0]), axis=-1)
    assert np.min(optical[:, :, 2]) > 0
    uv = cv2.projectPoints(optical.reshape(-1, 3), np.zeros(3), np.zeros(3),
                           np.array(calibration['camera_matrix']),
                           np.array(calibration['dist_coeffs']))[0].reshape(-1, 4, 2)
    if noise:
        uv += np.random.default_rng(seed).normal(0., noise, uv.shape)
    detections = [dict(id=tag_id, corners=corners.tolist(), center=corners.mean(axis=0).tolist(),
                       decision_margin=100., hamming=0, pose_valid=False, pose_attempted=False,
                       pose_device='none', pose_invalid_reason='deferred_multitag')
                  for tag_id, corners in zip(tags, uv)]
    return np.ascontiguousarray(uv), np.ascontiguousarray(points), detections


def detector(calibration=CAL):
    require_gpu()
    return NativeAprilTagPipeline({'mode': '3d', 'pose_device': 'cuda',
                                   'tag_size_m': SIZE, 'max_reprojection_error_px': 3.}, calibration)


def pipeline(tags, calibration=CAL, mount=None):
    require_gpu()
    cfg = dict(type='apriltag', settings={'backend': 'native', 'mode': '3d',
               'detector_device': 'cpu', 'pose_device': 'cuda', 'multitag': True,
               'tag_size_m': SIZE, 'max_ambiguity': .2, 'max_reprojection_error_px': 3.},
               calibration_data=calibration, robot_to_camera=mount)
    return app.make_detector(cfg, {'field_layout_data': field_layout(tags)})


def matrix_from_published(pose):
    w, x, y, z = pose['rotation_quaternion_wxyz']
    axis = np.array([x, y, z])
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    result = np.eye(4)
    result[:3, :3] = (w*w - axis @ axis) * np.eye(3) + 2 * np.outer(axis, axis) + 2 * w * skew
    result[:3, 3] = pose['translation_m']
    return result


def camera_from_native(result):
    rotation = cv2.Rodrigues(np.asarray(result['rvec_rad']))[0]
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack((rotation[2], -rotation[0], -rotation[1]))
    pose[:3, 3] = -rotation.T @ np.asarray(result['tvec_m'])
    return pose


def check_native_pose(result, uv, points, camera=None, expected_inliers=None, calibration=CAL):
    assert result['pose_valid'] and result['pose_device'] == 'cuda'
    json.dumps(result, allow_nan=False)
    indices = result['inlier_indices']
    assert len(indices) >= 2 and len(set(indices)) == len(indices)
    assert sorted(indices + result['rejected_indices']) == list(range(len(points)))
    if expected_inliers is not None:
        assert indices == expected_inliers
    rotation = cv2.Rodrigues(np.array(result['rvec_rad']))[0]
    translation = np.array(result['tvec_m'])
    accepted = points[indices].reshape(-1, 3)
    assert np.min((accepted @ rotation.T + translation)[:, 2]) > 0
    projected = cv2.projectPoints(points.reshape(-1, 3), np.array(result['rvec_rad']), translation,
                                  np.array(calibration['camera_matrix']),
                                  np.array(calibration['dist_coeffs']))[0].reshape(-1, 4, 2)
    per_tag = np.sqrt(np.sum((projected - uv)**2, axis=(1, 2)) / 4)
    assert len(result['per_tag_reprojection_error_px']) == len(points)
    np.testing.assert_allclose(result['per_tag_reprojection_error_px'], per_tag, atol=2e-6)
    residuals = projected[indices] - uv[indices]
    assert np.max(per_tag[indices]) <= 3.
    independent_error = float(np.sqrt(np.mean(np.sum(residuals**2, axis=2))))
    assert result['reprojection_error_px'] == pytest.approx(independent_error, abs=2e-6)
    if camera is not None:
        # Optical R,t maps WORLD points into camera; camera center is -R^T t.
        np.testing.assert_allclose(-rotation.T @ translation, camera[:3, 3], atol=5e-4)
        expected_rotation = np.stack((-camera[:3, 1], -camera[:3, 2], camera[:3, 0]))
        np.testing.assert_allclose(rotation, expected_rotation, atol=2e-4)


@pytest.mark.parametrize('planar', [False, True])
def test_native_joint_pose_matches_known_field_camera(planar):
    camera, tags = scene(planar)
    uv, world, _ = observation(tags, camera)
    p = detector()
    try:
        result = p.estimate_multitag(uv, world)
        check_native_pose(result, uv, world, camera, [0, 1, 2])
        assert result['pose_ambiguity'] <= .2
    finally:
        p.close()


@pytest.mark.parametrize('planar', [False, True])
def test_cuda_localization_preserves_mount_and_absolute_pose_conventions(planar):
    camera, tags = scene(planar)
    mount = {'translation_m': [.35, -.17, .42], 'rotation_rpy_deg': [5., -15., 25.]}
    expected_robot = camera @ np.linalg.inv(transform(mount['translation_m'], mount['rotation_rpy_deg']))
    _, _, detections = observation(tags, camera)
    p = pipeline(tags, mount=mount)
    try:
        output = p.localization.enrich(detections, (800, 1280))
        pose = output['localization']
        assert pose['valid'] and pose['pose_device'] == 'cuda'
        assert pose['used_tag_ids'] == [7, 12, 20]
        np.testing.assert_allclose(matrix_from_published(pose['field_to_camera']), camera, atol=5e-4)
        np.testing.assert_allclose(matrix_from_published(pose['field_to_robot']), expected_robot, atol=5e-4)
        for detection in output['detections']:
            assert detection['pose_source'] == 'field_layout_multitag'
            assert detection['pose_device'] == 'cuda'
            np.testing.assert_allclose(matrix_from_published(detection['robot_to_target']),
                                       np.linalg.inv(expected_robot) @ tags[detection['id']], atol=5e-4)
    finally:
        p.close()


@pytest.mark.parametrize('bad_corners', [1, 2, 4])
def test_outliers_are_removed_as_whole_tags(bad_corners):
    camera, tags = scene()
    uv, world, _ = observation(tags, camera)
    uv[2, :bad_corners] += [90., -40.]
    p = detector()
    try:
        result = p.estimate_multitag(uv, world)
        check_native_pose(result, uv, world, camera, [0, 1])
        assert result['rejected_indices'] == [2]
    finally:
        p.close()


def test_two_inconsistent_tags_do_not_manufacture_a_joint_pose():
    camera, tags = scene()
    tags = {key: tags[key] for key in (7, 12)}
    uv, world, _ = observation(tags, camera)
    uv[1] += [150., -75.]
    p = detector()
    try:
        result = p.estimate_multitag(uv, world)
        assert not result['pose_valid']
        assert result.get('pose_invalid_reason') or result.get('reason')
        json.dumps(result, allow_nan=False)
    finally:
        p.close()


def test_duplicate_observed_ids_are_excluded_before_gpu_joint_solving():
    camera, tags = scene()
    _, _, detections = observation(tags, camera)
    duplicated = detections + [copy.deepcopy(detections[0])]
    p = pipeline(tags)
    try:
        result = p.localization.enrich(duplicated, (800, 1280))['localization']
        assert result['valid'] and result['pose_device'] == 'cuda'
        assert result['duplicate_tag_ids'] == [7]
        assert result['used_tag_ids'] == [12, 20]
        np.testing.assert_allclose(matrix_from_published(result['field_to_camera']), camera, atol=5e-4)
        empty = p.localization.enrich([detections[0], copy.deepcopy(detections[0])], (800, 1280))['localization']
        assert not empty['valid'] and empty['invalid_reason'] == 'duplicate_tag_id'
        assert empty['field_to_camera'] is None
    finally:
        p.close()


def test_exact_frontoparallel_joint_ambiguity_remains_visible():
    calibration = dict(CAL, dist_coeffs=[0.] * 8)
    camera = transform([2., 3., 1.])
    tags = {7: transform([5., 2.7, 1.], [0., 0., 180.]),
            12: transform([5., 3.3, 1.], [0., 0., 180.])}
    uv, world, detections = observation(tags, camera, calibration)
    p = pipeline(tags, calibration)
    try:
        raw = p.estimate_multitag(uv, world)
        assert raw['pose_valid'] and raw['pose_ambiguity'] > .9
        result = p.localization.enrich(detections, (800, 1280))['localization']
        assert not result['valid'] and result['invalid_reason'] == 'ambiguous_pose'
        assert result['field_to_camera'] is None
    finally:
        p.close()


def test_noisy_nonplanar_joint_result_agrees_with_independent_cpu_solver():
    camera, tags = scene()
    p = pipeline(tags)
    reference = Localization({}, CAL, field_layout(tags))
    try:
        for seed in range(8):
            uv, world, detections = observation(tags, camera, noise=.2, seed=seed)
            cpu = reference.enrich(detections, (800, 1280))['localization']
            gpu = p.localization.enrich(detections, (800, 1280))['localization']
            assert cpu['valid'] and gpu['valid']
            assert gpu['pose_device'] == 'cuda'
            assert gpu['reprojection_error_px'] <= cpu['reprojection_error_px'] + .005
            np.testing.assert_allclose(matrix_from_published(gpu['field_to_camera']),
                                       matrix_from_published(cpu['field_to_camera']), atol=.01)
            check_native_pose(p.estimate_multitag(uv, world), uv, world, expected_inliers=[0, 1, 2])
    finally:
        p.close()


@pytest.mark.parametrize('kind', ['empty', 'single', 'over_capacity', 'corner_shape',
                                  'world_shape', 'length_mismatch', 'nonfinite_uv', 'nonfinite_world'])
def test_native_input_contract_rejects_unsupported_or_nonfinite_batches(kind):
    camera, tags = scene()
    uv, world, _ = observation(tags, camera)
    if kind == 'empty': uv, world = uv[:0], world[:0]
    elif kind == 'single': uv, world = uv[:1], world[:1]
    elif kind == 'over_capacity': uv, world = np.repeat(uv[:1], 257, axis=0), np.repeat(world[:1], 257, axis=0)
    elif kind == 'corner_shape': uv = uv[:, :3]
    elif kind == 'world_shape': world = world[:, :, :2]
    elif kind == 'length_mismatch': world = world[:2]
    elif kind == 'nonfinite_uv': uv[0, 0, 0] = np.nan
    else: world[0, 0, 0] = np.inf
    p = detector()
    try:
        with pytest.raises((ValueError, TypeError)):
            p.estimate_multitag(uv, world)
    finally:
        p.close()


def test_independent_camera_instances_and_same_instance_calls_are_isolated():
    camera, tags = scene()
    uv, world, _ = observation(tags, camera)
    first, second = detector(), detector()
    try:
        with ThreadPoolExecutor(max_workers=3) as pool:
            futures = [pool.submit(p.estimate_multitag, uv, world) for p in (first, second, first)]
            results = [future.result(timeout=30) for future in futures]
        for result in results:
            check_native_pose(result, uv, world, camera, [0, 1, 2])
        old = copy.deepcopy(results[0])
        shifted_camera = transform([2.2, 3.1, .8], [2., -3., 14.])
        shifted, _, _ = observation(tags, shifted_camera)
        first.estimate_multitag(shifted, world)
        assert results[0] == old, 'Published poses must not alias reusable CUDA buffers'
    finally:
        first.close()
        second.close()
    with pytest.raises(RuntimeError, match='closed'):
        first.estimate_multitag(uv, world)


def test_cuda_app_localization_never_calls_cpu_pnp_including_outlier_and_single_fallback(monkeypatch):
    camera, tags = scene()
    _, _, detections = observation(tags, camera)
    p = pipeline(tags)
    def forbidden(*args, **kwargs):
        raise AssertionError('CUDA runtime attempted a CPU PnP solve')
    try:
        monkeypatch.setattr('custom_vision.localization.estimate_tag_pose', forbidden)
        for name in ('solvePnP', 'solvePnPGeneric', 'solvePnPRansac', 'solvePnPRefineLM', 'solvePnPRefineVVS'):
            if hasattr(cv2, name):
                monkeypatch.setattr(cv2, name, forbidden)
        for observations, ids in [(detections, [7, 12, 20]), (detections[:1], [7])]:
            result = p.localization.enrich(observations, (800, 1280))['localization']
            assert result['valid'] and result['pose_device'] == 'cuda'
            assert result['used_tag_ids'] == ids
        outlier = copy.deepcopy(detections)
        outlier[2]['corners'] = (np.asarray(outlier[2]['corners']) + [90., -40.]).tolist()
        result = p.localization.enrich(outlier, (800, 1280))['localization']
        assert result['valid'] and result['pose_device'] == 'cuda'
        assert result['used_tag_ids'] == [7, 12]
        assert result['rejected_tag_ids'] == [20]
    finally:
        p.close()


@pytest.mark.parametrize('noise', [0., .15])
@pytest.mark.parametrize('shift,rpy', [([25., 40., -6.], [-15., 22., -75.]),
                                     ([1000., -800., 250.], [14., -20., 135.])])
def test_global_frame_rotation_and_large_origin_offsets_preserve_solution(shift,rpy,noise):
    camera, tags = scene()
    world_change = transform(shift, rpy)
    shifted_camera = world_change @ camera
    shifted_tags = {tag_id: world_change @ pose for tag_id, pose in tags.items()}
    uv, points, _ = observation(tags, camera, noise=noise, seed=339)
    shifted_uv, shifted_points, _ = observation(shifted_tags, shifted_camera, noise=noise, seed=339)
    np.testing.assert_allclose(shifted_uv, uv, atol=1e-9)
    p = detector()
    try:
        baseline = p.estimate_multitag(uv, points)
        shifted = p.estimate_multitag(shifted_uv, shifted_points)
        check_native_pose(baseline, uv, points, camera if noise == 0 else None, [0, 1, 2])
        check_native_pose(shifted, shifted_uv, shifted_points, shifted_camera if noise == 0 else None, [0, 1, 2])
        recovered = np.linalg.inv(world_change) @ camera_from_native(shifted)
        np.testing.assert_allclose(recovered, camera_from_native(baseline), atol=.002)
        for tag_id in tags:
            relative = np.linalg.inv(camera_from_native(shifted)) @ shifted_tags[tag_id]
            original = np.linalg.inv(camera_from_native(baseline)) @ tags[tag_id]
            np.testing.assert_allclose(relative, original, atol=.002)
        assert shifted['reprojection_error_px'] == pytest.approx(baseline['reprojection_error_px'], abs=.001)
    finally:
        p.close()


@pytest.mark.parametrize('depth_offset', [.0001, .003])
def test_nearly_coplanar_but_real_nonplanarity_is_not_flattened(depth_offset):
    camera, tags = scene(planar=True)
    tags[12][0, 3] += depth_offset
    tags[20][0, 3] -= depth_offset
    uv, points, _ = observation(tags, camera)
    assert np.linalg.svd(points.reshape(-1, 3)-points.mean(axis=(0, 1)), compute_uv=False)[-1] > 1e-6
    p = detector()
    try:
        result = p.estimate_multitag(uv, points)
        assert result['coplanar'] is False
        check_native_pose(result, uv, points, camera, [0, 1, 2])
    finally:
        p.close()


@pytest.mark.parametrize('frontal', [False, True])
def test_noisy_planar_joint_branches_preserve_cpu_ambiguity_decision(frontal):
    calibration = dict(CAL, dist_coeffs=[0.] * 8)
    if frontal:
        camera = transform([1., 3., 1.])
        tags = {7: transform([8., 2.75, 1.], [0., 0., 180.]),
                12: transform([8., 3.25, 1.], [0., 0., 180.])}
    else:
        camera, tags = scene(planar=True)
    uv, world, detections = observation(tags, camera, calibration, noise=.2, seed=63)
    cpu = Localization({}, calibration, field_layout(tags)).enrich(detections, (800, 1280))['localization']
    p = pipeline(tags, calibration)
    try:
        raw = p.estimate_multitag(uv, world)
        check_native_pose(raw, uv, world, expected_inliers=list(range(len(tags))), calibration=calibration)
        assert raw['coplanar'] is True
        gpu = p.localization.enrich(detections, (800, 1280))['localization']
        assert gpu['valid'] == cpu['valid']
        assert gpu['ambiguity'] == pytest.approx(cpu['ambiguity'], abs=.05)
        if frontal:
            assert not gpu['valid'] and gpu['invalid_reason'] == 'ambiguous_pose'
        else:
            assert gpu['valid']
            assert gpu['reprojection_error_px'] <= cpu['reprojection_error_px'] + .005
    finally:
        p.close()


def random_layout(count, seed):
    rng = np.random.default_rng(seed)
    camera = transform([1.5, 2.7, .8], [rng.uniform(-3, 3), rng.uniform(-6, 2), rng.uniform(10, 20)])
    tags = {}
    for i in range(count):
        # Spread across the image while varying wall depth and tag orientation;
        # this is a rigid-square field layout, not arbitrary unconstrained points.
        tags[100 + i] = transform([rng.uniform(4.8, 5.4), 2.2 + 2.*i/(count-1), rng.uniform(.6, 1.4)],
                                  [rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(165, 195)])
    return camera, tags


def test_two_noncontiguous_good_tags_win_over_multiple_unrelated_outliers():
    camera, tags = random_layout(6, 603)
    uv, world, _ = observation(tags, camera)
    good = [1, 4]
    bad = [0, 2, 3, 5]
    for index, shift in zip(bad, [[100., 80.], [-150., 60.], [60., -120.], [-70., -140.]]):
        uv[index] += shift
    p = detector()
    try:
        result = p.estimate_multitag(uv, world)
        check_native_pose(result, uv, world, camera, good)
        assert result['rejected_indices'] == bad
    finally:
        p.close()


@pytest.mark.parametrize('kind', ['collapsed', 'stretched', 'nonplanar_square', 'crossed_order'])
def test_invalid_field_square_geometry_cannot_produce_a_valid_joint_pose(kind):
    camera, tags = scene()
    uv, world, _ = observation(tags, camera)
    if kind == 'collapsed':
        world[:] = world.mean(axis=1, keepdims=True)
    elif kind == 'stretched':
        world[:] = world.mean(axis=1, keepdims=True) + 1.2*(world-world.mean(axis=1, keepdims=True))
    elif kind == 'nonplanar_square':
        world[:, 0, 0] += .02
    else:
        world = world[:, [0, 2, 1, 3]].copy()
    p = detector()
    try:
        try:
            result = p.estimate_multitag(uv, world)
        except ValueError:
            return  # Rejecting malformed world geometry at the API is valid too.
        assert not result['pose_valid']
        assert result.get('pose_invalid_reason') or result.get('reason')
        json.dumps(result, allow_nan=False)
    finally:
        p.close()


def test_malformed_field_square_cannot_hide_inside_other_valid_tag_consensus():
    camera, tags = scene()
    uv, world, _ = observation(tags, camera)
    # A small out-of-plane corner perturbation may be below a loose pixel RMS
    # threshold, but it is not a physically valid square of the configured size.
    world[0, 0, 0] += .003
    p = detector()
    try:
        try:
            result = p.estimate_multitag(uv, world)
        except ValueError:
            return
        if result['pose_valid']:
            assert 0 not in result['inlier_indices']
            assert result['inlier_indices'] == [1, 2]
    finally:
        p.close()


@pytest.mark.parametrize('count', [2, 4, 8])
@pytest.mark.parametrize('noise', [0., .15])
def test_seeded_varied_layouts_have_consistent_independent_tag_errors(count,noise):
    p = detector()
    try:
        for seed in (count*1086, count*1086+19):
            camera, tags = random_layout(count, seed)
            uv, points, _ = observation(tags, camera, noise=noise, seed=seed)
            result = p.estimate_multitag(uv, points)
            check_native_pose(result, uv, points, camera if noise == 0 else None, list(range(count)))
            assert result['reprojection_error_px'] < .5
            if noise:
                np.testing.assert_allclose(camera_from_native(result)[:3, 3], camera[:3, 3], atol=.1)
    finally:
        p.close()


def test_maximum_capacity_and_reused_workspace_do_not_leak_previous_consensus():
    """Exercise all 256 GPU slots, then shrink/change the same persistent solve."""
    camera, _ = scene(planar=True)
    tags = {
        1 + 16*row + col: transform([5., 1.2 + .22*col, .1 + .22*row], [0., 0., 180.])
        for row in range(16) for col in range(16)
    }
    uv, world, _ = observation(tags, camera)
    p = detector()
    try:
        maximum = p.estimate_multitag(uv, world)
        check_native_pose(maximum, uv, world, camera, list(range(256)))
        assert maximum['rejected_indices'] == []
        assert len(maximum['per_tag_reprojection_error_px']) == 256
        saved = copy.deepcopy(maximum)

        small_camera, small_tags = scene(planar=False)
        small_uv, small_world, _ = observation(small_tags, small_camera)
        small = p.estimate_multitag(small_uv, small_world)
        check_native_pose(small, small_uv, small_world, small_camera, [0, 1, 2])

        invalid = p.estimate_multitag(np.zeros_like(small_uv), small_world)
        assert not invalid['pose_valid']
        assert invalid['inlier_indices'] == []
        json.dumps(invalid, allow_nan=False)

        restored = p.estimate_multitag(small_uv, small_world)
        check_native_pose(restored, small_uv, small_world, small_camera, [0, 1, 2])
        assert maximum == saved, 'Subsequent GPU calls must not mutate returned results'
    finally:
        p.close()
