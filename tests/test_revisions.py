"""Public geometry revisions identify configuration without disclosing artifacts."""

import copy
import re

from custom_vision.revisions import geometry_revisions


def configured_geometry():
    pipeline = {'calibration_data': {'width': 640, 'height': 480,
        'camera_matrix': [[600, 0, 320], [0, 600, 240], [0, 0, 1]],
        'dist_coeffs': [0, 0, 0, 0, 0]},
        'robot_to_camera': {'translation_m': [0.2, 0, 0.3], 'rotation_rpy_deg': [0, 15, 0]}}
    config = {'field_layout_data': {'field': {'length': 16, 'width': 8}, 'tags': [
        {'ID': 2, 'pose': {'translation': {'x': 0, 'y': 1, 'z': 1},
            'rotation': {'quaternion': {'W': 1, 'X': 0, 'Y': 0, 'Z': 0}}}},
        {'ID': 1, 'pose': {'translation': {'x': 3, 'y': 1, 'z': 1},
            'rotation': {'quaternion': {'W': 1, 'X': 0, 'Y': 0, 'Z': 0}}}}]}}
    return pipeline, config


def test_missing_inputs_remain_null():
    assert geometry_revisions({}, {}) == {'calibration_revision': None,
        'mount_revision': None, 'field_layout_revision': None}


def test_revisions_are_deterministic_opaque_and_do_not_mutate_inputs():
    pipeline, config = configured_geometry()
    originals = copy.deepcopy((pipeline, config))
    expected = geometry_revisions(pipeline, config)
    assert all(re.fullmatch(r'sha256:[0-9a-f]{64}', value) for value in expected.values())
    assert geometry_revisions(pipeline, config) == expected
    assert (pipeline, config) == originals
    pipeline['calibration'] = '/private/camera-credentials.json'
    pipeline['calibration_data'].update(path='/private/data', token='SECRET',
        rms_px=0.1, benchmark_only=True, physical_verification=True)
    pipeline['robot_to_camera'].update(source_path='/private/mount', verified=True)
    pipeline['camera'] = {'source': 'rtsp://user:password@camera'}
    pipeline['poi'] = {'calibration_verified': True}
    config['field_layout'] = '/private/team-layout.json'
    config['field_layout_data']['metadata'] = {'password': 'SECRET'}
    assert geometry_revisions(pipeline, config) == expected


def test_numeric_types_negative_zero_tag_order_and_quaternion_sign_are_canonical():
    pipeline, config = configured_geometry()
    expected = geometry_revisions(pipeline, config)
    pipeline['calibration_data']['dist_coeffs'] = [-0., 0., 0., 0., 0.]
    pipeline['robot_to_camera']['rotation_rpy_deg'] = [-0., 15., 0.]
    config['field_layout_data']['field'] = {'width': 8., 'length': 16.}
    config['field_layout_data']['tags'].reverse()
    quaternion = config['field_layout_data']['tags'][0]['pose']['rotation']['quaternion']
    quaternion.update({key: -float(value) for key, value in quaternion.items()})
    assert geometry_revisions(pipeline, config) == expected


def test_public_geometry_changes_only_the_affected_revision():
    pipeline, config = configured_geometry()
    expected = geometry_revisions(pipeline, config)
    for revision, mutate in [
        ('calibration_revision', lambda p, c: p['calibration_data']['camera_matrix'][0].__setitem__(0, 601)),
        ('mount_revision', lambda p, c: p['robot_to_camera']['translation_m'].__setitem__(0, 0.21)),
        ('field_layout_revision', lambda p, c: c['field_layout_data']['tags'][0]['pose']['translation'].__setitem__('x', 0.1)),
    ]:
        changed_pipeline, changed_config = copy.deepcopy((pipeline, config))
        mutate(changed_pipeline, changed_config)
        actual = geometry_revisions(changed_pipeline, changed_config)
        assert actual[revision] != expected[revision]
        assert {key: value for key, value in actual.items() if key != revision} == {
            key: value for key, value in expected.items() if key != revision}
