"""Explicit calibration candidate activation/restore without cameras or workers."""
import copy
import json
from pathlib import Path

import pytest
import yaml

from custom_vision.control import RuntimeController
from custom_vision.revisions import geometry_revisions


def calibration(focal=500, width=640, height=480):
    return {'width':width,'height':height,
            'camera_matrix':[[focal,0,320],[0,focal,240],[0,0,1]],
            'dist_coeffs':[0,0,0,0,0]}


def metadata():
    return {'camera':{'physical_id':'measured-camera-a','user_label':'manual label is not identity',
                      'identity_source':'operator'},
            'mode':{'width':640,'height':480,'crop':[0,0,640,480],'binning':[1,1],
                    'focus':{'kind':'fixed','value':5,'locked':True}}}


@pytest.fixture
def controller(tmp_path,monkeypatch):
    class Timer:
        def __init__(self,*args): pass
        def start(self): pass
    monkeypatch.setattr('custom_vision.control.threading.Timer',Timer)
    path=tmp_path/'config'/'vision.yaml'
    path.parent.mkdir()
    provenance=metadata()
    pipeline={'name':'front_tags','type':'apriltag','enabled':True,
              'camera':{'source':0,'width':640,'height':480,'fps':30,
                        'physical_id':provenance['camera']['physical_id'],
                        'mode':provenance['mode']},
              'settings':{'backend':'pupil','mode':'2d'},'robot_to_camera':None,
              'calibration':'original-calibration.json',
              'poi':{'enabled':True,'calibration_verified':True,
                     'targets':[{'name':'aim','tag_id':7,'offset_m':[0,0,.2]}]}}
    (path.parent/pipeline['calibration']).write_text(json.dumps(calibration()))
    other=copy.deepcopy(pipeline)
    other['name']='rear_tags'
    config={'pipelines':[pipeline,other], 'dashboard':{'enabled':False,'port':5801},
            'networktables':{'enabled':False,'server':'https://private-user:private-password@example.invalid'}}
    path.write_text(yaml.safe_dump(config))
    return RuntimeController(path,lambda:None)


def history_record(controller,activation_id):
    return json.loads(controller._activation_path(activation_id).read_text())


def test_compatible_candidate_activation_preserves_artifacts_and_clears_verification(controller):
    before=controller.get_config()
    original=(controller.path.parent/before['pipelines'][0]['calibration']).read_bytes()
    candidate=calibration(515)
    provenance=metadata()
    snapshot=(copy.deepcopy(candidate),copy.deepcopy(provenance))
    validations=[]
    controller.validate_runtime=validations.append
    result=controller.activate_calibration_candidate('front_tags',candidate,provenance)
    saved=controller.get_config()
    assert result['saved'] and result['status']=='applied' and not result['restored']
    assert result['physical_verification'] is False
    assert result['poi_verification_reset']==['front_tags']
    assert saved['pipelines'][0]['poi']['calibration_verified'] is False
    assert validations[0]['pipelines'][0]['poi']['calibration_verified'] is False
    assert saved['pipelines'][0]['calibration']!=before['pipelines'][0]['calibration']
    assert saved['pipelines'][1]==before['pipelines'][1]
    assert (controller.path.parent/before['pipelines'][0]['calibration']).read_bytes()==original
    assert (candidate,provenance)==snapshot
    record=history_record(controller,result['activation_id'])
    assert record['previous_config_snapshot']==before
    assert record['previous_calibration_reference']==before['pipelines'][0]['calibration']
    assert record['previous_calibration_snapshot']['camera_matrix']==calibration()['camera_matrix']
    assert record['calibration_revision']==geometry_revisions({'calibration_data':candidate},{})['calibration_revision']
    assert record['calibration_revision']!=record['previous_calibration_revision']
    assert controller._activation_path(result['activation_id']).parent==controller.path.parent.parent/'data'/'calibration-activation-history'


@pytest.mark.parametrize('width,height',[(800,480),(640,600)])
def test_mismatched_calibration_resolution_is_rejected_before_writes(controller,width,height):
    original=controller.path.read_bytes()
    with pytest.raises(ValueError,match='configured camera mode'):
        controller.activate_calibration_candidate('front_tags',calibration(width=width,height=height),metadata())
    assert controller.path.read_bytes()==original
    assert controller.list_calibration_activations()==[]
    assert not (controller.path.parent.parent/'calibration').exists()


@pytest.mark.parametrize('section,field,replacement',[
    ('camera','physical_id','another-camera'),('mode','crop',[0,0,320,240]),
    ('mode','binning',[2,2]),('mode','focus',{'kind':'fixed','value':6,'locked':True}),
    ('mode','focus',{'kind':'adjustable','value':5,'locked':True}),
    ('mode','focus',{'kind':'fixed','value':5,'locked':False}),
])
def test_known_physical_provenance_mismatch_is_rejected(controller,section,field,replacement):
    original=controller.path.read_bytes()
    provenance=metadata()
    provenance[section][field]=replacement
    with pytest.raises(ValueError,match='does not match the configured camera'):
        controller.activate_calibration_candidate('front_tags',calibration(515),provenance)
    assert controller.path.read_bytes()==original
    assert controller.list_calibration_activations()==[]


@pytest.mark.parametrize('field,value',[('width',800),('height',600),('width',640.0),('height',True)])
def test_candidate_mode_metadata_must_agree_with_calibration(controller,field,value):
    provenance=metadata()
    provenance['mode'][field]=value
    with pytest.raises(ValueError,match='mode metadata'):
        controller.activate_calibration_candidate('front_tags',calibration(515),provenance)


def test_unknown_provenance_warns_and_never_claims_verification(controller):
    result=controller.activate_calibration_candidate('front_tags',calibration(515))
    assert result['physical_verification'] is False
    assert any('physical camera identity provenance is unknown' in warning for warning in result['warnings'])
    for field in ('crop','binning','focus'):
        assert any(field in warning and 'unknown' in warning for warning in result['warnings'])
    assert controller.get_config()['pipelines'][0]['poi']['calibration_verified'] is False


def test_manual_camera_label_and_source_are_not_physical_identity(controller):
    config=controller.get_config()
    config['pipelines'][0]['camera'].pop('physical_id')
    controller.path.write_text(yaml.safe_dump(config))
    provenance=metadata()
    provenance['camera']['user_label']='0'
    result=controller.activate_calibration_candidate('front_tags',calibration(515),provenance)
    assert any('physical camera identity provenance is unknown' in warning for warning in result['warnings'])


def test_unknown_focus_kind_and_default_lock_are_warnings_not_measured_conflicts(controller):
    provenance=metadata()
    provenance['mode']['focus']={'kind':'unknown','value':None,'locked':False}
    result=controller.activate_calibration_candidate('front_tags',calibration(515),provenance)
    for field in ('kind','value','locked'):
        assert any(f'focus {field} provenance is unknown' in warning for warning in result['warnings'])
    assert result['physical_verification'] is False


def test_explicit_same_calibration_activation_still_requires_new_verification(controller):
    before=controller.get_config()
    validations=[]
    controller.validate_runtime=validations.append
    result=controller.activate_calibration_candidate('front_tags',calibration(),metadata())
    assert controller.get_config()['pipelines'][0]['calibration']==before['pipelines'][0]['calibration']
    assert controller.get_config()['pipelines'][0]['poi']['calibration_verified'] is False
    assert validations[0]['pipelines'][0]['poi']['calibration_verified'] is False
    assert result['poi_verification_reset']==['front_tags']


def test_restore_changes_only_previous_calibration_and_verification(controller):
    before=controller.get_config()
    activated=controller.activate_calibration_candidate('front_tags',calibration(515),metadata())
    current=controller.get_config()
    current['pipelines'][0]['settings']['threads']=3
    current['pipelines'][0]['poi']['calibration_verified']=True
    current['pipelines'][1]['camera']['fps']=60
    current['dashboard']['port']=5810
    controller.apply_config(current)
    restored=controller.restore_calibration_activation(activated['activation_id'])
    saved=controller.get_config()
    assert restored['restored'] and restored['restore_status']=='applied'
    assert restored['poi_verification_reset']==['front_tags']
    assert saved['pipelines'][0]['calibration']==before['pipelines'][0]['calibration']
    assert saved['pipelines'][0]['poi']['calibration_verified'] is False
    assert saved['pipelines'][0]['settings']['threads']==3
    assert saved['pipelines'][1]==current['pipelines'][1]
    assert saved['dashboard']==current['dashboard']
    assert saved['networktables']==current['networktables']
    # Repeated restore is idempotent and cannot undo subsequent config work.
    second=controller.restore_calibration_activation(activated['activation_id'])
    assert second['already_restored'] and second['restored']
    assert controller.get_config()==saved


@pytest.mark.parametrize('previous_present',[False,True])
def test_restore_supports_no_previous_calibration(controller,previous_present):
    config=controller.get_config()
    selected=config['pipelines'][0]
    if previous_present:
        selected['calibration']=None
    else:
        selected.pop('calibration')
    selected['poi']['calibration_verified']=False
    controller.path.write_text(yaml.safe_dump(config))
    activated=controller.activate_calibration_candidate('front_tags',calibration(515),metadata())
    restored=controller.restore_calibration_activation(activated['activation_id'])
    saved=controller.get_config()['pipelines'][0]
    assert restored['restored']
    assert ('calibration' in saved)==previous_present
    assert saved.get('calibration') is None
    assert saved['poi']['calibration_verified'] is False


def test_restore_refuses_to_overwrite_subsequent_calibration_change(controller):
    activated=controller.activate_calibration_candidate('front_tags',calibration(515),metadata())
    controller.upload_calibration('front_tags',calibration(540))
    current=controller.path.read_bytes()
    with pytest.raises(ValueError,match='subsequent change'):
        controller.restore_calibration_activation(activated['activation_id'])
    assert controller.path.read_bytes()==current
    assert history_record(controller,activated['activation_id'])['restored'] is False


def test_restore_rejects_missing_previous_artifact_without_changing_config(controller):
    activated=controller.activate_calibration_candidate('front_tags',calibration(515),metadata())
    (controller.path.parent/'original-calibration.json').unlink()
    current=controller.path.read_bytes()
    with pytest.raises(ValueError,match='Previous calibration artifact is unavailable'):
        controller.restore_calibration_activation(activated['activation_id'])
    assert controller.path.read_bytes()==current


def test_restore_rejects_changed_previous_geometry_and_preserves_current_config(controller):
    activated=controller.activate_calibration_candidate('front_tags',calibration(515),metadata())
    (controller.path.parent/'original-calibration.json').write_text(json.dumps(calibration(600)))
    current=controller.path.read_bytes()
    with pytest.raises(ValueError,match='Previous calibration artifact changed since activation'):
        controller.restore_calibration_activation(activated['activation_id'])
    assert controller.path.read_bytes()==current
    assert history_record(controller,activated['activation_id'])['restored'] is False
    assert history_record(controller,activated['activation_id'])['previous_calibration_snapshot']['camera_matrix'][0][0]==500


def test_restore_preserves_new_camera_mode_and_rejects_incompatible_previous_calibration(controller):
    activated=controller.activate_calibration_candidate('front_tags',calibration(515),metadata())
    current=controller.get_config()
    current['pipelines'][0]['camera']['width']=800
    controller.path.write_text(yaml.safe_dump(current))
    original=controller.path.read_bytes()
    with pytest.raises(ValueError,match='current camera mode'):
        controller.restore_calibration_activation(activated['activation_id'])
    assert controller.path.read_bytes()==original


def test_durable_pending_history_precedes_each_config_mutation(controller,monkeypatch):
    apply=controller.apply_config
    observed=[]
    def check_pending(config):
        records=[json.loads(path.read_text()) for path in controller._activation_directory.glob('*.json')]
        assert len(records)==1
        record=records[0]
        observed.append((record['status'],record.get('restore_status')))
        assert record['status']=='pending' or record.get('restore_status')=='pending'
        return apply(config)
    monkeypatch.setattr(controller,'apply_config',check_pending)
    result=controller.activate_calibration_candidate('front_tags',calibration(515),metadata())
    controller.restore_calibration_activation(result['activation_id'])
    assert observed==[('pending',None),('applied','pending')]


def test_failed_activation_records_failure_and_preserves_original_config(controller):
    original=controller.path.read_bytes()
    def reject(_): raise RuntimeError('backend path /private/secret unavailable')
    controller.validate_runtime=reject
    with pytest.raises(RuntimeError,match='backend path'):
        controller.activate_calibration_candidate('front_tags',calibration(515),metadata())
    assert controller.path.read_bytes()==original
    history=controller.list_calibration_activations()
    assert len(history)==1 and history[0]['status']=='failed'
    assert '/private/secret' not in json.dumps(history)
    assert controller.get_config()['pipelines'][0]['poi']['calibration_verified'] is True
    with pytest.raises(ValueError,match='not applied'):
        controller.restore_calibration_activation(history[0]['activation_id'])


def test_history_write_failure_prevents_configuration_mutation(controller,monkeypatch):
    original=controller.path.read_bytes()
    def reject(_): raise OSError('history write rejected')
    monkeypatch.setattr(controller,'_write_activation',reject)
    with pytest.raises(OSError,match='history write rejected'):
        controller.activate_calibration_candidate('front_tags',calibration(515),metadata())
    assert controller.path.read_bytes()==original


def test_failed_restore_records_failure_and_keeps_active_candidate(controller):
    activated=controller.activate_calibration_candidate('front_tags',calibration(515),metadata())
    original=controller.path.read_bytes()
    def reject(_): raise RuntimeError('backend unavailable')
    controller.validate_runtime=reject
    with pytest.raises(RuntimeError,match='backend unavailable'):
        controller.restore_calibration_activation(activated['activation_id'])
    assert controller.path.read_bytes()==original
    record=history_record(controller,activated['activation_id'])
    assert record['restore_status']=='failed' and record['restored'] is False


@pytest.mark.parametrize('operation',['activate','restore'])
def test_restart_handoff_failure_records_completed_atomic_config_write(controller,monkeypatch,operation):
    activated=None
    if operation=='restore':
        activated=controller.activate_calibration_candidate('front_tags',calibration(515),metadata())
    before=controller.get_config()['pipelines'][0]['calibration']
    def reject_timer(*args): raise RuntimeError('restart handoff unavailable')
    monkeypatch.setattr('custom_vision.control.threading.Timer',reject_timer)
    with pytest.raises(RuntimeError,match='restart handoff unavailable'):
        if operation=='activate':
            controller.activate_calibration_candidate('front_tags',calibration(515),metadata())
        else:
            controller.restore_calibration_activation(activated['activation_id'])
    history=controller.list_calibration_activations()
    assert len(history)==1 and history[0]['status']=='applied'
    assert controller.get_config()['pipelines'][0]['calibration']!=before
    assert any('restart handoff failed' in warning for warning in history[0]['warnings'])
    assert history[0]['restored']==(operation=='restore')
    if operation=='restore': assert history[0]['restore_status']=='applied'


def test_pending_history_is_diagnostic_and_cannot_be_restored(controller):
    activated=controller.activate_calibration_candidate('front_tags',calibration(515),metadata())
    record=history_record(controller,activated['activation_id'])
    record['status']='pending'
    controller._write_activation(record)
    original=controller.path.read_bytes()
    with pytest.raises(ValueError,match='not applied'):
        controller.restore_calibration_activation(activated['activation_id'])
    assert controller.path.read_bytes()==original
    assert controller.list_calibration_activations()[0]['status']=='pending'


def test_history_is_persistent_and_public_responses_do_not_include_paths_or_secrets(controller):
    provenance=metadata()
    provenance['private_artifact_path']=str(controller.path.parent/'private-camera.json')
    result=controller.activate_calibration_candidate('front_tags',calibration(515),provenance)
    reloaded=RuntimeController(controller.path,lambda:None)
    history=reloaded.list_calibration_activations()
    assert history[0]['activation_id']==result['activation_id']
    encoded=json.dumps([result,history])
    for forbidden in (str(controller.path),'original-calibration.json','camera-',
                      'private-camera.json','private-password','manual label is not identity'):
        assert forbidden not in encoded
    assert not list(controller._activation_directory.glob('.pending-*'))


@pytest.mark.parametrize('identifier',['../vision.yaml','not-an-id','/tmp/record.json',None])
def test_restore_accepts_only_opaque_activation_ids(controller,identifier):
    with pytest.raises(ValueError,match='Invalid activation ID'):
        controller.restore_calibration_activation(identifier)


def test_unknown_activation_and_unknown_pipeline_are_rejected(controller):
    with pytest.raises(ValueError,match='Unknown activation ID'):
        controller.restore_calibration_activation('0'*32)
    with pytest.raises(ValueError,match='Unknown pipeline'):
        controller.activate_calibration_candidate('missing',calibration(),metadata())
    assert controller.list_calibration_activations()==[]
