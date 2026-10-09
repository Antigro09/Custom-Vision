"""Guided storage/review integration using declared fake recorded evidence.

These tests never open a camera or execute a calibration solver. Detector output
is controlled to exercise persistence, not claim corner or physical accuracy.
"""
import copy
import json
from pathlib import Path
import time

import cv2
import numpy as np
import pytest

from custom_vision.calibration_capture import CalibrationCapture, recorded_source
from custom_vision.calibration_guided import GuidedCalibrationJobs
from custom_vision.calibration_review import require_candidate_review
from custom_vision.camera_ownership import CameraBusyError


BOARD = {'cols':7,'rows':5,'square_size_m':.03}
MODE = {'width':320,'height':240,'crop':{'x':0,'y':0,'width':320,'height':240},
        'binning':[1,1],'focus':{'kind':'fixed','value':None,'locked':True}}
CAMERA = {'physical_id':'fake-recorded-camera-a','user_label':'Fake recorded fixture',
          'identity_source':'offline_metadata','capture_history':'Test fixture, no hardware'}


class FakeRecordedReader:
    def __init__(self):
        self.closed=False
        self.index=0

    def read(self):
        if self.closed:
            return False,None
        self.index+=1
        # A distinguishable lossless image, intentionally not a real board.
        image=np.full((240,320,3),(40,90,140),np.uint8)
        image[100:140,220:260]=(10,180,210)
        return True,image

    def release(self):
        self.closed=True
        return True


def fake_detector(_frame, board):
    x,y=np.meshgrid(np.arange(board.cols)*20+35,np.arange(board.rows)*20+45)
    return np.stack((x,y),axis=-1).reshape(-1,2).astype(float),{'sharpness_px':1.,'contrast':160.}


def runtime_calibration(quality='heuristics_passed_not_hardware_validated'):
    return {'width':320,'height':240,'camera_matrix':[[300,0,160],[0,305,120],[0,0,1]],
            'dist_coeffs':[0]*8,'quality_status':quality,'robot_mount_calibrated':False}


def ready_capture(capture, *, after_frame=None):
    deadline=time.monotonic()+1
    while time.monotonic()<deadline:
        status=capture.status()
        if status['frame'] is not None and (after_frame is None or status['frame']['frame_id']>after_frame):
            return status
        time.sleep(.005)
    pytest.fail('Bounded fake recorded preview failed to produce a frame')


@pytest.fixture
def capture():
    sources=[recorded_source('recorded-a','Fake recorded source A',lambda _:FakeRecordedReader(),
                             camera=copy.deepcopy(CAMERA),mode=copy.deepcopy(MODE)),
             recorded_source('recorded-b','Fake recorded source B',lambda _:FakeRecordedReader(),
                             camera=copy.deepcopy(CAMERA),mode=copy.deepcopy(MODE))]
    manager=CalibrationCapture(sources,detector=fake_detector,preview_fps=10)
    yield manager
    manager.close()


@pytest.fixture
def manager(tmp_path,capture,monkeypatch):
    monkeypatch.setattr(cv2,'VideoCapture',lambda *_args:pytest.fail('No device may be opened'))
    value=GuidedCalibrationJobs(tmp_path/'store',capture=capture)
    yield value
    value.close()


def create_capture(manager,source_id='recorded-a',board=None):
    return manager.create_session({'board':copy.deepcopy(board or BOARD),'camera':copy.deepcopy(CAMERA),
                                   'mode':copy.deepcopy(MODE),
                                   'input':{'kind':'capture','source_id':source_id}})


def start(manager,record):
    manager.capture_start({'session_id':record['session_id'],
                           'source_id':record['input']['source_id'],'confirmed':True})
    return ready_capture(manager.capture)


def save(manager,status):
    return manager.capture_snapshot({'capture_id':status['capture_id'],'frame_id':status['frame']['frame_id']})


def imported(manager,**changes):
    spec={'calibration':runtime_calibration(),'camera':copy.deepcopy(CAMERA),'mode':copy.deepcopy(MODE)}
    spec.update(changes)
    return manager.import_candidate(spec)


def fake_completed_candidate(manager,session):
    """An explicitly fabricated lifecycle result; no native fitting is implied."""
    public=imported(manager)
    candidate=manager._read('candidates',public['candidate_id'],'candidate.json')
    candidate.update(session_id=session['session_id'],capture_revision=session['capture_revision'],
                     source_kind='recorded',synthetic=False,solver='fake_no_solver')
    job=manager._read('jobs',candidate['job_id'],'job.json')
    job.update(session_id=session['session_id'],solver='fake_no_solver')
    manager._save_job(job)
    path=manager._directory('candidates',candidate['candidate_id'])/'candidate.json'
    path.write_text(json.dumps(candidate))
    return manager.get_candidate(candidate['candidate_id'])


def test_capture_requires_explicit_registered_source_and_confirmation_before_reads(manager):
    assert len(manager.capture_sources()['sources'])==2
    assert manager.capture_status()['state']=='idle'
    with pytest.raises(ValueError,match='registered'):
        create_capture(manager,'unregistered')
    record=create_capture(manager)
    with pytest.raises(ValueError,match='confirmation'):
        manager.capture_start({'session_id':record['session_id'],'source_id':'recorded-a'})
    with pytest.raises(ValueError,match='different source'):
        manager.capture_start({'session_id':record['session_id'],'source_id':'recorded-b','confirmed':True})
    assert manager.capture_status()['state']=='idle'


def test_failed_start_record_write_stops_and_releases_the_explicit_reader(manager,monkeypatch):
    from custom_vision import calibration_guided
    readers=[]
    def factory(_record):
        reader=FakeRecordedReader()
        readers.append(reader)
        return reader
    manager.capture._sources=[recorded_source('recorded-a','Fake recorded source',factory,
                                               camera=copy.deepcopy(CAMERA),mode=copy.deepcopy(MODE))]
    record=create_capture(manager)
    write=calibration_guided.write_json
    def fail_started_record(path,data):
        if Path(path).name=='record.json' and data.get('status')=='capturing':
            # Make the reader definitely active before injecting persistence failure.
            ready_capture(manager.capture)
            raise OSError('injected start record failure')
        return write(path,data)
    with monkeypatch.context() as injection:
        injection.setattr(calibration_guided,'write_json',fail_started_record)
        with pytest.raises(OSError,match='injected start record failure'):
            manager.capture_start({'session_id':record['session_id'],'source_id':'recorded-a','confirmed':True})
    assert len(readers)==1 and readers[0].closed
    stopped=manager.capture_status()
    assert stopped['state']=='stopped' and not stopped['connected'] and stopped['frame'] is None
    assert manager.get_session(record['session_id'])['status']=='new'
    # Failure must release ownership so an explicit retry acquires a fresh reader.
    retry=start(manager,record)
    assert retry['connected'] and len(readers)==2 and not readers[1].closed


def test_snapshot_preserves_supplied_exposure_midpoint_event_in_session_summary(manager,monkeypatch):
    record=create_capture(manager)
    status=start(manager,record)
    assert status['timing']['physical_exposure_timestamp'] is False
    retained=manager._session_record(record['session_id'])
    # Simulated trusted backend metadata only: the actual source remains the
    # fake recorded reader. No platform override, device or clock measurement.
    retained['source_kind']='v4l2'
    (manager._directory('sessions',record['session_id'])/'record.json').write_text(json.dumps(retained))
    snapshot=manager.capture.snapshot
    def with_exposure_metadata(capture_id,frame_id):
        value=snapshot(capture_id,frame_id)
        read_ns=value['timing']['host_read_complete_monotonic_ns']
        value['timing'].update(timestamp_source='hardware_exposure_midpoint',capture_event='exposure_midpoint',
                              capture_monotonic_ns=read_ns-10_000_000,physical_exposure_timestamp=True,
                              exposure_duration_ns=20_000_000,rolling_shutter_assumption='global_shutter',
                              clock_mapping_validated=True,measurement_kind='live',
                              note='Simulated trusted backend metadata; no actual clock or exposure measurement')
        return value
    monkeypatch.setattr(manager.capture,'snapshot',with_exposure_metadata)
    saved=save(manager,status)
    assert saved['session']['summary']['timestamp_source']=='hardware_exposure_midpoint'
    timing=saved['session']['views'][0]['timing']
    assert timing==saved['snapshot']['timing']
    assert timing['capture_event']=='exposure_midpoint' and timing['timestamp_unit']=='ns'
    assert timing['capture_monotonic_ns']==timing['host_read_complete_monotonic_ns']-10_000_000
    directory=manager.root/manager._session_record(record['session_id'])['selected_directory']
    manifest=json.loads((directory/'session.json').read_text())
    assert manifest['acquisition']['timestamp_source']=='hardware_exposure_midpoint'
    assert manifest['acquisition']['measured_capture_time_verified'] is False
    assert manifest['physical_validation'] is False


def test_lossless_snapshot_keeps_overlay_separate_and_repeated_save_is_idempotent(manager):
    record=create_capture(manager)
    status=start(manager,record)
    saved=save(manager,status)
    again=save(manager,status)
    assert saved['snapshot']['snapshot_id']==again['snapshot']['snapshot_id']
    assert saved['session']['capture_revision']==again['session']['capture_revision']==1
    assert len(again['session']['views'])==1 and again['capture']['snapshot_count']==1
    snapshot=saved['snapshot']
    raw=cv2.imdecode(np.frombuffer(manager.get_session_artifact(record['session_id'],snapshot['image'])['body'],np.uint8),cv2.IMREAD_COLOR)
    overlay=cv2.imdecode(np.frombuffer(manager.get_session_artifact(record['session_id'],snapshot['overlay'])['body'],np.uint8),cv2.IMREAD_COLOR)
    assert np.array_equal(raw[100:140,220:260],np.full((40,40,3),(10,180,210),np.uint8))
    assert raw.shape==overlay.shape==(240,320,3)
    assert not np.array_equal(raw,overlay)
    assert len(snapshot['corners_px'])==35
    assert snapshot['image_url']!=snapshot['overlay_url']
    assert snapshot['timing']['timestamp_source']=='host_frame_read_complete'
    assert snapshot['timing']['physical_exposure_timestamp'] is False
    manager.capture_stop({'capture_id':status['capture_id']})
    selected=manager.get_session(record['session_id'])
    assert selected['status']=='selected'
    directory=manager.root/manager._session_record(record['session_id'])['selected_directory']
    retained=json.loads((directory/'session.json').read_text())
    assert retained['complete'] and len(retained['views'])==1 and retained['capture_revision']==1
    assert retained['physical_validation'] is False


@pytest.mark.parametrize('stage',['overlay','overlay_partial','record','commit'])
def test_snapshot_storage_failure_preserves_previous_durable_revision_and_capture_eligibility(manager,monkeypatch,stage):
    record=create_capture(manager)
    status=start(manager,record)
    if stage in ('overlay','overlay_partial'):
        write=Path.write_bytes
        def fail_overlay(path,body):
            if path.parent.name=='overlays':
                if stage=='overlay_partial':
                    write(path,body[:len(body)//2])
                raise OSError('injected overlay store failure')
            return write(path,body)
        monkeypatch.setattr(Path,'write_bytes',fail_overlay)
    elif stage=='record':
        from custom_vision import calibration_guided
        write=calibration_guided.write_json
        failures=[]
        def fail_record_once(path,data):
            if Path(path).name=='record.json' and data['capture_revision']==1 and not failures:
                failures.append(True)
                raise OSError('injected record store failure')
            return write(path,data)
        monkeypatch.setattr(calibration_guided,'write_json',fail_record_once)
    else:
        monkeypatch.setattr(manager.capture,'commit_snapshot',lambda *_args:(_ for _ in ()).throw(OSError('injected volatile commit failure')))
    with pytest.raises(OSError,match='injected'):
        save(manager,status)
    retained=manager.get_session(record['session_id'])
    assert retained['capture_revision']==0 and retained['views']==[]
    assert retained['summary']['accepted_views']==0
    assert manager.capture_status()['snapshot_count']==0
    assert manager.capture_status()['snapshot_eligible']
    session_dir=manager._directory('sessions',record['session_id'])
    assert not list(session_dir.rglob('*.png'))
    for manifest in session_dir.rglob('session.json'):
        partial=json.loads(manifest.read_text())
        assert partial['views']==[] and partial['capture_revision']==0


@pytest.mark.parametrize('bad_frame_id',[True,1.0])
def test_noninteger_frame_id_cannot_replay_a_previous_integer_snapshot(manager,bad_frame_id):
    record=create_capture(manager)
    status=start(manager,record)
    saved=manager.capture_snapshot({'capture_id':status['capture_id'],'frame_id':1})
    assert saved['snapshot']['frame_id']==1
    with pytest.raises(ValueError,match='integer'):
        manager.capture_snapshot({'capture_id':status['capture_id'],'frame_id':bad_frame_id})
    assert manager.get_session(record['session_id'])['capture_revision']==1


def test_stop_resume_retains_saved_coverage_and_rejects_identical_observation(manager):
    record=create_capture(manager)
    first=start(manager,record)
    save(manager,first)
    manager.capture_stop({'capture_id':first['capture_id']})
    second=start(manager,manager.get_session(record['session_id']))
    assert second['capture_id']!=first['capture_id']
    assert second['frame']['frame_id']>first['frame']['frame_id']
    assert second['snapshot_count']==1 and second['coverage_fraction']>0
    assert not second['snapshot_eligible']
    with pytest.raises(ValueError,match='Already covered'):
        save(manager,second)
    assert manager.get_session(record['session_id'])['capture_revision']==1


def test_other_session_source_and_changed_board_never_mix_with_existing_capture(manager):
    first_record=create_capture(manager)
    first=start(manager,first_record)
    save(manager,first)
    second_record=create_capture(manager,'recorded-b',{'cols':8,'rows':5,'square_size_m':.04})
    with pytest.raises(CameraBusyError):
        start(manager,second_record)
    manager.capture_stop({'capture_id':first['capture_id']})
    second=start(manager,second_record)
    save(manager,second)
    assert manager.get_session(first_record['session_id'])['board']==BOARD
    assert manager.get_session(first_record['session_id'])['summary']['accepted_views']==1
    assert manager.get_session(second_record['session_id'])['board']['cols']==8
    assert len(manager.get_session(second_record['session_id'])['views'][0]['corners'])==40
    first_image=manager.snapshots(first_record['session_id'])['snapshots'][0]['image_url']
    second_image=manager.snapshots(second_record['session_id'])['snapshots'][0]['image_url']
    assert first_image!=second_image
    with pytest.raises(ValueError,match='different capture'):
        save(manager,first)


def test_interrupted_workspace_restart_retains_snapshots_without_automatic_acquisition(manager):
    record=create_capture(manager)
    status=start(manager,record)
    save(manager,status)
    manager.close()
    reopened=GuidedCalibrationJobs(manager.root)
    try:
        retained=reopened.get_session(record['session_id'])
        assert retained['status']=='selected' and retained['capture_revision']==1
        assert len(reopened.snapshots(record['session_id'])['snapshots'])==1
        assert reopened.capture_status()['state']=='idle'
        assert reopened.capture_sources()['sources']==[]
    finally:
        reopened.close()


def test_mosaic_contains_every_saved_image_and_real_overlay_pixels(manager):
    record=create_capture(manager)
    first=start(manager,record)
    save(manager,first)
    original=manager.capture.detector
    manager.capture.detector=lambda image,board:(original(image,board)[0]+[60,0],original(image,board)[1])
    second=ready_capture(manager.capture,after_frame=first['frame']['frame_id'])
    # The new detector must finish before requesting a second distinct view.
    deadline=time.monotonic()+1
    while second['frame']['corners_px'][0][0]<90 and time.monotonic()<deadline:
        second=ready_capture(manager.capture,after_frame=second['frame']['frame_id'])
    save(manager,second)
    snapshots=manager.snapshots(record['session_id'])
    assert len(snapshots['snapshots'])==2 and snapshots['coverage_fraction']>0
    mosaic=manager.mosaic(record['session_id'])
    assert mosaic['content_type']=='image/png'
    image=cv2.imdecode(np.frombuffer(mosaic['body'],np.uint8),cv2.IMREAD_COLOR)
    assert image.shape==(164,384,3)
    marker=((image[:,:,0]<35)&(image[:,:,1]>160)&(image[:,:,1]<205)&(image[:,:,2]>190)&(image[:,:,2]<240)).astype(np.uint8)
    count,_labels,stats,_centers=cv2.connectedComponentsWithStats(marker)
    assert sum(stats[index,cv2.CC_STAT_AREA]>100 for index in range(1,count))==2
    green=(image[:,:,1]>190)&(image[:,:,0]<70)&(image[:,:,2]<80)
    assert int(green[:,:192].sum())>35 and int(green[:,192:].sum())>35


def test_solver_requires_stopped_saved_preview_without_fabricating_temporal_validation(manager):
    record=create_capture(manager)
    status=start(manager,record)
    save(manager,status)
    with pytest.raises(ValueError,match='Stop the owned preview'):
        manager.create_job({'session_id':record['session_id'],'operation':'solve','solver':'mrcal'})
    manager.capture_stop({'capture_id':status['capture_id']})
    retained=manager.get_session(record['session_id'])
    assert retained['summary']['timestamp_source']=='preview_host_read_complete_not_sensor_time'
    assert manager._native_validation_basis(retained)=='image_groups'
    with pytest.raises(ValueError,match='already selected'):
        manager.create_job({'session_id':record['session_id'],'operation':'select'})


def test_import_requires_matching_resolution_and_preserves_downloadable_unapplied_result(manager):
    mismatched=copy.deepcopy(MODE)
    mismatched['width']=640
    with pytest.raises(ValueError,match='dimensions do not match'):
        imported(manager,mode=mismatched)
    candidate=imported(manager)
    assert candidate['solver']=='imported' and not candidate['reviewed']
    data=json.loads(manager.get_artifact(candidate['candidate_id'],'intrinsics.candidate.json')['body'])
    assert data['calibration_verified'] is False and data['robot_mount_calibrated'] is False
    assert data['camera_matrix']==runtime_calibration()['camera_matrix']
    diagnostics=manager.diagnostics(candidate['candidate_id'])
    assert diagnostics['status']=='unavailable' and diagnostics['board'] is None
    assert diagnostics['frames']==[]  # importing intrinsics cannot invent solved board geometry
    assert candidate['review_status']['review_allowed']
    reviewed=manager.review(candidate['candidate_id'],{'confirmed':True})
    assert reviewed['reviewed'] and not reviewed['review_status']['physical_verification']
    require_candidate_review(reviewed['calibration'],manager.candidate_metadata(candidate['candidate_id']))


def test_imported_needs_review_is_downloadable_but_confirmation_never_bypasses_quality_gate(manager):
    candidate=imported(manager,calibration=runtime_calibration('needs_review'))
    assert not candidate['review_status']['review_allowed'] and not candidate['reviewed']
    assert json.loads(manager.get_artifact(candidate['candidate_id'],'intrinsics.candidate.json')['body'])['quality_status']=='needs_review'
    with pytest.raises(ValueError,match='quality'):
        manager.review(candidate['candidate_id'],{'confirmed':True})


def test_imported_unknown_camera_can_be_reviewed_for_export_without_physical_eligibility(manager):
    camera=copy.deepcopy(CAMERA)
    camera['physical_id']=None
    mode=copy.deepcopy(MODE)
    mode.update(crop=None,binning=None,focus={'kind':'unknown','value':None,'locked':False})
    candidate=imported(manager,camera=camera,mode=mode)
    reviewed=manager.review(candidate['candidate_id'],{'confirmed':True})
    assert reviewed['reviewed'] and not reviewed['review_status']['activation_provenance_available']
    with pytest.raises(ValueError):
        require_candidate_review(reviewed['calibration'],manager.candidate_metadata(candidate['candidate_id']))


@pytest.mark.parametrize('location',['calibration','report','nested_provenance'])
def test_import_cannot_strip_synthetic_provenance_by_supplied_false_flag(manager,location):
    data=runtime_calibration()
    report={'status':data['quality_status'],'issues':[]}
    if location=='calibration': data['synthetic']=True
    elif location=='report': report['source_kind']='synthetic'
    else: data['calibration_provenance']={'source_kind':'synthetic','synthetic':True}
    candidate=imported(manager,calibration=data,report=report,synthetic=False)
    reviewed=manager.review(candidate['candidate_id'],{'confirmed':True})
    assert reviewed['synthetic'] is True
    assert not reviewed['review_status']['activation_provenance_available']
    with pytest.raises(ValueError,match='Synthetic'):
        require_candidate_review(reviewed['calibration'],manager.candidate_metadata(candidate['candidate_id']))


def test_new_saved_observation_stales_previously_reviewed_candidate_without_changing_geometry(manager):
    record=create_capture(manager)
    status=start(manager,record)
    current=save(manager,status)['session']
    candidate=fake_completed_candidate(manager,current)
    reviewed=manager.review(candidate['candidate_id'],{'confirmed':True})
    assert reviewed['reviewed'] and not reviewed['stale']
    detector=manager.capture.detector
    manager.capture.detector=lambda image,board:(detector(image,board)[0]+[60,0],detector(image,board)[1])
    newer=ready_capture(manager.capture,after_frame=status['frame']['frame_id'])
    save(manager,newer)
    stale=manager.get_candidate(candidate['candidate_id'])
    assert stale['stale'] and not stale['reviewed']
    assert stale['calibration']['camera_matrix']==reviewed['calibration']['camera_matrix']
    with pytest.raises(ValueError,match='stale'):
        manager.review(candidate['candidate_id'],{'confirmed':True})
    with pytest.raises(ValueError,match='stale'):
        require_candidate_review(stale['calibration'],manager.candidate_metadata(candidate['candidate_id']))
