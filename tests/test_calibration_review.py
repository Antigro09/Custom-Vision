"""Reviewing software evidence does not authorize unknown/synthetic hardware use."""
import copy

import pytest

from custom_vision.calibration_review import (candidate_provenance, candidate_review_status,
                                            require_candidate_review, review_candidate)


def calibration():
    return {'width':640,'height':480,'camera_matrix':[[500,0,320],[0,500,240],[0,0,1]],
            'dist_coeffs':[0]*5,'quality_status':'heuristics_passed_not_hardware_validated'}


def metadata():
    return {'camera':{'physical_id':'camera-serial-a'},
            'mode':{'width':640,'height':480,'crop':{'x':0,'y':0,'width':640,'height':480},
                    'binning':[1,1],'focus':{'kind':'fixed','value':None,'locked':True}},
            'report':{'status':'heuristics_passed_not_hardware_validated','issues':[]}}


def test_review_does_not_mutate_inputs_or_certify_physical_hardware():
    data, provenance = calibration(), metadata()
    original=copy.deepcopy((data,provenance))
    pending=candidate_review_status(data,provenance)
    assert pending['review_allowed'] and not pending['reviewed']
    reviewed=review_candidate(data,provenance,confirmed=True)
    status=require_candidate_review(data,reviewed)
    assert status['reviewed'] and status['activation_provenance_available']
    assert status['physical_verification'] is False
    assert (data,provenance)==original


@pytest.mark.parametrize('change',['geometry','quality','issues','stale'])
def test_review_cannot_survive_geometry_or_quality_changes(change):
    data, provenance=calibration(),metadata()
    reviewed=review_candidate(data,provenance,confirmed=True)
    if change=='geometry': data['camera_matrix'][0][0]=510
    elif change=='quality': data['quality_status']='needs_review'
    elif change=='stale': reviewed['stale']=True
    else: reviewed['report']['issues']=['unresolved lens model disagreement']
    with pytest.raises(ValueError): require_candidate_review(data,reviewed)


@pytest.mark.parametrize('quality',['needs_review','failed','selection_requires_review',None])
def test_unknown_or_unacceptable_quality_cannot_be_confirmed(quality):
    data=calibration()
    data['quality_status']=quality
    provenance=metadata()
    provenance.pop('report')
    provenance['review']={'confirmed':True,'quality_status':quality}
    assert candidate_review_status(data,provenance)['review_allowed'] is False
    with pytest.raises(ValueError): review_candidate(data,provenance,confirmed=True)


@pytest.mark.parametrize('confirmed',[False,None,1,'true'])
def test_review_requires_literal_explicit_confirmation(confirmed):
    with pytest.raises(ValueError,match='explicit confirmation'):
        review_candidate(calibration(),metadata(),confirmed=confirmed)


def test_diagnostic_baseline_is_distinct_from_native_quality():
    data=calibration()
    data['quality_status']='baseline_not_hardware_validated'
    provenance=metadata()
    provenance['report']['status']=data['quality_status']
    result=review_candidate(data,provenance,confirmed=True)
    status=require_candidate_review(data,result)
    assert status['diagnostic_baseline'] and not status['physical_verification']


@pytest.mark.parametrize('synthetic_metadata',[{'synthetic':True}, {'source_kind':'synthetic'},
                                             {'input_kind':'synthetic'}, {'source':{'kind':'synthetic'}}])
def test_synthetic_can_be_reviewed_and_exported_but_never_physically_activated(synthetic_metadata):
    provenance=metadata()
    provenance.update(synthetic_metadata)
    reviewed=review_candidate(calibration(),provenance,confirmed=True)
    status=candidate_review_status(calibration(),reviewed)
    assert status['reviewed'] and status['review_allowed']
    assert not status['activation_provenance_available']
    with pytest.raises(ValueError,match='Synthetic'):
        require_candidate_review(calibration(),reviewed)


def test_unknown_camera_and_mode_can_be_reviewed_for_export_only():
    reviewed=review_candidate(calibration(),{},confirmed=True)
    status=candidate_review_status(calibration(),reviewed)
    assert status['reviewed'] and not status['activation_provenance_available']
    with pytest.raises(ValueError,match='camera and mode'):
        require_candidate_review(calibration(),reviewed)


@pytest.mark.parametrize('section,field,value',[
    ('camera','physical_id','unknown'),('camera','physical_id',None),
    ('mode','width',640.),('mode','height',True),('mode','crop',None),
    ('mode','crop',{'x':0,'y':0,'width':False,'height':480}),('mode','binning',[1,True]),
    ('mode','focus',{'kind':'unknown','value':None,'locked':False}),
    ('mode','focus',{'kind':'manual','value':None,'locked':True}),
    ('mode','focus',{'kind':'manual','value':float('nan'),'locked':True}),
    ('mode','focus',{'kind':'fixed','value':None,'locked':False}),
])
def test_exact_provenance_is_required(section,field,value):
    provenance=metadata()
    provenance[section][field]=value
    assert not candidate_review_status(calibration(),provenance)['activation_provenance_available']
    with pytest.raises(ValueError): candidate_provenance(provenance)


def test_status_does_not_expose_camera_identity_private_paths_or_labels():
    provenance=metadata()
    provenance['private_path']='/private/calibration.json'
    provenance['camera']['label']='private label'
    status=candidate_review_status(calibration(),provenance)
    assert 'private' not in repr(status)
    assert 'camera-serial-a' not in repr(status)
