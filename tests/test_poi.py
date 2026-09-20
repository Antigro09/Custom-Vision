import copy
import math
import numpy as np
import pytest
from custom_vision.poi import PointOfInterestTracker, validate_poi
from custom_vision.overlay import annotate
from custom_vision.localization import Localization

CAL = dict(width=1280, height=800, camera_matrix=[[750.,0,640.],[0,750.,400.],[0,0,1]], dist_coeffs=[0.]*8)
TAG = dict(id=7, pose_valid=True, rvec_rad=[0.,0.,math.pi], tvec_m=[0.,0.,2.],
           pose_ambiguity=.05,reprojection_error_px=.1,
           center=[640.,400.], corners=[[670,370],[610,370],[610,430],[670,430]])

def tracker(offset=(0,0,0), **kw):
    return PointOfInterestTracker(dict(enabled=True,calibration_verified=True,
        targets=[dict(name='aim',tag_id=7,offset_m=list(offset))],**kw),CAL)

@pytest.mark.parametrize('offset,point,signs', [
    ([0,0,0],[0,0,2],[0,0]), ([0,.2,.4],[.2,-.4,2],[1,1]),
    ([-.3,-.2,-.1],[-.2,.1,2.3],[-1,-1]), ([.3,0,0],[0,0,1.7],[0,0])])
def test_tag_frame_offsets_and_angle_signs(offset,point,signs):
    pipe=tracker(offset);before=copy.deepcopy(TAG)
    result=pipe.process([TAG],(800,1280,3),12.5);out=result['targets'][0]
    assert result['valid'] and result['selected_name']=='aim'
    assert not result['uses_odometry'] and not result['uses_field_layout']
    assert out['camera_translation_m']==pytest.approx(point)
    assert [np.sign(out['tx_deg']),np.sign(out['ty_deg'])]==signs
    assert out['robot_translation_m'] is None
    assert TAG==before


def test_mount_offset_changes_robot_bearing_not_camera_angles():
    pipe=tracker([0,.2,.4])
    mounted=PointOfInterestTracker(pipe.settings,CAL,dict(translation_m=[.5,.1,.2],rotation_rpy_deg=[0,0,90]))
    a=pipe.process([TAG],(800,1280),1)['targets'][0]
    b=mounted.process([TAG],(800,1280),1)['targets'][0]
    assert b['camera_translation_m']==a['camera_translation_m']
    assert b['robot_translation_m']==pytest.approx([.7,2.1,.6])
    assert b['robot_yaw_deg']==pytest.approx(math.degrees(math.atan2(2.1,.7)))


@pytest.mark.parametrize('roll', [0., 90., 180., 270.])
def test_offset_rotates_with_decoded_tag_not_image_axes(roll):
    # Upright tag +Y points image-right. Rolling the printed tag clockwise
    # rotates that same physical offset toward image-down, left, then up.
    angle=math.radians(roll)
    tag=dict(TAG,rvec_rad=[0.,0.,math.pi+angle])
    target=tracker([0,.3,.1]).process([tag],(800,1280),1)['targets'][0]
    assert target['valid']
    assert target['camera_translation_m']==pytest.approx(
        [.3*math.cos(angle)+.1*math.sin(angle),
         .3*math.sin(angle)-.1*math.cos(angle),2.],abs=1e-12)


def test_distortion_changes_preview_pixel_not_physical_aim_angle():
    pipe=tracker([0,.5,.2])
    distorted=PointOfInterestTracker(pipe.settings,dict(CAL,dist_coeffs=[.2,-.01,.005,-.004,0.]))
    a=pipe.process([TAG],(800,1280),1)['targets'][0]
    b=distorted.process([TAG],(800,1280),1)['targets'][0]
    assert a['pixel']!=b['pixel']
    assert a['tx_deg']==b['tx_deg'] and a['ty_deg']==b['ty_deg']
    assert a['camera_translation_m']==b['camera_translation_m']


def test_field_derived_tag_pose_cannot_claim_independent_aim():
    result=tracker().process([dict(TAG,pose_source='field_layout_multitag')],(800,1280),1)
    assert not result['valid'] and not result['uses_field_layout']
    assert result['targets'][0]['invalid_reason']=='field_derived_tag_pose'
    assert not result['targets'][0]['geometry_valid']


@pytest.mark.parametrize('case,reason', [('absent','tag_not_visible'),('duplicate','duplicate_tag_id'),
    ('invalid','reprojection_error'),('ambiguous','ambiguous_tag_pose'),('high_error','reprojection_error'),
    ('quality_missing','unknown_pose_quality')])
def test_invalid_aim_and_no_stale_history(case,reason):
    pipe=tracker();assert pipe.process([TAG],(800,1280),1)['valid']
    tag=copy.deepcopy(TAG);ds=[tag]
    if case=='absent':ds=[]
    elif case=='duplicate':ds=[tag,copy.deepcopy(tag)]
    elif case=='invalid':tag.update(pose_valid=False,pose_invalid_reason='reprojection_error')
    elif case=='ambiguous':tag['pose_ambiguity']=.8
    elif case=='high_error':tag['reprojection_error_px']=999
    elif case=='quality_missing':tag.pop('pose_ambiguity')
    result=pipe.process(ds,(800,1280),2)
    assert not result['valid'] and result['selected_name'] is None
    assert result['targets'][0]['invalid_reason']==reason


@pytest.mark.parametrize('calibration,verified', [(CAL,False),(dict(CAL,benchmark_only=True),True)])
def test_nominal_calibration_is_preview_only_even_with_loose_detector_threshold(calibration,verified):
    pipe=PointOfInterestTracker(dict(enabled=True,calibration_verified=verified,targets=[dict(name='aim',tag_id=7,offset_m=[0,0,.2])]),calibration)
    result=pipe.process([TAG],(800,1280),1)
    assert not result['valid'] and result['targets'][0]['geometry_valid']
    assert result['targets'][0]['invalid_reason']=='calibration_not_verified'


def test_behind_camera_and_resolution_fail_closed():
    assert tracker([3,0,0]).process([TAG],(800,1280),1)['targets'][0]['invalid_reason']=='poi_behind_camera'
    assert tracker().process([TAG],(720,1280),1)['invalid_reason']=='calibration_resolution_mismatch'


def test_out_of_frame_point_can_have_valid_geometry_but_no_mount_bearing():
    target=tracker([0,5,0]).process([TAG],(800,1280),1)['targets'][0]
    assert target['valid'] and not target['in_image']
    assert target['tx_deg']==pytest.approx(math.degrees(math.atan2(5,2)))
    assert target['robot_yaw_deg'] is None


@pytest.mark.parametrize('key,value', [('pose_ambiguity',-1),('pose_ambiguity',True),
    ('pose_ambiguity',float('nan')),('reprojection_error_px',float('inf')),
    ('reprojection_error_px',None),('reprojection_error_px','0.1')])
def test_unknown_quality_never_aims(key,value):
    target=tracker().process([dict(TAG,**{key:value})],(800,1280),1)['targets'][0]
    assert not target['valid'] and target['geometry_valid']
    assert target['invalid_reason']=='unknown_pose_quality'


@pytest.mark.parametrize('key,value', [('rvec_rad',[0.,float('nan'),0.]),
    ('tvec_m',[0.,0.,float('inf')]),('rvec_rad',[0.,0.]),('tvec_m',None)])
def test_malformed_pose_never_produces_geometry(key,value):
    target=tracker().process([dict(TAG,**{key:value})],(800,1280),1)['targets'][0]
    assert not target['valid'] and not target['geometry_valid']
    assert target['pixel'] is None
    assert target['invalid_reason']=='malformed_pose'


def test_missing_calibration_fails_closed_and_timestamp_is_exact():
    result=PointOfInterestTracker(tracker().settings).process([TAG],(800,1280),1.234567)
    assert result['capture_monotonic_us']==1234567
    assert not result['valid'] and result['invalid_reason']=='no_calibration'
    assert result['targets'][0]['tx_deg'] is None


def test_configuration_priority_and_invisible_fallback():
    settings=dict(enabled=True,calibration_verified=True,targets=[dict(name='first',tag_id=8,offset_m=[0,0,0]),dict(name='second',tag_id=7,offset_m=[0,0,0])])
    pipe=PointOfInterestTracker(settings,CAL)
    assert pipe.process([TAG],(800,1280),1)['selected_name']=='second'
    assert pipe.process([TAG,dict(TAG,id=8)],(800,1280),2)['selected_name']=='first'


@pytest.mark.parametrize('settings', [dict(enabled=True),dict(enabled='yes'),dict(targets='bad'),
    dict(targets=[dict(name='x',tag_id=True,offset_m=[0,0,0])]),
    dict(targets=[dict(name='x',tag_id=1,offset_m=[0,float('nan'),0])]),
    dict(targets=[dict(name='x',tag_id=1,offset_m=[0,0,0],frame='cv')]),dict(max_ambiguity=2),dict(max_reprojection_error_px=1000),dict(extra=1)])
def test_validation(settings):
    with pytest.raises(ValueError):validate_poi(settings)


def test_box_depth_and_poi_overlay_preserve_raw_pixels():
    frame=np.zeros((800,1280,3),np.uint8)
    short=annotate(frame,[TAG],CAL)
    cube=annotate(frame,[TAG],CAL,box_depth_ratio=1.)
    assert not frame.any() and np.count_nonzero(short!=cube)>100
    poi=tracker([0,.3,.3]).process([TAG],frame.shape,1)
    marked=annotate(frame,[TAG],CAL,poi=poi)
    x,y=np.rint(poi['targets'][0]['pixel']).astype(int)
    assert np.count_nonzero(marked[y-10:y+10,x-10:x+10])>10
    assert not frame.any()
    assert np.array_equal(annotate(frame,[TAG],None,poi=poi),annotate(frame,[TAG],None))


def test_failed_native_pose_is_not_recomputed_on_cpu(monkeypatch):
    pipe=Localization({},CAL,None,None)
    def forbidden(*a,**kw):raise AssertionError('silent retry')
    monkeypatch.setattr('custom_vision.localization.estimate_tag_pose',forbidden)
    tag=dict(TAG,pose_valid=False,pose_attempted=True,pose_device='cuda',pose_invalid_reason='cuda_pnp_failed')
    for key in ('rvec_rad','tvec_m','pose_ambiguity'):tag.pop(key)
    out=pipe.enrich([tag],(800,1280))['detections'][0]
    assert not out['pose_valid'] and out['pose_invalid_reason']=='cuda_pnp_failed'


def test_poi_factory_keeps_single_tag_solve_independent_of_field_map(monkeypatch):
    from custom_vision import app, localization, native_apriltags
    class Localizer:
        field_tags={7:None,8:None}
        def __init__(self,*_):pass
    class Detector:
        def __init__(self,settings,calibration):self.settings=settings
        def _estimate_pose(self,_):raise AssertionError('not called in constructor')
    monkeypatch.setattr(localization,'Localization',Localizer)
    monkeypatch.setattr(native_apriltags,'NativeAprilTagPipeline',Detector)
    cfg=dict(type='apriltag',settings={'backend':'native','multitag':True},calibration_data=CAL,
             poi={'enabled':True,'targets':[{'name':'aim','tag_id':7,'offset_m':[0,0,0]}]})
    detector=app.make_detector(cfg,{'field_layout_data':{'tags':[]}})
    assert not detector.settings.get('skip_single_when_multi')
    assert detector.poi is not None
    cfg['poi']['enabled']=False
    assert app.make_detector(cfg,{'field_layout_data':{'tags':[]}}).settings['skip_single_when_multi']


def test_nt4_poi_round_trip_and_clear(tmp_path):
    import socket
    import time
    from custom_vision.publisher import Publisher
    ntcore=pytest.importorskip('ntcore')
    with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    def wait(predicate):
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            if predicate():return True
            time.sleep(.01)
        return False
    server=ntcore.NetworkTableInstance.create();pub=Publisher({'enabled':False});subscriptions=[]
    try:
        server.startServer(str(tmp_path/'poi-nt.json'),'127.0.0.1',0,port)
        pub.instance=ntcore.NetworkTableInstance.create();pub.instance.setServer('127.0.0.1',port)
        pub.instance.startClient4('poi-contract-test')
        table=server.getTable('/CustomVision/tags')
        valid=table.getBooleanTopic('poi_valid').subscribe(False)
        name=table.getStringTopic('poi_name').subscribe('')
        tag=table.getIntegerTopic('poi_tag_id').subscribe(-1)
        tx=table.getDoubleTopic('poi_tx_deg').subscribe(0.)
        ty=table.getDoubleTopic('poi_ty_deg').subscribe(0.)
        xyz=table.getDoubleArrayTopic('poi_camera_xyz').subscribe([])
        robot_xyz=table.getDoubleArrayTopic('poi_robot_xyz').subscribe([])
        robot_yaw=table.getDoubleArrayTopic('poi_robot_yaw_deg').subscribe([])
        fps=table.getDoubleTopic('fps').subscribe(0.)
        subscriptions.extend([valid,name,tag,tx,ty,xyz,robot_xyz,robot_yaw,fps])
        assert wait(pub.instance.isConnected)
        mounted=PointOfInterestTracker(tracker([0,.2,.3]).settings,CAL,
                                      dict(translation_m=[.5,.1,.2],rotation_rpy_deg=[0,0,0]))
        poi=mounted.process([TAG],(800,1280),time.monotonic())
        data=dict(pipeline='tags',connected=True,frame_id=1,detections=[TAG],latency_ms=1.,fps=29.5,poi=poi)
        pub.publish(data)
        assert wait(lambda: valid.get() and name.get()=='aim' and tag.get()==7 and len(xyz.get())==3
                    and len(robot_xyz.get())==3 and len(robot_yaw.get())==1 and fps.get()==29.5)
        assert tx.get()==pytest.approx(math.degrees(math.atan2(.2,2.)))
        assert ty.get()==pytest.approx(math.degrees(math.atan2(.3,2.)))
        assert robot_xyz.get()==pytest.approx([2.5,-.1,.5])
        assert robot_yaw.get()==pytest.approx([math.degrees(math.atan2(-.1,2.5))])
        # A nominal-calibration geometry preview must clear all typed aim output.
        nominal=PointOfInterestTracker(tracker().settings,dict(CAL,benchmark_only=True))
        pub.publish(dict(data,poi=nominal.process([TAG],(800,1280),time.monotonic())))
        assert wait(lambda:not valid.get() and name.get()=='' and tag.get()==-1 and len(xyz.get())==0
                    and len(robot_xyz.get())==0 and len(robot_yaw.get())==0 and tx.get()==0. and ty.get()==0.)
        pub.publish(data);assert wait(valid.get)
        # Watchdog/shutdown error has no POI extras and must invalidate immediately.
        pub.publish(dict(data,connected=False,poi=None,fps=0.,detections=[]))
        assert wait(lambda:not valid.get() and len(xyz.get())==0 and fps.get()==0.)
    finally:
        pub.close()
        for sub in subscriptions:sub.close()
        server.stopServer();ntcore.NetworkTableInstance.destroy(server)
