"""Real native CPU and optional GPU integration, never simulated GPU results."""
from concurrent.futures import ThreadPoolExecutor
import math

import cv2
import numpy as np
import pytest

from custom_vision.native_apriltags import NativeAprilTagPipeline, native_capabilities

CAL = {'width':1280,'height':800,
       'camera_matrix':[[760.,0.,640.],[0.,765.,400.],[0.,0.,1.]],
       'dist_coeffs':[-.12,.03,.001,-.001,.001,.01,-.004,0.]}
OBJ = np.array([[-.08255,.08255,0.],[.08255,.08255,0.],
                [.08255,-.08255,0.],[-.08255,-.08255,0.]])


def pipeline(device, **settings):
    caps=native_capabilities()
    if not caps.get('available'):
        pytest.skip('native extension is not built')
    if device=='cuda' and (not caps.get('cuda_pose_compiled') or not caps.get('cuda_pose_devices')):
        pytest.skip('real CUDA pose build AND GPU required; CPU math tests are separate')
    return NativeAprilTagPipeline(dict(mode='3d',pose_device=device,**settings),CAL)


def samples(count, noise=0.):
    rng=np.random.default_rng(1086)
    points=[];truth=[]
    for _ in range(count):
        r=np.array([rng.uniform(-.8,.8),rng.uniform(-.8,.8),rng.uniform(-math.pi,math.pi)])
        t=np.array([rng.uniform(-.4,.4),rng.uniform(-.3,.3),rng.uniform(.6,4.)])
        uv=cv2.projectPoints(OBJ,r,t,np.array(CAL['camera_matrix']),np.array(CAL['dist_coeffs']))[0].reshape(4,2)
        points.append(uv+rng.normal(0,noise,uv.shape));truth.append((r,t))
    return np.array(points,dtype=np.float64).reshape(-1,4,2),truth


@pytest.mark.parametrize('device',['cpu','cuda'])
def test_batch_api_recovers_known_poses_and_chunks(device):
    p=pipeline(device)
    try:
        corners,truth=samples(300)
        outputs=p.estimate_poses(corners)
        assert len(outputs)==300
        for result,(r,t) in zip(outputs,truth):
            assert result['pose_valid'] and result['pose_device']==device and result['pose_attempted']
            assert result['reprojection_error_px']<1e-3
            np.testing.assert_allclose(result['tvec_m'],t,atol=1e-4)
        assert p.last_timings['pose_ms']>=0
        if device=='cuda':
            assert p.last_timings['pose_kernel_ms']>0
        old=outputs[0]['tvec_m'][:]
        assert p.estimate_poses(np.empty((0,4,2)))==[]
        p.estimate_poses(corners[:2])
        assert outputs[0]['tvec_m']==old, 'Results must not alias reusable CUDA buffers'
    finally:p.close()


@pytest.mark.parametrize('device',['cpu','cuda'])
def test_bad_shapes_nonfinite_close_and_2d_contract(device):
    p=pipeline(device)
    try:
        for shape in [(4,2),(1,3,2),(4097,4,2)]:
            with pytest.raises(ValueError):p.estimate_poses(np.zeros(shape))
        with pytest.raises(ValueError):p.estimate_poses(np.full((1,4,2),np.nan))
    finally:p.close()
    with pytest.raises(RuntimeError,match='closed'):p.estimate_poses(np.zeros((1,4,2)))


def test_cuda_request_never_silently_falls_back():
    caps=native_capabilities()
    if not caps.get('available'):pytest.skip('native extension is not built')
    if caps.get('cuda_pose_compiled') and caps.get('cuda_pose_devices'):pytest.skip('covered by executing GPU tests')
    with pytest.raises((RuntimeError,ValueError),match='CUDA|cuda'):
        NativeAprilTagPipeline({'mode':'3d','pose_device':'cuda'},CAL)


def test_cuda_requires_supported_camera_and_3d():
    p=pipeline('cuda');p.close()
    for settings,cal in [({'mode':'2d','pose_device':'cuda'},CAL),
                         ({'mode':'3d','pose_device':'cuda'},None),
                         ({'mode':'3d','pose_device':'cuda'},dict(CAL,dist_coeffs=[0.]*12))]:
        with pytest.raises(ValueError):NativeAprilTagPipeline(settings,cal)


def test_cuda_noise_matches_cpu_and_is_thread_safe():
    first=pipeline('cuda');second=pipeline('cuda');cpu=pipeline('cpu')
    try:
        corners,_=samples(64,.2)
        expected=cpu.estimate_poses(corners)
        with ThreadPoolExecutor(max_workers=3) as pool:
            # Calls sharing first serialize; different instances have separate streams.
            futures=[pool.submit(p.estimate_poses,corners) for p in (first,second,first)]
            results=[f.result(timeout=30) for f in futures]
        for outputs in results:
            for ref,result in zip(expected,outputs):
                assert result['pose_valid']==ref['pose_valid']
                if result['pose_valid']:
                    assert result['reprojection_error_px']<=ref['reprojection_error_px']+.003
        for outputs in results[1:]:
            for x,y in zip(results[0],outputs):
                np.testing.assert_allclose(x['tvec_m'],y['tvec_m'],atol=1e-8)
    finally:
        first.close();second.close();cpu.close()


@pytest.mark.parametrize('angle',[0.,math.pi/2,math.pi,-math.pi/2])
def test_cuda_frontal_rotation_and_ambiguity(angle):
    p=pipeline('cuda')
    try:
        cal=np.array(CAL['camera_matrix']);dist=np.array(CAL['dist_coeffs'])
        uv=cv2.projectPoints(OBJ,np.array([0.,0.,angle]),np.array([0.,0.,2.]),cal,dist)[0].reshape(4,2)
        value=p.estimate_poses(uv[None])[0]
        assert value['pose_valid'] and value['reprojection_error_px']<1e-3
        assert value['pose_ambiguity']>.99, 'Frontal ambiguity must not be disguised by refinement'
    finally:p.close()


def test_cuda_reported_errors_match_independent_opencv_projection():
    p=pipeline('cuda')
    try:
        corners,_=samples(80,.5)
        output=p.estimate_poses(corners)
        for observed,value in zip(corners,output):
            assert value['pose_valid']
            for prefix in ('','alternate_'):
                r=np.array(value[prefix+'rvec_rad']);t=np.array(value[prefix+'tvec_m'])
                projected=cv2.projectPoints(OBJ,r,t,np.array(CAL['camera_matrix']),np.array(CAL['dist_coeffs']))[0].reshape(4,2)
                measured_error=float(np.sqrt(np.sum((projected-observed)**2)/4))
                assert value[prefix+'reprojection_error_px']==pytest.approx(measured_error,abs=1e-7)
                R=cv2.Rodrigues(r)[0]
                assert np.min((OBJ@R.T+t)[:,2])>0
    finally:p.close()


def test_cuda_mixed_degenerate_batch_does_not_contaminate_other_tags():
    p=pipeline('cuda')
    try:
        corners,_=samples(4)
        invalid=np.array([np.zeros((4,2)),[[0.,0.],[1.,1.],[2.,2.],[3.,3.]],
                          [[0.,0.],[1.,1.],[0.,1.],[1.,0.]]])
        combined=np.concatenate([corners[:2],invalid,corners[2:]])
        output=p.estimate_poses(combined)
        assert [r['pose_valid'] for r in output]==[True,True,False,False,False,True,True]
        repeated=p.estimate_poses(corners)
        for previous,current in zip(output[:2]+output[-2:],repeated):
            np.testing.assert_allclose(previous['tvec_m'],current['tvec_m'],atol=1e-10)
            np.testing.assert_allclose(previous['rvec_rad'],current['rvec_rad'],atol=1e-10)
    finally:p.close()
