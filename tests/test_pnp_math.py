"""Execute the actual shared C++ numerical math on CPU; not GPU validation."""
import ctypes
from pathlib import Path
import shutil
import subprocess
import cv2
import numpy as np
import pytest
from custom_vision.localization import estimate_tag_pose

ROOT=Path(__file__).resolve().parents[1]
PTR=np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')

@pytest.fixture(scope='module')
def mathlib(tmp_path_factory):
    compiler=shutil.which('g++')
    if not compiler:pytest.skip('C++ compiler required for shared CUDA-math host tests')
    out=tmp_path_factory.mktemp('pnp-math')/'math.so'
    subprocess.run([compiler,'-std=c++17','-O2','-fPIC','-shared','-I'+str(ROOT/'native'),str(ROOT/'tests/pnp_math_bridge.cpp'),'-o',str(out)],check=True)
    lib=ctypes.CDLL(str(out));lib.vision_pose_solve.argtypes=[PTR,PTR,ctypes.c_double,ctypes.c_int,PTR]
    lib.vision_pose_project.argtypes=[PTR,PTR,PTR,PTR,PTR,PTR]
    lib.vision_pose_homography.argtypes=[PTR,PTR]
    return lib

C=np.array([750.,755.,640.,400.,-.15,.03,.001,-.001,.005,.001,0.,0.])
K=np.array([[C[0],0,C[2]],[0,C[1],C[3]],[0,0,1.]])
H=.1651/2
POINTS=np.array([[-H,H,0],[H,H,0],[H,-H,0],[-H,-H,0]])

def run(lib,uv,c=C,iterations=30,size=.1651):
    out=np.zeros(30)
    return bool(lib.vision_pose_solve(np.ascontiguousarray(c),np.ascontiguousarray(uv),size,iterations,out)),out

@pytest.mark.parametrize('noise',[0.,.2,1.])
def test_300_seeded_poses_per_noise_level_agree_with_opencv(mathlib,noise):
    rng=np.random.default_rng(2026)
    for _ in range(300):
        r=np.array([*rng.uniform(-.7,.7,2),rng.uniform(-np.pi,np.pi)])
        t=np.array([*rng.uniform(-.5,.5,2),rng.uniform(.4,5)])
        uv=cv2.projectPoints(POINTS,r,t,K,C[4:])[0].reshape(4,2)+rng.normal(0,noise,(4,2))
        ok,out=run(mathlib,uv);assert ok
        ref=estimate_tag_pose(uv,.1651,K,C[4:],max_reprojection_error_px=1000)
        assert ref['pose_valid']
        assert out[12]<=ref['reprojection_error_px']+.002
        assert out[12]<=out[14]+1e-10 # refinement never worsens initialization
        assert 0<=out[13]<=1
        if noise==0:
            np.testing.assert_allclose(out[9:12],t,atol=1e-5)
            np.testing.assert_allclose(out[:9].reshape(3,3),cv2.Rodrigues(r)[0],atol=1e-5)

@pytest.mark.parametrize('angle',[0.,np.pi/2,np.pi,-np.pi/2])
def test_exact_frontal_rotation_and_ambiguity(mathlib,angle):
    c=C.copy();c[4:]=0
    uv=cv2.projectPoints(POINTS,np.array([0.,0.,angle]),np.array([0.,0.,2.]),K,c[4:])[0].reshape(4,2)
    ok,out=run(mathlib,uv,c)
    assert ok and out[12]<1e-6 and out[13]==pytest.approx(1.,abs=1e-5)
    np.testing.assert_allclose(out[:9].reshape(3,3),cv2.Rodrigues(np.array([0.,0.,angle]))[0],atol=1e-6)


def test_analytic_jacobian_against_finite_differences_and_opencv(mathlib):
    R=cv2.Rodrigues(np.array([.3,-.4,2.9]))[0];t=np.array([.2,-.1,1.]);p=POINTS[0].copy()
    uv=np.zeros(2);J=np.zeros((2,6))
    assert mathlib.vision_pose_project(C,R,t,p,uv,J)
    expected=cv2.projectPoints(p[None,:],cv2.Rodrigues(R)[0],t,K,C[4:])[0].reshape(2)
    np.testing.assert_allclose(uv,expected,atol=1e-9)
    for axis in range(6):
        step=np.zeros(6);step[axis]=1e-6;values=[]
        for sign in (1,-1):
            rr=cv2.Rodrigues(step[:3]*sign)[0]@R;tt=t+sign*step[3:];q=np.zeros(2)
            assert mathlib.vision_pose_project(C,rr,tt,p,q,np.zeros((2,6)))
            values.append(q)
        np.testing.assert_allclose(J[:,axis],(values[0]-values[1])/2e-6,rtol=1e-6,atol=1e-6)

@pytest.mark.parametrize('uv',[np.zeros((4,2)),np.array([[0.,0],[1,1],[2,2],[3,3]]),np.full((4,2),np.nan),np.array([[0.,0],[1,1],[0,1],[1,0]])])
def test_degenerate_and_nonfinite_input_invalid(mathlib,uv):
    assert not run(mathlib,uv)[0]


def test_closed_form_homography_matches_projective_maps(mathlib):
    rng=np.random.default_rng(513)
    square=np.c_[POINTS[:,:2]/H,np.ones(4)]
    for _ in range(500):
        expected=np.array([[rng.uniform(.01,.4),rng.uniform(-.03,.03),rng.uniform(-1,1)],
                           [rng.uniform(-.03,.03),rng.uniform(.01,.4),rng.uniform(-1,1)],
                           [rng.uniform(-.3,.3),rng.uniform(-.3,.3),1.]])
        rays=square@expected.T
        corners=np.ascontiguousarray(rays[:,:2]/rays[:,2,None])
        actual=np.zeros((3,3))
        assert mathlib.vision_pose_homography(corners,actual)
        np.testing.assert_allclose(actual,expected,atol=1e-10,rtol=1e-10)
    assert not mathlib.vision_pose_homography(np.zeros((4,2)),np.zeros((3,3)))


@pytest.mark.parametrize('size',[.01,.1651,1.])
@pytest.mark.parametrize('depth',[.6,5.,100.])
def test_tag_scale_and_far_off_axis_translation(mathlib,size,depth):
    r=np.array([.3,-.4,2.1]);t=np.array([.35*depth,-.2*depth,depth])*(size/.1651)
    points=POINTS*(size/.1651)
    uv=cv2.projectPoints(points,r,t,K,C[4:])[0].reshape(4,2)
    ok,out=run(mathlib,uv,size=size)
    assert ok and out[12]<1e-7
    np.testing.assert_allclose(out[9:12],t,atol=1e-7,rtol=1e-8)
    R=out[:9].reshape(3,3)
    np.testing.assert_allclose(R.T@R,np.eye(3),atol=1e-10)
    assert np.linalg.det(R)==pytest.approx(1.,abs=1e-10)


def test_refinement_preserves_independent_alternate_and_seed_ambiguity(mathlib):
    r=np.array([.2,-.1,2.7]);t=np.array([.1,-.2,2.5])
    uv=cv2.projectPoints(POINTS,r,t,K,C[4:])[0].reshape(4,2)
    uv+=np.array([[.13,-.05],[-.2,.1],[.12,.25],[.02,-.1]])
    ok,seed=run(mathlib,uv,iterations=0);refined_ok,refined=run(mathlib,uv,iterations=30)
    assert ok and refined_ok and seed[17]==1
    assert seed[16]==0 and 1<=refined[16]<=30
    assert refined[12]<seed[12]
    np.testing.assert_array_equal(refined[13:16],seed[13:16])
    np.testing.assert_array_equal(refined[18:],seed[18:])
    alternate_uv=cv2.projectPoints(POINTS,cv2.Rodrigues(seed[18:27].reshape(3,3))[0],seed[27:30],K,C[4:])[0].reshape(4,2)
    alternate_error=np.sqrt(np.sum((alternate_uv-uv)**2)/4)
    assert alternate_error==pytest.approx(seed[15],abs=1e-9)
    assert seed[13]==pytest.approx(seed[14]/alternate_error,abs=1e-9)


@pytest.mark.parametrize('index,value',[(0,0.),(1,-1.),(2,np.nan),(3,np.inf),(5,np.nan)])
def test_invalid_camera_rejected(mathlib,index,value):
    uv=cv2.projectPoints(POINTS,np.array([.3,-.4,1.]),np.array([.1,-.2,2.]),K,C[4:])[0].reshape(4,2)
    camera=C.copy();camera[index]=value
    assert not run(mathlib,uv,camera)[0]


def test_arbitrary_rotations_and_oblique_noisy_corners(mathlib):
    rng=np.random.default_rng(93454)
    valid=0
    for _ in range(200):
        axis=rng.normal(0,1,3);r=axis/np.linalg.norm(axis)*rng.uniform(0,np.pi)
        t=np.array([rng.uniform(-.5,.5),rng.uniform(-.4,.4),rng.uniform(.4,8)])
        uv=cv2.projectPoints(POINTS,r,t,K,C[4:])[0].reshape(4,2)+rng.normal(0,.2,(4,2))
        if not cv2.isContourConvex(uv.astype(np.float32)):
            continue
        ok,out=run(mathlib,uv)
        assert ok
        ref=estimate_tag_pose(uv,.1651,K,C[4:],max_reprojection_error_px=1000)
        assert ref['pose_valid']
        assert out[12]<=ref['reprojection_error_px']+.002
        R=out[:9].reshape(3,3)
        np.testing.assert_allclose(R.T@R,np.eye(3),atol=1e-9)
        assert np.linalg.det(R)==pytest.approx(1.,abs=1e-9)
        valid+=1
    assert valid>180
