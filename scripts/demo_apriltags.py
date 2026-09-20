#!/usr/bin/env python3
"""Explicit synthetic bench demo: browser controls and MultiTag, with NT disabled."""
import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np
import yaml


def fixture():
    width,height=1280,800
    camera=np.array([1.,2.,.6])
    matrix=np.array([[900.,0,640],[0,900.,400],[0,0,1.]])
    half=.1651/2
    local=np.array([[0,half,half],[0,-half,half],[0,-half,-half],[0,half,-half]])
    # Known upright wall tag poses, facing a forward-facing camera at (1,2,.6).
    rotations=np.diag([-1.,-1.,1.])
    nwu_to_cv=np.array([[0,-1,0],[0,0,-1],[1,0,0.]])
    placements=[(7,[3,1.6,1.0]),(12,[3,2.4,1.0]),(20,[3.3,2.0,.8])]
    image=np.full((height,width),200,np.uint8)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    tags=[]
    src=np.array([[240.,0],[0,0],[0,240],[240,240]],np.float32)
    for ident,position in placements:
        xyz=np.array(position)
        points=(local@rotations.T+xyz-camera)@nwu_to_cv.T
        projected=points@matrix.T
        pixels=(projected[:,:2]/projected[:,2:]).astype(np.float32)
        marker=cv2.aruco.generateImageMarker(dictionary,ident,240)
        transform=cv2.getPerspectiveTransform(src,pixels)
        warped=cv2.warpPerspective(marker,transform,(width,height),flags=cv2.INTER_NEAREST,borderValue=255)
        mask=cv2.warpPerspective(np.full((240,240),255,np.uint8),transform,(width,height),flags=cv2.INTER_NEAREST,borderValue=0)
        image[mask>0]=warped[mask>0]
        tags.append({'ID':ident,'pose':{'translation':dict(zip('xyz',position)),'rotation':{'quaternion':{'W':0,'X':0,'Y':0,'Z':1}}}})
    return cv2.cvtColor(image,cv2.COLOR_GRAY2BGR),dict(width=width,height=height,camera_matrix=matrix.tolist(),dist_coeffs=[0]*5),{'field':{'length':16.5,'width':8.2},'tags':tags}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=5802)
    parser.add_argument('--backend',choices=['native','pupil'],default='native')
    parser.add_argument('--detector-device',choices=['cpu','cuda'],default='cpu')
    parser.add_argument('--pose-device',choices=['cpu','cuda'],default='cpu')
    parser.add_argument('--poi',action='store_true',help='Show preview-only tag-relative offset geometry')
    parser.add_argument('--max-frames',type=int,default=0)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    directory=root/'data'/'apriltag-demo'
    directory.mkdir(parents=True,exist_ok=True)
    image,calibration,layout=fixture()
    # This is a mathematically defined scene, never a measured physical camera.
    (directory/'calibration.json').write_text(json.dumps(dict(calibration,benchmark_only=True),indent=2))
    (directory/'field.json').write_text(json.dumps(layout,indent=2))
    config=yaml.safe_load((root/'config'/'vision.yaml').read_text())
    config['networktables']['enabled']=False
    config['dashboard'].update(host='127.0.0.1',port=args.port)
    config['field_layout']='field.json'
    config['pipelines']=config['pipelines'][:1]
    cfg=config['pipelines'][0]
    cfg['name']='synthetic_demo'
    cfg['camera'].update(source='synthetic-demo',width=1280,height=800,fps=30)
    cfg['input_kind']='synthetic'
    cfg['calibration']='calibration.json'
    cfg['settings']['backend']=args.backend
    cfg['settings']['detector_device']=args.detector_device
    cfg['settings']['pose_device']=args.pose_device
    cfg['settings']['max_ambiguity']=.3
    cfg['robot_to_camera']={'translation_m':[.25,.1,.45],'rotation_rpy_deg':[0,0,0]}
    if args.poi:
        cfg['poi']={'enabled':True,'calibration_verified':False,'max_ambiguity':.3,
                    'targets':[{'name':'demo_aim','tag_id':7,'offset_m':[0.,0.,.1]}]}
        cfg['preview']['box_depth_ratio']=1.
    path=directory/'vision.yaml'
    path.write_text(yaml.safe_dump(config,sort_keys=False))
    from custom_vision import app
    class SyntheticCamera:
        def __init__(self,settings):
            self.period=1/settings['fps']
            self.next_frame=time.monotonic()
            self.last_capture_monotonic=0
        def read(self):
            delay=self.next_frame-time.monotonic()
            if delay>0: time.sleep(delay)
            self.last_capture_monotonic=time.monotonic()
            self.next_frame=self.last_capture_monotonic+self.period
            return True,image
        def release(self): pass
    app.open_camera=SyntheticCamera  # Scoped to this explicit bench-demo process only.
    print(f'SYNTHETIC DEMO ONLY. NetworkTables disabled. http://127.0.0.1:{args.port}',flush=True)
    return app.main(['--config',str(path),'--no-nt','--max-frames',str(args.max_frames)])


if __name__=='__main__':
    raise SystemExit(main())
