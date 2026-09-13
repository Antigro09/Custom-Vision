#!/usr/bin/env python3
"""Synthetic floor-ball geometry/UI demonstration; NetworkTables always disabled."""
import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import yaml


def fixture():
    from custom_vision.localization import CV_TO_NWU, _rpy_rotation
    width,height=1280,800
    matrix=np.array([[600.,0,640],[0,600.,400],[0,0,1.]])
    origin=np.array([.25,0,.65]);mount={'translation_m':origin.tolist(),'rotation_rpy_deg':[0,35,0]}
    points=np.array([[1.1,-.25,.05],[1.7,.3,.05],[.85,.15,.05]])
    rotation=_rpy_rotation(mount['rotation_rpy_deg'])@CV_TO_NWU
    camera_points=(points-origin)@rotation
    pixels=camera_points@matrix.T;pixels=pixels[:,:2]/pixels[:,2:]
    image=np.full((height,width,3),25,np.uint8)
    for uv,point in zip(pixels,camera_points):
        radius=round(.05*600/point[2])
        cv2.circle(image,tuple(np.round(uv).astype(int)),radius,(240,240,240),-1)
    calibration={'width':width,'height':height,'camera_matrix':matrix.tolist(),'dist_coeffs':[0]*5}
    return image,calibration,mount,points


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',type=int,default=5802)
    p.add_argument('--max-frames',type=int,default=0)
    p.add_argument('--headless',action='store_true')
    p.add_argument('--stdout-json',action='store_true')
    p.add_argument('--clear-after',type=int,default=0,help='After this many frames render empty floor')
    a=p.parse_args()
    if a.max_frames<0 or a.clear_after<0:p.error('frame counts must be nonnegative')
    root=Path(__file__).resolve().parents[1];directory=root/'data'/'object-demo';directory.mkdir(parents=True,exist_ok=True)
    image,calibration,mount,_=fixture()
    (directory/'calibration.json').write_text(json.dumps(calibration))
    config=yaml.safe_load((root/'config'/'objects.yaml').read_text())
    config['networktables']['enabled']=False
    config['dashboard'].update(host='127.0.0.1',port=a.port)
    config['pipelines']=config['pipelines'][:1];cfg=config['pipelines'][0]
    cfg.update(name='synthetic_objects',input_kind='synthetic',calibration='calibration.json',robot_to_camera=mount)
    cfg['camera'].update(source='synthetic-demo',width=1280,height=800,fps=30)
    cfg['settings']={'backend':'contour','label':'synthetic_ball_candidate','threshold':180,'min_circularity':.6,'max_detections':16}
    cfg['geometry'].update(target_height_m=.05,intake_offset_m=[.5,0])
    path=directory/'vision.yaml';path.write_text(yaml.safe_dump(config,sort_keys=False))
    from custom_vision import app
    class SyntheticCamera:
        def __init__(self,settings):self.period=1/settings['fps'];self.next_frame=time.monotonic();self.count=0;self.last_capture_monotonic=0
        def read(self):
            delay=self.next_frame-time.monotonic()
            if delay>0:time.sleep(delay)
            self.last_capture_monotonic=time.monotonic();self.next_frame=self.last_capture_monotonic+self.period
            self.count+=1
            frame=np.full_like(image,25) if a.clear_after and self.count>a.clear_after else image
            return True,frame
        def release(self):pass
    app.open_camera=SyntheticCamera
    print(f'SYNTHETIC geometry demo; contour candidates, no trained model, NT disabled. http://127.0.0.1:{a.port}',file=sys.stderr)
    args=['--config',str(path),'--no-nt','--max-frames',str(a.max_frames)]
    if a.headless:args.append('--headless')
    if a.stdout_json:args.append('--stdout-json')
    return app.main(args)


if __name__=='__main__':raise SystemExit(main())
