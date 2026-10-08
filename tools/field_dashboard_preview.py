#!/usr/bin/env python3
"""Local camera-free dashboard preview. Explicit synthetic fixture replay only.

No detector, camera, NT instance or runtime worker is created. Demo configuration
is read-only; all display imports stay in the browser. Ctrl-C stops this server.
"""
from __future__ import annotations
import argparse
import copy
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from custom_vision.dashboard import Dashboard

class ReplayConfig:
    writable=False
    def __init__(self,mount):self.mount=mount
    def get_config(self):
        return {'pipelines':[{'name':'front_tags','type':'apriltag','enabled':True,
            'camera':{'source':'Synthetic replay only','width':1280,'height':800,'fps':1,'fourcc':'MJPG'},
            'settings':{'mode':'3d','backend':'pupil','pose_device':'cpu'},'robot_to_camera':self.mount}],
            'dashboard':{'stream_fps':1},'networktables':{'enabled':False}}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=5844)
    parser.add_argument('--fixture',default='single_tag.json',choices=['single_tag.json','camera_only_localization.json','missing_field_layout.json','same_frame_watchdog.json'])
    args=parser.parse_args()
    mount=None if args.fixture=='camera_only_localization.json' else {'translation_m':[.35,-.17,.42],'rotation_rpy_deg':[5.,-15.,25.]}
    dashboard=Dashboard({'host':'127.0.0.1','port':args.port},ReplayConfig(mount))
    layout=json.loads((ROOT/'protocol/fixture-layouts.json').read_text())['layouts']['scene']
    packet=json.loads((ROOT/'protocol/fixtures'/args.fixture).read_text())
    dashboard.set_field_geometry(layout,{'front_tags':mount})
    dashboard._device_cache={'devices':[],'note':'Synthetic preview; device discovery disabled.'}
    dashboard._device_cache_until=float('inf')
    print(f'SYNTHETIC FIXTURE PREVIEW: http://127.0.0.1:{args.port} (no cameras, NT or robot)',flush=True)
    count=0
    try:
        while True:
            current=copy.deepcopy(packet)
            # Labeled fixture playback identities, not real captures or publications.
            current['packet_seq']=count
            current['frame_id']=42+count
            current['input_kind']='synthetic'
            dashboard.update(current)
            count+=1
            time.sleep(.8)
    except KeyboardInterrupt:
        pass
    finally:
        dashboard.close()

if __name__=='__main__':main()
