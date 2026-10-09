#!/usr/bin/env python3
"""Local camera-free dashboard preview. Explicit synthetic fixture replay only.

No detector, camera, NT instance or runtime worker is created. Demo configuration
is read-only; all display imports stay in the browser. --calibration-data enables
the compatible offline calibration workspace on the same page, with local import
jobs and export only. Ctrl-C stops this server and cancels its owned jobs.
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
            'dashboard':{'stream_fps':1,'preview_notice':'SYNTHETIC LOCAL PREVIEW — field poses are fixture playback. Calibration accepts local imports only. No camera acquisition, NetworkTables, robot control or hardware profile activation.'},
            'networktables':{'enabled':False}}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=5844)
    parser.add_argument('--fixture',default='single_tag.json',choices=['single_tag.json','camera_only_localization.json','missing_field_layout.json','same_frame_watchdog.json'])
    parser.add_argument('--calibration-data',type=Path,help='Enable offline calibration imports/export on this page using this local store; no runtime activation')
    args=parser.parse_args()
    if not 0 <= args.port <= 65535: parser.error('Port must be 0..65535')
    mount=None if args.fixture=='camera_only_localization.json' else {'translation_m':[.35,-.17,.42],'rotation_rpy_deg':[5.,-15.,25.]}
    jobs=None
    if args.calibration_data is not None:
        from custom_vision.calibration_jobs import CalibrationJobs
        jobs=CalibrationJobs(args.calibration_data)
    dashboard=Dashboard({'host':'127.0.0.1','port':args.port,'offline_preview':True},ReplayConfig(mount),calibration_jobs=jobs)
    layout=json.loads((ROOT/'protocol/fixture-layouts.json').read_text())['layouts']['scene']
    packet=json.loads((ROOT/'protocol/fixtures'/args.fixture).read_text())
    dashboard.set_field_geometry(layout,{'front_tags':mount})
    dashboard._device_cache={'devices':[],'note':'Synthetic preview; device discovery disabled.'}
    dashboard._device_cache_until=float('inf')
    print(f'SYNTHETIC FIXTURE PREVIEW: http://127.0.0.1:{dashboard.server.server_port} (no cameras, NT or robot)',flush=True)
    if jobs is not None: print('Offline calibration is on the same page; local import jobs/export only. No hardware activation.',flush=True)
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
