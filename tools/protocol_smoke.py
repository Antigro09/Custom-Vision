#!/usr/bin/env python3
"""Small CPU/file-input smoke; no camera, NT client, service or hardware."""
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import cv2
import numpy as np
import yaml
from custom_vision.app import main


def run():
    with tempfile.TemporaryDirectory(prefix='custom-vision-smoke-') as temporary:
        folder=Path(temporary)
        writer=cv2.VideoWriter(str(folder/'blank.avi'),cv2.VideoWriter_fourcc(*'MJPG'),30,(64,48))
        assert writer.isOpened(),'File-only MJPEG encoder unavailable'
        for _ in range(3):writer.write(np.zeros((48,64,3),np.uint8))
        writer.release()
        config={'max_frame_age_ms':500,'networktables':{'enabled':False},
                'dashboard':{'enabled':False},'pipelines':[]}
        for name,kind in [('tags','apriltag'),('objects','object')]:
            config['pipelines'].append({'name':name,'type':kind,
                'camera':{'source':'blank.avi','width':64,'height':48,'fps':30},'settings':{}})
        path=folder/'vision.yaml'
        path.write_text(yaml.safe_dump(config))
        output=io.StringIO()
        with redirect_stdout(output):
            result=main(['--config',str(path),'--headless','--no-nt','--stdout-json','--max-frames','3'])
        assert result==0
        packets=[json.loads(line) for line in output.getvalue().splitlines()]
        assert len(packets)==8
        for pipeline in ('tags','objects'):
            stream=[p for p in packets if p['pipeline']==pipeline]
            assert [p['packet_seq'] for p in stream]==[0,1,2,3]
            assert [p['frame_id'] for p in stream]==[0,1,2,2]
            assert all(p['connected'] and not p['detections'] for p in stream[:3])
            assert not stream[-1]['connected'] and stream[-1]['error']=='Runtime stopped'
            assert stream[-1]['capture_monotonic_us']==stream[-2]['capture_monotonic_us']
            assert all(p['capture_server_us'] is None and p['time_sync_valid'] is False for p in stream)
        print('PASS: 2 shared file-input pipelines, 3 empty frames each, same-frame shutdown; 8 profile packets.')


if __name__=='__main__':run()
