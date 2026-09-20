"""Atomic browser configuration updates and validated calibration/layout imports.

POI verification belongs to the selected calibration. Controller changes clear
the acknowledgement; manually overwriting a calibration file at the same path
is outside this controller and requires manually clearing/rechecking it too.
"""
import copy
import hashlib
import json
import os
from pathlib import Path
import tempfile
import threading

import yaml

from .config import load_config, validate_config
from .calibration import validate_calibration


def atomic_write(path, text):
    path=Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    fd, temporary=tempfile.mkstemp(prefix='.pending-',dir=path.parent)
    try:
        with os.fdopen(fd,'w') as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary,path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


class RuntimeController:
    def __init__(self,path,on_change,validate_runtime=None):
        self.path=Path(path).resolve()
        self.on_change=on_change
        self.validate_runtime=validate_runtime
        self.lock=threading.RLock()

    def get_config(self):
        with self.lock:
            return yaml.safe_load(self.path.read_text())

    def apply_config(self,data):
        with self.lock:
            if not isinstance(data,dict):
                raise ValueError('Configuration must be an object')
            clean=copy.deepcopy(data)
            clean.pop('field_layout_data',None)
            for pipeline in clean.get('pipelines',[]):
                if isinstance(pipeline,dict): pipeline.pop('calibration_data',None)
            normalized=validate_config(clean,self.path.parent)
            previous={pipeline['name']:pipeline for pipeline in self.get_config()['pipelines']}
            verification_reset=[]
            for pipeline,checked in zip(clean['pipelines'],normalized['pipelines']):
                if pipeline['type']!='apriltag' or not isinstance(pipeline.get('poi'),dict):
                    continue
                old=previous.get(pipeline['name'],{}).get('calibration')
                new=pipeline.get('calibration')
                old_path=(self.path.parent/old).resolve() if old else None
                new_path=(self.path.parent/new).resolve() if new else None
                if old_path!=new_path:
                    if pipeline['poi'].get('calibration_verified'):
                        verification_reset.append(pipeline['name'])
                    pipeline['poi']['calibration_verified']=False
                    checked['poi']['calibration_verified']=False
            if self.validate_runtime:
                self.validate_runtime(normalized)
            atomic_write(self.path,yaml.safe_dump(clean,sort_keys=False))
            # Allow the HTTP response to complete before rebuilding workers/server.
            timer=threading.Timer(.25,self.on_change)
            timer.daemon=True
            timer.start()
            message='Saved; camera workers will restart and targets clear briefly.'
            if verification_reset:
                message+=' POI calibration verification cleared for '+', '.join(verification_reset)+'. Validate the new calibration before acknowledging it again.'
            return {'saved':True,'restart_required':True,'message':message,
                    'poi_verification_reset':verification_reset}

    def _save_artifact(self,kind,data):
        encoded=json.dumps(data,allow_nan=False,indent=2)+'\n'
        digest=hashlib.sha256(encoded.encode()).hexdigest()[:16]
        path=self.path.parent.parent/'calibration'/f'{kind}-{digest}.json'
        if not path.exists(): atomic_write(path,encoded)
        return os.path.relpath(path,self.path.parent)

    def upload_calibration(self,pipeline,data):
        validated=validate_calibration(data)
        with self.lock:
            config=self.get_config()
            selected=next((p for p in config['pipelines'] if p['name']==pipeline),None)
            if selected is None: raise ValueError('Unknown pipeline')
            # Re-uploading the same normalized calibration is not a new lens
            # model. Preserve the existing artifact path and acknowledgement,
            # including when JSON key order or formatting differs.
            existing=selected.get('calibration')
            if existing:
                try:
                    current=validate_calibration(json.loads((self.path.parent/existing).read_text()))
                except (OSError,ValueError,TypeError):
                    current=None
                if current==validated:
                    return self.apply_config(config)
            selected['calibration']=self._save_artifact('camera',validated)
            return self.apply_config(config)

    def upload_field_layout(self,data):
        # Localization constructor validates the complete WPILib layout contract.
        from .localization import Localization
        Localization({},field_layout=data)
        with self.lock:
            config=self.get_config()
            config['field_layout']=self._save_artifact('field',data)
            return self.apply_config(config)
