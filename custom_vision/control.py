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
import time
import uuid

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

    def _calibration_reference(self,selected,validated):
        # Re-uploading the same normalized calibration is not a new lens model.
        # Keep the original artifact reference, including different JSON order.
        existing=selected.get('calibration')
        if existing:
            try:
                current=validate_calibration(json.loads((self.path.parent/existing).read_text()))
            except (OSError,ValueError,TypeError):
                current=None
            if current==validated:
                return existing
        return self._save_artifact('camera',validated)

    def upload_calibration(self,pipeline,data,*,_clear_verification=False):
        validated=validate_calibration(data)
        with self.lock:
            config=self.get_config()
            selected=next((p for p in config['pipelines'] if p['name']==pipeline),None)
            if selected is None: raise ValueError('Unknown pipeline')
            selected['calibration']=self._calibration_reference(selected,validated)
            # Explicit candidate activation must require a new physical check,
            # including when the candidate has identical intrinsic parameters.
            if _clear_verification and isinstance(selected.get('poi'),dict):
                selected['poi']['calibration_verified']=False
            return self.apply_config(config)

    @property
    def _activation_directory(self):
        return self.path.parent.parent/'data'/'calibration-activation-history'

    def _activation_path(self,activation_id):
        try:
            identifier=uuid.UUID(str(activation_id)).hex
        except (ValueError,TypeError,AttributeError) as exc:
            raise ValueError('Invalid activation ID') from exc
        return self._activation_directory/f'{identifier}.json'

    def _write_activation(self,record):
        atomic_write(self._activation_path(record['activation_id']),
                     json.dumps(record,allow_nan=False,indent=2)+'\n')

    def _config_write_token(self):
        stat=self.path.stat()
        return stat.st_dev,stat.st_ino,stat.st_mtime_ns,stat.st_size

    @staticmethod
    def _public_activation(record):
        # The local record contains a config snapshot, original references and
        # provenance. None of those (including labels, credentials or paths)
        # belongs in the history GET response.
        keys=('activation_id','pipeline','status','restored','restore_status',
              'created_unix_us','restored_unix_us','calibration_revision',
              'previous_calibration_revision','physical_verification','warnings')
        return {key:copy.deepcopy(record[key]) for key in keys if key in record}

    @staticmethod
    def _calibration_revision(data):
        if data is None: return None
        from .revisions import geometry_revisions
        return geometry_revisions({'calibration_data':data},{})['calibration_revision']

    @staticmethod
    def _candidate_compatibility(selected,validated,metadata):
        """Check known mode/provenance without claiming physical verification.

        Candidate metadata follows the session contract: camera.physical_id and
        mode.{width,height,crop,binning,focus:{kind,value,locked}}. Camera source
        indices/URLs and manual labels never identify a physical camera.
        """
        if metadata is None: metadata={}
        if not isinstance(metadata,dict):
            raise ValueError('Candidate metadata must be an object')
        candidate_camera=metadata.get('camera') or {}
        candidate_mode=metadata.get('mode') or {}
        if not isinstance(candidate_camera,dict) or not isinstance(candidate_mode,dict):
            raise ValueError('Candidate camera and mode metadata must be objects')
        camera=selected.get('camera') or {}
        for field,default in (('width',640),('height',480)):
            if validated[field]!=camera.get(field,default):
                raise ValueError(f'Candidate calibration {field} does not match the configured camera mode')
            declared=candidate_mode.get(field)
            if declared is not None and (type(declared) is not int or declared!=validated[field]):
                raise ValueError(f'Candidate mode metadata {field} does not match its calibration')
        warnings=['Configured mode compatibility does not verify the physical camera or calibration accuracy.']
        current_mode=camera.get('mode') or {}
        if not isinstance(current_mode,dict):
            raise ValueError('Configured camera mode metadata must be an object')

        def known(value):
            return value is not None and not (isinstance(value,str) and value in ('','unknown'))

        def compare(field,current,candidate):
            if not known(current) or not known(candidate):
                warnings.append(f'{field} provenance is unknown for the current camera or candidate.')
            elif current!=candidate:
                raise ValueError(f'Candidate {field} does not match the configured camera')

        compare('physical camera identity',camera.get('physical_id'),candidate_camera.get('physical_id'))
        for field in ('crop','binning'):
            compare(field,current_mode.get(field,camera.get(field)),candidate_mode.get(field))
        current_focus=current_mode.get('focus',camera.get('focus'))
        if current_focus is None and isinstance(camera.get('controls'),dict):
            current_focus=camera['controls'].get('focus')
        candidate_focus=candidate_mode.get('focus')
        if isinstance(current_focus,dict) or isinstance(candidate_focus,dict):
            # A numeric configured focus control supplies only its value, not a
            # proof of focus kind or locking. Compare each known component.
            current_focus=current_focus if isinstance(current_focus,dict) else {'value':current_focus}
            candidate_focus=candidate_focus if isinstance(candidate_focus,dict) else {'value':candidate_focus}
            for field in ('kind','value','locked'):
                current=current_focus.get(field)
                candidate=candidate_focus.get(field)
                if field=='locked':
                    # Session metadata defaults an unknown focus to unlocked.
                    # That default is not a measured physical locking state.
                    if not known(current_focus.get('kind')): current=None
                    if not known(candidate_focus.get('kind')): candidate=None
                compare(f'focus {field}',current,candidate)
        else:
            compare('focus',current_focus,candidate_focus)
        return warnings

    def activate_calibration_candidate(self,pipeline,data,metadata=None):
        """Explicitly activate a compatible candidate with private restore history."""
        validated=validate_calibration(data)
        with self.lock:
            config=self.get_config()
            selected=next((p for p in config['pipelines'] if p['name']==pipeline),None)
            if selected is None: raise ValueError('Unknown pipeline')
            warnings=self._candidate_compatibility(selected,validated,metadata)
            previous_reference=selected.get('calibration')
            previous_data=None
            if previous_reference:
                try:
                    previous_data=validate_calibration(json.loads((self.path.parent/previous_reference).read_text()))
                except (OSError,ValueError,TypeError):
                    warnings.append('Previous calibration artifact is unavailable or invalid; its reference was retained for manual recovery.')
            activated_reference=self._calibration_reference(selected,validated)
            record={'format':'calibration-activation-v1','activation_id':uuid.uuid4().hex,
                    'pipeline':pipeline,'status':'pending','restored':False,
                    'created_unix_us':time.time_ns()//1000,'physical_verification':False,
                    'warnings':warnings,'previous_config_snapshot':config,
                    'previous_calibration_present':'calibration' in selected,
                    'previous_calibration_reference':previous_reference,
                    'previous_calibration_snapshot':previous_data,
                    'activated_calibration_reference':activated_reference,
                    'candidate_metadata':copy.deepcopy(metadata),
                    'calibration_revision':self._calibration_revision(validated),
                    'previous_calibration_revision':self._calibration_revision(previous_data)}
            # A durable pending record is required before the config mutation.
            # Artifacts are immutable content-addressed imports; previous files
            # stay untouched and the original references/snapshot remain private.
            self._write_activation(record)
            before_write=self._config_write_token()
            try:
                result=self.upload_calibration(pipeline,validated,_clear_verification=True)
            except Exception as exc:
                # apply_config writes atomically before scheduling its restart.
                # A restart-handoff exception cannot turn a completed config
                # replacement into a failed activation in the audit history.
                record['status']='applied' if self._config_write_token()!=before_write else 'failed'
                record['failure_type']=type(exc).__name__
                if record['status']=='applied':
                    record['warnings'].append('Configuration was saved, but runtime restart handoff failed; restart the runtime before using it.')
                self._write_activation(record)
                raise
            record['status']='applied'
            self._write_activation(record)
            result.update(self._public_activation(record))
            result['message']='Candidate activated; physically validate it before acknowledging calibration verification.'
            if isinstance(selected.get('poi'),dict) and selected['poi'].get('calibration_verified'):
                result['poi_verification_reset']=list(dict.fromkeys(result['poi_verification_reset']+[pipeline]))
            return result

    def restore_calibration_activation(self,activation_id):
        """Restore only this pipeline's calibration if its activation still owns it."""
        with self.lock:
            path=self._activation_path(activation_id)
            try:
                record=json.loads(path.read_text())
            except FileNotFoundError as exc:
                raise ValueError('Unknown activation ID') from exc
            if record.get('restored'):
                return {**self._public_activation(record),'already_restored':True,
                        'message':'This activation has already been restored.'}
            if record.get('status')!='applied':
                raise ValueError('Activation was not applied')
            config=self.get_config()
            selected=next((p for p in config['pipelines'] if p['name']==record['pipeline']),None)
            if selected is None: raise ValueError('Activation pipeline no longer exists')
            if selected.get('calibration')!=record['activated_calibration_reference']:
                raise ValueError('Calibration changed after activation; restore would overwrite a subsequent change')
            previous=record.get('previous_calibration_reference')
            if previous:
                try:
                    restored_data=validate_calibration(json.loads((self.path.parent/previous).read_text()))
                except (OSError,ValueError,TypeError) as exc:
                    raise ValueError('Previous calibration artifact is unavailable or invalid') from exc
                if self._calibration_revision(restored_data)!=record.get('previous_calibration_revision'):
                    raise ValueError('Previous calibration artifact changed since activation; recover the original saved snapshot manually')
                camera=selected.get('camera') or {}
                if any(restored_data[field]!=camera.get(field,default) for field,default in (('width',640),('height',480))):
                    raise ValueError('Previous calibration does not match the current camera mode')
            if record.get('previous_calibration_present'):
                selected['calibration']=previous
            else:
                selected.pop('calibration',None)
            verification_was_set=isinstance(selected.get('poi'),dict) and selected['poi'].get('calibration_verified')
            if isinstance(selected.get('poi'),dict):
                selected['poi']['calibration_verified']=False
            record['restore_status']='pending'
            self._write_activation(record)
            before_write=self._config_write_token()
            try:
                result=self.apply_config(config)
            except Exception as exc:
                changed=self._config_write_token()!=before_write
                record['restore_status']='applied' if changed else 'failed'
                if changed:
                    record.update(restored=True,restored_unix_us=time.time_ns()//1000)
                    record['warnings'].append('Configuration was restored, but runtime restart handoff failed; restart the runtime before using it.')
                record['restore_failure_type']=type(exc).__name__
                self._write_activation(record)
                raise
            record.update(restored=True,restore_status='applied',restored_unix_us=time.time_ns()//1000)
            self._write_activation(record)
            result.update(self._public_activation(record))
            if verification_was_set:
                result['poi_verification_reset']=list(dict.fromkeys(result['poi_verification_reset']+[record['pipeline']]))
            result['message']='Previous calibration reference restored; physical calibration verification must be acknowledged again.'
            return result

    def list_calibration_activations(self):
        with self.lock:
            records=[]
            for path in self._activation_directory.glob('*.json'):
                try:
                    record=json.loads(path.read_text())
                    if record.get('format')!='calibration-activation-v1': continue
                    if self._activation_path(record['activation_id'])!=path: continue
                    records.append(self._public_activation(record))
                except (OSError,ValueError,TypeError,KeyError):
                    continue
            return sorted(records,key=lambda record:record.get('created_unix_us',0),reverse=True)

    def upload_field_layout(self,data):
        # Localization constructor validates the complete WPILib layout contract.
        from .localization import Localization
        Localization({},field_layout=data)
        with self.lock:
            config=self.get_config()
            config['field_layout']=self._save_artifact('field',data)
            return self.apply_config(config)
