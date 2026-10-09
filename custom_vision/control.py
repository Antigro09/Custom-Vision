"""Atomic browser configuration updates and validated calibration/layout imports.

POI verification belongs to the selected calibration and camera mode. Controller
changes clear the acknowledgement. Reviewed candidates are bound to exact camera
provenance and geometry; active status and rollback detect later replacements.
Legacy raw-file editing still requires clearing/rechecking physical validation.
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
from .calibration_review import candidate_provenance, require_candidate_review


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
                old_pipeline=previous.get(pipeline['name'],{})
                old=old_pipeline.get('calibration')
                new=pipeline.get('calibration')
                old_path=(self.path.parent/old).resolve() if old else None
                new_path=(self.path.parent/new).resolve() if new else None
                if (old_path!=new_path or self._camera_signature(old_pipeline) != self._camera_signature(pipeline)):
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
        needs_review=validated.get('quality_status')=='needs_review'
        with self.lock:
            config=self.get_config()
            selected=next((p for p in config['pipelines'] if p['name']==pipeline),None)
            if selected is None: raise ValueError('Unknown pipeline')
            selected['calibration']=self._calibration_reference(selected,validated)
            # Explicit candidate activation must require a new physical check,
            # including when the candidate has identical intrinsic parameters.
            if (_clear_verification or needs_review) and isinstance(selected.get('poi'),dict):
                selected['poi']['calibration_verified']=False
            result=self.apply_config(config)
            if needs_review:
                result['quality_warning']='Imported calibration requires investigation; physical verification was cleared.'
            return result

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
              'previous_calibration_revision','physical_verification','software_review','warnings')
        return {key:copy.deepcopy(record[key]) for key in keys if key in record}

    @staticmethod
    def _calibration_revision(data):
        if data is None: return None
        from .revisions import geometry_revisions
        return geometry_revisions({'calibration_data':data},{})['calibration_revision']

    @staticmethod
    def _camera_signature(selected):
        """Private capture identity/mode snapshot; never returned by status APIs."""
        camera=selected.get('camera') or {}
        mode=camera.get('mode') or {}
        if not isinstance(camera,dict) or not isinstance(mode,dict):
            raise ValueError('Configured camera mode metadata must be an object')
        focus=mode.get('focus',camera.get('focus'))
        if focus is None and isinstance(camera.get('controls'),dict):
            focus=camera['controls'].get('focus')
        return {'physical_id':camera.get('physical_id'),
                'source':camera.get('source',0),'backend':camera.get('backend','auto'),
                'fourcc':camera.get('fourcc'),'fps':camera.get('fps',30),
                'width':camera.get('width',640),'height':camera.get('height',480),
                'declared_dimensions':{key:mode.get(key) for key in ('width','height')},
                'crop':mode.get('crop',camera.get('crop')),
                'binning':mode.get('binning',camera.get('binning')),
                'focus':focus,'focus_control':(camera.get('controls') or {}).get('focus')}

    @staticmethod
    def _candidate_compatibility(selected,validated,metadata):
        """Prove exact declared identity/mode; software review is a separate gate."""
        current=RuntimeController._camera_signature(selected)
        declared_mode=(selected.get('camera') or {}).get('mode') or {}
        for field in ('width','height'):
            if field in declared_mode and declared_mode[field]!=current[field]:
                raise ValueError(f'Configured camera mode metadata {field} conflicts with its capture resolution')
            if validated[field]!=current[field]:
                raise ValueError(f'Candidate calibration {field} does not match the configured camera mode')
        candidate=candidate_provenance(metadata)
        for field in ('width','height'):
            if candidate[field]!=validated[field]:
                raise ValueError(f'Candidate mode metadata {field} does not match its calibration')
        configured=candidate_provenance({'camera':{'physical_id':current['physical_id']},
                                        'mode':{key:current[key] for key in
                                                ('width','height','crop','binning','focus')}})
        labels={'physical_id':'physical camera identity'}
        for field in candidate:
            if candidate[field]!=configured[field]:
                raise ValueError(f'Candidate {labels.get(field,field)} does not match the configured camera')
        if current['focus_control'] is not None and current['focus_control']!=configured['focus']['value']:
            raise ValueError('Configured focus metadata does not match the configured camera focus control')
        return ['Exact declared camera/mode compatibility does not verify calibration accuracy or the physical mount.']

    def activate_calibration_candidate(self,pipeline,data,metadata=None):
        """Explicitly activate a compatible candidate with private restore history."""
        validated=validate_calibration(data)
        with self.lock:
            config=self.get_config()
            selected=next((p for p in config['pipelines'] if p['name']==pipeline),None)
            if selected is None: raise ValueError('Unknown pipeline')
            warnings=self._candidate_compatibility(selected,validated,metadata)
            review_status=require_candidate_review(validated,metadata)
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
                    'activated_camera_signature':self._camera_signature(selected),
                    'software_review':review_status,
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
            expected_camera=record.get('activated_camera_signature')
            if expected_camera is None:
                previous_pipeline=next((p for p in record.get('previous_config_snapshot',{}).get('pipelines',[])
                                        if p.get('name')==record['pipeline']),None)
                if previous_pipeline is None:
                    raise ValueError('Activation camera provenance is unavailable; restore requires manual recovery')
                expected_camera=self._camera_signature(previous_pipeline)
            if self._camera_signature(selected)!=expected_camera:
                raise ValueError('Camera identity or current camera mode changed since activation; restore would overwrite incompatible state')
            try:
                active_data=validate_calibration(json.loads((self.path.parent/selected['calibration']).read_text()))
            except (OSError,ValueError,TypeError) as exc:
                raise ValueError('Active calibration artifact is unavailable or invalid') from exc
            if self._calibration_revision(active_data)!=record.get('calibration_revision'):
                raise ValueError('Active calibration artifact changed since activation; restore would overwrite a subsequent change')
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

    def calibration_status(self,pipeline=None):
        """Path-free active calibration states, separate from unapplied candidates.

        "Validated" requires both a strict reviewed activation for this exact
        camera/mode and the existing explicit physical-check acknowledgement.
        Legacy/raw imports stay conspicuously unverified; a checkbox alone is
        insufficient to infer camera/mode provenance.
        """
        with self.lock:
            selected=self.get_config()['pipelines']
            if pipeline is not None:
                selected=[item for item in selected if item['name']==pipeline]
                if not selected: raise ValueError('Unknown pipeline')
            records=[]
            for path in self._activation_directory.glob('*.json'):
                try:
                    record=json.loads(path.read_text())
                    if (record.get('format')=='calibration-activation-v1'
                            and self._activation_path(record['activation_id'])==path
                            and record.get('status')=='applied' and not record.get('restored')):
                        records.append(record)
                except (OSError,ValueError,TypeError,KeyError):
                    continue
            records.sort(key=lambda item:item.get('created_unix_us',0),reverse=True)
            result=[]
            for item in selected:
                status={'pipeline':item['name'],'state':'missing','active':False,
                        'physical_verification':False,'software_reviewed':False,
                        'calibration_revision':None,'reason':'No custom calibration is selected'}
                reference=item.get('calibration')
                if reference:
                    try:
                        data=validate_calibration(json.loads((self.path.parent/reference).read_text()))
                    except FileNotFoundError:
                        status['reason']='Selected calibration artifact is missing'
                    except (OSError,ValueError,TypeError):
                        status.update(state='stale',reason='Selected calibration artifact is unreadable or invalid')
                    else:
                        revision=self._calibration_revision(data)
                        status.update(calibration_revision=revision,active=True,
                                      state='custom_active_unverified',
                                      reason='Custom calibration is active without validated camera/mode review')
                        is_default=(item.get('calibration_source')=='default'
                                    or data.get('calibrator') in ('default','nominal')
                                    or data.get('quality_status') in ('default','nominal'))
                        if is_default:
                            status.update(state='default',reason='Default or nominal calibration is active')
                        elif any(data[key]!=self._camera_signature(item)[key] for key in ('width','height')):
                            status.update(state='mode_mismatch',active=False,
                                          reason='Custom calibration resolution differs from the current camera mode')
                        else:
                            activation=next((record for record in records
                                             if record['pipeline']==item['name']
                                             and record.get('activated_calibration_reference')==reference),None)
                            if activation is not None:
                                if activation.get('calibration_revision')!=revision:
                                    status.update(state='stale',active=False,
                                                  reason='Calibration geometry changed after review')
                                elif self._camera_signature(item)!=activation.get('activated_camera_signature'):
                                    status.update(state='mode_mismatch',active=False,
                                                  reason='Camera identity or mode changed after activation')
                                else:
                                    try:
                                        checked=require_candidate_review(data,activation.get('candidate_metadata'))
                                    except ValueError:
                                        status['reason']='Active calibration has no current accepted review'
                                    else:
                                        status.update(software_reviewed=True,
                                                      diagnostic_baseline=checked['diagnostic_baseline'],
                                                      reason='Reviewed custom calibration is active; physical validation is unacknowledged')
                                        if (item.get('poi') or {}).get('calibration_verified') is True:
                                            status.update(state='custom_active_validated',physical_verification=True,
                                                          reason='Reviewed custom calibration for this camera/mode has an explicit physical-check acknowledgement')
                result.append(status)
            return result[0] if pipeline is not None else result

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
