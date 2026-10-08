"""Atomic JSON and typed NT4 topics with synchronized host-read timestamps."""
import json
import threading
import time

from .nt_clock import NtClockAdapter

PROTOCOL_PROFILE = 'custom-vision-schema2-2026.1'
MAX_PACKET_SEQ = 2**53 - 1


def wire_values(value):
    """Micrometer/subpixel precision bounds JSON size without altering host data."""
    if isinstance(value,float): return round(value,6)
    if isinstance(value,dict): return {key:wire_values(item) for key,item in value.items()}
    if isinstance(value,(list,tuple)): return [wire_values(item) for item in value]
    return value


def wire_packet(payload):
    """One canonical metric target per object; compact references avoid triplication."""
    # Prune duplicated metric targets BEFORE recursively copying/rounding them.
    # Copy every container we change: the dashboard and typed NT topics still
    # consume the original full-precision targets and their shared references.
    if not isinstance(payload.get('objects'),dict):
        return wire_values(payload)
    objects=payload['objects']
    targets=objects.get('targets',[])
    ids=[target['track_id'] for target in targets]
    if len(ids)!=len(set(ids)):
        raise ValueError('objects.targets must have unique local track IDs')
    selected_id=objects.get('selected_track_id')
    if selected_id is not None and selected_id not in ids:
        raise ValueError('selected_track_id must resolve in objects.targets')
    compact=dict(payload)
    compact['objects']=dict(payload['objects'])
    compact['objects'].pop('selected_target',None)  # Resolve selected_track_id in targets.
    if 'detections' in payload:
        detections=[]
        for detection in payload['detections']:
            target=detection.get('robot_relative')
            if isinstance(target,dict) and target.get('valid'):
                if target['track_id'] not in ids:
                    raise ValueError('robot_relative reference must resolve in objects.targets')
                detection=dict(detection,robot_relative={'valid':True,'track_id':target['track_id']})
            detections.append(detection)
        compact['detections']=detections
    return wire_values(compact)


class Publisher:
    def __init__(self,config,stdout=False,*,clock_adapter=None):
        self.stdout=stdout
        self.lock=threading.Lock()
        self.instance=None
        self.tables={}
        self.packet_sequences={}
        self.clock_adapter=clock_adapter
        self.root=config.get('table','/CustomVision')
        self.period=config.get('period_ms',10)/1000
        if config.get('enabled'):
            import ntcore
            self.instance=ntcore.NetworkTableInstance.create()
            self.instance.startClient4('CustomVision-'+self.root.replace('/','-'))
            if config.get('server'): self.instance.setServer(config['server'])
            else: self.instance.setServerTeam(config['team'])

    def publish(self,payload):
        # Clock sampling, validity, serialization and sequence allocation share
        # one boundary. A delayed worker cannot publish before an invalidation
        # whose packet_seq was already allocated by another thread.
        with self.lock:
            return self._publish_locked(payload)

    def _publish_locked(self,payload):
        # Convert the host read-completion time to NT's synchronized server clock.
        # This is not hardware exposure time; a measured capture correction may be supplied.
        if self.instance is not None:
            import ntcore
        if self.instance is not None and self.clock_adapter is None:
            self.clock_adapter=NtClockAdapter(ntcore_module=ntcore)
        capture=None
        if self.instance is not None and payload.get('connected'):
            capture=self.clock_adapter.capture_server_us(self.instance,payload.get('capture_monotonic_us'),
                                                       payload.get('capture_latency_offset_ms',0))
        payload['capture_server_us']=capture
        payload['time_sync_valid']=capture is not None
        # The dashboard consumes the original payload, not this JSON. A fully
        # disabled publisher must neither require NTCore nor spend time encoding.
        if self.instance is None and not self.stdout:
            return
        key=(payload.get('boot_id'),payload['pipeline'])
        sequence=self.packet_sequences.get(key,0)
        if sequence>MAX_PACKET_SEQ:
            raise OverflowError('packet_seq exhausted; start a new producer boot before publishing')
        payload['packet_seq']=sequence
        payload['protocol_profile']=PROTOCOL_PROFILE
        for field in ('calibration_revision','mount_revision','field_layout_revision'):
            payload.setdefault(field,None)
        payload.setdefault('timing',{'clock_domain':'nt_server','timestamp_unit':'us',
                                    'capture_event':'host_frame_read_complete',
                                    'capture_correction_verified':False,
                                    'capture_correction_uncertainty_ms':None})
        encoded=json.dumps(wire_packet(payload),allow_nan=False,separators=(',',':'))
        self.packet_sequences[key]=sequence+1
        if self.stdout: print(encoded,flush=True)
        if self.instance is None: return
        name=payload['pipeline']
        if name not in self.tables:
            table=self.instance.getTable(f'{self.root}/{name}')
            options=ntcore.PubSubOptions(periodic=self.period,sendAll=True,keepDuplicates=True)
            types={'result':'String','connected':'Boolean','has_target':'Boolean','frame_id':'Integer','packet_seq':'Integer','count':'Integer',
                   'latency_ms':'Double','tag_ids':'IntegerArray','pose_valid':'Boolean','field_to_robot':'DoubleArray',
                   'used_tag_ids':'IntegerArray','capture_server_us':'Integer','time_sync_valid':'Boolean',
                   'target_valid':'Boolean','selected_track_id':'Integer','selected_target_robot':'DoubleArray',
                   'approach_robot_xy':'DoubleArray','object_track_ids':'IntegerArray',
                   'fps':'Double','poi_valid':'Boolean','poi_name':'String','poi_tag_id':'Integer',
                   'poi_tx_deg':'Double','poi_ty_deg':'Double','poi_camera_xyz':'DoubleArray',
                   'poi_robot_xyz':'DoubleArray','poi_robot_yaw_deg':'DoubleArray'}
            self.tables[name]={key:getattr(table,f'get{kind}Topic')(key).publish(options) for key,kind in types.items()}
        topics=self.tables[name]
        localization=payload.get('localization') or {}
        robot_pose=localization.get('field_to_robot')
        valid=bool(payload.get('connected') and localization.get('valid') and robot_pose)
        objects=payload.get('objects') or {}
        selected=next((target for target in objects.get('targets',[])
                       if target.get('track_id')==objects.get('selected_track_id')), {})
        target_valid=bool(payload.get('connected') and objects.get('valid') and selected.get('valid') and selected.get('observed'))
        poi=payload.get('poi') or {}
        aim=next((p for p in poi.get('targets',[]) if p.get('name')==poi.get('selected_name') and p.get('valid')),None)
        aim=aim if payload.get('connected') and poi.get('valid') else None
        values={'result':encoded,'connected':payload['connected'],'has_target':bool(payload.get('connected') and payload['detections']),
                'frame_id':payload['frame_id'],'packet_seq':payload['packet_seq'],'count':len(payload['detections']) if payload.get('connected') else 0,'latency_ms':payload['latency_ms'],
                'fps':payload.get('fps',0.) if payload.get('connected') else 0.,
                'poi_valid':bool(aim),'poi_name':aim['name'] if aim else '',
                'poi_tag_id':aim['tag_id'] if aim else -1,
                'poi_tx_deg':aim['tx_deg'] if aim else 0.,'poi_ty_deg':aim['ty_deg'] if aim else 0.,
                'poi_camera_xyz':aim['camera_translation_m'] if aim else [],
                'poi_robot_xyz':(aim.get('robot_translation_m') or []) if aim else [],
                'poi_robot_yaw_deg':[aim['robot_yaw_deg']] if aim and aim.get('robot_yaw_deg') is not None else [],
                'tag_ids':[d['id'] for d in payload['detections'] if 'id' in d] if payload.get('connected') else [],'pose_valid':valid,
                'field_to_robot':(robot_pose['translation_m']+robot_pose['rotation_quaternion_wxyz']) if valid else [],
                'used_tag_ids':localization.get('used_tag_ids',[]) if valid else [],
                'capture_server_us':payload.get('capture_server_us') or 0,'time_sync_valid':payload['time_sync_valid'],
                'target_valid':target_valid,'selected_track_id':selected.get('track_id',0) if target_valid else 0,
                'selected_target_robot':selected['translation_m'] if target_valid else [],
                'approach_robot_xy':selected['approach']['translation_m'][:2] if target_valid else [],
                'object_track_ids':[target['track_id'] for target in objects.get('targets',[]) if target.get('valid') and target.get('observed')] if payload.get('connected') else []}
        for key,value in values.items(): topics[key].set(value)
        self.instance.flush()  # NTCore rate-limits network flushes; receiver requests 10ms periodic too.
    def close(self):
        with self.lock:
            self._close_locked()

    def _close_locked(self):
        if self.instance:
            for topics in self.tables.values():
                for topic in topics.values(): topic.close()
            self.instance.stopClient()
            self.instance.destroy(self.instance)
            self.instance=None
