"""Atomic JSON and typed NT4 topics with synchronized host-read timestamps."""
import json
import threading
import time


def wire_values(value):
    """Micrometer/subpixel precision bounds JSON size without altering host data."""
    if isinstance(value,float): return round(value,6)
    if isinstance(value,dict): return {key:wire_values(item) for key,item in value.items()}
    if isinstance(value,(list,tuple)): return [wire_values(item) for item in value]
    return value


class Publisher:
    def __init__(self,config,stdout=False):
        self.stdout=stdout
        self.lock=threading.Lock()
        self.instance=None
        self.tables={}
        self.root=config.get('table','/CustomVision')
        self.period=config.get('period_ms',10)/1000
        if config.get('enabled'):
            import ntcore
            self.instance=ntcore.NetworkTableInstance.create()
            self.instance.startClient4('CustomVision-'+self.root.replace('/','-'))
            if config.get('server'): self.instance.setServer(config['server'])
            else: self.instance.setServerTeam(config['team'])

    def publish(self,payload):
        # Convert the host read-completion time to NT's synchronized server clock.
        # This is not hardware exposure time; a measured capture correction may be supplied.
        import ntcore
        offset=self.instance.getServerTimeOffset() if self.instance else None
        if offset is not None and 'capture_monotonic_us' in payload and payload.get('connected'):
            nt_now=ntcore._now()
            monotonic_us=time.monotonic_ns()//1000
            age_us=monotonic_us-payload['capture_monotonic_us']
            correction_us=int(payload.get('capture_latency_offset_ms',0)*1000)
            payload['capture_server_us']=nt_now+offset-age_us-correction_us
            payload['time_sync_valid']=True
        else:
            payload['capture_server_us']=None
            payload['time_sync_valid']=False
        encoded=json.dumps(wire_values(payload),allow_nan=False,separators=(',',':'))
        with self.lock:
            if self.stdout: print(encoded,flush=True)
            if self.instance is None: return
            name=payload['pipeline']
            if name not in self.tables:
                table=self.instance.getTable(f'{self.root}/{name}')
                options=ntcore.PubSubOptions(periodic=self.period,sendAll=True,keepDuplicates=True)
                types={'result':'String','connected':'Boolean','has_target':'Boolean','frame_id':'Integer','count':'Integer',
                       'latency_ms':'Double','tag_ids':'IntegerArray','pose_valid':'Boolean','field_to_robot':'DoubleArray',
                       'used_tag_ids':'IntegerArray','capture_server_us':'Integer','time_sync_valid':'Boolean'}
                self.tables[name]={key:getattr(table,f'get{kind}Topic')(key).publish(options) for key,kind in types.items()}
            topics=self.tables[name]
            localization=payload.get('localization') or {}
            robot_pose=localization.get('field_to_robot')
            valid=bool(payload.get('connected') and localization.get('valid') and robot_pose)
            values={'result':encoded,'connected':payload['connected'],'has_target':bool(payload['detections']),
                    'frame_id':payload['frame_id'],'count':len(payload['detections']),'latency_ms':payload['latency_ms'],
                    'tag_ids':[d['id'] for d in payload['detections'] if 'id' in d],'pose_valid':valid,
                    'field_to_robot':(robot_pose['translation_m']+robot_pose['rotation_quaternion_wxyz']) if valid else [],
                    'used_tag_ids':localization.get('used_tag_ids',[]) if valid else [],
                    'capture_server_us':payload.get('capture_server_us') or 0,'time_sync_valid':payload['time_sync_valid']}
            for key,value in values.items(): topics[key].set(value)
            self.instance.flush()  # NTCore rate-limits network flushes; receiver requests 10ms periodic too.

    def close(self):
        if self.instance:
            for topics in self.tables.values():
                for topic in topics.values(): topic.close()
            self.instance.stopClient()
            self.instance.destroy(self.instance)
            self.instance=None
