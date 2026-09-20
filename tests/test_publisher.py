"""Robot-facing clock conversion and invalidation without a physical roboRIO."""
import json

import ntcore
import pytest

from custom_vision.publisher import Publisher


class Topic:
    def __init__(self):
        self.value = None
    def publish(self, options):
        return self
    def set(self, value):
        self.value = value


class Instance:
    def __init__(self, offset):
        self.offset = offset
        self.topics = {}
        self.flushes = 0
    def getServerTimeOffset(self):
        return self.offset
    def getTable(self, path):
        self.path = path
        return self
    def __getattr__(self, name):
        if name.startswith('get') and name.endswith('Topic'):
            return lambda key: self.topics.setdefault(key, Topic())
        raise AttributeError(name)
    def flush(self):
        self.flushes += 1


def packet():
    return dict(schema_version=2, pipeline='front', connected=True, frame_id=42, boot_id='boot-a',
                capture_monotonic_us=970000, capture_latency_offset_ms=4, latency_ms=30,
                detections=[{'id': 7}], localization={'valid': True, 'used_tag_ids': [7, 12],
                    'field_to_robot': {'translation_m': [1, 2, 3], 'rotation_quaternion_wxyz': [1, 0, 0, 0]}})


def test_timestamp_is_server_clock_minus_frame_age_and_measured_offset(monkeypatch):
    monkeypatch.setattr(ntcore, '_now', lambda: 500000)
    monkeypatch.setattr('custom_vision.publisher.time.monotonic_ns', lambda: 1000000000)
    pub = Publisher({'enabled': False, 'table': '/CustomVision/jetson-tags'})
    pub.instance = Instance(2000000)
    data = packet()
    pub.publish(data)
    topics = pub.instance.topics
    assert topics['capture_server_us'].value == 2466000
    assert topics['time_sync_valid'].value is True
    decoded = json.loads(topics['result'].value)
    assert decoded['capture_server_us'] == 2466000
    assert decoded['frame_id'] == 42 and decoded['boot_id'] == 'boot-a'
    assert topics['field_to_robot'].value == [1, 2, 3, 1, 0, 0, 0]
    assert pub.instance.path == '/CustomVision/jetson-tags/front'
    data.update(connected=False, detections=[], localization={'valid': False})
    pub.publish(data)
    for key in ('field_to_robot', 'used_tag_ids', 'tag_ids'):
        assert topics[key].value == []
    for key in ('pose_valid', 'time_sync_valid', 'has_target'):
        assert topics[key].value is False
    assert topics['capture_server_us'].value == 0
    assert json.loads(topics['result'].value)['capture_server_us'] is None


def test_unsynchronized_time_and_missing_mount_never_create_robot_pose():
    pub = Publisher({'enabled': False})
    pub.instance = Instance(None)
    data = packet()
    data['localization']['field_to_robot'] = None
    pub.publish(data)
    assert pub.instance.topics['pose_valid'].value is False
    assert pub.instance.topics['capture_server_us'].value == 0
    assert json.loads(pub.instance.topics['result'].value)['time_sync_valid'] is False


def test_wire_precision_preserves_integer_timestamps_and_host_values():
    from custom_vision.publisher import wire_values
    raw={'capture_server_us':1234567890123456,'translation_m':[1.23456789,-.12345678], 'yaw_deg':35.98765432}
    compact=wire_values(raw)
    assert compact['capture_server_us']==raw['capture_server_us']
    assert compact['translation_m']==[1.234568,-.123457]
    assert raw['translation_m'][0]==1.23456789
    assert abs(compact['yaw_deg']-raw['yaw_deg']) < .0000005


def test_object_selection_topics_clear_on_lost_target_or_disconnect():
    pub=Publisher({'enabled':False})
    pub.instance=Instance(None)
    data=packet()
    target={'valid':True,'observed':True,'track_id':17,'translation_m':[2,.2,.05],
            'approach':{'translation_m':[1.4,.2,0]}}
    data['objects']={'valid':True,'selected_target':target,'targets':[target]}
    pub.publish(data)
    topics=pub.instance.topics
    assert topics['target_valid'].value is True
    assert topics['selected_track_id'].value==17
    assert topics['selected_target_robot'].value==[2,.2,.05]
    assert topics['approach_robot_xy'].value==[1.4,.2]
    assert topics['object_track_ids'].value==[17]
    for connected in [True,False]:
        data.update(connected=connected,objects={'valid':False,'selected_target':None,'targets':[]})
        pub.publish(data)
        assert topics['target_valid'].value is False
        assert topics['selected_track_id'].value==0
        assert all(topics[key].value==[] for key in ['selected_target_robot','approach_robot_xy','object_track_ids'])


def test_object_wire_targets_remain_coherent_without_mutating_dashboard_payload():
    from custom_vision.publisher import wire_packet
    target={'valid':True,'observed':True,'track_id':7,'translation_m':[1.23456789,.2,.05],
            'capture_monotonic_us':1234567890123456,'uncertainty':{'range_std_m':.012345678},
            'approach':{'translation_m':[.8,.2,0]}}
    data={'detections':[{'robot_relative':target}, {'robot_relative':{'valid':False,'reason':'horizon'}}],
          'objects':{'valid':True,'targets':[target],'selected_target':target,'selected_track_id':7}}
    wire=wire_packet(data)
    assert 'selected_target' not in wire['objects']
    assert wire['detections'][0]['robot_relative']=={'valid':True,'track_id':7}
    assert wire['detections'][1]['robot_relative']=={'valid':False,'reason':'horizon'}
    canonical=next(t for t in wire['objects']['targets'] if t['track_id']==wire['objects']['selected_track_id'])
    assert canonical['translation_m']==[1.234568,.2,.05]
    assert canonical['capture_monotonic_us']==1234567890123456
    assert canonical['uncertainty']['range_std_m']==.012346
    assert canonical['approach']['translation_m']==[.8,.2,0]
    assert data['objects']['selected_target'] is target
    assert data['detections'][0]['robot_relative']['translation_m'][0]==1.23456789


@pytest.mark.parametrize('invalidation', ['absent','invalid','wrong_selection','disconnected','preview_only'])
def test_poi_topics_clear_every_metric_when_selected_target_is_not_valid(invalidation):
    pub=Publisher({'enabled':False})
    pub.instance=Instance(None)
    data=packet()
    aim={'valid':True,'name':'speaker','tag_id':7,'tx_deg':12.5,'ty_deg':5.,
         'camera_translation_m':[.5,-.2,2.], 'robot_translation_m':[2.2,-.5,.7],
         'robot_yaw_deg':-12.8}
    data.update(fps=58.2,poi={'valid':True,'selected_name':'speaker','targets':[aim]})
    pub.publish(data)
    topics=pub.instance.topics
    assert topics['poi_valid'].value is True
    assert topics['poi_name'].value=='speaker'
    assert topics['poi_tag_id'].value==7
    assert topics['poi_tx_deg'].value==12.5 and topics['poi_ty_deg'].value==5.
    assert topics['poi_camera_xyz'].value==[.5,-.2,2.]
    assert topics['poi_robot_xyz'].value==[2.2,-.5,.7]
    assert topics['poi_robot_yaw_deg'].value==[-12.8]
    assert topics['fps'].value==58.2
    if invalidation=='absent':data.pop('poi')
    elif invalidation=='invalid':data['poi']['valid']=False
    elif invalidation=='wrong_selection':data['poi']['selected_name']='other'
    elif invalidation=='disconnected':data['connected']=False
    else:aim.update(valid=False,geometry_valid=True,calibration_verified=False)
    pub.publish(data)
    assert topics['poi_valid'].value is False
    assert topics['poi_name'].value=='' and topics['poi_tag_id'].value==-1
    assert topics['poi_tx_deg'].value==0. and topics['poi_ty_deg'].value==0.
    for key in ('poi_camera_xyz','poi_robot_xyz','poi_robot_yaw_deg'):
        assert topics[key].value==[]
    if invalidation=='disconnected':assert topics['fps'].value==0.


def test_poi_missing_mount_keeps_camera_aim_without_inventing_robot_coordinates():
    pub=Publisher({'enabled':False})
    pub.instance=Instance(None)
    data=packet()
    data['poi']={'valid':True,'selected_name':'aim','targets':[
        {'valid':True,'name':'aim','tag_id':7,'tx_deg':3.,'ty_deg':4.,
         'camera_translation_m':[.1,-.1,2.],'robot_translation_m':None,'robot_yaw_deg':None}]}
    pub.publish(data)
    topics=pub.instance.topics
    assert topics['poi_valid'].value is True
    assert topics['poi_camera_xyz'].value==[.1,-.1,2.]
    assert topics['poi_robot_xyz'].value==[] and topics['poi_robot_yaw_deg'].value==[]
