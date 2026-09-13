"""Robot-facing clock conversion and invalidation without a physical roboRIO."""
import json

import ntcore

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
