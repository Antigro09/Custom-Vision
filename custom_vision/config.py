"""Validated configuration; all artifact paths resolve relative to its YAML file."""
import copy
import json
import math
from pathlib import Path

import yaml


def mapping(value, name):
    if not isinstance(value, dict):
        raise ValueError(f'{name} must be a mapping')
    return value


def finite(value, name, minimum, maximum, integer=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f'{name} must be between {minimum} and {maximum}')
    if integer and not isinstance(value, int):
        raise ValueError(f'{name} must be an integer')
    return value


def boolean(value, name):
    if not isinstance(value, bool):
        raise ValueError(f'{name} must be boolean')
    return value


def preview_settings(settings, name='preview'):
    mapping(settings, name)
    for key, low, high in [('stream_fps', 1, 30), ('stream_width', 160, 1920), ('jpeg_quality', 20, 95)]:
        if key in settings:
            finite(settings[key], f'{name}.{key}', low, high, integer=key != 'stream_fps')
    if 'box_depth_ratio' in settings:
        finite(settings['box_depth_ratio'], f'{name}.box_depth_ratio', .1, 2.)
    if settings.get('rotation_deg', 0) not in (0,90,180,270):
        raise ValueError(f'{name}.rotation_deg must be 0, 90, 180 or 270')


def validate_config(data, directory):
    config = copy.deepcopy(mapping(data, 'Configuration'))
    directory = Path(directory).resolve()
    pipelines = config.get('pipelines')
    if not isinstance(pipelines, list) or not 1 <= len(pipelines) <= 8:
        raise ValueError('At least one pipeline is required (maximum eight)')
    names = set()
    for pipeline in pipelines:
        mapping(pipeline, 'Each pipeline')
        name = pipeline.get('name', '')
        if not isinstance(name, str) or not name or not name.isascii() or not name.replace('_','').replace('-','').isalnum() or name in names:
            raise ValueError('Pipeline names must be unique ASCII letters, numbers, underscores or hyphens')
        names.add(name)
        if pipeline.get('type') not in ('apriltag','object'):
            raise ValueError(f'Unsupported pipeline type for {name}')
        boolean(pipeline.setdefault('enabled',True),'pipeline.enabled')
        camera = mapping(pipeline.setdefault('camera',{}),'camera')
        camera.setdefault('source',0)
        for key, default in [('width',640),('height',480),('fps',30)]:
            value=camera.setdefault(key,default)
            finite(value,f'camera.{key}',1,8192 if key!='fps' else 240,integer=key!='fps')
        source=camera['source']
        if isinstance(source,bool) or not isinstance(source,(str,int)) or (isinstance(source,int) and source<0) or (isinstance(source,str) and not source):
            raise ValueError('camera.source must be a nonnegative device index or nonempty path')
        if camera.get('backend','auto') not in ('auto','v4l2','gstreamer'):
            raise ValueError('camera.backend must be auto, v4l2 or gstreamer')
        fourcc=camera.get('fourcc')
        if fourcc is not None and (not isinstance(fourcc,str) or len(fourcc)!=4 or not fourcc.isascii()):
            raise ValueError('camera.fourcc must be exactly four ASCII characters')
        finite(camera.get('capture_latency_offset_ms',0),'camera.capture_latency_offset_ms',0,100)
        controls=mapping(camera.setdefault('controls',{}),'camera.controls')
        for key,value in controls.items():
            if not isinstance(key,str) or not key.replace('_','').isalnum():
                raise ValueError('Invalid UVC control name')
            finite(value,'camera control',-2**31,2**31-1,integer=True)
        if isinstance(source,str) and camera.get('backend')!='gstreamer' and not source.startswith(('/','rtsp://','http://','https://')):
            camera['source']=str(directory / source)
        pipeline['calibration_data']=None
        if pipeline.get('calibration'):
            with (directory / pipeline['calibration']).open() as stream:
                pipeline['calibration_data']=json.load(stream)
        settings=mapping(pipeline.setdefault('settings',{}),'settings')
        if pipeline['type']=='apriltag':
            from .poi import validate_poi
            validate_poi(pipeline.get('poi'))
            if settings.get('pose_device','cpu') not in ('cpu','cuda'):
                raise ValueError('pose_device must be cpu or cuda')
            if settings.get('pose_device','cpu') == 'cuda' and settings.get('backend','pupil') != 'native':
                raise ValueError('CUDA pose requires the native backend')
            if 'cuda_pose_iterations' in settings:
                finite(settings['cuda_pose_iterations'],'cuda_pose_iterations',1,100,True)
            if settings.get('backend','pupil') not in ('native','pupil'):
                raise ValueError('AprilTag backend must be native or pupil')
            if settings.get('mode','3d') not in ('2d','3d'):
                raise ValueError('AprilTag mode must be 2d or 3d')
            if settings.get('detector_device','cpu') not in ('cpu','cuda'):
                raise ValueError('detector_device must be cpu or cuda')
            if settings.get('backend','pupil') == 'pupil' and settings.get('detector_device','cpu') != 'cpu':
                raise ValueError('CUDA detection requires the native backend')
            if settings.get('preprocess','cpu') not in ('cpu','cuda'):
                raise ValueError('preprocess must be cpu or cuda')
            for key,default,low,high,integer in [('threads',2,1,6,True),('quad_decimate',2,1,4,False),('tag_size_m',.1651,.001,2,False),('min_decision_margin',30,0,1000,False),('max_ambiguity',.2,0,1,False)]:
                finite(settings.get(key,default),f'settings.{key}',low,high,integer)
            for key in ('multitag','always_single_tag'):
                if key in settings: boolean(settings[key],f'settings.{key}')
        if settings.get('model_path'):
            settings['model_path']=str((directory / settings['model_path']).resolve())
        if pipeline.get('robot_to_camera') is not None:
            extrinsics=mapping(pipeline['robot_to_camera'],'robot_to_camera')
            for key in ('translation_m','rotation_rpy_deg'):
                values=extrinsics.get(key)
                if not isinstance(values,list) or len(values)!=3:
                    raise ValueError(f'robot_to_camera.{key} requires three numbers')
                for value in values: finite(value,f'robot_to_camera.{key}',-1000,1000)
        if pipeline['type']=='object':
            from .object_geometry import validate_geometry_settings
            validate_geometry_settings(mapping(pipeline.setdefault('geometry',{}),'geometry'))
            if settings.get('task','detect') not in ('detect','segment'):
                raise ValueError('Object task must be detect or segment')
            if settings.get('output_format','yolov8_raw') not in ('yolov8_raw','yolo26_end2end'):
                raise ValueError('Unsupported YOLO output format')
        preview_settings(pipeline.setdefault('preview',{}))
    if not any(p['enabled'] for p in pipelines):
        raise ValueError('At least one pipeline must be enabled')
    nt=mapping(config.setdefault('networktables',{}),'networktables')
    boolean(nt.setdefault('enabled',False),'networktables.enabled')
    if nt.get('server') is not None and not isinstance(nt['server'],str):
        raise ValueError('networktables.server must be a hostname string')
    if nt['enabled'] and not nt.get('server'):
        finite(nt.get('team'),'networktables.team',1,99999,True)
    if not isinstance(nt.get('table','/CustomVision'),str) or not nt.get('table','/CustomVision').startswith('/'):
        raise ValueError('networktables.table must be an absolute topic path')
    finite(nt.get('period_ms',10),'networktables.period_ms',5,1000)
    finite(config.setdefault('max_frame_age_ms',100),'max_frame_age_ms',10,1000)
    dashboard=mapping(config.setdefault('dashboard',{'enabled':True,'host':'0.0.0.0','port':5801}),'dashboard')
    boolean(dashboard.get('enabled',False),'dashboard.enabled')
    finite(dashboard.get('port',5801),'dashboard.port',1,65535,True)
    if not isinstance(dashboard.get('host','0.0.0.0'),str):
        raise ValueError('dashboard.host must be a string')
    preview_settings(dashboard,'dashboard')
    config['field_layout_data']=None
    if config.get('field_layout'):
        with (directory/config['field_layout']).open() as stream:
            config['field_layout_data']=json.load(stream)
    return config


def load_config(path):
    path=Path(path).resolve()
    with path.open() as stream:
        data=yaml.safe_load(stream)
    return validate_config(data,path.parent)
