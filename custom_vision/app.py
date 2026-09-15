"""Low-latency AprilTag workers, asynchronous browser controls and NT4 publication."""
import argparse
import logging
import signal
import threading
import time
import uuid
from pathlib import Path

import cv2
import numpy as np

from .config import load_config
from .camera import LatestFrameCapture
from .dashboard import Dashboard
from .publisher import Publisher

LOG = logging.getLogger('custom_vision')


def open_camera(settings):
    source=settings['source']
    backend=settings.get('backend','auto')
    api={'auto':cv2.CAP_ANY,'v4l2':cv2.CAP_V4L2,'gstreamer':cv2.CAP_GSTREAMER}.get(backend)
    if api is None: raise ValueError(f'Unknown camera backend: {backend}')
    if backend=='auto' and (isinstance(source,int) or str(source).startswith('/dev/')): api=cv2.CAP_V4L2
    cap=cv2.VideoCapture(source,api)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f'Camera unavailable: {source}. Check connection and PhotonVision ownership.')
    try:
        if settings.get('fourcc'): cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*settings['fourcc']))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,settings['width'])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,settings['height'])
        cap.set(cv2.CAP_PROP_FPS,settings['fps'])
        cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
        if settings.get('controls'):
            from .device_controls import apply_controls
            apply_controls(source,settings['controls'])
        if isinstance(source,int) or str(source).startswith('/dev/') or backend=='gstreamer':
            return LatestFrameCapture(cap)
        return cap
    except Exception:
        cap.release()
        raise


def make_detector(cfg, config):
    if cfg['type']=='apriltag':
        from .localization import Localization
        settings=dict(cfg['settings'])
        layout=config.get('field_layout_data')
        localizer=Localization(settings,cfg.get('calibration_data'),layout,cfg.get('robot_to_camera'))
        if layout is not None and settings.get('multitag',True) and not settings.get('always_single_tag',False):
            settings['known_tag_ids']=list(localizer.field_tags)
            settings['skip_single_when_multi']=True
        if settings.get('backend','pupil')=='native':
            from .native_apriltags import NativeAprilTagPipeline
            detector=NativeAprilTagPipeline(settings,cfg.get('calibration_data'))
        else:
            from .apriltags import AprilTagPipeline
            detector=AprilTagPipeline(settings,cfg.get('calibration_data'))
        detector.localization=localizer
        return detector
    from .objects import ObjectPipeline
    from .object_geometry import ObjectGeometry
    geometry=ObjectGeometry(cfg.get('geometry',{}),cfg.get('calibration_data'),cfg.get('robot_to_camera'))
    detector=ObjectPipeline(cfg['settings'],cfg.get('calibration_data'))
    detector.geometry=geometry
    return detector


def camera_settings_identity(settings):
    """Resolve UVC aliases so a device has exactly one reader in this runtime."""
    normalized=dict(settings)
    source=settings['source']
    if settings.get('backend') != 'gstreamer':
        if isinstance(source,int): source=f'/dev/video{source}'
        if str(source).startswith('/dev/'):
            source=str(Path(source).resolve())
    normalized['source']=str(source)
    return normalized


def validate_runtime(config):
    detectors=[]
    cameras={}
    try:
        for cfg in config['pipelines']:
            if not cfg.get('enabled',True): continue
            identity=camera_settings_identity(cfg['camera'])
            source=identity['source']
            if source in cameras and cameras[source]!=identity:
                raise ValueError('Pipelines sharing a camera must use identical camera settings')
            cameras[source]=identity
            detectors.append(make_detector(cfg,config))
    finally:
        for detector in detectors:
            if hasattr(detector,'close'): detector.close()


def draw_detections(frame,detections):
    from .overlay import annotate
    return annotate(frame,detections)


class Runtime:
    def __init__(self,config,*,stdout=False,max_frames=0,config_path=None):
        cv2.setNumThreads(1)  # Two camera workers own their detector thread budgets.
        self.config=config
        self.stop=threading.Event()
        self.restart_requested=False
        self.capture_shutdown_failed=False
        self.closed=False
        self.had_error=False
        self.max_frames=max_frames
        self.boot_id=str(uuid.uuid4())
        self.threads=[]
        self.lock=threading.RLock()
        self.states={}
        self.groups={}
        for cfg in config['pipelines']:
            if not cfg.get('enabled',True): continue
            detector=make_detector(cfg,config)
            identity=camera_settings_identity(cfg['camera'])
            source=identity['source']
            if source in self.groups and camera_settings_identity(self.groups[source][0][0]['camera'])!=identity:
                raise ValueError('Pipelines sharing a camera must use identical camera settings')
            self.groups.setdefault(source,[]).append((cfg,detector))
            self.states[cfg['name']]={'last_frame':time.monotonic(),'frame_id':0,'failed':False,'fps':0.,'last_publish':0.}
        self.publisher=Publisher(config['networktables'],stdout=stdout)
        self.controller=None
        if config_path:
            from .control import RuntimeController
            self.controller=RuntimeController(config_path,self.request_restart,validate_runtime)
        self.dashboard=Dashboard(config['dashboard'],controller=self.controller) if config['dashboard'].get('enabled') else None
        if self.dashboard:
            by_name={p['name']:p for p in config['pipelines']}
            def renderer(payload,frame):
                from .overlay import annotate
                cfg=by_name[payload['pipeline']]
                return annotate(frame,payload['detections'],cfg.get('calibration_data'),cfg['settings'].get('tag_size_m',.1651),objects=payload.get('objects'))
            self.dashboard.set_renderer(renderer)

    def request_restart(self):
        with self.lock:
            if getattr(self,'capture_shutdown_failed',False):
                return
            self.restart_requested=True
            self.stop.set()

    def release_camera(self,cap):
        # OpenCV and existing test captures return None on success; the latest
        # frame wrapper explicitly returns False if its reader still owns UVC.
        if cap.release() is False:
            with self.lock:
                self.capture_shutdown_failed=True
                self.had_error=True
                self.restart_requested=False
                self.stop.set()
            LOG.error('Camera reader is still blocked; stopping runtime and refusing to reopen its device')
            return False
        return True

    def emit(self,cfg,frame_id,captured,detections,*,frame=None,error=None,extras=None):
        with self.lock:
            if self.closed or (self.stop.is_set() and error is None): return
            # A worker can wait here behind another camera or the watchdog.
            # Recheck freshness after acquiring the publication lock; a cached
            # detector-end time must never revive an already invalidated target.
            now=time.monotonic()
            max_age=self.config.get('max_frame_age_ms',500)
            if error is None and (now-captured)*1000>max_age:
                error=f'Frame exceeded {max_age:g} ms age limit'
                detections=[]
                frame=None
                extras=None
            payload={'schema_version':2,'boot_id':getattr(self,'boot_id','test'),'pipeline':cfg['name'],'type':cfg['type'],
                     'mode':cfg.get('settings',{}).get('mode','3d') if cfg['type']=='apriltag' else cfg.get('settings',{}).get('task','detect'),
                     'backend':cfg.get('settings',{}).get('backend','pupil' if cfg['type']=='apriltag' else 'contour'),
                     'detector_device':('cuda' if cfg.get('settings',{}).get('backend')=='tensorrt' else 'cpu') if cfg['type']=='object' else cfg.get('settings',{}).get('detector_device','cpu'),
                     'input_kind':cfg.get('input_kind','camera'),'connected':error is None,'frame_id':frame_id,'capture_monotonic_us':int(captured*1e6),
                     'publish_unix_us':time.time_ns()//1000,'latency_ms':max(0.,(now-captured)*1000),
                     'timestamp_source':'host_frame_read_complete',
                     'capture_latency_offset_ms':cfg.get('camera',{}).get('capture_latency_offset_ms',0),
                     'detections':detections,'error':error,'preview_settings':cfg.get('preview',{})}
            if extras: payload.update(extras)
            self.publisher.publish(payload)
            if self.dashboard: self.dashboard.update(payload,frame)

    def camera_worker(self,group):
        cap=None
        count=frame_id=0
        geometry_capture={}
        max_age=self.config.get('max_frame_age_ms',500)
        try:
            while not self.stop.is_set() and (not self.max_frames or count<self.max_frames):
                try:
                    if cap is None: cap=open_camera(group[0][0]['camera'])
                    ok,frame=cap.read()
                    captured=getattr(cap,'last_capture_monotonic',time.monotonic())
                    if self.stop.is_set(): break
                    if not ok or frame is None: raise RuntimeError('Camera read failed or input video reached its end')
                    if count==0: LOG.info('Camera %s delivering %dx%d',group[0][0]['camera']['source'],frame.shape[1],frame.shape[0])
                    for cfg,detector in group:
                        if self.stop.is_set(): break
                        started=time.monotonic()
                        try:
                            # A previous pipeline on this camera (or capture delay)
                            # may have already consumed the entire freshness budget.
                            # Do not enqueue inference that can only be discarded.
                            if (started-captured)*1000>max_age:
                                if hasattr(detector,'geometry'): detector.geometry.reset()
                                with self.lock:
                                    self.states[cfg['name']].update(last_frame=captured,frame_id=frame_id,failed=True)
                                self.emit(cfg,frame_id,captured,[],error=f'Frame exceeded {max_age:g} ms age limit')
                                continue
                            detections=detector.process(frame)
                            detector_end=time.monotonic()
                            extras={}
                            if hasattr(detector,'localization'):
                                enrichment=detector.localization.enrich(detections,frame.shape)
                                detections=enrichment['detections']
                                extras['localization']=enrichment['localization']
                            if hasattr(detector,'geometry'):
                                previous_capture=geometry_capture.get(cfg['name'])
                                if previous_capture is not None and (captured-previous_capture)*1000>max_age:
                                    detector.geometry.reset()
                                if (detector_end-captured)*1000 <= max_age:
                                    enrichment=detector.geometry.enrich(detections,frame.shape,captured)
                                    detections=enrichment['detections']
                                    extras['objects']=enrichment['objects']
                                geometry_capture[cfg['name']]=captured
                                extras['inference_timings']=getattr(detector,'last_timings',{})
                            finished=time.monotonic()
                            with self.lock:
                                state=self.states[cfg['name']]
                                interval=finished-state.get('last_publish',0)
                                fps=1/interval if state.get('last_publish',0) and interval>0 else 0
                                previous=state.get('fps',0)
                                fps=.2*fps+.8*previous if previous else fps
                                state.update(last_frame=captured,frame_id=frame_id,failed=False,last_publish=finished,fps=fps)
                            extras.update(frame_size=[frame.shape[1],frame.shape[0]],processing_ms=(finished-started)*1000,detector_ms=(detector_end-started)*1000,
                                          localization_ms=(finished-detector_end)*1000,queue_ms=max(0.,(started-captured)*1000),
                                          fps=fps,dropped_frames=getattr(cap,'dropped_frames',0),
                                          native_timings=getattr(detector,'last_timings',{}))
                            if (finished-captured)*1000>max_age:
                                if hasattr(detector,'geometry'): detector.geometry.reset()
                                self.emit(cfg,frame_id,captured,[],error=f'Frame exceeded {max_age:g} ms age limit')
                            else:
                                self.emit(cfg,frame_id,captured,detections,frame=frame,extras=extras)
                        except Exception as exc:
                            self.had_error=True
                            if hasattr(detector,'geometry'): detector.geometry.reset()
                            LOG.exception('Pipeline %s failed',cfg['name'])
                            with self.lock: self.states[cfg['name']].update(last_frame=started,frame_id=frame_id,failed=True)
                            self.emit(cfg,frame_id,captured,[],error=str(exc))
                    frame_id+=1
                    count+=1
                except (RuntimeError,ValueError,TypeError,cv2.error,OSError) as exc:
                    self.had_error=True
                    LOG.warning('%s',exc)
                    for cfg,detector in group:
                        if hasattr(detector,'geometry'): detector.geometry.reset()
                        self.emit(cfg,frame_id,time.monotonic(),[],error=str(exc))
                    if cap is not None:
                        self.release_camera(cap)
                        cap=None
                    if self.max_frames: break
                    self.stop.wait(1.)
        finally:
            if cap is not None: self.release_camera(cap)

    def run(self):
        max_age=self.config.get('max_frame_age_ms',500)
        try:
            for group in self.groups.values():
                thread=threading.Thread(target=self.camera_worker,args=(group,),daemon=True)
                self.threads.append(thread)
                thread.start()
            while not self.stop.wait(min(.05,max_age/2000)):
                if not any(thread.is_alive() for thread in self.threads): break
                with self.lock:
                    for cfg in self.config['pipelines']:
                        if cfg['name'] not in self.states: continue
                        state=self.states[cfg['name']]
                        if (time.monotonic()-state['last_frame'])*1000>max_age:
                            self.emit(cfg,state['frame_id'],time.monotonic(),[],error=f'No fresh frame within {max_age:g} ms')
        finally:
            self.stop.set()
            for thread in self.threads: thread.join(timeout=2)
            for cfg in self.config['pipelines']:
                if cfg['name'] in self.states: self.emit(cfg,self.states[cfg['name']]['frame_id'],time.monotonic(),[],error='Runtime stopped')
            with self.lock:
                self.closed=True
                if self.dashboard: self.dashboard.close()
                self.publisher.close()
            if not any(thread.is_alive() for thread in self.threads):
                for group in self.groups.values():
                    for _,detector in group:
                        if hasattr(detector,'close'): detector.close()
            elif getattr(self,'restart_requested',False):
                self.restart_requested=False
                self.had_error=True
                LOG.error('A capture worker did not stop; refusing to open a duplicate camera during reload')
        return 1 if self.had_error and (self.max_frames or not getattr(self,'restart_requested',False)) else 0


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default='config/local.yaml' if Path('config/local.yaml').exists() else 'config/vision.yaml')
    parser.add_argument('--check',action='store_true',help='Validate configuration and load backends without cameras or networking')
    parser.add_argument('--no-nt',action='store_true')
    parser.add_argument('--headless','--no-dashboard',dest='no_dashboard',action='store_true',help='Run without browser UI/preview')
    parser.add_argument('--stdout-json',action='store_true')
    parser.add_argument('--max-frames',type=int,default=0)
    args=parser.parse_args(argv)
    if args.max_frames<0: parser.error('--max-frames must be nonnegative')
    logging.basicConfig(level=logging.INFO,format='%(levelname)s %(message)s')
    try:
        if args.check:
            validate_runtime(load_config(args.config))
            print('Configuration and detector initialization OK')
            return 0
        while True:
            config=load_config(args.config)
            if args.no_nt: config['networktables']['enabled']=False
            if args.no_dashboard: config['dashboard']['enabled']=False
            runtime=Runtime(config,stdout=args.stdout_json,max_frames=args.max_frames,config_path=args.config)
            for signum in (signal.SIGINT,signal.SIGTERM): signal.signal(signum,lambda *_:runtime.stop.set())
            result=runtime.run()
            if not runtime.restart_requested: return result
    except (ValueError,OSError,RuntimeError,ImportError) as exc:
        LOG.error('%s',exc)
        return 1


if __name__=='__main__':
    raise SystemExit(main())
