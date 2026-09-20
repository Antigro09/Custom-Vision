"""Current-frame tag-relative aiming, independent of field maps and odometry.

Offsets use WPILib's TAG frame: +X out of the tag's front, +Y to the right
when viewing an upright printed tag, +Z up. This is not the camera CV frame.
A camera-bearing result is not a shooter solution or a robot-heading command.
"""
from __future__ import annotations
import math
import cv2
import numpy as np
from .calibration import validate_calibration
from .localization import CV_TO_NWU, RAW_FROM_WP_TAG, _rpy_rotation, validate_robot_to_camera


def validate_poi(settings):
    if settings is None:
        settings = {}
    if not isinstance(settings, dict):
        raise ValueError('poi must be an object')
    unknown = set(settings) - {'enabled', 'calibration_verified', 'targets', 'max_ambiguity', 'max_reprojection_error_px'}
    if unknown:
        raise ValueError('Unknown poi setting(s): ' + ', '.join(map(str, unknown)))
    out = dict(enabled=False, calibration_verified=False, targets=[], max_ambiguity=.2, max_reprojection_error_px=3.)
    out.update(settings)
    for key in ('enabled', 'calibration_verified'):
        if not isinstance(out[key], bool):
            raise ValueError(f'poi.{key} must be boolean')
    for key, lo, hi in (('max_ambiguity', 0, 1), ('max_reprojection_error_px', .01, 10)):
        x = out[key]
        if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) or not lo <= x <= hi:
            raise ValueError(f'poi.{key} must be in [{lo}, {hi}]')
    targets = out['targets']
    if not isinstance(targets, list) or len(targets) > 32 or (out['enabled'] and not targets):
        raise ValueError('Enabled poi requires 1..32 targets')
    normalized, names = [], set()
    for target in targets:
        if not isinstance(target, dict) or set(target) != {'name', 'tag_id', 'offset_m'}:
            raise ValueError('Each POI requires exactly name, tag_id and offset_m')
        name, tag_id, offset = target['name'], target['tag_id'], target['offset_m']
        if not isinstance(name, str) or not 1 <= len(name) <= 64 or not name.isascii() or not name.replace('_','').replace('-','').isalnum() or name in names:
            raise ValueError('POI names must be unique ASCII letters, numbers, underscores or hyphens (1..64)')
        if isinstance(tag_id, bool) or not isinstance(tag_id, int) or not 0 <= tag_id <= 2**31-1:
            raise ValueError('POI tag_id must be a nonnegative integer')
        if not isinstance(offset, (list, tuple)) or len(offset) != 3:
            raise ValueError('POI offset_m requires [out_of_tag, right, up] in meters')
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or abs(v)>20 for v in offset):
            raise ValueError('POI offsets must be finite numbers within +/-20 meters')
        names.add(name)
        normalized.append(dict(name=name, tag_id=tag_id, offset_m=list(map(float, offset))))
    out['targets'] = normalized
    return out


class PointOfInterestTracker:
    def __init__(self, settings, calibration=None, robot_to_camera=None):
        self.settings = validate_poi(settings)
        self.calibration = validate_calibration(calibration) if calibration is not None else None
        self.mount = validate_robot_to_camera(robot_to_camera)
        self.K = self.distortion = None
        if self.calibration is not None:
            self.K = np.asarray(self.calibration['camera_matrix'], np.float64)
            self.distortion = np.asarray(self.calibration['dist_coeffs'], np.float64)
        self.mount_R = _rpy_rotation(self.mount['rotation_rpy_deg']) if self.mount is not None else None
        self.offsets = [(item, RAW_FROM_WP_TAG @ np.array(item['offset_m'])) for item in self.settings['targets']]

    def process(self, detections, image_shape, captured_s):
        """No history: an absent/ambiguous tag never leaves a stale aim target."""
        h, w = image_shape[:2]
        result = dict(valid=False, selected_name=None, targets=[], invalid_reason='disabled',
                      capture_monotonic_us=int(captured_s*1e6), uses_odometry=False, uses_field_layout=False)
        if not self.settings['enabled']:
            return result
        problem = ('no_calibration' if self.calibration is None else
                   'calibration_resolution_mismatch' if (w,h) != (self.calibration['width'],self.calibration['height']) else None)
        tagged = {}
        for detection in detections:
            tagged.setdefault(detection.get('id'), []).append(detection)
        for target, raw_offset in self.offsets:
            entry = dict(target, offset_frame='tag_wpilib', valid=False, geometry_valid=False,
                         calibration_verified=False, invalid_reason=problem, pixel=None,
                         camera_translation_m=None, tx_deg=None, ty_deg=None,
                         robot_translation_m=None, robot_yaw_deg=None, robot_elevation_deg=None)
            result['targets'].append(entry)
            if problem:
                continue
            candidates = tagged.get(target['tag_id'], [])
            if len(candidates) != 1:
                entry['invalid_reason'] = 'tag_not_visible' if not candidates else 'duplicate_tag_id'
                continue
            detection = candidates[0]
            if not detection.get('pose_valid'):
                entry['invalid_reason'] = detection.get('pose_invalid_reason') or 'invalid_tag_pose'
                continue
            # Runtime calls this tracker before field-localization enrichment.
            # Keep that guarantee at this boundary too: a derived MultiTag pose
            # has already used the uploaded field layout, even if it looks like
            # an ordinary tag-to-camera transform to a later caller.
            if detection.get('pose_source') in ('field_layout_multitag', 'field_layout_cuda_multitag'):
                entry['invalid_reason'] = 'field_derived_tag_pose'
                continue
            try:
                rvec = np.asarray(detection['rvec_rad'], np.float64).reshape(3)
                tvec = np.asarray(detection['tvec_m'], np.float64).reshape(3)
                if not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
                    raise ValueError('Nonfinite pose')
                rotation = cv2.Rodrigues(rvec)[0]
                point = rotation @ raw_offset + tvec
                if not np.isfinite(point).all() or point[2] <= 1e-6:
                    entry['invalid_reason'] = 'poi_behind_camera'
                    continue
                pixel = cv2.projectPoints(point.reshape(1,3), np.zeros(3), np.zeros(3), self.K, self.distortion)[0].reshape(2)
                if not np.isfinite(pixel).all():
                    raise ValueError('Nonfinite projection')
                x, y, z = map(float, point)
                # tx: right positive; ty: up positive. Both are optical-axis
                # planar angles; elevation differs from ty when tx is nonzero.
                entry.update(geometry_valid=True, pixel=pixel.tolist(), in_image=bool(0<=pixel[0]<w and 0<=pixel[1]<h),
                             camera_translation_m=point.tolist(), camera_frame='opencv_right_down_forward',
                             tx_deg=math.degrees(math.atan2(x,z)), ty_deg=math.degrees(math.atan2(-y,z)),
                             distance_m=float(np.linalg.norm(point)))
                if self.mount is not None:
                    robot = self.mount_R @ (CV_TO_NWU @ point) + np.asarray(self.mount['translation_m'])
                    horizontal = math.hypot(robot[0],robot[1])
                    entry['robot_translation_m'] = robot.tolist()
                    if horizontal>1e-9:
                        entry['robot_yaw_deg'] = math.degrees(math.atan2(robot[1],robot[0]))
                        entry['robot_elevation_deg'] = math.degrees(math.atan2(robot[2],horizontal))
                ambiguity = detection.get('pose_ambiguity')
                error = detection.get('reprojection_error_px')
                trusted = self.settings['calibration_verified'] and not self.calibration.get('benchmark_only',False)
                entry['calibration_verified'] = bool(trusted)
                entry['invalid_reason'] = (
                    'calibration_not_verified' if not trusted else
                    'unknown_pose_quality' if any(isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) or v<0 for v in (ambiguity,error)) else
                    'ambiguous_tag_pose' if ambiguity>self.settings['max_ambiguity'] else
                    'reprojection_error' if error>self.settings['max_reprojection_error_px'] else None)
                entry['valid'] = entry['invalid_reason'] is None
            except (KeyError, TypeError, ValueError, cv2.error):
                entry.update(valid=False, geometry_valid=False, invalid_reason='malformed_pose', pixel=None)
        # Explicit configuration order is priority. Do not average cameras or
        # choose a different physical target just because its tag is closer.
        selected = next((x for x in result['targets'] if x['valid']), None)
        result.update(valid=selected is not None, selected_name=selected['name'] if selected else None,
                      invalid_reason=None if selected else problem or 'no_valid_poi')
        return result
