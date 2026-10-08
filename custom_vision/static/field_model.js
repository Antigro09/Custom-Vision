/* Browser-only field display model. No pose fusion, odometry or wire mutation.
 * All metric poses remain WPILib NWU, with WXYZ quaternions and a fixed origin.
 * receiptMs/nowMs use one browser monotonic clock (performance.now()).
 */
(function (root, factory) {
  'use strict';
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.CVFieldModel = api;
}(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';
  const finite = value => typeof value === 'number' && Number.isFinite(value);
  const integer = value => Number.isSafeInteger(value) && value >= 0;
  const text = value => typeof value === 'string' && value.length > 0 && value.length <= 256;
  const vector = (value, length) => Array.isArray(value) && value.length === length && value.every(finite);
  const copy = value => JSON.parse(JSON.stringify(value));
  const nullableNumber = value => finite(value) ? value : null;
  const nullableInteger = value => integer(value) ? value : null;
  const nullableText = value => text(value) ? value : null;
  const tagIds = value => Array.isArray(value) ? [...new Set(value.slice(0, 1024).filter(id => integer(id) && id <= 2147483647))] : [];

  function normalizePose(value) {
    if (!value || value.frame !== 'wpilib_nwu' || !vector(value.translation_m, 3) ||
        !vector(value.rotation_quaternion_wxyz, 4)) return null;
    const norm = Math.hypot(...value.rotation_quaternion_wxyz);
    // Match the producer's layout tolerance; normalize serialization drift only.
    if (!finite(norm) || Math.abs(norm - 1) > 0.001) return null;
    const pose = {translation_m: value.translation_m.slice(),
      rotation_quaternion_wxyz: value.rotation_quaternion_wxyz.map(v => v / norm), frame: 'wpilib_nwu'};
    if (vector(value.rotation_rpy_deg, 3)) pose.rotation_rpy_deg = value.rotation_rpy_deg.slice();
    return pose;
  }

  function transformPoint(value, point) {
    const pose = normalizePose(value);
    if (!pose || !vector(point, 3)) return null;
    const [w, x, y, z] = pose.rotation_quaternion_wxyz;
    const [px, py, pz] = point;
    const result = [
      (1 - 2 * (y*y + z*z))*px + 2*(x*y - z*w)*py + 2*(x*z + y*w)*pz,
      2*(x*y + z*w)*px + (1 - 2*(x*x + z*z))*py + 2*(y*z - x*w)*pz,
      2*(x*z - y*w)*px + 2*(y*z + x*w)*py + (1 - 2*(x*x + y*y))*pz
    ].map((v, i) => v + pose.translation_m[i]);
    return result.every(finite) ? result : null;
  }

  function poseAxes(value, length = 0.35) {
    const pose = normalizePose(value);
    if (!pose || !finite(length) || length <= 0) return [];
    const axes = ['x', 'y', 'z'].map((axis, index) => {
      const point = [0, 0, 0]; point[index] = length;
      return {axis, color: ['#ef5350', '#66bb6a', '#42a5f5'][index],
        from: pose.translation_m.slice(), to: transformPoint(pose, point)};
    });
    return axes.every(axis => axis.to !== null) ? axes : [];
  }

  // Explicit right-handed display conversion: NWU +X -> scene +X,
  // NWU +Y -> scene -Z, NWU +Z -> scene +Y. No alliance/origin transform.
  function nwuToScene(point) { return vector(point, 3) ? [point[0], point[2], -point[1]] : null; }

  function mountPose(value) {
    if (!value || !vector(value.translation_m, 3) || !vector(value.rotation_rpy_deg, 3)) return null;
    const [roll, pitch, yaw] = value.rotation_rpy_deg.map(v => v * Math.PI / 360);
    const [cr, sr, cp, sp, cy, sy] = [Math.cos(roll), Math.sin(roll), Math.cos(pitch), Math.sin(pitch), Math.cos(yaw), Math.sin(yaw)];
    return normalizePose({frame: 'wpilib_nwu', translation_m: value.translation_m,
      rotation_rpy_deg: value.rotation_rpy_deg,
      rotation_quaternion_wxyz: [cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy]});
  }

  function layoutScene(value) {
    if (!value || !value.field || !finite(value.field.length) || value.field.length <= 0 ||
        !finite(value.field.width) || value.field.width <= 0 || !Array.isArray(value.tags) ||
        value.tags.length < 1 || value.tags.length > 1024) return null;
    const tags = [], ids = new Set();
    for (const tag of value.tags) {
      if (!tag || !integer(tag.ID) || tag.ID > 2147483647 || ids.has(tag.ID)) return null;
      const translation = tag.pose?.translation, q = tag.pose?.rotation?.quaternion;
      const pose = normalizePose({frame: 'wpilib_nwu', translation_m: [translation?.x, translation?.y, translation?.z],
        rotation_quaternion_wxyz: [q?.W, q?.X, q?.Y, q?.Z]});
      if (!pose) return null;
      ids.add(tag.ID); tags.push({id: tag.ID, pose});
    }
    return {field: {length: value.field.length, width: value.field.width}, tags};
  }

  function family(pose, status, reason) {
    return {pose, status, reason, actionable: status === 'valid'};
  }

  function createStore(options = {}) {
    const staleAfterMs = options.staleAfterMs ?? 1500;
    const maxTrailPoints = options.maxTrailPoints ?? 180;
    const maxSources = options.maxSources ?? 8;
    if (!finite(staleAfterMs) || staleAfterMs <= 0 || staleAfterMs > 60000 ||
        !integer(maxTrailPoints) || maxTrailPoints < 1 || maxTrailPoints > 1000 ||
        !integer(maxSources) || maxSources < 1 || maxSources > 32) throw new RangeError('Invalid field display bounds');
    const entries = new Map();

    function ingest(packet, receiptMs, sourceId = 'dashboard') {
      const hasSequence = packet?.packet_seq !== undefined && packet?.packet_seq !== null;
      if (!packet || !text(sourceId) || !text(packet.pipeline) || !text(packet.boot_id) ||
          (hasSequence && !integer(packet.packet_seq)) || !integer(packet.frame_id) || !finite(receiptMs)) return false;
      // A disabled Publisher leaves the internal dashboard payload unsequenced.
      // This identity is display-only, never a synthesized protocol packet_seq.
      const publicationIdentity = JSON.stringify([packet.frame_id, nullableInteger(packet.capture_monotonic_us),
        nullableInteger(packet.publish_unix_us), packet.connected === true, packet.localization?.valid === true,
        nullableText(packet.error), nullableText(packet.localization?.invalid_reason)]);
      const key = JSON.stringify([sourceId, packet.pipeline]);
      let entry = entries.get(key);
      const bootChanged = entry && entry.bootId !== packet.boot_id;
      if (entry && entry.retiredBoots.includes(packet.boot_id)) return false;
      if (entry && !bootChanged) {
        if (hasSequence && entry.packetSeq !== null && packet.packet_seq <= entry.packetSeq) return false;
        if (!hasSequence && (entry.packetSeq !== null || publicationIdentity === entry.publicationIdentity)) return false;
      }
      if (entry && receiptMs < entry.receiptMs) return false;
      const revisions = {calibration: nullableText(packet.calibration_revision),
        mount: nullableText(packet.mount_revision), fieldLayout: nullableText(packet.field_layout_revision)};
      const revisionChanged = entry && JSON.stringify(revisions) !== JSON.stringify(entry.revisions);
      if (!entry) {
        if (entries.size >= maxSources) {
          const oldest = [...entries].reduce((a, b) => a[1].receiptMs <= b[1].receiptMs ? a : b);
          entries.delete(oldest[0]);
        }
        entry = {retiredBoots: [], cameraTrail: [], robotTrail: [], trailBreak: {camera: true, robot: true},
          lastTrailFrame: -1, invalidatedFrame: -1, lastMeasurementFrame: -1};
        entries.set(key, entry);
      }
      if (bootChanged || revisionChanged) {
        if (bootChanged) entry.retiredBoots = [...entry.retiredBoots, entry.bootId].slice(-16);
        entry.cameraTrail = []; entry.robotTrail = [];
        entry.trailBreak = {camera: true, robot: true};
        entry.lastTrailFrame = -1; entry.invalidatedFrame = -1; entry.lastMeasurementFrame = -1;
      }
      entry.sourceId = sourceId; entry.pipeline = packet.pipeline; entry.bootId = packet.boot_id;
      entry.packetSeq = hasSequence ? packet.packet_seq : null;
      entry.publicationIdentity = publicationIdentity; entry.frameId = packet.frame_id; entry.receiptMs = receiptMs;
      entry.revisions = revisions; entry.inputKind = nullableText(packet.input_kind) || 'unknown';
      entry.poseMeaning = 'vision_estimate';
      entry.timing = {captureServerUs: nullableInteger(packet.capture_server_us),
        captureMonotonicUs: nullableInteger(packet.capture_monotonic_us), timeSyncValid: packet.time_sync_valid === true};
      const localization = packet.localization;
      const invalidReason = nullableText(packet.error) || nullableText(localization?.invalid_reason);
      entry.quality = {method: nullableText(localization?.method), poseDevice: nullableText(localization?.pose_device),
        inlierTagCount: nullableInteger(localization?.inlier_tag_count),
        reprojectionErrorPx: nullableNumber(localization?.reprojection_error_px), ambiguity: nullableNumber(localization?.ambiguity)};
      entry.observedTagIds = packet.connected === true && Array.isArray(packet.detections)
        ? tagIds(packet.detections.slice(0, 1024).map(detection => detection?.id)) : [];
      entry.usedTagIds = [];
      // Publication invalidation is applied before any frame deduplication.
      // Invalidation never leaves an old pose looking current/actionable.
      if (packet.connected !== true || packet.error) {
        entry.invalidatedFrame = Math.max(entry.invalidatedFrame, entry.lastMeasurementFrame, packet.frame_id);
        entry.camera = family(null, 'invalid', invalidReason || 'source_disconnected');
        entry.robot = family(null, 'invalid', invalidReason || 'source_disconnected');
      } else if (!localization || localization.valid !== true) {
        const status = localization ? 'invalid' : 'unavailable';
        entry.camera = family(null, status, invalidReason || 'no_field_localization');
        entry.robot = family(null, status, invalidReason || 'no_field_localization');
      } else if (packet.frame_id <= entry.invalidatedFrame || packet.frame_id < entry.lastMeasurementFrame) {
        entry.camera = family(null, 'invalid', 'invalidated_or_older_frame');
        entry.robot = family(null, 'invalid', 'invalidated_or_older_frame');
      } else {
        const newMeasurement = packet.frame_id > entry.lastMeasurementFrame;
        const previousCameraReceipt = entry.camera?.receiptMs;
        const previousRobotReceipt = entry.robot?.receiptMs;
        for (const name of ['camera', 'robot']) {
          if (entry[name]?.receiptMs !== undefined && receiptMs - entry[name].receiptMs >= staleAfterMs) entry.trailBreak[name] = true;
        }
        const cameraPose = normalizePose(localization.field_to_camera);
        const robotPose = cameraPose ? normalizePose(localization.field_to_robot) : null;
        entry.camera = cameraPose ? family(cameraPose, 'valid', null) : family(null, 'invalid', 'invalid_field_camera_pose');
        entry.robot = robotPose ? family(robotPose, 'valid', null)
          : family(null, cameraPose ? 'unavailable' : 'invalid', nullableText(localization.robot_pose_invalid_reason) || (cameraPose ? 'no_field_robot_pose' : 'invalid_field_camera_pose'));
        if (cameraPose) entry.camera.receiptMs = newMeasurement || previousCameraReceipt === undefined ? receiptMs : previousCameraReceipt;
        if (robotPose) entry.robot.receiptMs = newMeasurement ? receiptMs : (previousRobotReceipt ?? previousCameraReceipt ?? receiptMs);
        entry.usedTagIds = cameraPose ? tagIds(localization.used_tag_ids) : [];
        entry.lastMeasurementFrame = Math.max(entry.lastMeasurementFrame, packet.frame_id);
        if (packet.frame_id > entry.lastTrailFrame) {
          for (const name of ['camera', 'robot']) {
            if (entry[name].pose) {
              const trail = entry[`${name}Trail`];
              trail.push({translation_m: entry[name].pose.translation_m.slice(), frameId: packet.frame_id,
                packetSeq: entry.packetSeq, receiptMs, break: entry.trailBreak[name]});
              entry.trailBreak[name] = false;
              if (trail.length > maxTrailPoints) trail.splice(0, trail.length - maxTrailPoints);
            }
          }
          entry.lastTrailFrame = packet.frame_id;
        }
      }
      for (const name of ['camera', 'robot']) {
        if (entry[name].status !== 'valid') entry.trailBreak[name] = true;
      }
      return true;
    }

    function snapshot(nowMs, filter = {}) {
      if (!finite(nowMs) || nowMs < 0) throw new RangeError('Field snapshot needs a monotonic browser time');
      const sources = [];
      for (const entry of entries.values()) {
        if ((filter.pipeline !== undefined && filter.pipeline !== entry.pipeline) ||
            (filter.sourceId !== undefined && filter.sourceId !== entry.sourceId)) continue;
        const age = Math.max(0, nowMs - entry.receiptMs);
        const source = copy(entry);
        delete source.retiredBoots; delete source.lastTrailFrame;
        delete source.trailBreak;
        delete source.publicationIdentity;
        delete source.invalidatedFrame; delete source.lastMeasurementFrame;
        source.receiptAgeMs = age;
        source.staleAfterMs = staleAfterMs;
        for (const name of ['camera', 'robot']) {
          source[name].receiptAgeMs = source[name].pose ? Math.max(0, nowMs - source[name].receiptMs) : null;
          if (source[name].pose && source[name].receiptAgeMs >= staleAfterMs) {
            source[name].status = 'stale'; source[name].reason = 'measurement_receipt_expired';
            source[name].actionable = false;
          }
        }
        // A stale retained pose can be drawn only as a labeled historical ghost.
        if (age >= staleAfterMs) {
          for (const name of ['camera', 'robot']) {
            source[name].actionable = false;
            if (source[name].pose) source[name].status = 'stale';
          }
          source.status = 'stale'; source.reason = 'receipt_expired';
          source.usedTagIds = []; source.observedTagIds = [];
        } else {
          source.status = source.camera.status;
          source.reason = source.camera.reason;
          if (source.status === 'stale') { source.usedTagIds = []; source.observedTagIds = []; }
        }
        source.cameraPose = source.camera.pose; source.robotPose = source.robot.pose;
        sources.push(source);
      }
      return {sources, staleAfterMs};
    }
    function clearTrails() {
      for (const entry of entries.values()) {
        entry.cameraTrail = []; entry.robotTrail = []; entry.trailBreak = {camera: true, robot: true};
      }
    }
    return {ingest, snapshot, clearTrails, clear: () => entries.clear()};
  }
  return {createStore, normalizePose, transformPoint, poseAxes, nwuToScene, mountPose, layoutScene};
}));
