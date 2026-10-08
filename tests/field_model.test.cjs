const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {test} = require('node:test');
const model = require('../custom_vision/static/field_model.js');
const fixture = name => JSON.parse(fs.readFileSync(path.join(__dirname, '../protocol/fixtures', `${name}.json`), 'utf8'));
const pose = (xyz = [1, 2, 0.5], q = [1, 0, 0, 0]) => ({translation_m: xyz, rotation_quaternion_wxyz: q, frame: 'wpilib_nwu'});
function packet(seq = 0, frame = seq + 1) {
  return {boot_id: 'test-boot', pipeline: 'front_tags', packet_seq: seq, frame_id: frame,
    connected: true, input_kind: 'synthetic', capture_monotonic_us: 1234567890123 + frame,
    capture_server_us: null, time_sync_valid: false,
    calibration_revision: 'cal-a', mount_revision: 'mount-a', field_layout_revision: 'field-a',
    localization: {valid: true, method: 'multitag_pnp', used_tag_ids: [1, 2], inlier_tag_count: 2,
      reprojection_error_px: 0.3, ambiguity: 0.04, field_to_camera: pose(), field_to_robot: pose([0.8, 2, 0])},
    detections: [{id: 1}, {id: 2}]};
}
const first = (store, now) => store.snapshot(now).sources[0];
const close = (actual, expected) => actual.forEach((v, i) => assert.ok(Math.abs(v - expected[i]) < 1e-10, `${v} vs ${expected[i]}`));

test('real producer camera-only fixture retains camera without inventing robot pose', () => {
  const store = model.createStore(), data = fixture('camera_only_localization'), unchanged = structuredClone(data);
  assert.equal(store.ingest(data, 100), true);
  const view = first(store, 120);
  assert.equal(view.camera.status, 'valid'); assert.equal(view.robot.status, 'unavailable');
  assert.equal(view.robot.reason, 'no_robot_to_camera'); assert.equal(view.robotPose, null);
  assert.equal(view.camera.pose.frame, 'wpilib_nwu');
  assert.deepEqual(view.camera.pose.translation_m, [2, 3, 0.7]);
  assert.equal(view.poseMeaning, 'vision_estimate'); assert.equal(view.inputKind, 'synthetic');
  assert.equal(view.cameraTrail.length, 1); assert.equal(view.robotTrail.length, 0);
  assert.equal(view.timing.captureServerUs, 1234569890123);
  assert.deepEqual(data, unchanged);
});

test('real single-tag packet exposes separate vision estimates and quality', () => {
  const store = model.createStore(); store.ingest(fixture('single_tag'), 10);
  const view = first(store, 25);
  assert.equal(view.robot.status, 'valid'); assert.equal(view.robot.actionable, true);
  assert.notDeepEqual(view.robotPose.translation_m, view.cameraPose.translation_m);
  assert.deepEqual(view.usedTagIds, [1]); assert.deepEqual(view.observedTagIds, [1]);
  assert.equal(view.quality.method, 'single_tag_pnp'); assert.equal(view.quality.inlierTagCount, 1);
  assert.equal(view.quality.reprojectionErrorPx, 0); assert.equal(view.receiptAgeMs, 15);
});

test('repeated polling cannot refresh receipt expiry and stale pose is a non-actionable ghost', () => {
  const store = model.createStore({staleAfterMs: 100}), data = packet();
  store.ingest(data, 0);
  assert.equal(store.ingest(data, 90), false);
  const view = first(store, 100);
  assert.equal(view.status, 'stale'); assert.equal(view.camera.status, 'stale');
  assert.equal(view.camera.actionable, false); assert.equal(view.robot.actionable, false);
  assert.ok(view.cameraPose); assert.equal(view.receiptAgeMs, 100);
  assert.deepEqual(view.observedTagIds, []); assert.deepEqual(view.usedTagIds, []);
});

test('already-old first server status is stale even before browser has dwelled', () => {
  const store = model.createStore();
  assert.equal(store.ingest(packet(), 20 - 2000), true);
  const view = first(store, 20);
  assert.equal(view.receiptAgeMs, 2000); assert.equal(view.status, 'stale');
  assert.equal(view.camera.receiptAgeMs, 2000); assert.equal(view.camera.actionable, false);
});

test('unsequenced disabled-publisher dashboard packets deduplicate without synthetic sequence', () => {
  const store = model.createStore({staleAfterMs: 100}), data = packet(); delete data.packet_seq;
  assert.equal(store.ingest(data, 0), true); assert.equal(store.ingest(structuredClone(data), 90), false);
  assert.equal(first(store, 100).status, 'stale'); assert.equal(first(store, 100).packetSeq, null);
  const invalidation = {...data, connected: false, error: 'watchdog', localization: {valid: false}};
  assert.equal(store.ingest(invalidation, 110), true);
  assert.equal(first(store, 110).cameraPose, null); assert.equal(first(store, 110).status, 'invalid');
  assert.equal(store.ingest(invalidation, 190), false);
  assert.equal(first(store, 210).status, 'stale');
});

test('unsequenced repeated invalidation publications can use logging identity without clock math', () => {
  const store = model.createStore(), data = {...packet(), connected: false, error: 'watchdog', publish_unix_us: 1791456000123456};
  delete data.packet_seq;
  store.ingest(data, 0);
  assert.equal(store.ingest({...data, publish_unix_us: data.publish_unix_us + 10}, 200), true);
  assert.equal(first(store, 200).cameraPose, null); assert.equal(first(store, 200).receiptAgeMs, 0);
});

test('same-frame watchdog invalidation wins before measurement deduplication', () => {
  const store = model.createStore(); store.ingest(packet(0, 42), 0);
  const invalid = {...packet(1, 42), connected: false, error: 'camera watchdog', localization: {valid: false}};
  assert.equal(store.ingest(invalid, 20), true);
  let view = first(store, 20);
  assert.equal(view.cameraPose, null); assert.equal(view.robotPose, null);
  assert.equal(view.camera.actionable, false); assert.equal(view.cameraTrail.length, 1);
  assert.equal(view.status, 'invalid'); assert.deepEqual(view.observedTagIds, []);
  assert.equal(store.ingest({...invalid, packet_seq: 2}, 30), true);
  assert.equal(store.ingest(packet(3, 42), 40), true);
  view = first(store, 40);
  assert.equal(view.cameraPose, null); assert.equal(view.reason, 'invalidated_or_older_frame');
  store.ingest(packet(4, 43), 50); assert.equal(first(store, 50).status, 'valid');
});

test('out-of-order publications cannot revive geometry or refresh freshness', () => {
  const store = model.createStore(); store.ingest(packet(5, 6), 50);
  assert.equal(store.ingest(packet(4, 5), 60), false);
  assert.equal(store.ingest({...packet(6, 7), connected: false}, 40), false);
  assert.equal(first(store, 70).packetSeq, 5); assert.equal(first(store, 70).receiptAgeMs, 20);
  // A newer publication with an older capture frame is accepted as invalid display data.
  assert.equal(store.ingest(packet(6, 5), 80), true);
  assert.equal(first(store, 80).cameraPose, null);
});

test('same-frame newer publication never adds duplicate trails or extends measurement expiry', () => {
  const store = model.createStore({staleAfterMs: 100}); store.ingest(packet(0, 42), 0);
  store.ingest(packet(1, 42), 90);
  const view = first(store, 100);
  assert.equal(view.cameraTrail.length, 1); assert.equal(view.robotTrail.length, 1);
  assert.equal(view.receiptAgeMs, 10); assert.equal(view.camera.receiptAgeMs, 100);
  assert.equal(view.status, 'stale'); assert.equal(view.reason, 'measurement_receipt_expired');
});

test('trails break after errors and rejected localization rather than connecting invented motion', () => {
  for (const update of [data => ({...data, connected: false, error: 'camera lost'}),
    data => ({...data, localization: {valid: false, invalid_reason: 'ambiguous'}})]) {
    const store = model.createStore(); store.ingest(packet(), 0); store.ingest(packet(1), 10);
    store.ingest(update(packet(2)), 20); store.ingest(packet(3), 30);
    const view = first(store, 30);
    assert.deepEqual(view.cameraTrail.map(point => point.break), [true, false, true]);
    assert.deepEqual(view.robotTrail.map(point => point.break), [true, false, true]);
    assert.deepEqual(view.cameraTrail.map(point => point.frameId), [1, 2, 4]);
  }
});

test('receipt gaps break trails even if no snapshot happened while stale', () => {
  const store = model.createStore({staleAfterMs: 100}); store.ingest(packet(), 0);
  store.ingest(packet(1), 150);
  assert.deepEqual(first(store, 150).cameraTrail.map(point => point.break), [true, true]);
  store.ingest(packet(2), 160);
  assert.deepEqual(first(store, 160).cameraTrail.map(point => point.break), [true, true, false]);
});

test('camera-only intervals break robot trails while continuous camera trails remain continuous', () => {
  const store = model.createStore(); store.ingest(packet(), 0);
  const cameraOnly = packet(1); cameraOnly.localization.field_to_robot = null;
  cameraOnly.localization.robot_pose_invalid_reason = 'no_robot_to_camera';
  store.ingest(cameraOnly, 10); store.ingest(packet(2), 20);
  assert.deepEqual(first(store, 20).cameraTrail.map(point => point.break), [true, false, false]);
  assert.deepEqual(first(store, 20).robotTrail.map(point => point.break), [true, true]);
});

test('clearTrails preserves current geometry, publication eligibility and future segment break', () => {
  const store = model.createStore(); const data = packet(); store.ingest(data, 0);
  const before = first(store, 0); store.clearTrails();
  const after = first(store, 10);
  assert.deepEqual(after.cameraPose, before.cameraPose); assert.equal(after.packetSeq, before.packetSeq);
  assert.deepEqual(after.cameraTrail, []); assert.deepEqual(after.robotTrail, []);
  assert.equal(store.ingest(data, 20), false);
  store.ingest(packet(1), 30);
  assert.equal(first(store, 30).cameraTrail[0].break, true);
});

test('boot changes clear trails and retired boot packets are rejected', () => {
  const store = model.createStore(); store.ingest(packet(), 0); store.ingest(packet(1), 10);
  const reboot = {...packet(), boot_id: 'new-boot'}; store.ingest(reboot, 20);
  assert.equal(first(store, 20).cameraTrail.length, 1); assert.equal(first(store, 20).bootId, 'new-boot');
  assert.equal(store.ingest(packet(99), 30), false); assert.equal(first(store, 30).bootId, 'new-boot');
});

test('each public geometry revision clears pending historical trails', () => {
  for (const key of ['calibration_revision', 'mount_revision', 'field_layout_revision']) {
    const store = model.createStore(); store.ingest(packet(), 0); store.ingest(packet(1), 10);
    store.ingest({...packet(2), [key]: 'changed'}, 20);
    assert.equal(first(store, 20).cameraTrail.length, 1, key);
    assert.equal(first(store, 20).robotTrail.length, 1, key);
  }
});

test('invalid or missing localization clears current geometry while retaining labeled history', () => {
  for (const localization of [undefined, {valid: false, invalid_reason: 'ambiguous', field_to_camera: pose()}]) {
    const store = model.createStore(); store.ingest(packet(), 0);
    store.ingest({...packet(1), localization}, 10);
    const view = first(store, 10);
    assert.equal(view.cameraPose, null); assert.equal(view.robotPose, null);
    assert.equal(view.cameraTrail.length, 1); assert.deepEqual(view.usedTagIds, []);
    assert.equal(view.status, localization ? 'invalid' : 'unavailable');
  }
});

test('invalid camera pose cannot validate robot pose; bad robot pose preserves valid camera', () => {
  const store = model.createStore(), data = packet();
  data.localization.field_to_camera = pose([Infinity, 0, 0]); store.ingest(data, 0);
  assert.equal(first(store, 0).robotPose, null); assert.equal(first(store, 0).status, 'invalid');
  const second = packet(1); second.localization.field_to_robot = pose([0, NaN, 0]); store.ingest(second, 10);
  assert.equal(first(store, 10).camera.status, 'valid'); assert.equal(first(store, 10).robotPose, null);
});

test('source/pipeline scopes have independent receipt expiry and selection', () => {
  const store = model.createStore({staleAfterMs: 100}); store.ingest(packet(), 0, 'jetson-a');
  store.ingest(packet(), 80, 'jetson-b'); store.ingest({...packet(), pipeline: 'rear_tags'}, 90, 'jetson-a');
  const sources = store.snapshot(100).sources;
  assert.equal(sources.length, 3); assert.equal(sources[0].status, 'stale');
  assert.equal(sources[1].status, 'valid'); assert.equal(sources[2].status, 'valid');
  assert.equal(store.snapshot(100, {pipeline: 'rear_tags'}).sources.length, 1);
  assert.equal(store.snapshot(100, {sourceId: 'jetson-b'}).sources.length, 1);
});

test('bounded histories and source eviction cap display memory', () => {
  const store = model.createStore({maxTrailPoints: 3, maxSources: 2});
  for (let i = 0; i < 12; i++) store.ingest(packet(i), i, 'first');
  const trail = first(store, 12).cameraTrail;
  assert.equal(trail.length, 3); assert.deepEqual(trail.map(point => point.frameId), [10, 11, 12]);
  store.ingest(packet(), 20, 'second'); store.ingest(packet(), 30, 'third');
  assert.deepEqual(store.snapshot(30).sources.map(source => source.sourceId), ['second', 'third']);
  store.clear(); assert.deepEqual(store.snapshot(30).sources, []);
});

test('snapshot modifications and original payload modifications cannot mutate model state', () => {
  const store = model.createStore(), data = packet(); store.ingest(data, 0);
  data.localization.field_to_camera.translation_m[0] = 900;
  const view = first(store, 0); view.cameraPose.translation_m[0] = 800; view.cameraTrail[0].translation_m[0] = 700;
  assert.equal(first(store, 0).cameraPose.translation_m[0], 1); assert.equal(first(store, 0).cameraTrail[0].translation_m[0], 1);
});

test('NWU to scene is explicit and right handed; quaternion axes preserve left-positive yaw', () => {
  assert.deepEqual(model.nwuToScene([1, 2, 3]), [1, 3, -2]);
  assert.equal(model.nwuToScene([0, NaN, 0]), null);
  const rotated = pose([2, 3, 4], [Math.SQRT1_2, 0, 0, Math.SQRT1_2]);
  close(model.transformPoint(rotated, [1, 0, 0]), [2, 4, 4]);
  const axes = model.poseAxes(rotated, 1);
  close(axes[0].to, [2, 4, 4]); close(axes[1].to, [1, 3, 4]); close(axes[2].to, [2, 3, 5]);
  assert.deepEqual(axes.map(axis => axis.axis), ['x', 'y', 'z']);
});

test('pose validation normalizes only small serialization drift and rejects optical/Rodrigues aliases', () => {
  assert.equal(model.normalizePose({...pose(), frame: 'opencv_optical'}), null);
  assert.equal(model.normalizePose(pose([0, 0, 0], [2, 0, 0, 0])), null);
  assert.equal(model.normalizePose(pose([0, 0, 0], [0, 0, 0, 0])), null);
  assert.equal(model.normalizePose(pose([0, 0, 0], [NaN, 0, 0, 1])), null);
  assert.deepEqual(model.normalizePose(pose([0, 0, 0], [0.999999, 0, 0, 0])).rotation_quaternion_wxyz, [1, 0, 0, 0]);
  assert.deepEqual(model.poseAxes(null), []); assert.deepEqual(model.poseAxes(pose(), Infinity), []);
});

test('measured mount adapter preserves Rz Ry Rx signs without supplying a missing mount', () => {
  assert.equal(model.mountPose(null), null);
  const mount = model.mountPose({translation_m: [0.3, 0.1, 0.5], rotation_rpy_deg: [0, -90, 0]});
  close(model.transformPoint(mount, [1, 0, 0]), [0.3, 0.1, 1.5]);
  const yaw = model.mountPose({translation_m: [0, 0, 0], rotation_rpy_deg: [0, 0, 90]});
  close(model.transformPoint(yaw, [1, 0, 0]), [0, 1, 0]);
});

test('layout adapter preserves metric origin and known tag height; no default backdrop exists', () => {
  const data = {field: {length: 12.25, width: 6.75}, tags: [{ID: 3,
    pose: {translation: {x: 4, y: 5, z: 1.7}, rotation: {quaternion: {W: 1, X: 0, Y: 0, Z: 0}}}}]};
  const result = model.layoutScene(data);
  assert.deepEqual(result.field, {length: 12.25, width: 6.75});
  assert.deepEqual(result.tags[0].pose.translation_m, [4, 5, 1.7]);
  assert.equal(model.layoutScene(null), null); assert.equal(model.layoutScene({...data, tags: []}), null);
  assert.equal(model.layoutScene({...data, tags: [data.tags[0], data.tags[0]]}), null);
  data.tags[0].pose.rotation.quaternion.W = 4; assert.equal(model.layoutScene(data), null);
});

test('unsafe identity, bounds and timestamps fail explicitly without changing accepted state', () => {
  const store = model.createStore(); store.ingest(packet(), 0);
  for (const invalid of [{...packet(1), packet_seq: -1}, {...packet(1), packet_seq: 2**53},
    {...packet(1), frame_id: false}, {...packet(1), boot_id: ''}]) assert.equal(store.ingest(invalid, 10), false);
  assert.equal(store.ingest(packet(1), NaN), false); assert.equal(first(store, 0).packetSeq, 0);
  const data = packet(1); data.capture_server_us = 2**53; data.capture_monotonic_us = 1.25;
  store.ingest(data, 10); assert.equal(first(store, 10).timing.captureServerUs, null);
  assert.equal(first(store, 10).timing.captureMonotonicUs, null);
  assert.throws(() => store.snapshot(NaN), RangeError);
  assert.throws(() => model.createStore({maxTrailPoints: 1001}), RangeError);
  assert.throws(() => model.createStore({maxSources: 0}), RangeError);
});
