'use strict';
const $ = (id) => document.getElementById(id);
const state = {config: null, selected: null, token: '', writable: false, devices: [], modes: [], controls: [], status: {}, imageURL: null, previewBusy: false, busy: false};
const n = (id) => Number($(id).value);
const value = (id, v) => { $(id).value = v ?? ''; };
const selectedPipeline = () => state.config?.pipelines?.find(p => p.name === state.selected);

async function request(path, data, retryToken = true) {
  const response = await fetch(path, data === undefined ? {cache: 'no-store'} : {
    method: 'POST', headers: {'Content-Type': 'application/json', 'X-Custom-Vision-CSRF': state.token}, body: JSON.stringify(data)
  });
  const body = await response.json();
  // Restarting the runtime rotates the server token. Refresh only after an
  // explicit rejection, so an accepted write is never replayed on uncertainty.
  // Keep the user's current form/configuration intact during this handshake.
  if (data !== undefined && retryToken && response.status === 403 &&
      typeof body.error === 'string' && body.error.startsWith('Missing or invalid setup token')) {
    const setup = await request('/api/config');
    state.token = setup.csrf_token;
    return request(path, data, false);
  }
  if (!response.ok) throw new Error(body.error || `Request failed (${response.status})`);
  return body;
}
function notice(message, error = false) {
  $('notice').textContent = message;
  $('notice').className = error ? 'error' : '';
  $('notice').hidden = !message;
}
function setBusy(busy) {
  state.busy = busy;
  $('save').disabled = busy || !state.writable;
  $('save').textContent = busy ? 'Applying…' : 'Save & apply settings';
  $('refresh').disabled = busy;
}
function option(select, text, val, selected = false) {
  const el = document.createElement('option'); el.textContent = text; el.value = val; el.selected = selected; select.append(el); return el;
}
function hideFrame() {
  $('preview-image').hidden = true;
  $('preview-image').removeAttribute('src');
  $('preview-empty').hidden = false;
  if (state.imageURL) URL.revokeObjectURL(state.imageURL);
  state.imageURL = null;
}
function deviceForSource(source) {
  const normalized = typeof source === 'number' ? `/dev/video${source}` : source;
  return state.devices.find(d => d.source === normalized);
}
function populateModes(device, camera) {
  const select = $('capture-mode'); select.replaceChildren();
  state.modes = [...(device?.modes || [])];
  let index = state.modes.findIndex(m => m.width === camera.width && m.height === camera.height && Math.abs(m.fps - camera.fps) < .01 && m.fourcc === camera.fourcc);
  if (index < 0) {
    state.modes.unshift({width: camera.width || 1280, height: camera.height || 800, fps: camera.fps || 120, fourcc: camera.fourcc || 'MJPG', configured: true});
    index = 0;
  }
  state.modes.forEach((m, i) => option(select, `${m.width} × ${m.height} · ${m.fps} FPS · ${m.fourcc}${m.configured ? ' (configured; unverified)' : ''}`, i, i === index));
  $('mode-note').textContent = device?.error || (device ? `${device.name}: ${device.modes.length} driver-advertised mode tuples. Changing resolution requires a matching calibration.` : 'No matching device is attached. The saved mode is shown as unverified; camera availability and capture negotiation are checked at runtime.');
}
function populateControls(device, saved) {
  state.controls = [];
  const root = $('camera-controls'); root.replaceChildren();
  if (!device?.controls?.length) { const p = document.createElement('p'); p.className = 'hint'; p.textContent = 'No controls reported for this camera.'; root.append(p); return; }
  for (const c of device.controls) {
    const label = document.createElement('label'); label.textContent = c.name.replaceAll('_', ' ');
    const input = document.createElement('select'); input.dataset.control = c.name;
    const unchanged = `Leave unchanged${c.value === undefined ? '' : ` (current: ${c.value})`}`;
    option(input, unchanged, '');
    if (c.type === 'menu' || c.type === 'intmenu' || c.type === 'bool') {
      const items = Object.keys(c.menu || {}).length ? c.menu : {0: 'Off', 1: 'On'};
      for (const [key, text] of Object.entries(items)) option(input, `${text} (${key})`, key, saved[c.name] === Number(key));
      label.append(input);
      input.disabled = !c.writable;
      state.controls.push({name: c.name, element: input});
    } else if (c.type === 'int' && c.min !== undefined && c.max !== undefined) {
      const number = document.createElement('input'); number.type = 'number'; number.min = c.min; number.max = c.max; number.step = c.step || 1;
      number.placeholder = `${c.value ?? 'Current'} · ${c.min} to ${c.max}`; number.value = saved[c.name] ?? '';
      number.disabled = !c.writable; label.append(number); state.controls.push({name: c.name, element: number});
    } else { input.disabled = true; label.append(input); }
    if (!c.writable) { const hint = document.createElement('span'); hint.className = 'subtle'; hint.textContent = (c.flags || []).join(', ') || 'Read-only or unsupported type'; label.append(hint); }
    root.append(label);
  }
}
function updateMountInputs() {
  for (const id of ['mount-x','mount-y','mount-z','mount-roll','mount-pitch','mount-yaw']) $(id).disabled = !$('mount-measured').checked;
}
$('mount-measured').onchange = updateMountInputs;
function updateDetectorDevice() {
  $('tag-device').disabled = selectedPipeline()?.type !== 'apriltag' || $('tag-backend').value !== 'native';
  if ($('tag-backend').value !== 'native') value('tag-device', 'cpu');
  $('pose-device').disabled = selectedPipeline()?.type !== 'apriltag' || $('tag-backend').value !== 'native' || $('tag-mode').value === '2d';
  if ($('pose-device').disabled) value('pose-device','cpu');
}
$('tag-backend').onchange = updateDetectorDevice;
$('tag-mode').onchange = updateDetectorDevice;
const objectNeuralFields = ['object-task','object-format','object-model','object-labels','object-width','object-height','object-confidence'];
function updateObjectBackend() {
  const neural = ['tensorrt', 'opencv_onnx'].includes($('object-backend').value);
  $('object-neural-settings').hidden = !neural;
  for (const id of objectNeuralFields) $(id).disabled = selectedPipeline()?.type === 'apriltag' || !neural;
  $('object-backend-note').textContent = neural
    ? 'Use a trained model already stored on this Jetson. Labels must match training class order; input dimensions and output format must match the exported model.'
    : 'This shape or color baseline reports candidates, not a trained neural detector. Its score describes circularity or color fill, not model probability.';
}
$('object-backend').onchange = updateObjectBackend;
function populatePipeline() {
  const pipeline = selectedPipeline(); if (!pipeline) return;
  const camera = pipeline.camera || {}, tags = pipeline.settings || {}, mount = pipeline.robot_to_camera || {};
  const isObject = pipeline.type !== 'apriltag', geometry = pipeline.geometry || {};
  const preview = {...(state.config.dashboard || {}), ...(pipeline.preview || {})};
  $('pipeline-title').textContent = pipeline.name;
  $('pipeline-description').textContent = isObject ? 'Object detection and calibrated robot-relative targets' : 'AprilTag aiming and calibrated robot localization';
  $('apriltag-panel').hidden = isObject; $('object-panel').hidden = !isObject;
  $('field-settings').hidden = isObject; $('object-geometry').hidden = !isObject;
  $('mount-title').textContent = isObject ? 'Robot & target plane' : 'Robot & field';
  $('preview-image').alt = isObject ? 'Camera preview with object boxes and selected targets' : 'Camera preview with AprilTag overlays';
  for (const id of ['object-backend','object-limit','object-target-height','object-anchor','object-max-range','object-max-std','object-intake-x','object-intake-y','object-standoff']) $(id).disabled = !isObject;
  $('pipeline-enabled').checked = pipeline.enabled !== false;
  value('camera-source', camera.source); value('camera-backend', camera.backend || 'auto');
  const device = deviceForSource(camera.source);
  $('device-select').replaceChildren(); option($('device-select'), 'Use configured source', '');
  state.devices.forEach(d => option($('device-select'), `${d.source} · ${d.name}`, d.source, device === d));
  populateModes(device, camera); populateControls(device, camera.controls || {});
  value('tag-mode', tags.mode || '3d'); value('tag-backend', tags.backend || 'native');
  value('tag-device', tags.detector_device || 'cpu'); value('pose-device', tags.pose_device || 'cpu'); updateDetectorDevice();
  const poi = pipeline.poi || {};
  $('poi-enabled').checked = poi.enabled === true; $('poi-verified').checked = poi.calibration_verified === true;
  value('poi-targets', JSON.stringify(poi.targets || [], null, 2));
  for (const id of ['poi-enabled','poi-verified','poi-targets']) $(id).disabled = isObject;
  $('box-depth-field').hidden = isObject; $('box-depth-note').hidden = isObject;
  $('box-depth').disabled = isObject;
  value('tag-size', tags.tag_size_m ?? .1651); value('tag-threads', tags.threads ?? 2);
  value('tag-decimate', tags.quad_decimate ?? 1); value('tag-margin', tags.min_decision_margin ?? 20);
  $('multitag').checked = tags.multitag !== false;
  for (const id of ['tag-mode','tag-backend','tag-size','tag-threads','tag-decimate','tag-margin','multitag']) $(id).disabled = pipeline.type !== 'apriltag';
  value('object-backend', isObject ? tags.backend || 'contour' : 'contour');
  value('object-task', tags.task || 'detect'); value('object-format', tags.output_format || 'yolov8_raw');
  value('object-model', tags.model_path || ''); value('object-labels', (tags.labels || []).join('\n'));
  const inputSize = Array.isArray(tags.input_size) ? tags.input_size : [tags.input_size || 640, tags.input_size || 640];
  value('object-width', inputSize[0]); value('object-height', inputSize[1]);
  value('object-confidence', tags.confidence_threshold ?? .35); value('object-limit', tags.max_detections ?? 16);
  value('object-target-height', geometry.target_height_m); value('object-anchor', geometry.anchor || 'bbox_center');
  value('object-max-range', geometry.max_range_m ?? 5); value('object-max-std', geometry.max_position_std_m ?? .3);
  value('object-intake-x', geometry.intake_offset_m?.[0] ?? 0); value('object-intake-y', geometry.intake_offset_m?.[1] ?? 0);
  value('object-standoff', geometry.approach_standoff_m ?? .1); updateObjectBackend();
  $('mount-measured').checked = pipeline.robot_to_camera != null; updateMountInputs();
  const translation = mount.translation_m || [0, 0, 0], rotation = mount.rotation_rpy_deg || [0, 0, 0];
  ['mount-x','mount-y','mount-z'].forEach((id, i) => value(id, translation[i]));
  ['mount-roll','mount-pitch','mount-yaw'].forEach((id, i) => value(id, rotation[i]));
  value('preview-rotation', preview.rotation_deg ?? 0); value('preview-fps', preview.stream_fps ?? 10);
  value('preview-width', preview.stream_width ?? 640); value('preview-quality', preview.jpeg_quality ?? 70); value('box-depth', preview.box_depth_ratio ?? .5);
  $('calibration-path').textContent = pipeline.calibration || 'No calibration uploaded';
  $('field-path').textContent = state.config.field_layout || 'No field layout uploaded';
  const nt = state.config.networktables || {};
  $('nt-description').textContent = nt.enabled ? `Team ${nt.team || '—'} · ${nt.server || 'Automatic roboRIO discovery'} · ${nt.table || '/CustomVision'}` : 'NetworkTables publication is disabled in the saved configuration.';
  for (const button of $('pipelines').children) button.classList.toggle('selected', button.dataset.name === pipeline.name);
  updateStatus();
}
function buildNav() {
  $('pipelines').replaceChildren();
  for (const p of state.config.pipelines || []) {
    const button = document.createElement('button'); button.className = 'pipeline'; button.dataset.name = p.name;
    const title = document.createElement('strong'); title.textContent = p.name;
    const subtitle = document.createElement('small'); subtitle.textContent = `${p.type === 'apriltag' ? 'AprilTags' : 'Objects'} · ${p.enabled === false ? 'disabled' : 'enabled'}`;
    button.append(title, subtitle); button.onclick = () => { state.selected = p.name; hideFrame(); populatePipeline(); };
    $('pipelines').append(button);
  }
}
async function loadConfiguration() {
  const [setup, devices] = await Promise.all([request('/api/config'), request('/api/devices')]);
  state.config = setup.config; state.token = setup.csrf_token; state.writable = setup.writable;
  state.devices = devices.devices || [];
  if (!selectedPipeline()) state.selected = state.config.pipelines?.[0]?.name;
  buildNav(); populatePipeline(); setBusy(false);
  if (!state.writable) notice('This dashboard is read-only. Start the runtime with a configuration controller to enable setup.');
  $('connection').textContent = 'Connected'; $('connection').className = 'badge live';
}
function collectSettings() {
  const config = structuredClone(state.config), p = config.pipelines.find(p => p.name === state.selected);
  if (!p) throw new Error('No pipeline selected');
  p.enabled = $('pipeline-enabled').checked;
  const source = $('camera-source').value.trim();
  p.camera = {...p.camera, ...state.modes[Number($('capture-mode').value)], source: /^\d+$/.test(source) ? Number(source) : source, backend: $('camera-backend').value};
  delete p.camera.configured;
  const controls = {...(p.camera.controls || {})};
  for (const {name, element} of state.controls) {
    if (element.disabled) continue;
    if (element.value === '') delete controls[name]; else controls[name] = Number(element.value);
  }
  p.camera.controls = controls;
  if (p.type === 'apriltag') p.settings = {...p.settings, mode: $('tag-mode').value, backend: $('tag-backend').value, detector_device: $('tag-device').value || 'cpu', pose_device: $('pose-device').value || 'cpu', tag_size_m: n('tag-size'), threads: n('tag-threads'), quad_decimate: n('tag-decimate'), min_decision_margin: n('tag-margin'), multitag: $('multitag').checked};
  else {
    p.settings = {...p.settings, backend: $('object-backend').value, max_detections: n('object-limit')};
    if (['tensorrt', 'opencv_onnx'].includes(p.settings.backend)) Object.assign(p.settings, {
      task: $('object-task').value, output_format: $('object-format').value,
      model_path: $('object-model').value.trim(),
      labels: $('object-labels').value.split(/[,\n]/).map(label => label.trim()).filter(Boolean),
      input_size: [n('object-width'), n('object-height')], confidence_threshold: n('object-confidence')
    });
    p.geometry = {...p.geometry, target_height_m: $('object-target-height').value.trim() === '' ? null : n('object-target-height'),
      anchor: $('object-anchor').value, max_range_m: n('object-max-range'), max_position_std_m: n('object-max-std'),
      intake_offset_m: [n('object-intake-x'), n('object-intake-y')], approach_standoff_m: n('object-standoff')};
  }
  if (p.type === 'apriltag') {
    let targets;
    try { targets = JSON.parse($('poi-targets').value || '[]'); } catch (_) { throw new Error('POI targets must be a valid JSON array'); }
    if (!Array.isArray(targets)) throw new Error('POI targets must be a JSON array');
    p.poi = {...p.poi, enabled: $('poi-enabled').checked, calibration_verified: $('poi-verified').checked, targets};
  }
  p.robot_to_camera = $('mount-measured').checked ? {translation_m: ['mount-x','mount-y','mount-z'].map(n), rotation_rpy_deg: ['mount-roll','mount-pitch','mount-yaw'].map(n)} : null;
  p.preview = {...p.preview, rotation_deg: n('preview-rotation'), stream_fps: n('preview-fps'), stream_width: n('preview-width'), jpeg_quality: n('preview-quality')};
  if (p.type === 'apriltag') p.preview.box_depth_ratio = n('box-depth');
  return config;
}
function applyMessage(result) {
  return result.message || (result.restart_required ? 'Saved. A runtime restart is required before these settings take effect.' : 'Settings accepted. Camera reconfiguration may briefly interrupt frames; check pipeline health below.');
}
$('settings-form').addEventListener('submit', async event => {
  event.preventDefault(); if (!state.writable || state.busy) return;
  setBusy(true); notice('Validating and applying configuration…');
  try { const result = await request('/api/config', collectSettings()); await loadConfiguration(); notice(applyMessage(result)); }
  catch (error) { notice(error.message, true); }
  finally { setBusy(false); }
});
async function upload(input, kind) {
  const file = input.files[0]; if (!file || state.busy) return;
  if (file.size > 1000000) { notice('JSON upload is too large. Maximum file size is 1 MB.', true); return; }
  setBusy(true); notice(`Validating ${kind}…`);
  try {
    const data = JSON.parse(await file.text());
    const result = await request(kind === 'calibration' ? '/api/calibration' : '/api/field-layout', kind === 'calibration' ? {pipeline: state.selected, data} : {data});
    await loadConfiguration(); notice(applyMessage(result));
  } catch (error) { notice(error.message, true); }
  finally { input.value = ''; setBusy(false); }
}
$('calibration-upload').addEventListener('change', event => upload(event.target, 'calibration'));
$('field-upload').addEventListener('change', event => upload(event.target, 'field layout'));
$('refresh').onclick = async () => { try { notice(''); await loadConfiguration(); } catch (error) { notice(error.message, true); } };
$('device-select').onchange = () => {
  const d = state.devices.find(d => d.source === $('device-select').value); if (!d) return;
  value('camera-source', d.source); populateModes(d, selectedPipeline()?.camera || {}); populateControls(d, {});
};
$('camera-source').onchange = () => { const d = deviceForSource(/^\d+$/.test($('camera-source').value) ? Number($('camera-source').value) : $('camera-source').value); populateModes(d, selectedPipeline()?.camera || {}); populateControls(d, {}); };
function updateStatus() {
  const status = state.status[state.selected];
  const connected = status?.connected === true;
  const isObject = selectedPipeline()?.type !== 'apriltag';
  $('preview-label').textContent = connected ? (status.input_kind === 'synthetic' ? 'Synthetic preview' : 'Live') : (selectedPipeline()?.enabled === false ? 'Disabled' : 'Waiting for camera');
  $('preview-label').className = connected ? 'badge live' : 'badge';
  const backend = status?.backend;
  const objectBackends = {tensorrt: 'TensorRT / GPU', opencv_onnx: 'ONNX / CPU', contour: 'Shape baseline', hsv: 'Color baseline'};
  $('backend-label').textContent = backend === 'native' ? (status.detector_device === 'cuda' ? 'CUDA / GPU' : 'C++ / CPU') : (objectBackends[backend] || backend || '—');
  const latency = status?.latency_ms;
  $('latency').replaceChildren(document.createTextNode(Number.isFinite(latency) && connected ? latency.toFixed(1) : '—'));
  const unit = document.createElement('small'); unit.textContent = ' ms'; $('latency').append(unit);
  $('target-count').textContent = connected ? status.detections?.length || 0 : 0;
  $('target-count-label').textContent = isObject ? 'Visible objects' : 'Visible tags';
  $('pose-label').textContent = isObject ? 'Object geometry' : 'Pose estimate';
  $('object-selection').hidden = !isObject;
  const fps = connected && Number.isFinite(status.fps) && status.fps >= 0 ? status.fps : null;
  const fpsText = fps === null ? '—' : fps.toFixed(1);
  $('processing-fps').textContent = connected ? fpsText : '0.0';
  const ms = x => Number.isFinite(x) && x >= 0 ? x.toFixed(2) : '—';
  let stageTimings = 'Awaiting data';
  if (connected) {
    if (isObject) {
      const times = status.inference_timings || {};
      stageTimings = ['tensorrt', 'opencv_onnx'].includes(backend)
        ? `Preprocess ${ms(times.preprocess_ms)} ms · inference ${ms(times.inference_ms)} ms · decode ${ms(times.decode_ms)} ms`
        : `Detect ${ms(status.detector_ms)} ms`;
      stageTimings += ` · geometry ${ms(status.localization_ms)} ms`;
    } else {
      const times = status.native_timings || {};
      const poseDevice = {cuda: 'CUDA', cpu: 'CPU', mixed: 'CPU + CUDA'}[status.pose_device] || 'device unreported';
      if (status.mode === '2d' || status.pose_device === 'none') {
        stageTimings = `Detect ${ms(times.detect_ms ?? status.detector_ms)} ms · single-tag pose not run${status.mode === '2d' ? ' (2D)' : ''}`;
      } else if (Number.isFinite(times.detect_ms) || Number.isFinite(times.pose_ms)) {
        stageTimings = `Detect ${ms(times.detect_ms)} ms · single-tag pose ${ms(status.single_tag_pose_ms ?? times.pose_ms)} ms (${poseDevice})`;
      } else {
        stageTimings = `Detection + single-tag pose ${ms(status.detector_ms)} ms (${poseDevice})`;
      }
      const fieldPoseDevice = {cpu: 'CPU', cuda: 'CUDA', mixed: 'CPU + CUDA'}[status.localization?.pose_device];
      stageTimings += ` · localization / POI ${ms(status.localization_ms)} ms${fieldPoseDevice ? ` (field solve ${fieldPoseDevice})` : ''}`;
    }
    stageTimings += ` · queue ${ms(status.queue_ms)} ms`;
  }
  $('pipeline-timings').textContent = stageTimings;
  const poi = connected ? status.poi : null;
  const aim = poi?.valid ? poi.targets?.find(x => x.valid && x.name === poi.selected_name) : null;
  const previewAim = aim || poi?.targets?.find(x => x.geometry_valid);
  $('poi-status').hidden = isObject || (!selectedPipeline()?.poi?.enabled && !poi?.targets?.length);
  $('poi-selected').textContent = aim ? `${aim.name} · tag ${aim.tag_id}` : previewAim ? `${previewAim.name} · PREVIEW ONLY` : 'No valid POI';
  $('poi-angles').textContent = previewAim && Number.isFinite(previewAim.tx_deg) && Number.isFinite(previewAim.ty_deg)
    ? `tx ${previewAim.tx_deg.toFixed(2)}° right · ty ${previewAim.ty_deg.toFixed(2)}° up${aim ? '' : ' · ' + (previewAim.invalid_reason || 'unverified').replaceAll('_',' ')}`
    : 'Requires a currently visible tag with an unambiguous pose and validated intrinsics.';
  const localization = status?.localization || status?.field_pose || status?.robot_pose;
  const pose = connected && localization?.valid === true && localization?.field_to_camera;
  $('pose-state').textContent = !connected ? 'Awaiting data' : (pose ? (localization.field_to_robot ? 'Robot field pose available' : 'Camera field pose · mount not measured') : (status.detections?.some(d => d.pose_valid && d.camera_to_target) ? 'Camera-relative pose available' : 'No field pose'));
  $('pose-note').textContent = status?.error || status?.localization_error || localization?.invalid_reason?.replaceAll('_', ' ') || '3D uses matched calibration; multi-tag uses known field coordinates and the measured camera mounting transform.';
  $('frame-label').textContent = connected ? `${fpsText} processed FPS · ${status.mode || 'unknown'} detection` : (status?.error || 'No frame received');
  if (isObject) {
    const objects = connected ? status?.objects : null;
    const selected = objects?.valid === true && objects.selected_target?.valid === true ? objects.selected_target : null;
    const position = selected?.translation_m;
    const hasPosition = Array.isArray(position) && position.length === 3 && position.every(Number.isFinite) && Number.isFinite(selected?.range_xy_m);
    const validCount = objects?.valid === true ? (objects.targets || []).filter(target => target.valid === true).length : 0;
    $('pose-state').textContent = !connected ? 'Awaiting data' : validCount ? `${validCount} robot-relative target${validCount === 1 ? '' : 's'}` : '2D only · no valid range';
    $('pose-note').textContent = status?.error || objects?.invalid_reason?.replaceAll('_', ' ') || (hasPosition
      ? 'Target position is in the robot frame at capture time, without motion compensation.'
      : 'Ranging needs matching intrinsics, a measured camera mount and target plane.');
    $('object-selected').textContent = hasPosition ? `${selected.label || 'Object'} · track ${selected.track_id ?? objects.selected_track_id ?? '—'}` : 'None';
    $('object-position').textContent = hasPosition
      ? `X ${position[0].toFixed(2)} m forward · Y ${position[1].toFixed(2)} m left · range ${selected.range_xy_m.toFixed(2)} m${Number.isFinite(selected.uncertainty?.max_position_std_m) ? ` · uncertainty ${selected.uncertainty.max_position_std_m.toFixed(2)} m` : ''}`
      : 'No current robot-relative target.';
    const detectionLabel = {detect: 'box detection', segment: 'instance segmentation'}[status?.task || status?.mode] || 'object detection';
    $('frame-label').textContent = connected ? `${fpsText} processed FPS · ${detectionLabel}` : (status?.error || 'No frame received');
  }
  $('raw-results').textContent = JSON.stringify(status || {}, null, 2);
  if (!connected) hideFrame();
}
async function pollStatus() {
  try { state.status = await request('/api/status'); $('connection').textContent = 'Connected'; $('connection').className = 'badge live'; updateStatus(); }
  catch (_) { $('connection').textContent = 'Unavailable'; $('connection').className = 'badge error'; state.status = {}; updateStatus(); }
  finally { setTimeout(pollStatus, document.hidden ? 2000 : 500); }
}
async function pollPreview() {
  const pipeline = state.selected;
  if (!document.hidden && pipeline && state.status[pipeline]?.connected && !state.previewBusy) {
    state.previewBusy = true;
    try {
      const response = await fetch(`/frame/${encodeURIComponent(pipeline)}`, {cache: 'no-store'});
      if (!response.ok) throw new Error('No current frame');
      const blob = await response.blob();
      if (pipeline === state.selected && state.status[pipeline]?.connected) {
        if (state.imageURL) URL.revokeObjectURL(state.imageURL);
        state.imageURL = URL.createObjectURL(blob); $('preview-image').src = state.imageURL;
        $('preview-image').hidden = false; $('preview-empty').hidden = true;
      }
    } catch (_) { if (pipeline === state.selected) hideFrame(); }
    finally { state.previewBusy = false; }
  }
  const fps = selectedPipeline()?.preview?.stream_fps || state.config?.dashboard?.stream_fps || 10;
  setTimeout(pollPreview, document.hidden ? 1500 : 1000 / Math.min(30, Math.max(1, fps)));
}
$('host').textContent = location.host;
loadConfiguration().catch(error => notice(error.message, true));
pollStatus(); pollPreview();
