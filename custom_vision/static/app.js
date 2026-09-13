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
}
$('tag-backend').onchange = updateDetectorDevice;
function populatePipeline() {
  const pipeline = selectedPipeline(); if (!pipeline) return;
  const camera = pipeline.camera || {}, tags = pipeline.settings || {}, mount = pipeline.robot_to_camera || {};
  const preview = {...(state.config.dashboard || {}), ...(pipeline.preview || {})};
  $('pipeline-title').textContent = pipeline.name;
  $('pipeline-description').textContent = pipeline.type === 'apriltag' ? 'AprilTag aiming and calibrated robot localization' : 'Object pipeline · existing backend configuration';
  $('pipeline-enabled').checked = pipeline.enabled !== false;
  value('camera-source', camera.source); value('camera-backend', camera.backend || 'auto');
  const device = deviceForSource(camera.source);
  $('device-select').replaceChildren(); option($('device-select'), 'Use configured source', '');
  state.devices.forEach(d => option($('device-select'), `${d.source} · ${d.name}`, d.source, device === d));
  populateModes(device, camera); populateControls(device, camera.controls || {});
  value('tag-mode', tags.mode || '3d'); value('tag-backend', tags.backend || 'native');
  value('tag-device', tags.detector_device || 'cpu'); updateDetectorDevice();
  value('tag-size', tags.tag_size_m ?? .1651); value('tag-threads', tags.threads ?? 2);
  value('tag-decimate', tags.quad_decimate ?? 1); value('tag-margin', tags.min_decision_margin ?? 20);
  $('multitag').checked = tags.multitag !== false;
  for (const id of ['tag-mode','tag-backend','tag-size','tag-threads','tag-decimate','tag-margin','multitag']) $(id).disabled = pipeline.type !== 'apriltag';
  $('mount-measured').checked = pipeline.robot_to_camera != null; updateMountInputs();
  const translation = mount.translation_m || [0, 0, 0], rotation = mount.rotation_rpy_deg || [0, 0, 0];
  ['mount-x','mount-y','mount-z'].forEach((id, i) => value(id, translation[i]));
  ['mount-roll','mount-pitch','mount-yaw'].forEach((id, i) => value(id, rotation[i]));
  value('preview-rotation', preview.rotation_deg ?? 0); value('preview-fps', preview.stream_fps ?? 10);
  value('preview-width', preview.stream_width ?? 640); value('preview-quality', preview.jpeg_quality ?? 70);
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
  if (p.type === 'apriltag') p.settings = {...p.settings, mode: $('tag-mode').value, backend: $('tag-backend').value, detector_device: $('tag-device').value || 'cpu', tag_size_m: n('tag-size'), threads: n('tag-threads'), quad_decimate: n('tag-decimate'), min_decision_margin: n('tag-margin'), multitag: $('multitag').checked};
  p.robot_to_camera = $('mount-measured').checked ? {translation_m: ['mount-x','mount-y','mount-z'].map(n), rotation_rpy_deg: ['mount-roll','mount-pitch','mount-yaw'].map(n)} : null;
  p.preview = {...p.preview, rotation_deg: n('preview-rotation'), stream_fps: n('preview-fps'), stream_width: n('preview-width'), jpeg_quality: n('preview-quality')};
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
  $('preview-label').textContent = connected ? (status.input_kind === 'synthetic' ? 'Synthetic preview' : 'Live') : (selectedPipeline()?.enabled === false ? 'Disabled' : 'Waiting for camera');
  $('preview-label').className = connected ? 'badge live' : 'badge';
  const backend = status?.backend;
  $('backend-label').textContent = backend === 'native' ? (status.detector_device === 'cuda' ? 'CUDA / GPU' : 'C++ / CPU') : (backend || '—');
  const latency = status?.latency_ms;
  $('latency').replaceChildren(document.createTextNode(Number.isFinite(latency) && connected ? latency.toFixed(1) : '—'));
  const unit = document.createElement('small'); unit.textContent = ' ms'; $('latency').append(unit);
  $('target-count').textContent = connected ? status.detections?.length || 0 : 0;
  $('frame-id').textContent = connected ? status.frame_id ?? '—' : '—';
  const localization = status?.localization || status?.field_pose || status?.robot_pose;
  const pose = connected && localization?.valid === true && localization?.field_to_camera;
  $('pose-state').textContent = !connected ? 'Awaiting data' : (pose ? (localization.field_to_robot ? 'Robot field pose available' : 'Camera field pose · mount not measured') : (status.detections?.some(d => d.pose_valid && d.camera_to_target) ? 'Camera-relative pose available' : 'No field pose'));
  $('pose-note').textContent = status?.error || status?.localization_error || localization?.invalid_reason?.replaceAll('_', ' ') || '3D uses matched calibration; multi-tag uses known field coordinates and the measured camera mounting transform.';
  $('frame-label').textContent = connected ? `Frame ${status.frame_id ?? '—'} · ${status.mode || 'unknown'} detection` : (status?.error || 'No frame received');
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
