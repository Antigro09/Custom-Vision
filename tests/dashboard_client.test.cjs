// Regression tests for request/reload behavior and status truthfulness. The DOM
// is stubbed: visual layout is verified separately in an actual browser.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const {test} = require('node:test');
const source = fs.readFileSync(path.join(__dirname, '../custom_vision/static/app.js'), 'utf8');

function harness() {
  const elements = new Map();
  function element() {
    return {get value() { return this._value || ''; }, set value(v) { this._value = String(v); }, checked: false, disabled: false, textContent: '', hidden: false,
      className: '', classList: {toggle() {}}, dataset: {}, children: [], addEventListener() {}, removeAttribute() {},
      append(...children) { this.children.push(...children); },
      replaceChildren(...children) { this.children = children; }};
  }
  const document = {hidden: true, getElementById(id) {
    if (!elements.has(id)) elements.set(id, element());
    return elements.get(id);
  }, createElement: element, createTextNode: text => ({textContent: text})};
  const context = vm.createContext({document, location: {host: '127.0.0.1'},
    URL: {revokeObjectURL() {}}, structuredClone, setTimeout() {},
    // Suspend automatic initial fetches; each test supplies its own responses.
    fetch: () => new Promise(() => {})});
  vm.runInContext(source, context);
  vm.runInContext(`state.config = {pipelines: [{name:'front_tags', type:'apriltag', settings:{mode:'3d', backend:'native', detector_device:'cpu'}}]}; state.selected='front_tags'; state.token='old-token';`, context);
  return {context, elements, run: code => vm.runInContext(code, context)};
}
function response(status, body) { return {status, ok: status >= 200 && status < 300, json: async () => body}; }

test('second save refreshes a rotated token once and preserves edited form/config', async () => {
  const h = harness(), calls = [];
  h.run(`$('tag-size').value='0.1732'; state.config.pendingEdit='keep-me';`);
  h.context.fetch = async (url, options) => {
    calls.push({url, options});
    if (calls.length === 1) return response(403, {error: 'Missing or invalid setup token; reload the page'});
    if (calls.length === 2) return response(200, {csrf_token: 'new-token', config: {unrelated: true}});
    return response(200, {saved: true});
  };
  const result = await h.run(`request('/api/config', {pipelines:[{name:'front_tags'}]})`);
  assert.equal(result.saved, true);
  assert.deepEqual(calls.map(c => c.url), ['/api/config', '/api/config', '/api/config']);
  assert.equal(calls[0].options.headers['X-Custom-Vision-CSRF'], 'old-token');
  assert.equal(calls[2].options.headers['X-Custom-Vision-CSRF'], 'new-token');
  assert.equal(calls[0].options.body, calls[2].options.body);
  assert.equal(h.run('state.config.pendingEdit'), 'keep-me');
  assert.equal(h.elements.get('tag-size').value, '0.1732');
});

test('non-token rejection never replays a write', async () => {
  const h = harness(); let requests = 0;
  h.context.fetch = async () => { requests++; return response(403, {error: 'Cross-origin writes are not allowed'}); };
  await assert.rejects(h.run(`request('/api/config', {})`), /Cross-origin/);
  assert.equal(requests, 1);
});

test('token retry is bounded and uncertain failures never replay a write', async () => {
  const h = harness(); let requests = 0;
  h.context.fetch = async () => {
    requests++;
    return requests === 2 ? response(200, {csrf_token: 'new-token'}) : response(403, {error: 'Missing or invalid setup token; reload the page'});
  };
  await assert.rejects(h.run(`request('/api/config', {})`), /setup token/);
  assert.equal(requests, 3);
  requests = 0;
  h.context.fetch = async () => { requests++; throw new Error('Network disconnected'); };
  await assert.rejects(h.run(`request('/api/config', {})`), /Network disconnected/);
  assert.equal(requests, 1);
});

test('status identifies synthetic input and the actual mode/device instead of saved settings', () => {
  const h = harness();
  h.run(`state.status.front_tags={connected:true, frame_id:7, mode:'2d', backend:'native', detector_device:'cuda', input_kind:'synthetic', detections:[], latency_ms:12.3}; updateStatus();`);
  assert.equal(h.elements.get('preview-label').textContent, 'Synthetic preview');
  assert.equal(h.elements.get('backend-label').textContent, 'CUDA / GPU');
  assert.equal(h.elements.get('frame-label').textContent, 'Frame 7 · 2d detection');
  h.run(`state.status.front_tags.detector_device='cpu'; state.status.front_tags.input_kind='camera'; updateStatus();`);
  assert.equal(h.elements.get('backend-label').textContent, 'C++ / CPU');
  assert.equal(h.elements.get('preview-label').textContent, 'Live');
});

test('pupil selection disables CUDA and native selection can save CUDA independently of preprocess', () => {
  const h = harness();
  h.run(`$('tag-backend').value='pupil'; $('tag-device').value='cuda'; updateDetectorDevice();`);
  assert.equal(h.elements.get('tag-device').disabled, true);
  assert.equal(h.elements.get('tag-device').value, 'cpu');
  h.run(`$('tag-backend').value='native'; updateDetectorDevice(); $('tag-device').value='cuda';
    state.config.pipelines[0].settings.preprocess='cpu';
    state.modes=[{width:1280,height:800,fps:120,fourcc:'MJPG'}];
    $('capture-mode').value='0'; $('camera-source').value='0';
    $('camera-backend').value='v4l2'; $('tag-mode').value='2d';`);
  assert.equal(h.elements.get('tag-device').disabled, false);
  const result = h.run('collectSettings()');
  assert.equal(result.pipelines[0].settings.detector_device, 'cuda');
  assert.equal(result.pipelines[0].settings.preprocess, 'cpu');
  assert.equal(result.pipelines[0].robot_to_camera, null);
});

test('object settings round trip neural model fields and explicit unmeasured geometry', () => {
  const h = harness();
  h.run(`state.config.pipelines.push({name:'objects',type:'object',camera:{source:1,width:1280,height:800,fps:60,fourcc:'MJPG'},settings:{backend:'tensorrt',task:'segment',output_format:'yolo26_end2end',model_path:'/models/piece.engine',labels:['piece','other'],input_size:[640,384],confidence_threshold:.42,max_detections:20,iou_threshold:.5},geometry:{target_height_m:null,anchor:'mask_centroid',max_range_m:8,max_position_std_m:.3,intake_offset_m:[.4,-.1],approach_standoff_m:.2}});
    state.selected='objects'; populatePipeline();
    $('capture-mode').value='0'; $('object-labels').value='piece, other\\n third';`);
  assert.equal(h.elements.get('apriltag-panel').hidden, true);
  assert.equal(h.elements.get('object-panel').hidden, false);
  assert.equal(h.elements.get('field-settings').hidden, true);
  assert.equal(h.elements.get('object-target-height').value, '');
  const p = h.run('collectSettings().pipelines[1]');
  assert.equal(p.settings.task, 'segment');
  assert.equal(p.settings.output_format, 'yolo26_end2end');
  assert.equal(p.settings.model_path, '/models/piece.engine');
  assert.deepEqual(Array.from(p.settings.input_size), [640, 384]);
  assert.deepEqual(Array.from(p.settings.labels), ['piece', 'other', 'third']);
  assert.equal(p.settings.iou_threshold, .5);
  assert.equal(p.settings.confidence_threshold, .42);
  assert.equal(p.geometry.target_height_m, null);
  assert.equal(p.geometry.anchor, 'mask_centroid');
  assert.deepEqual(Array.from(p.geometry.intake_offset_m), [.4, -.1]);
  assert.equal(p.robot_to_camera, null);
  h.run(`$('object-target-height').value='0';`);
  assert.equal(h.run('collectSettings().pipelines[1].geometry.target_height_m'), 0);
  h.run(`$('object-target-height').value='-0.2';`);
  assert.equal(h.run('collectSettings().pipelines[1].geometry.target_height_m'), -.2);
});

test('baseline settings hide neural requirements and switching back preserves AprilTag controls', () => {
  const h = harness();
  h.run(`state.config.pipelines.push({name:'objects',type:'object',settings:{backend:'hsv',hsv_lower:[0,70,80]}}); state.selected='objects'; populatePipeline();`);
  assert.equal(h.elements.get('object-model').disabled, true);
  assert.equal(h.elements.get('object-neural-settings').hidden, true);
  assert.match(h.elements.get('object-backend-note').textContent, /not a trained neural detector/);
  h.run(`state.selected='front_tags'; populatePipeline();`);
  assert.equal(h.elements.get('apriltag-panel').hidden, false);
  assert.equal(h.elements.get('object-panel').hidden, true);
  assert.equal(h.elements.get('object-target-height').disabled, true);
  assert.equal(h.elements.get('field-settings').hidden, false);
  assert.equal(h.elements.get('tag-size').disabled, false);
  assert.equal(h.elements.get('tag-device').disabled, false);
});

test('object status shows current robot coordinates and clears selection on invalidation or disconnect', () => {
  const h = harness();
  h.run(`state.config.pipelines[0].type='object';
    const target={valid:true,track_id:7,label:'piece',translation_m:[2,-.3,.1],range_xy_m:2.02,uncertainty:{max_position_std_m:.08}};
    state.status.front_tags={connected:true,backend:'tensorrt',task:'segment',frame_id:12,detections:[{}],objects:{valid:true,targets:[target],selected_track_id:7,selected_target:target}};updateStatus();`);
  assert.equal(h.elements.get('backend-label').textContent, 'TensorRT / GPU');
  assert.equal(h.elements.get('target-count-label').textContent, 'Visible objects');
  assert.equal(h.elements.get('object-selected').textContent, 'piece · track 7');
  assert.match(h.elements.get('object-position').textContent, /X 2.00 m forward · Y -0.30 m left · range 2.02 m/);
  assert.equal(h.elements.get('pose-note').textContent, 'Target position is in the robot frame at capture time, without motion compensation.');
  assert.equal(h.elements.get('frame-label').textContent, 'Frame 12 · instance segmentation');
  h.run(`state.status.front_tags.task='detect';updateStatus();`);
  assert.equal(h.elements.get('frame-label').textContent, 'Frame 12 · box detection');
  h.run(`state.status.front_tags.objects.valid=false; state.status.front_tags.objects.invalid_reason='target_plane_not_measured';updateStatus();`);
  assert.equal(h.elements.get('object-selected').textContent, 'None');
  assert.equal(h.elements.get('pose-state').textContent, '2D only · no valid range');
  assert.equal(h.elements.get('pose-note').textContent, 'target plane not measured');
  h.run(`state.status.front_tags.objects.valid=true;state.status.front_tags.connected=false;updateStatus();`);
  assert.equal(h.elements.get('object-selected').textContent, 'None');
  assert.equal(h.elements.get('pose-state').textContent, 'Awaiting data');
});
