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
    return {value: '', checked: false, disabled: false, textContent: '', hidden: false,
      className: '', dataset: {}, children: [], addEventListener() {}, removeAttribute() {},
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
