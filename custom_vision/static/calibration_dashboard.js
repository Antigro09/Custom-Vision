/* Calibration workspace. Imports and CPU solves do not open a camera.
 * Guided capture requires explicit source ownership; activation remains explicit.
 */
(function(root, factory) {
  'use strict';
  const api = factory(root);
  if (typeof module === 'object' && module.exports) module.exports = api;
  else {
    root.CVCalibrationDashboard = api;
    const start = () => {const el = document.getElementById('calibration-workspace'); if (el) api.mount(el);};
    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start, {once:true}); else start();
  }
})(typeof globalThis !== 'undefined' ? globalThis : this, function(root) {
  'use strict';
  const terminal = new Set(['completed','succeeded','failed','canceled','cancelled','unavailable','timed_out']);
  const active = job => !!job && !terminal.has(job.status);
  const finite = value => typeof value === 'number' && Number.isFinite(value);
  const display = (value, digits=2) => finite(value) ? value.toFixed(digits) : 'Not computed';
  const bounded = (value, label, min, max, integer=false) => {
    if (!finite(value) || value < min || value > max || (integer && !Number.isInteger(value))) throw new Error(`${label} must be ${integer?'an integer':'a finite number'} between ${min} and ${max}.`);
    return value;
  };

  function millimetersToMeters(value) {
    if(value === '' || value === null || value === undefined)return null;
    const mm=Number(value);if(!finite(mm))throw new Error('Measured square spacing must be a finite number of millimeters.');
    return mm/1000;
  }
  function metersToMillimeters(value) {return finite(value)?value*1000:null;}
  function captureSpecification(values,source) {
    if(!source?.source_id||!source.mode)throw new Error('Select a source with an explicit configured camera mode.');
    const mode=source.mode,camera=source.camera||{},empty=value=>value === '' || value === null || value === undefined;
    const canonical=value=>Array.isArray(value)?value.map(canonical):value&&typeof value==='object'?Object.fromEntries(Object.keys(value).sort().map(key=>[key,canonical(value[key])])):value;
    const equal=(a,b)=>JSON.stringify(canonical(a))===JSON.stringify(canonical(b));
    const declared={...values,input_kind:'capture',source_id:source.source_id,user_label:values.user_label||camera.user_label||source.label};
    const physical=String(values.physical_id||'').trim(),sourcePhysical=camera.physical_id||null;
    if(physical&&physical!==sourcePhysical)throw new Error('Declared physical camera ID conflicts with the explicitly selected source.');
    declared.physical_id=sourcePhysical||'';
    for(const key of ['width','height'])if(empty(declared[key]))declared[key]=mode[key];
    const cropKeys=['crop_x','crop_y','crop_width','crop_height'];
    if(cropKeys.every(key=>empty(declared[key])))for(const [key,name]of cropKeys.map((key,i)=>[key,['x','y','width','height'][i]]))declared[key]=mode.crop?.[name]??'';
    if(['binning_x','binning_y'].every(key=>empty(declared[key]))){declared.binning_x=mode.binning?.[0]??'';declared.binning_y=mode.binning?.[1]??'';}
    const sourceFocus=mode.focus||{kind:'unknown',value:null,locked:false};
    if((!declared.focus_kind||declared.focus_kind==='unknown')&&empty(declared.focus_value)&&declared.focus_locked!==true){declared.focus_kind=sourceFocus.kind;declared.focus_value=sourceFocus.value??'';declared.focus_locked=sourceFocus.locked===true;}
    const spec=normalizeSpec(declared,[]);
    const sourceMode={width:mode.width,height:mode.height,crop:mode.crop??null,binning:mode.binning??null,focus:{kind:sourceFocus.kind,value:sourceFocus.value??null,locked:sourceFocus.locked===true}};
    if(!equal(spec.mode,sourceMode))throw new Error('Declared resolution, crop, binning or focus conflicts with the selected source mode. Clear the conflicting declaration or select the matching source.');
    return spec;
  }

  function solverPreferencesForSession(inputKind,current) {return inputKind==='capture'?{...current,solver:'mrcal',min_views:40,no_spline:false}:{...current};}
  async function openSessionAfterStop(selectedId,stop,load) {if(!selectedId)throw new Error('Choose a saved session.');await stop();return load(selectedId);}
  function canSolveSession({session,solver,solvers,working=false,capturing=false,reviewed=false}) {return !working&&!capturing&&session?.status==='selected'&&(session.summary?.accepted_views||0)>0&&reviewed===true&&solvers?.[solver]?.available===true;}
  function selectionPresentation(inputKind,capturing) {return {show_selection:inputKind!=='capture',capture_note:inputKind==='capture'?(capturing?'Saved snapshots are already selected. Stop preview, then review these observations.':'Saved snapshots are already selected. Review the retained observations before solving.'):''};}

  function createJobMonitor({request,onUpdate=()=>{},onFinished=()=>{},onError=()=>{},isHidden=()=>false,
    setTimer=root.setTimeout.bind(root),clearTimer=root.clearTimeout.bind(root)}) {
    let generation=0, current=null, timer=null, outstanding=false, stopped=false;
    function schedule(delay) {if (timer!==null)clearTimer(timer);timer=setTimer(poll,delay);}
    async function poll() {
      timer=null;if(stopped||!current||!active(current))return;
      if(outstanding){schedule(1000);return;}
      const version=generation,id=current.job_id;outstanding=true;
      try {
        const job=await request(`/api/calibration/jobs/${encodeURIComponent(id)}`);
        if(stopped||version!==generation)return;
        current=job;onUpdate(job);
        if(!active(job)){current=null;await onFinished(job);}
      } catch(error) {if(!stopped&&version===generation)onError(error);}
      finally {outstanding=false;if(!stopped&&version===generation&&current&&active(current))schedule(isHidden()?5000:1000);}
    }
    return {start(job){generation++;current=job;stopped=false;if(timer!==null)clearTimer(timer);timer=null;
      onUpdate(job);if(active(job))schedule(isHidden()?5000:1000);else{current=null;Promise.resolve(onFinished(job)).catch(onError);}},
      stop(){generation++;stopped=true;current=null;if(timer!==null)clearTimer(timer);timer=null;},
      current:()=>current, isPolling:()=>outstanding};
  }

  function normalizeSpec(values, assetIds, timestampsId=null) {
    const maybe = value => value === '' || value === null || value === undefined ? null : Number(value);
    const width=maybe(values.width),height=maybe(values.height);
    if((width===null)!==(height===null))throw new Error('Declare both width and height, or leave both unknown.');
    if(width!==null){bounded(width,'Declared width',1,8192,true);bounded(height,'Declared height',1,8192,true);}
    const cropValues=['crop_x','crop_y','crop_width','crop_height'].map(key=>maybe(values[key]));
    let crop=null;
    if(cropValues.some(v=>v!==null)) {
      if(cropValues.some(v=>v===null))throw new Error('Crop declaration needs X, Y, width and height.');
      cropValues.forEach((v,i)=>bounded(v,'Crop '+['X','Y','width','height'][i],i<2?0:1,8192,true));
      crop={x:cropValues[0],y:cropValues[1],width:cropValues[2],height:cropValues[3]};
    }
    const bx=maybe(values.binning_x),by=maybe(values.binning_y);
    if((bx===null)!==(by===null))throw new Error('Binning declaration needs both X and Y factors.');
    if(bx!==null){bounded(bx,'X binning',1,16,true);bounded(by,'Y binning',1,16,true);}
    const focusKind=values.focus_kind||'unknown',focusValue=maybe(values.focus_value);
    if(!['fixed','manual','unknown'].includes(focusKind))throw new Error('Choose an explicit focus declaration.');
    if(focusValue!==null)bounded(focusValue,'Focus value',-1000000,1000000);
    if(focusKind!=='manual'&&focusValue!==null)throw new Error('A numeric focus value applies only to manual focus.');
    if(!['images','video','capture'].includes(values.input_kind))throw new Error('Choose image, recorded video or explicit capture input.');
    if(values.input_kind==='capture'&&(!values.source_id||typeof values.source_id!=='string'))throw new Error('Explicit capture requires a selected source.');
    if(!Array.isArray(assetIds)||(values.input_kind==='capture'?assetIds.length!==0:!assetIds.length)||assetIds.length>100||(values.input_kind==='video'&&assetIds.length!==1))throw new Error('Import 1–100 images, or exactly one video.');
    const label=String(values.user_label||'').trim(),physical=String(values.physical_id||'').trim();
    if(!label||label.length>128||physical.length>128)throw new Error('Provide a camera label (maximum 128 characters).');
    if(!['manual','offline_metadata'].includes(values.identity_source||'manual'))throw new Error('Choose manual identity or provided offline metadata.');
    const square=Number(values.square_size_m);if(!finite(square)||square<=0||square>=1)throw new Error('Measured square size must be between 0 and 1 meter.');
    const history=String(values.capture_history||'').trim();if(history.length>2048)throw new Error('Recording history must be at most 2048 characters.');
    return {board:{cols:bounded(Number(values.cols),'Board inner-corner columns',3,40,true),rows:bounded(Number(values.rows),'Board inner-corner rows',3,40,true),
        square_size_m:square},
      camera:{physical_id:physical||null,user_label:label,identity_source:values.identity_source||'manual',capture_history:history||null},
      mode:{width,height,crop,binning:bx===null?null:[bx,by],focus:{kind:focusKind,value:focusValue,locked:values.focus_locked===true}},
      input:{kind:values.input_kind,asset_ids:assetIds.slice(),timestamps_asset_id:values.input_kind==='capture'?null:timestampsId,...(values.input_kind==='capture'?{source_id:values.source_id}:{})},
      selection:{max_views:bounded(Number(values.max_views),'Maximum selected views',1,180,true),interval_s:bounded(Number(values.interval_s),'Minimum interval',0.001,60),
        novelty:bounded(Number(values.novelty),'Novelty threshold',0,1),max_sharpness_px:bounded(Number(values.max_sharpness_px),'Edge-transition limit',0.01,100),
        min_contrast:bounded(Number(values.min_contrast),'Minimum board contrast',0,255)}};
  }

  async function fileBase64(file) {
    const bytes=new Uint8Array(await file.arrayBuffer());let text='';
    for(let i=0;i<bytes.length;i+=16384)text+=String.fromCharCode(...bytes.subarray(i,i+16384));
    return root.btoa(text);
  }

  function mount(container, options={}) {
    if(container.__calibrationDashboard)return container.__calibrationDashboard;
    const doc=container.ownerDocument||document;
    container.classList.add('calibration-workspace');
    container.innerHTML=`
      <div class="cal-heading"><div><div class="eyebrow">CHESSBOARD LENS LAB / EXPLICIT SOURCES</div><h2>Calibration workbench</h2><p>Preview a measured chessboard or import a recording. Review observations, solve intrinsics, then review a candidate.</p></div><span class="badge" id="cal-connection">Loading capabilities</span></div>
      <ol class="cal-stages" aria-label="Calibration stages"><li data-stage="setup">01 <span>Setup</span></li><li data-stage="select">02 <span>Select</span></li><li data-stage="review">03 <span>Review</span></li><li data-stage="solve">04 <span>Solve</span></li><li data-stage="export">05 <span>Export</span></li></ol>
      <div id="cal-message" class="cal-message" role="status" aria-live="polite" hidden></div>
      <div class="cal-toolbar"><label>Saved local session<select id="cal-sessions"><option value="">New session</option></select></label><button id="cal-resume" class="cal-secondary">Open session</button><button id="cal-refresh" class="cal-secondary">Refresh sessions</button><button id="cal-new" class="cal-secondary">New setup</button><span id="cal-session-id" class="subtle">No session created</span></div>
      <div class="cal-main-grid"><section class="cal-card cal-setup"><div class="cal-card-title"><h3>Board &amp; camera declaration</h3><span>01 / SETUP</span></div>
      <form id="cal-setup-form"><div class="cal-three"><label>Inner-corner columns<input id="cal-cols" type="number" min="3" max="40" step="1" placeholder="7" required></label><label>Inner-corner rows<input id="cal-rows" type="number" min="3" max="40" step="1" placeholder="5" required></label><label>Measured square spacing (mm)<input id="cal-square" type="number" min="0.001" max="999.999" step="0.001" placeholder="30.000" required></label></div><p class="cal-hint">Count intersections inside the board, not squares. Measure the printed square spacing in millimeters; the session stores meters; preview defaults do not supply measurements.</p>
      <label>Camera / lens label<input id="cal-camera-label" maxlength="128" placeholder="Example: front OV9281 or OV2311 · lens identifier" required></label><div class="cal-two"><label>Physical camera ID, if known<input id="cal-physical-id" maxlength="128" placeholder="Serial or your manual identifier"></label><label>Identity declaration<select id="cal-identity-source"><option value="manual">Manual declaration</option><option value="offline_metadata">Provided offline metadata</option></select></label></div>
      <label>Recording processing history (optional)<textarea id="cal-capture-history" maxlength="2048" rows="2" placeholder="OBS/export resolution, scaling, crop, codec or re-encoding history; unknown if not supplied"></textarea></label><details class="cal-mode-details"><summary>Resolution, crop, binning &amp; focus metadata</summary><p class="cal-hint">Optional declarations. The worker reports actual decoded dimensions. Model names do not select a mode or establish sensor identity.</p><div class="cal-two"><label>Declared width (px)<input id="cal-width" type="number" min="1" step="1" placeholder="Unknown"></label><label>Declared height (px)<input id="cal-height" type="number" min="1" step="1" placeholder="Unknown"></label></div><div class="cal-four"><label>Crop X<input id="cal-crop-x" type="number" min="0" step="1" placeholder="None"></label><label>Crop Y<input id="cal-crop-y" type="number" min="0" step="1" placeholder="None"></label><label>Crop width<input id="cal-crop-width" type="number" min="1" step="1" placeholder="None"></label><label>Crop height<input id="cal-crop-height" type="number" min="1" step="1" placeholder="None"></label></div><div class="cal-two"><label>X binning<input id="cal-binning-x" type="number" min="1" step="1" placeholder="Unknown"></label><label>Y binning<input id="cal-binning-y" type="number" min="1" step="1" placeholder="Unknown"></label><label>Focus kind<select id="cal-focus-kind"><option value="unknown">Unknown</option><option value="fixed">Fixed focus (declared)</option><option value="manual">Manual focus (declared)</option></select></label><label>Manual focus value<input id="cal-focus-value" type="number" step="any" placeholder="Unknown" disabled></label></div><label class="cal-check"><input id="cal-focus-locked" type="checkbox"> Focus was held fixed during this recording</label></details>
      <div id="cal-capture-settings" hidden></div><div class="cal-card-title cal-divider"><h3>Offline source files</h3><span>NO CAMERA ACCESS</span></div><div class="cal-upload-row"><label>Input kind<select id="cal-input-kind"><option value="images">Board images</option><option value="video">Recorded video</option></select></label><label class="cal-file-button">Import source files<input id="cal-files" type="file" accept="image/png,image/jpeg,image/webp" multiple></label><label class="cal-file-button">Timestamp sidecar<input id="cal-timestamps" type="file" accept=".jsonl"></label></div><p id="cal-upload-status" class="cal-hint">No assets imported. Files upload one at a time to this local workspace.</p><ul id="cal-assets" class="cal-assets"></ul>
      <details><summary>Selection thresholds</summary><div class="cal-two"><label>Maximum selected views<input id="cal-max-views" type="number" value="180" min="1" max="180" step="1"></label><label>Minimum interval (s)<input id="cal-interval" type="number" value="0.7" min="0.001" step="0.001"></label><label>Novelty (normalized RMS)<input id="cal-novelty" type="number" value="0.025" min="0" step="0.001"></label><label>Max edge transition (px)<input id="cal-sharpness" type="number" value="3" min="0.01" step="0.01"></label><label>Minimum board contrast<input id="cal-contrast" type="number" value="40" min="0" max="255" step="1"></label></div><p class="cal-hint">Selection heuristics reject duplicates, blur and low contrast. They do not establish calibration accuracy.</p></details>
      <button id="cal-create" class="primary" type="submit">Create local session</button></form></section>
      <section class="cal-card cal-review"><div id="cal-capture-preview" hidden></div><div class="cal-card-title"><h3>Selection &amp; coverage review</h3><span>02–03 / OBSERVATIONS</span></div><div class="cal-review-preview"><img id="cal-preview" alt="Actual selected-board preview" hidden><div id="cal-preview-empty"><span class="cal-target">⌖</span><strong>No selected observations yet</strong><p>Run selection on imported files. Lossless accepted frames remain the solver input.</p></div></div><div class="cal-review-stats"><div><span>Accepted views</span><strong id="cal-accepted">—</strong></div><div><span>Decoded frames</span><strong id="cal-processed">—</strong></div><div><span>Corner-cell coverage</span><strong id="cal-coverage">—</strong></div></div><div class="cal-coverage-area"><div id="cal-coverage-grid" class="cal-coverage-grid" role="img" aria-label="Coverage grid unavailable"></div><p id="cal-coverage-note" class="cal-hint">No coverage grid available. Coverage describes sampling, not physical accuracy.</p></div><p id="cal-resolution" class="cal-hint">Decoded resolution unknown</p><p id="cal-clock" class="cal-hint">Source timestamp method unknown</p><ul id="cal-rejections" class="cal-issues"></ul><button id="cal-select" class="primary" disabled>Select useful board views</button><p id="cal-capture-review-note" class="cal-hint" hidden></p><label class="cal-check cal-reviewed"><input id="cal-review-confirm" type="checkbox" disabled> I reviewed coverage, rejected frames and the camera/board declarations</label><details><summary>Accepted view metrics</summary><div class="cal-table-wrap"><table><thead><tr><th>Frame</th><th>Source time</th><th>Edge width</th><th>Contrast</th></tr></thead><tbody id="cal-views"></tbody></table></div><p class="cal-hint">First 30 accepted views. Smaller edge-transition widths indicate sharper board edges.</p></details></section></div>
      <div id="cal-diagnostics" hidden></div><section class="cal-job-bar"><div><span class="eyebrow">BOUNDED CPU JOB</span><strong id="cal-job-state">Idle</strong><p id="cal-job-progress">No worker job running</p></div><button id="cal-cancel" class="cal-secondary" disabled>Cancel job</button></section>
      <div class="cal-main-grid cal-bottom-grid"><section class="cal-card"><div class="cal-card-title"><h3>Choose a solver</h3><span>04 / SOLVE</span></div><label>Solver<select id="cal-solver"><option value="opencv">OpenCV baseline</option><option value="mrcal" disabled>Native mrcal (checking availability)</option></select></label><p id="cal-solver-note" class="cal-hint">Checking installed capabilities.</p><div class="cal-two"><label>Minimum selected views<input id="cal-min-views" type="number" min="3" max="180" step="1" value="10"></label><label>Initial FOV seed (deg)<input id="cal-fov" type="number" min="20" max="170" value="90"></label><label>Corner detector<select id="cal-corner-detector"><option value="auto">Auto (reported by worker)</option><option value="opencv-sb">OpenCV SB</option><option value="mrgingham">mrgingham (if installed)</option></select></label><label>Held-out RMS heuristic (px)<input id="cal-validation-rms" type="number" min="0.001" value="0.6" step="0.001"></label><label>Worker timeout (s)<input id="cal-timeout" type="number" min="1" max="600" value="120" step="1"></label></div><label class="cal-check"><input id="cal-no-spline" type="checkbox"> Skip spline reference model</label><button id="cal-solve" class="primary" disabled>Solve a calibration candidate</button><p class="cal-hint">The initial FOV is a solver seed, not a measured lens specification. Solving does not activate a candidate.</p><details><summary>Exact local capabilities</summary><pre id="cal-capabilities" class="cal-json">Loading…</pre></details></section>
      <section class="cal-card cal-result"><div class="cal-card-title"><h3>Candidate evidence &amp; export</h3><span>05 / REVIEW REQUIRED</span></div><div id="cal-calibration-states" class="cal-calibration-states" hidden><p id="cal-active-calibration" class="cal-active-calibration">No runtime camera calibration is active in this offline workspace</p><p id="cal-runtime-states" class="cal-hint">Runtime status unavailable</p><div class="cal-state-cards"><div><strong>Default</strong><span id="cal-default-state">Missing / unvalidated</span></div><div><strong>Custom file</strong><span id="cal-custom-state">No file imported</span><label class="cal-file-button">Import runtime JSON<input id="cal-import-candidate" type="file" accept=".json,application/json"></label></div><div><strong>Latest result</strong><span id="cal-latest-state">Not Applied · no result</span></div></div></div><p id="cal-candidate-state" class="cal-result-state">No candidate solved</p><div class="cal-evidence"><div><span>Fit residual</span><strong id="cal-fit">Not computed</strong><small>Training fit; not uncertainty</small></div><div><span>Held-out validation</span><strong id="cal-heldout">Not computed</strong><small>Independent of training residual</small></div><div><span>Projection uncertainty</span><strong id="cal-uncertainty">Not computed</strong><small>Sampling uncertainty; not total accuracy</small></div></div><ul id="cal-candidate-issues" class="cal-issues"></ul><ul id="cal-candidate-warnings" class="cal-warnings"></ul><div id="cal-downloads" class="cal-downloads"></div><div id="cal-result-images" class="cal-result-images"></div><p class="cal-hint">Exported intrinsics are a candidate for the declared camera/lens/mode. Board survey, lens model, focus, motion and physical validation remain separate.</p><div id="cal-review-gate" hidden><label class="cal-check"><input id="cal-candidate-review-confirm" type="checkbox"> I reviewed the native/runtime results and exact camera/mode declarations; this is not physical verification</label><button id="cal-candidate-review" type="button" class="cal-secondary" disabled>Mark eligible result reviewed</button><p id="cal-candidate-review-note" class="cal-hint"></p></div><div id="cal-activation" hidden><label>Runtime pipeline<select id="cal-activation-pipeline"></select></label><label class="cal-check"><input id="cal-activation-confirm" type="checkbox"> Apply this reviewed candidate to the selected runtime pipeline</label><button id="cal-activate" class="cal-secondary" disabled>Activate explicitly</button><div id="cal-activation-history"></div></div><details><summary>Raw solver report</summary><pre id="cal-report" class="cal-json">No report available</pre></details></section></div>`;
    const $=id=>container.querySelector('#'+id);
    const state={capabilities:null,assets:[],timestamps:null,session:null,job:null,candidate:null,busy:false,cancelPending:false,destroyed:false,token:'',sessions:[],runtimeStatus:null,offlineSolverPreferences:null};
    let captureUI=null,diagnostics=null;
    const path=kind=>'/api/calibration/'+kind;
    const say=(message,error=false)=>{$('cal-message').textContent=message;$('cal-message').hidden=!message;$('cal-message').classList.toggle('error',error);};
    const setText=(id,value)=>{$(id).textContent=value;};
    function stage(name){container.querySelectorAll('[data-stage]').forEach(el=>{el.classList.toggle('current',el.dataset.stage===name);el.setAttribute('aria-current',el.dataset.stage===name?'step':'false');});}
    async function defaultRequest(url,data,retry=true){
      const response=await root.fetch(url,data===undefined?{cache:'no-store'}:{method:'POST',keepalive:url.endsWith('/capture/stop'),headers:{'Content-Type':'application/json','X-Custom-Vision-CSRF':state.token},body:JSON.stringify(data)});
      const body=await response.json();
      if(data!==undefined&&retry&&response.status===403&&String(body.error||'').startsWith('Missing or invalid setup token')){
        const setup=await defaultRequest('/api/config');state.token=setup.csrf_token;return defaultRequest(url,data,false);
      }
      if(!response.ok)throw new Error(body.error||`Request failed (${response.status})`);return body;
    }
    const request=options.request||defaultRequest;
    function controls(){
      const capturing=root.CVCalibrationCapture?.isActive(captureUI?.current()),working=state.busy||active(state.job),selected=state.session?.status==='selected'&&(state.session?.summary?.accepted_views||0)>0;
      $('cal-sessions').disabled=working;
      for(const id of ['cal-create','cal-files','cal-timestamps','cal-input-kind','cal-resume','cal-refresh','cal-new'])$(id).disabled=working;
      const selection=selectionPresentation(state.session?.input?.kind,capturing);$('cal-select').hidden=!selection.show_selection;$('cal-select').disabled=working||capturing||!state.session||!selection.show_selection;$('cal-capture-review-note').hidden=!selection.capture_note;$('cal-capture-review-note').textContent=selection.capture_note;
      $('cal-review-confirm').disabled=working||capturing||!selected;
      $('cal-solve').disabled=!canSolveSession({session:state.session,solver:$('cal-solver').value,solvers:state.capabilities?.solvers,working,capturing,reviewed:$('cal-review-confirm').checked});
      $('cal-cancel').disabled=!active(state.job)||state.cancelPending;
      $('cal-activate').disabled=working||!reviewableCandidate(state.candidate)||!(state.candidate?.reviewed||state.candidate?.review_status?.reviewed)||state.candidate?.review_status?.activation_provenance_available!==true||!$('cal-activation-confirm').checked||!$('cal-activation-pipeline').value;
      $('cal-candidate-review').disabled=working||!reviewableCandidate(state.candidate)||!$('cal-candidate-review-confirm').checked;
      $('cal-import-candidate').disabled=working;
      const editableCapture=state.session?.input?.kind==='capture';
      for(const el of $('cal-setup-form').querySelectorAll('input,select,button'))if(!el.closest('#cal-capture-settings'))el.disabled=working||!!state.session&&!editableCapture;
      $('cal-create').disabled=working||!!state.session;
      $('cal-focus-value').disabled=working||!!state.session&&!editableCapture||$('cal-focus-kind').value!=='manual';
      captureUI?.sync();
    }
    async function action(fn){if(state.busy)return;state.busy=true;say('');controls();try{await fn();}catch(error){say(error.message,true);}finally{state.busy=false;controls();}}
    function list(rootEl,items){rootEl.replaceChildren();for(const item of (Array.isArray(items)?items:[]).slice(0,100)){const li=doc.createElement('li');li.textContent=typeof item==='string'?item:JSON.stringify(item);rootEl.append(li);}}
    function assetList(){
      list($('cal-assets'),state.assets.map(a=>`${a.file_name} · ${(a.size_bytes/1048576).toFixed(2)} MiB · ${a.sha256?.slice(0,12)||'hash unavailable'}`));
      setText('cal-upload-status',`${state.assets.length} source asset${state.assets.length===1?'':'s'} imported${state.timestamps?' · timestamp sidecar imported':''}. Uploads stay local.`);
    }
    function clearCandidate(){
      state.candidate=null;diagnostics?.clearCandidate();setText('cal-candidate-state','No candidate solved');setText('cal-latest-state','Not Applied · no result');$('cal-candidate-review-confirm').checked=false;
      for(const id of ['cal-fit','cal-heldout','cal-uncertainty'])setText(id,'Not computed');
      for(const id of ['cal-downloads','cal-candidate-issues','cal-candidate-warnings','cal-result-images'])$(id).replaceChildren();
      setText('cal-report','No report available');$('cal-activation-confirm').checked=false;
    }
    function clearReview(){
      for(const id of ['cal-accepted','cal-processed','cal-coverage'])setText(id,'—');
      for(const id of ['cal-coverage-grid','cal-views','cal-rejections'])$(id).replaceChildren();
      $('cal-preview').hidden=true;$('cal-preview').removeAttribute('src');$('cal-preview-empty').hidden=false;
      setText('cal-resolution','Decoded resolution unknown');setText('cal-clock','Source timestamp method unknown');
      setText('cal-coverage-note','No coverage grid available. Coverage describes sampling, not physical accuracy.');
    }
    async function upload(files,kind){
      if(!state.capabilities)throw new Error('Wait for local capabilities before importing files.');
      const limits=state.capabilities.limits||{},inputKind=$('cal-input-kind').value;
      const actualKind=kind==='timestamps'?'timestamps':inputKind==='video'?'video':'image';
      if(actualKind==='video'&&files.length!==1)throw new Error('Import exactly one video file.');
      if(actualKind==='image'&&state.assets.length+files.length>(limits.max_images||100))throw new Error('Image input exceeds the local session image limit.');
      for(const [i,file]of files.entries()){
        const limit=limits[actualKind+'_bytes']||({image:12,video:32,timestamps:2}[actualKind]*1048576);
        if(!file.size||file.size>limit)throw new Error(`${file.name}: file must be nonempty and no larger than ${(limit/1048576).toFixed(0)} MiB.`);
        setText('cal-upload-status',`Importing ${i+1}/${files.length}: ${file.name} (one file at a time)`);
        const result=await request(path('assets'),{file_name:file.name,data_base64:await fileBase64(file),kind:actualKind});
        if(actualKind==='timestamps')state.timestamps=result;else if(actualKind==='video')state.assets=[result];else state.assets.push(result);
      }
      assetList();captureUI?.refresh();
    }
    function formValues(){
      const keys={cols:'cols',rows:'rows',square_size_m:'square',user_label:'camera-label',physical_id:'physical-id',identity_source:'identity-source',
        width:'width',height:'height',crop_x:'crop-x',crop_y:'crop-y',crop_width:'crop-width',crop_height:'crop-height',binning_x:'binning-x',binning_y:'binning-y',
        focus_kind:'focus-kind',focus_value:'focus-value',input_kind:'input-kind',max_views:'max-views',interval_s:'interval',novelty:'novelty',max_sharpness_px:'sharpness',min_contrast:'contrast'};
      const values={};for(const [key,id]of Object.entries(keys))values[key]=$('cal-'+id).value;values.focus_locked=$('cal-focus-locked').checked;
      values.square_size_m=millimetersToMeters($('cal-square').value);values.capture_history=$('cal-capture-history').value;return values;
    }
    function setForm(session){
      const board=session.board||{},camera=session.camera||{},mode=session.declared_mode||{crop:session.mode?.crop,binning:session.mode?.binning,focus:session.mode?.focus},selection=session.selection||{};
      const values={cols:board.cols,rows:board.rows,square:metersToMillimeters(board.square_size_m),'camera-label':camera.user_label,'physical-id':camera.physical_id,'identity-source':camera.identity_source,
        'capture-history':camera.capture_history,width:mode.width,height:mode.height,'crop-x':mode.crop?.x,'crop-y':mode.crop?.y,'crop-width':mode.crop?.width,'crop-height':mode.crop?.height,
        'binning-x':mode.binning?.[0],'binning-y':mode.binning?.[1],'focus-kind':mode.focus?.kind,'focus-value':mode.focus?.value,
        'input-kind':session.input?.kind==='capture'?'images':session.input?.kind,'max-views':selection.max_views,interval:selection.interval_s,novelty:selection.novelty,sharpness:selection.max_sharpness_px,contrast:selection.min_contrast};
      for(const [id,value]of Object.entries(values))if(value!==undefined)$('cal-'+id).value=value??'';
      $('cal-focus-locked').checked=mode.focus?.locked===true;focusControls();
    }
    function focusControls(){$('cal-focus-value').disabled=$('cal-focus-kind').value!=='manual';if($('cal-focus-value').disabled)$('cal-focus-value').value='';}
    function review(session){
      state.session=session;const summary=session.summary||{};
      setText('cal-session-id',session.session_id);setText('cal-accepted',summary.accepted_views??'—');setText('cal-processed',summary.processed_frames??'—');
      setText('cal-coverage',finite(summary.coverage_fraction)?(summary.coverage_fraction*100).toFixed(0)+'%':'—');
      const grid=summary.coverage_grid,gridEl=$('cal-coverage-grid');gridEl.replaceChildren();
      const hasGrid=Array.isArray(grid)&&grid.length>0&&grid.length<=16&&grid.every(row=>Array.isArray(row)&&row.length===grid[0].length&&row.length>0&&row.length<=32&&row.every(v=>v===0||v===1||typeof v==='boolean'));
      if(hasGrid){gridEl.style.gridTemplateColumns=`repeat(${grid[0].length},1fr)`;for(const row of grid)for(const cell of row){const el=doc.createElement('span');el.className=cell?'covered':'uncovered';gridEl.append(el);}}
      gridEl.setAttribute('aria-label',hasGrid?`Actual ${grid[0].length} by ${grid.length} corner-cell coverage grid`:'Coverage grid unavailable');
      setText('cal-coverage-note',hasGrid?'Covered corner cells from accepted observations. Coverage is a sampling heuristic, not physical accuracy.':'No coverage grid provided by the worker.');
      setText('cal-resolution',session.width&&session.height?`Decoded ${session.width} × ${session.height} px${session.declared_mode?.width?' · declared '+session.declared_mode.width+' × '+session.declared_mode.height:''}`:'Decoded resolution unknown');
      setText('cal-clock','Source timestamp method: '+(summary.timestamp_source||'unknown'));
      list($('cal-rejections'),Object.entries(summary.rejections_by_reason||{}).map(([reason,count])=>`${count} rejected · ${reason}`).concat(session.warnings||[]));
      const preview=summary.preview_artifact;
      $('cal-preview').hidden=!preview;$('cal-preview-empty').hidden=!!preview;
      if(preview)$('cal-preview').src=path(`sessions/${encodeURIComponent(session.session_id)}/artifacts/${encodeURIComponent(preview)}`);else $('cal-preview').removeAttribute('src');
      $('cal-views').replaceChildren();for(const view of (session.views||[]).slice(0,30)){
        const tr=doc.createElement('tr');for(const value of [view.source_frame_id,display(view.source_time_s)+' s',display(view.sharpness_px)+' px',display(view.contrast,1)]){const td=doc.createElement('td');td.textContent=value??'—';tr.append(td);}$('cal-views').append(tr);
      }
      diagnostics?.loadSession(session.session_id);stage(summary.accepted_views>0?'review':'select');controls();
    }
    async function refreshSessions(){
      state.sessions=await request(path('sessions'));$('cal-sessions').replaceChildren();
      const empty=doc.createElement('option');empty.value='';empty.textContent='Choose saved session';$('cal-sessions').append(empty);
      for(const session of state.sessions.slice(0,100)){const option=doc.createElement('option');option.value=session.session_id;option.textContent=`${session.camera?.user_label||session.session_id} · ${session.status||'saved'} · ${session.summary?.accepted_views||0} views`;option.selected=session.session_id===state.session?.session_id;$('cal-sessions').append(option);}
    }
    function updateJob(job){
      state.job=job;setText('cal-job-state',`${job.operation||'Job'} · ${job.status}`);
      const p=job.progress||{};setText('cal-job-progress',`${job.stage||'Worker'} · ${p.processed_frames??0}${p.total_frames!==null&&p.total_frames!==undefined?'/'+p.total_frames:''} frames · ${p.accepted_views??0} accepted${p.last_reason?' · '+p.last_reason:''}`);
      stage(job.operation==='solve'?'solve':'select');controls();
    }
    async function completed(job){
      state.cancelPending=false;state.job=job;
      if(state.session)review(await request(path('sessions/'+encodeURIComponent(state.session.session_id))));
      if(job.result?.candidate_id)await candidate(job.result.candidate_id);
      if(['failed','unavailable','timed_out'].includes(job.status))say(job.error||`Job ${job.status}`,true);
      if(['canceled','cancelled'].includes(job.status))say('Job cancelled. Saved inputs are retained; run selection again to review observations.');
      await refreshSessions();controls();
    }
    const monitor=createJobMonitor({request,onUpdate:updateJob,onFinished:completed,onError:error=>say('Worker status unavailable: '+error.message,true),isHidden:()=>doc.hidden});
    async function candidate(id){
      const item=await request(path('candidates/'+encodeURIComponent(id)));state.candidate=item;const report=item.report||{},model=report.models?.opencv8||{};
      setText('cal-candidate-state',`${item.solver||'Solver'} · ${item.quality_status||report.status||'candidate'} · not physically qualified`);
      const fit=finite(report.training_rms_px)?report.training_rms_px:(finite(model.training_rms_px)?model.training_rms_px:(finite(item.calibration?.rms_error_px)?item.calibration.rms_error_px:model.rms_px));
      setText('cal-fit',finite(fit)?display(fit,3)+' px':'Not computed');
      const holdout=model.holdout||report.holdout,uncertainty=model.uncertainty||report.uncertainty;
      setText('cal-heldout',finite(holdout?.rms_px)?display(holdout.rms_px,3)+' px RMS':'Not computed');
      setText('cal-uncertainty',finite(uncertainty?.p95_px)?display(uncertainty.p95_px,3)+' px p95':'Not computed');
      list($('cal-candidate-issues'),report.issues);list($('cal-candidate-warnings'),report.warnings);
      $('cal-downloads').replaceChildren();for(const artifact of (item.artifacts||[]).slice(0,64)){
        const a=doc.createElement('a');a.href=path(`candidates/${encodeURIComponent(id)}/artifacts/${encodeURIComponent(artifact.name)}`);a.textContent='Download '+artifact.name;a.className='cal-download';a.setAttribute('download','');$('cal-downloads').append(a);
      }
      $('cal-result-images').replaceChildren();
      if(item.solver==='mrcal'&&finite(uncertainty?.p95_px))for(const artifact of (item.artifacts||[]).filter(a=>/uncertainty[^/]*\.png$/i.test(a.name)).slice(0,3)){
        const figure=doc.createElement('figure'),image=doc.createElement('img'),caption=doc.createElement('figcaption');
        image.src=path(`candidates/${encodeURIComponent(id)}/artifacts/${encodeURIComponent(artifact.name)}`);
        image.alt='Provided native sampling projection uncertainty heatmap';caption.textContent=artifact.name+' · sampling uncertainty, not total physical accuracy';
        figure.append(image,caption);$('cal-result-images').append(figure);
      }
      setText('cal-report',JSON.stringify(report,null,2));$('cal-activation-confirm').checked=false;$('cal-candidate-review-confirm').checked=false;
      setText('cal-latest-state',`${item.quality_status||report.status||'unknown'} · ${item.reviewed||item.review_status?.reviewed?'Reviewed':'Not reviewed'} · Not Applied`);
      setText('cal-candidate-review-note',reviewableCandidate(item)?'Eligible software result: review remains separate from physical camera/mount verification.':'This quality status cannot be activated or marked reviewed. Retain artifacts for diagnosis.');
      diagnostics?.loadCandidate(id);stage('export');controls();
    }
    async function runJob(operation){
      const options={min_views:bounded(Number($('cal-min-views').value),'Minimum selected views',$('cal-solver').value==='opencv'?3:10,180,true),fov_deg:bounded(Number($('cal-fov').value),'Initial FOV seed',20,170),
        corner_detector:$('cal-corner-detector').value,no_spline:$('cal-no-spline').checked,max_validation_rms:bounded(Number($('cal-validation-rms').value),'Held-out RMS heuristic',0.001,10),
        timeout_s:bounded(Number($('cal-timeout').value),'Worker timeout',0.1,600)};
      if(operation==='solve'&&!$('cal-review-confirm').checked)throw new Error('Review selected observations before starting a solve.');
      if(operation==='select')$('cal-review-confirm').checked=false;
      clearCandidate();
      const job=await request(path('jobs'),{session_id:state.session.session_id,operation,solver:$('cal-solver').value,options});state.cancelPending=false;monitor.start(job);
    }
    function reviewableCandidate(item){return !!item&&['heuristics_passed_not_hardware_validated','baseline_not_hardware_validated'].includes(item.quality_status||item.report?.status)&&item.review_status?.review_allowed!==false;}
    function solverPreferences(){return {solver:$('cal-solver').value,min_views:Number($('cal-min-views').value),no_spline:$('cal-no-spline').checked};}
    function applySolverPreferences(values){$('cal-solver').value=values.solver;$('cal-min-views').value=String(values.min_views);$('cal-no-spline').checked=values.no_spline===true;}
    function enterCaptureSolver(){if(!state.offlineSolverPreferences)state.offlineSolverPreferences=solverPreferences();const defaults=solverPreferencesForSession('capture',solverPreferences());if(!state.capabilities?.solvers?.mrcal?.available)defaults.solver=$('cal-solver').value;applySolverPreferences(defaults);solverControls();}
    function restoreOfflineSolverPreferences(){if(state.offlineSolverPreferences){applySolverPreferences(state.offlineSolverPreferences);state.offlineSolverPreferences=null;solverControls();}}
    function solverControls(){
      const native=$('cal-solver').value==='mrcal',available=state.capabilities?.solvers?.mrcal;
      setText('cal-solver-note',native?`Native mrcal ${state.capabilities?.solver_versions?.mrcal||state.capabilities?.versions?.mrcal||'version unavailable'}. ${available?.reason||'Held-out checks and sampling uncertainty are reported only when computed.'}`:
        'Explicit OpenCV baseline: training residual only. Held-out validation, projection uncertainty and native models are not computed. This choice never substitutes for a requested native mrcal solve.');
      for(const id of ['cal-fov','cal-corner-detector','cal-validation-rms'])$(id).disabled=!native;
      $('cal-no-spline').disabled=!native||available?.spline!==true;
      $('cal-corner-detector').querySelector('option[value="mrgingham"]').disabled=!state.capabilities?.tools?.mrgingham_available;
      $('cal-min-views').min=native?'10':'3';
      if($('cal-no-spline').disabled)$('cal-no-spline').checked=true;controls();
    }
    async function calibrationStatus(){
      if(!state.capabilities?.live_capture)return;
      try{state.runtimeStatus=await request(path('status'));const value=state.runtimeStatus,pipelines=value.pipelines||[];const validated=pipelines.filter(p=>{const c=p.calibration||p;return c.active===true&&c.state==='custom_active_validated'&&c.physical_verification===true;});
        setText('cal-active-calibration',validated.length?'USING VALIDATED CUSTOM CALIBRATION · '+validated.map(p=>p.name||p.pipeline).join(', '):value.runtime_activation_available?'No active validated custom calibration reported · inspect missing/default/stale/mode-mismatch status':'OFFLINE WORKSPACE · no runtime camera calibration active');
        $('cal-active-calibration').classList.toggle('validated',validated.length>0);setText('cal-default-state',value.default_state||'Missing / unvalidated');
        if(pipelines.length){const notes=pipelines.map(p=>{const c=p.calibration||p;return `${p.name||p.pipeline}: ${c.state||'unvalidated'}${c.reason?' · '+c.reason:''}`;});setText('cal-runtime-states',notes.join(' · '));}else setText('cal-runtime-states','Offline session review only · no runtime camera activation is available');
      }catch(error){setText('cal-active-calibration','Runtime calibration state unavailable · active custom calibration is not asserted');}
    }
    async function activationHistory(){
      if(!state.capabilities?.runtime_activation)return;
      const history=await request(path('activations'));$('cal-activation-history').replaceChildren();
      for(const record of history.slice(0,10)){
        const div=doc.createElement('div'),label=doc.createElement('span'),button=doc.createElement('button');
        label.textContent=`${record.pipeline||'Pipeline'} · ${record.activation_id||record.id||'activation'}`;
        button.textContent='Restore previous calibration';button.className='cal-secondary';
        button.disabled=record.status!=='applied'||record.restored===true;
        button.onclick=()=>action(async()=>{const id=record.activation_id||record.id;await request(path(`activations/${encodeURIComponent(id)}/restore`),{confirmed:true});say('Previous calibration restored by explicit request.');await activationHistory();await calibrationStatus();});
        div.append(label,button);$('cal-activation-history').append(div);
      }
    }
    $('cal-setup-form').onsubmit=event=>{event.preventDefault();action(async()=>{
      const spec=normalizeSpec(formValues(),state.assets.map(asset=>asset.asset_id),state.timestamps?.asset_id||null);
      const session=await request(path('sessions'),spec);clearCandidate();review(session);await refreshSessions();say('Session saved. Run selection before reviewing and solving.');
    });};
    $('cal-files').onchange=event=>{const files=Array.from(event.target.files);event.target.value='';action(async()=>{try{await upload(files,'input');}finally{assetList();}});};
    $('cal-timestamps').onchange=event=>{const files=Array.from(event.target.files);event.target.value='';if(files.length)action(async()=>{try{await upload(files,'timestamps');}finally{assetList();}});};
    $('cal-input-kind').onchange=()=>{state.assets=[];state.timestamps=null;assetList();const video=$('cal-input-kind').value==='video';$('cal-files').multiple=!video;$('cal-files').accept=video?'video/mp4,video/x-msvideo,video/quicktime,.avi,.mkv,.mov,.mp4':'image/png,image/jpeg,image/webp';};
    $('cal-focus-kind').onchange=focusControls;$('cal-review-confirm').onchange=controls;$('cal-solver').onchange=solverControls;
    $('cal-select').onclick=()=>action(()=>runJob('select'));$('cal-solve').onclick=()=>action(()=>runJob('solve'));
    $('cal-cancel').onclick=()=>action(async()=>{if(!active(state.job)||state.cancelPending)return;state.cancelPending=true;controls();try{const job=await request(path(`jobs/${encodeURIComponent(state.job.job_id)}/cancel`),{});updateJob(job);say('Cancellation requested; waiting for the worker to stop.');}catch(error){state.cancelPending=false;throw error;}});
    $('cal-refresh').onclick=()=>action(refreshSessions);
    $('cal-resume').onclick=()=>action(async()=>{const id=$('cal-sessions').value;const session=await openSessionAfterStop(id,()=>captureUI?.stop('Opening another saved session.'),selected=>request(path('sessions/'+encodeURIComponent(selected))));monitor.stop();state.job=null;clearCandidate();if(session.input?.kind==='capture')enterCaptureSolver();else restoreOfflineSolverPreferences();setForm(session);$('cal-review-confirm').checked=false;review(session);
      const candidateId=session.latest_candidate_id||session.candidate_ids?.at(-1);if(candidateId)await candidate(candidateId);
      const jobId=session.latest_job_id;if(jobId){const job=await request(path('jobs/'+encodeURIComponent(jobId)));if(active(job))monitor.start(job);}
    });
    $('cal-new').onclick=()=>{captureUI?.stop('New setup requested.');diagnostics?.loadSession(null);monitor.stop();state.session=null;clearCandidate();clearReview();state.job=null;state.assets=[];state.timestamps=null;$('cal-review-confirm').checked=false;$('cal-setup-form').reset();restoreOfflineSolverPreferences();assetList();setText('cal-session-id','No session created');setText('cal-job-state','Idle');setText('cal-job-progress','No worker job running');stage('setup');controls();};
    $('cal-activation-confirm').onchange=controls;$('cal-activation-pipeline').onchange=controls;
    $('cal-activate').onclick=()=>action(async()=>{if(!$('cal-activation-confirm').checked)throw new Error('Confirm the reviewed candidate and runtime pipeline.');await request(path(`candidates/${encodeURIComponent(state.candidate.candidate_id)}/activate`),{pipeline:$('cal-activation-pipeline').value,confirmed:true});say('Candidate activated by explicit request. Physical verification remains required.');$('cal-activation-confirm').checked=false;await activationHistory();await calibrationStatus();});
    $('cal-candidate-review-confirm').onchange=controls;
    $('cal-candidate-review').onclick=()=>action(async()=>{if(!$('cal-candidate-review-confirm').checked||!reviewableCandidate(state.candidate))throw new Error('This candidate is not eligible for review.');const id=state.candidate.candidate_id;await request(path(`candidates/${encodeURIComponent(id)}/review`),{confirmed:true});await candidate(id);say('Eligible candidate marked reviewed. No runtime calibration was activated.');});
    $('cal-import-candidate').onchange=event=>{const file=event.target.files?.[0];event.target.value='';if(!file)return;action(async()=>{if(file.size>1048576)throw new Error('Runtime calibration JSON must be at most 1 MiB.');let imported;try{imported=JSON.parse(await file.text());}catch(error){throw new Error('Invalid runtime calibration JSON.');}const calibration=imported.calibration||imported;let camera=imported.camera,mode=imported.mode;if(!camera||!mode){const spec=normalizeSpec({...formValues(),input_kind:'capture',source_id:'imported-file'},[]);camera=spec.camera;mode=spec.mode;}const quality=imported.quality_status||calibration.quality_status||calibration.metadata?.quality_status||calibration.metadata?.review?.quality_status;const item=await request(path('candidates/import'),{calibration,camera,mode,quality_status:quality,report:imported.report,confirmed:false});setText('cal-custom-state',`Imported ${file.name} · Not Applied`);await candidate(item.candidate_id);say('Custom file imported for the declared camera/mode. Review and activation remain explicit.');});};
    const api={destroy(){state.destroyed=true;monitor.stop();captureUI?.destroy().catch(()=>{});diagnostics?.destroy();container.__calibrationDashboard=null;},refresh:()=>action(refreshSessions)};
    container.__calibrationDashboard=api;stage('setup');controls();
    root.addEventListener?.('pagehide',()=>api.destroy(),{once:true});
    (async()=>{try{
      const [capabilities,setup]=await Promise.all([request(path('capabilities')),request('/api/config')]);if(state.destroyed)return;
      if(capabilities.available===false){container.innerHTML='<details class="cal-launch-hint"><summary>Offline calibration workspace</summary><p>Import images or recordings in the separate local desktop calibration application. Vision pipelines and camera ownership stay separate.</p><code>python tools/calibration_dashboard.py</code></details>';return;}
      if(capabilities.desktop_mode)doc.body.classList.add('calibration-only');
      state.capabilities=capabilities;state.token=setup.csrf_token||options.token||'';
      setText('cal-connection',capabilities.live_capture?'Local guided / offline workspace':'Local offline workspace');
      if(capabilities.live_capture){
        $('cal-calibration-states').hidden=false;$('cal-review-gate').hidden=false;
        if(root.CVCalibrationDiagnostics){$('cal-diagnostics').hidden=false;diagnostics=root.CVCalibrationDiagnostics.mount($('cal-diagnostics'),{request,onError:error=>say(error.message,true)});}
        if(root.CVCalibrationCapture){$('cal-capture-settings').hidden=false;$('cal-capture-preview').hidden=false;
          captureUI=root.CVCalibrationCapture.mount($('cal-capture-settings'),$('cal-capture-preview'),{request,getBoard:()=>({cols:Number($('cal-cols').value),rows:Number($('cal-rows').value),square_size_m:millimetersToMeters($('cal-square').value)}),getSession:()=>state.session,isBusy:()=>state.busy||active(state.job),
          createSession:async source=>request(path('sessions'),captureSpecification(formValues(),source)),
          onStatus:()=>controls(),onSession:session=>{enterCaptureSolver();setForm(session);clearCandidate();review(session);refreshSessions().catch(error=>say(error.message,true));},onSnapshot:async()=>{if(state.session){review(await request(path('sessions/'+encodeURIComponent(state.session.session_id))));await refreshSessions();}},
          onStopped:async()=>{if(state.session?.input?.kind==='capture'){review(await request(path('sessions/'+encodeURIComponent(state.session.session_id))));await refreshSessions();}},onInvalidated:()=>{if(state.session?.input?.kind==='capture'){state.session=null;clearCandidate();clearReview();setText('cal-session-id','Previous session preserved · start a new session for changed settings');controls();}},onError:error=>say(error.message,true)});
          for(const input of $('cal-setup-form').querySelectorAll('input,select'))if(!input.closest('#cal-capture-settings')&&!['cal-files','cal-timestamps','cal-input-kind'].includes(input.id))input.addEventListener('input',()=>{if(state.session?.input?.kind==='capture'||captureUI.current()?.capture_id)captureUI.invalidate();else captureUI.sync();});
        }
        await calibrationStatus();
      }setText('cal-capabilities',JSON.stringify(capabilities,null,2));
      const mrcal=$('cal-solver').querySelector('option[value="mrcal"]');mrcal.disabled=!capabilities.solvers?.mrcal?.available;mrcal.textContent=capabilities.solvers?.mrcal?.available?'Native mrcal':'Native mrcal · unavailable';
      $('cal-activation').hidden=!capabilities.runtime_activation;
      if(capabilities.runtime_activation){for(const p of setup.config?.pipelines||[]){const option=doc.createElement('option');option.value=p.name;option.textContent=p.name;$('cal-activation-pipeline').append(option);}await activationHistory();}
      solverControls();await refreshSessions();controls();
    }catch(error){say(error.message,true);setText('cal-connection','Workspace unavailable');}})();
    return api;
  }
  return {mount,createJobMonitor,normalizeSpec,captureSpecification,solverPreferencesForSession,selectionPresentation,canSolveSession,openSessionAfterStop,millimetersToMeters,metersToMillimeters,fileBase64,displayNumber:display,isActiveJob:active};
});
