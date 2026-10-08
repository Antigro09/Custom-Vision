const assert = require('node:assert/strict');
const {test} = require('node:test');
const ui = require('../custom_vision/static/calibration_dashboard.js');

function declaration() {
  return {cols:'7',rows:'5',square_size_m:'0.03',user_label:'Synthetic OV9281-labelled recording',physical_id:'',identity_source:'manual',
    width:'',height:'',focus_kind:'unknown',focus_value:'',focus_locked:false,input_kind:'images',max_views:'180',interval_s:'.7',novelty:'.025',max_sharpness_px:'3',min_contrast:'40'};
}
function timers() {
  let next=0;const pending=new Map();
  return {setTimer(fn,delay){const id=++next;pending.set(id,{fn,delay});return id;},clearTimer:id=>pending.delete(id),
    pending,fire(){const [id,job]=pending.entries().next().value;pending.delete(id);return job.fn();}};
}
const tick = () => new Promise(resolve=>setImmediate(resolve));

test('manual camera model names never supply sensor mode, identity, focus or measurements', () => {
  const spec=ui.normalizeSpec(declaration(),['asset-a']);
  assert.equal(spec.camera.physical_id,null);assert.equal(spec.camera.capture_history,null);
  assert.deepEqual(spec.mode,{width:null,height:null,crop:null,binning:null,focus:{kind:'unknown',value:null,locked:false}});
  assert.deepEqual(spec.board,{cols:7,rows:5,square_size_m:.03});
  assert.equal(spec.input.timestamps_asset_id,null);
  assert.throws(()=>ui.normalizeSpec({...declaration(),cols:'',rows:'',square_size_m:''},['asset-a']),/square size/);
});

test('declared crop, binning, fixed focus and recording history survive session specification', () => {
  const input={...declaration(),width:'640',height:'480',crop_x:'0',crop_y:'10',crop_width:'640',crop_height:'480',
    binning_x:'2',binning_y:'2',focus_kind:'fixed',focus_locked:true,capture_history:'Synthetic lossless local fixture; no camera'};
  const spec=ui.normalizeSpec(input,['asset-a'],'timestamps-a');
  assert.deepEqual(spec.mode.crop,{x:0,y:10,width:640,height:480});assert.deepEqual(spec.mode.binning,[2,2]);
  assert.equal(spec.mode.focus.locked,true);assert.equal(spec.camera.capture_history,input.capture_history);
  assert.equal(spec.input.timestamps_asset_id,'timestamps-a');
});

test('partial declarations and excessive input budgets fail before a session write', () => {
  for(const change of [{width:'640'},{crop_width:'300'},{binning_x:'2'},{focus_kind:'fixed',focus_value:'1'},
    {max_views:'181'},{interval_s:'NaN'},{square_size_m:'1'},{identity_source:'guessed_by_model'}]) {
    assert.throws(()=>ui.normalizeSpec({...declaration(),...change},['asset-a']));
  }
  assert.throws(()=>ui.normalizeSpec(declaration(),[]),/Import/);
  assert.throws(()=>ui.normalizeSpec({...declaration(),input_kind:'video'},['a','b']),/exactly one video/);
});

test('residual reporting preserves measured zero and does not turn null into computed metrics', () => {
  assert.equal(ui.displayNumber(0,3),'0.000');assert.equal(ui.displayNumber(null),'Not computed');
  assert.equal(ui.displayNumber(NaN),'Not computed');assert.equal(ui.displayNumber('0'),'Not computed');
});

test('one outstanding worker poll remains bounded even if a replacement job starts', async () => {
  const clock=timers(),updates=[],requests=[];let resolveFirst;
  const monitor=ui.createJobMonitor({...clock,request:path=>{requests.push(path);return new Promise(resolve=>{resolveFirst=resolve;});},onUpdate:job=>updates.push(job.job_id)});
  monitor.start({job_id:'first',status:'running'});const first=clock.fire();
  monitor.start({job_id:'replacement',status:'running'});await clock.fire();
  assert.equal(requests.length,1);assert.equal(monitor.isPolling(),true);
  resolveFirst({job_id:'first',status:'completed'});await first;
  assert.deepEqual(updates,['first','replacement']);assert.equal(monitor.current().job_id,'replacement');
  assert.equal(clock.pending.size,1);monitor.stop();
});

test('polling terminal canceled status stops without treating cancellation as failure', async () => {
  const clock=timers(),finished=[];
  const monitor=ui.createJobMonitor({...clock,request:async()=>({job_id:'job',status:'canceled'}),onFinished:job=>finished.push(job.status)});
  monitor.start({job_id:'job',status:'running'});await clock.fire();
  assert.deepEqual(finished,['canceled']);assert.equal(clock.pending.size,0);assert.equal(monitor.current(),null);
});

test('visibility throttles job polling and destroy ignores late responses', async () => {
  const clock=timers(),finished=[],updates=[];let resolve;
  const monitor=ui.createJobMonitor({...clock,isHidden:()=>true,request:()=>new Promise(r=>{resolve=r;}),onUpdate:job=>updates.push(job.status),onFinished:job=>finished.push(job.status)});
  monitor.start({job_id:'job',status:'running'});assert.equal([...clock.pending.values()][0].delay,5000);
  const pending=clock.fire();monitor.stop();resolve({job_id:'job',status:'completed'});await pending;
  assert.deepEqual(updates,['running']);assert.deepEqual(finished,[]);assert.equal(clock.pending.size,0);
});

test('worker status failure surfaces once and only retries the read', async () => {
  const clock=timers(),errors=[];let calls=0;
  const monitor=ui.createJobMonitor({...clock,request:async()=>{calls++;throw new Error('temporary connection failure');},onError:error=>errors.push(error.message)});
  monitor.start({job_id:'job',status:'running'});await clock.fire();
  assert.equal(calls,1);assert.deepEqual(errors,['temporary connection failure']);assert.equal([...clock.pending.values()][0].delay,1000);
  monitor.stop();await tick();
});
