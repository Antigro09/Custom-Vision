const assert = require('node:assert/strict');
const {test} = require('node:test');
const renderer = require('../custom_vision/static/field_renderer.js');
const pose = (t=[0,0,0],q=[1,0,0,0]) => ({translation_m:t,rotation_quaternion_wxyz:q,frame:'wpilib_nwu'});
const near = (actual,expected) => actual.forEach((v,i) => assert.ok(Math.abs(v-expected[i]) < 1e-10, `${actual} != ${expected}`));

function harness(rect={width:800,height:500}) {
  const calls=[],listeners=new Map(),timers=new Map();let time=0,nextTimer=0;
  const drawingState={globalAlpha:1},saved=[];
  drawingState.save=() => {saved.push({...drawingState});calls.push({name:'save',args:[]});};
  drawingState.restore=() => {Object.assign(drawingState,saved.pop());calls.push({name:'restore',args:[]});};
  const ctx=new Proxy(drawingState, {get(target,key) {return target[key] || ((...args) => calls.push({name:key,args}));},
    set(target,key,value) {target[key]=value;calls.push({name:'set:'+key,args:[value]});return true;}});
  const canvas={width:0,height:0,getContext:() => ctx,getBoundingClientRect:() => rect,
    addEventListener:(name,fn) => listeners.set(name,fn),removeEventListener:name => listeners.delete(name),
    focus(){},setPointerCapture(){},hasPointerCapture:() => false};
  const instance=renderer.createRenderer(canvas,{now:() => time,
    setTimer:(fn,delay) => {const id=++nextTimer;timers.set(id,{fn,due:time+delay});return id;},clearTimer:id => timers.delete(id)});
  return {instance,canvas,calls,listeners,timers,flush(value) {
    time=value;
    for (let i=0;i<20;i++) {
      const ready=[...timers].find(([,timer]) => timer.due<=time);
      if (!ready) return;
      timers.delete(ready[0]);ready[1].fn();
    }
    throw new Error('Scheduler failed to settle');
  },event(name,event={}) {
    let prevented=false;listeners.get(name)({...event,preventDefault(){prevented=true;}});return prevented;
  }};
}

test('WXYZ rotates axes directly in NWU, including yaw and pitch', () => {
  const yaw=renderer.poseAxes(pose([2,3,.7],[Math.SQRT1_2,0,0,Math.SQRT1_2]),1);
  near(yaw.origin,[2,3,.7]);near(yaw.x,[2,4,.7]);near(yaw.y,[1,3,.7]);near(yaw.z,[2,3,1.7]);
  const pitch=renderer.poseAxes(pose([0,0,0],[Math.SQRT1_2,0,Math.SQRT1_2,0]),1);
  near(pitch.x,[0,0,-1]);near(pitch.z,[1,0,0]);
  assert.equal(renderer.normalizePose({...pose(),frame:'optical'}),null);
  assert.equal(renderer.normalizePose(pose([0,0,NaN])),null);
  assert.equal(renderer.normalizePose(pose([0,0,0],[0,0,0,0])),null);
  assert.equal(renderer.normalizePose(pose([0,0,0],[1e308,1e308,1e308,1e308])),null);
});

test('2D keeps field X right, Y up and ignores height only in projection', () => {
  const p=renderer.projection({mode:'2d',target:[0,0,0],distance:10,zoom:1},800,500);
  assert.ok(p([1,0,0]).x>p([0,0,0]).x);
  assert.ok(p([0,1,0]).y<p([0,0,0]).y);
  assert.deepEqual(p([1,2,0]),p([1,2,8]));
  assert.equal(p([1e308,1e308,0]).visible,false);
});

test('3D perspective responds to actual Z and rejects behind-camera points', () => {
  const p=renderer.projection({mode:'3d',target:[0,0,0],distance:10,zoom:1,azimuth:0,elevation:Math.PI/4},800,500);
  assert.ok(p([0,0,1]).y<p([0,0,0]).y);
  assert.notEqual(p([0,0,1]).depth,p([0,0,0]).depth);
  assert.ok(p([0,1,0]).x>p([0,0,0]).x);
  assert.equal(p([20,0,20]).visible,false);
});

test('missing field dimensions define only a viewport, never a field', () => {
  assert.equal(renderer.fieldBounds(null),null);
  assert.equal(renderer.fieldBounds({length:16,width:null}),null);
  const home=renderer.homeView({},'2d');assert.deepEqual(home.target,[0,0,0]);
  const h=harness();h.flush(0);
  assert.ok(h.calls.some(c => c.name==='fillText' && c.args[0].includes('Field geometry unavailable')));
  h.instance.setScene({field:{length:16.5,width:8.2}});h.instance.home();
  assert.deepEqual(h.instance.getView().target,[8.25,4.1,0]);h.instance.destroy();
});

test('unknown height or unknown base never becomes a solid; holes persist', () => {
  const ring=[[0,0],[2,0],[2,2],[0,2]],hole=[[.5,.5],[1,.5],[1,1],[.5,1]];
  const result=renderer.polygonData({obstacles:[
    {vertices_m:ring,holes_m:[hole],reviewed:true,base_z_m:null,height_m:1},
    {vertices_m:ring,reviewed:false,base_z_m:0,height_m:1},
    {vertices_m:ring,holes_m:[hole],reviewed:true,base_z_m:.2,height_m:1},
  ]});
  assert.equal(result[0].height,null);assert.equal(result[0].base,null);assert.equal(result[0].holes.length,1);
  assert.equal(result[1].height,null);assert.equal(result[2].height,1);assert.equal(result[2].base,.2);
  const h=harness();h.instance.setMode('3d');
  h.instance.setScene({geometry:{obstacles:[{vertices_m:ring,holes_m:[hole],reviewed:true,base_z_m:.2,height_m:1}]}});h.flush(0);
  assert.ok(h.calls.some(c => c.name==='fill' && c.args[0]==='evenodd'));
  h.calls.length=0;h.instance.setScene({geometry:{obstacles:[{vertices_m:ring,reviewed:true,base_z_m:null,height_m:null}]}});h.flush(67);
  assert.equal(h.calls.filter(c => c.name==='fill').length,0);
  assert.ok(h.calls.some(c => c.name==='fillText' && c.args[0].includes('z / height unknown')));h.instance.destroy();
});

test('trail break starts at its point, preserving gaps and a bounded history', () => {
  const points=[0,1,2,3].map(x => ({translation_m:[x,0,0],break:x===0 || x===2}));
  assert.deepEqual(renderer.trailSegments(points),[[[0,0,0],[1,0,0]],[[2,0,0],[3,0,0]]]);
  assert.deepEqual(renderer.trailSegments([points[0],null,points[3]]),[[[0,0,0]],[[3,0,0]]]);
  const many=Array.from({length:1000},(_,x) => ({translation_m:[x,0,0]}));
  assert.equal(renderer.trailSegments(many)[0].length,160);
  assert.equal(renderer.trailSegments(many)[0][0][0],840);
});

test('camera-only localization and stale robot estimate render independently', () => {
  const h=harness();
  h.instance.setScene({sources:[{pipeline:'front',camera:{pose:pose([1,1,.7]),status:'valid'},
    robot:{pose:null,status:'unavailable'}}]});h.flush(0);
  let labels=h.calls.filter(c => c.name==='fillText').map(c => c.args[0]);
  assert.ok(labels.some(t => t==='front · vision camera'));
  assert.equal(labels.some(t => t==='front · vision robot'),false);
  h.calls.length=0;h.instance.setScene({sources:[{pipeline:'front',camera:{pose:pose([1,1,.7]),status:'valid'},
    robot:{pose:pose([1,1,.3]),status:'stale'}}]});h.flush(67);
  labels=h.calls.filter(c => c.name==='fillText').map(c => c.args[0]);
  assert.ok(labels.includes('front · vision robot · stale'));
  assert.ok(h.calls.some(c => c.name==='set:globalAlpha' && c.args[0]===.4));h.instance.destroy();
});

test('observed tags remain visible when their localization solve is rejected', () => {
  const h=harness();
  h.instance.setScene({tags:[{id:7,pose:pose([1,1,1])}],sources:[
    {status:'invalid',observedTagIds:[7],usedTagIds:[],camera:{pose:null,status:'invalid'},robot:{pose:null,status:'invalid'}}]});
  h.flush(0);
  const tagStroke=h.calls.findIndex(c => c.name==='strokeRect');
  assert.ok(tagStroke>0);
  const recentColor=h.calls.slice(0,tagStroke).filter(c => c.name==='set:strokeStyle').at(-1);
  assert.equal(recentColor.args[0],'#85c7ff');h.instance.destroy();
});

test('render scheduling coalesces packet bursts and caps frames at 15 per second', () => {
  const h=harness();h.flush(0);assert.equal(h.instance.getStats().drawCount,1);
  for (let i=0;i<100;i++) h.instance.setScene({sources:[]});
  assert.equal(h.timers.size,1);h.flush(30);assert.equal(h.instance.getStats().drawCount,1);
  h.flush(67);assert.equal(h.instance.getStats().drawCount,2);assert.equal(h.timers.size,0);
  h.instance.draw();h.instance.destroy();assert.equal(h.timers.size,0);assert.equal(h.listeners.size,0);
});

test('backing canvas pixel budget remains bounded for oversized panels', () => {
  const h=harness({width:100_000,height:80_000});h.flush(0);
  assert.ok(h.canvas.width<=4096);assert.ok(h.canvas.height<=4096);
  assert.ok(h.canvas.width*h.canvas.height<=8_010_000);h.instance.destroy();
});

test('pointer and keyboard controls change only the view and Home restores it', () => {
  const h=harness();h.flush(0);h.instance.setMode('3d');
  const original=h.instance.getView();assert.equal(h.event('keydown',{key:'ArrowLeft'}),true);
  assert.notEqual(h.instance.getView().azimuth,original.azimuth);
  h.event('wheel',{deltaY:-200});assert.ok(h.instance.getView().zoom>1);
  h.event('keydown',{key:'Home'});assert.deepEqual(h.instance.getView(),original);
  h.instance.setMode('2d');h.event('pointerdown',{button:0,pointerId:1,clientX:100,clientY:100});
  h.event('pointermove',{pointerId:1,clientX:120,clientY:130});assert.notDeepEqual(h.instance.getView().target,[0,0,0]);
  h.event('pointerup',{pointerId:1});assert.throws(() => h.instance.setMode('vr'),/Mode must/);h.instance.destroy();
});

test('planar image texture uses bounded mesh and draws without inferred elevation', () => {
  const h=harness();h.instance.setMode('3d');
  h.instance.setScene({backgroundImage:{image:{width:200,height:100},corners:[[0,0],[2,0],[2,1],[0,1]]}});h.flush(0);
  const textured=h.calls.filter(c => c.name==='drawImage');assert.equal(textured.length,128);h.instance.destroy();
});


test('a maximum-size closed metric ring is preserved rather than silently dropped', () => {
  const outer=Array.from({length:512},(_,i)=>[2+Math.cos(i*2*Math.PI/512),2+Math.sin(i*2*Math.PI/512)]);
  outer.push(outer[0].slice());
  const data=renderer.polygonData({obstacles:[{id:'bounded-ring',vertices_m:outer,holes_m:[],reviewed:true,base_z_m:null,height_m:null}]});
  assert.equal(data.length,1);assert.equal(data[0].points.length,513);
});
