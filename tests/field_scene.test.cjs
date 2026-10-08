// Portable map import tests; no browser renderer, network, camera or planner.
const assert=require('node:assert/strict');
const crypto=require('node:crypto');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const {test}=require('node:test');
const scene=require('../custom_vision/static/field_scene.js');
const renderer=require('../custom_vision/static/field_renderer.js');
const clone=value=>JSON.parse(JSON.stringify(value));
const provenance={kind:'manual',label:'Synthetic reviewed geometry',uri:null};
const approvedReview=()=>({state:'approved',reviewed_revision:1});
function fixture() {
  const map={schema_version:'frc-field-map/1',
    map:{id:'synthetic-test',revision:1,season:null,variant:'synthetic',frame:'wpilib_nwu',units:'m',width_m:10,height_m:5},
    image:{file_name:'synthetic-field.png',sha256:'0'.repeat(64),width_px:1000,height_px:500,mime_type:'image/png',attribution:'Synthetic test',license:null},
    source:{kind:'synthetic',label:'Synthetic test only',uri:null},
    calibration:{model:'affine',image_to_field:[.01,0,0,0,-.01,5,0,0,1],
      control_points:[{pixel:[0,0],field_m:[0,5]},{pixel:[1000,0],field_m:[10,5]},{pixel:[0,500],field_m:[0,0]}],
      distortion:'not_applicable',fit_error_m:0,independent_check_error_m:0,
      independent_check_points:[{pixel:[500,250],field_m:[5,2.5]}]},
    boundary:{outer:[[0,0],[10,0],[10,5],[0,5],[0,0]],holes:[],review:approvedReview(),provenance:clone(provenance)},
    obstacles:[{id:'obstacle-1',outer:[[2,1],[3,1],[3,2],[2,2],[2,1]],holes:[],review:approvedReview(),provenance:clone(provenance),vertical_range_m:null}],
    approval:{state:'approved',reviewed_revision:1,content_sha256:null}};
  map.approval.content_sha256=scene.canonicalDigest(map);
  return map;
}
const layout={field:{length:10,width:5},tags:[]};

test('pure SHA-256 matches published vectors and bounded byte samples without WebCrypto',()=>{
  for(const value of ['', 'abc', 'a'.repeat(55), 'a'.repeat(56), 'a'.repeat(64), 'a'.repeat(1000), 'café 🚀']) {
    const bytes=new TextEncoder().encode(value);
    assert.equal(scene.sha256(bytes),crypto.createHash('sha256').update(bytes).digest('hex'));
  }
  const bytes=Uint8Array.from({length:4097},(_,i)=>(i*17)%256);
  assert.equal(scene.sha256(bytes),crypto.createHash('sha256').update(bytes).digest('hex'));
  assert.throws(()=>scene.sha256('abc'),/Uint8Array/);
});

test('canonical digest excludes only top approval, sorts keys, encodes f64 and normalizes negative zero',()=>{
  const expected='{"a":"f64:3ff0000000000000","nested":{"approval":"included","z":"f64:0000000000000000"},"s":"café 🚀"}';
  const value={s:'café 🚀',nested:{z:-0,approval:'included'},a:1,approval:{ignored:true}};
  assert.equal(scene.canonicalDigest(value),crypto.createHash('sha256').update(expected,'utf8').digest('hex'));
  assert.equal(scene.canonicalDigest(value),scene.canonicalDigest({a:1,nested:{approval:'included',z:0},s:'café 🚀'}));
  assert.notEqual(scene.canonicalDigest(value),scene.canonicalDigest({...value,a:1.0000001}));
  assert.throws(()=>scene.canonicalDigest({n:NaN}),/finite/);
});

test('shared A* golden map and image match exact hashes, landmark projection and unchanged physical polygons',()=>{
  const directory=path.join(__dirname,'fixtures/field-map');
  const input=JSON.parse(fs.readFileSync(path.join(directory,'synthetic-approved.json'),'utf8'));
  const bytes=fs.readFileSync(path.join(directory,input.image.file_name));
  assert.equal(scene.canonicalDigest(input),'e022b3c0b20e83f71f8a6519946d635bc2e730c38d20f65392dae404d76f0163');
  assert.equal(scene.sha256(bytes),'9f8ad23da16dcf864327ca31558ad9e1cae8864a8355e9c2f9feba48a828ebeb');
  assert.equal(scene.sha256(bytes),input.image.sha256);
  assert.equal(bytes.readUInt32BE(16),input.image.width_px); assert.equal(bytes.readUInt32BE(20),input.image.height_px);
  const view=scene.validateFieldMap(input,{field:{length:8,width:4}});
  assert.equal(view.approved,true); assert.equal(view.bindingValid,true);
  assert.deepEqual(view.field,{length:8,width:4});
  assert.deepEqual(view.geometry.boundary.vertices_m,input.boundary.outer);
  assert.deepEqual(view.geometry.obstacles[0].vertices_m,input.obstacles[0].outer);
  assert.deepEqual(view.geometry.obstacles[0].holes_m,input.obstacles[0].holes);
  const polygons=renderer.polygonData(view.geometry);
  assert.deepEqual(polygons[0].points,input.obstacles[0].outer.map(([x,y])=>[x,y,0]));
  assert.deepEqual(polygons[0].holes,input.obstacles[0].holes.map(ring=>ring.map(([x,y])=>[x,y,0])));
  assert.equal(polygons[0].height,null); assert.equal(polygons[0].base,null);
  const matrix=input.calibration.image_to_field,determinant=matrix[0]*matrix[4]-matrix[1]*matrix[3];
  for(const landmark of [...input.calibration.control_points,...input.calibration.independent_check_points]) {
    const [u,v]=landmark.pixel;
    const world=[matrix[0]*u+matrix[1]*v+matrix[2],matrix[3]*u+matrix[4]*v+matrix[5],0];
    assert.deepEqual(world,[...landmark.field_m,0]);
    const x=world[0]-matrix[2],y=world[1]-matrix[5];
    assert.deepEqual([(matrix[4]*x-matrix[1]*y)/determinant,(-matrix[3]*x+matrix[0]*y)/determinant].map(value=>value===0?0:value),landmark.pixel);
    const s=u/input.image.width_px,t=v/input.image.height_px,[a,b,c,d]=view.background.corners;
    const texture=[0,1].map(i=>(1-s)*(1-t)*a[i]+s*(1-t)*b[i]+s*t*c[i]+(1-s)*t*d[i]);
    assert.deepEqual(texture,landmark.field_m);
    for(const mode of ['2d','3d']) {
      const project=renderer.projection(renderer.homeView({field:view.field},mode),800,500);
      assert.deepEqual(project([...texture,0]),project(world));
    }
  }
  assert.deepEqual(scene.validateFieldMap(view.map,{field:{length:8,width:4}}).geometry,view.geometry);
  const edited=clone(input); edited.source.label+=' edited';
  assert.equal(scene.validateFieldMap(edited,{field:{length:8,width:4}}).approved,false);
});

test('approved scene preserves width-X height-Y fixed NWU and planar image corners',()=>{
  const input=fixture(),original=clone(input),view=scene.validateFieldMap(input,layout);
  assert.equal(view.approved,true); assert.equal(view.bindingValid,true); assert.equal(view.status,'approved');
  assert.deepEqual(view.field,{length:10,width:5});
  assert.deepEqual(view.background.corners,[[0,5],[10,5],[10,0],[0,0]]);
  assert.deepEqual(view.geometry.boundary.vertices_m,input.boundary.outer);
  assert.equal(view.geometry.boundary.reviewed,true);
  assert.equal(view.geometry.obstacles[0].height_m,null);
  assert.equal(view.geometry.obstacles[0].base_z_m,null);
  view.map.boundary.outer[0][0]=99;
  assert.deepEqual(input,original);
  assert.match(view.warnings.join(' '),/Synthetic/);
});

test('dimension mismatch prevents overlay while missing layout has an explicit warning',()=>{
  const mismatch=scene.validateFieldMap(fixture(),{field:{length:16,width:8}});
  assert.equal(mismatch.bindingValid,false); assert.equal(mismatch.status,'layout_mismatch');
  assert.match(mismatch.warnings.join(' '),/no field overlay/);
  const unbound=scene.validateFieldMap(fixture(),null);
  assert.equal(unbound.bindingValid,true);
  assert.match(unbound.warnings.join(' '),/No WPILib layout/);
});

test('edited content and stale polygon revision invalidate approval and withhold vertical geometry',()=>{
  const input=fixture(); input.obstacles[0].vertical_range_m={min:.2,max:1.2};
  input.approval.content_sha256=scene.canonicalDigest(input);
  let view=scene.validateFieldMap(input,layout);
  assert.equal(view.approved,true); assert.equal(view.geometry.obstacles[0].base_z_m,.2);
  assert.equal(view.geometry.obstacles[0].height_m,1);
  input.obstacles[0].provenance.label='Edited geometry metadata';
  view=scene.validateFieldMap(input,layout);
  assert.equal(view.approved,false); assert.equal(view.status,'draft');
  assert.equal(view.geometry.obstacles[0].height_m,null);
  assert.match(view.warnings.join(' '),/stale/);
  input.approval.content_sha256=scene.canonicalDigest(input);
  input.obstacles[0].review.reviewed_revision=2;
  input.approval.content_sha256=scene.canonicalDigest(input);
  assert.equal(scene.validateFieldMap(input,layout).approved,false);
});

test('missing fit or independent check evidence downgrades otherwise approved content and withholds solids',()=>{
  for(const change of [x=>x.calibration.fit_error_m=null,x=>x.calibration.independent_check_error_m=null,
                      x=>x.calibration.independent_check_points=[]]) {
    const input=fixture(); input.obstacles[0].vertical_range_m={min:0,max:1}; change(input);
    input.approval.content_sha256=scene.canonicalDigest(input);
    const view=scene.validateFieldMap(input,layout);
    assert.equal(view.approved,false); assert.equal(view.status,'draft');
    assert.equal(view.geometry.obstacles[0].height_m,null); assert.equal(view.geometry.obstacles[0].base_z_m,null);
    assert.match(view.warnings.join(' '),/Approval requires provided finite fit error/);
  }
  assert.throws(()=>scene.validateFieldMap({}),/root must contain exactly/);
});

test('matching RMS residuals are reported without inventing an accuracy threshold or physical qualification',()=>{
  const input=fixture();
  input.calibration.control_points[0].field_m=[0,4.7];
  input.calibration.control_points[1].field_m=[10,4.6];
  input.calibration.fit_error_m=.5/Math.sqrt(3);
  input.calibration.independent_check_points[0].field_m=[4.7,2.1];
  input.calibration.independent_check_error_m=.5;
  input.approval.content_sha256=scene.canonicalDigest(input);
  const view=scene.validateFieldMap(input,layout);
  assert.equal(view.approved,true);
  assert.match(view.warnings.join(' '),/0\.288675/); assert.match(view.warnings.join(' '),/0\.5 m/);
  assert.match(view.warnings.join(' '),/Computed affine RMS/);
  assert.match(view.warnings.join(' '),/no accuracy acceptance threshold/);
  assert.match(view.warnings.join(' '),/not hardware qualification/);
});

test('declared fit and independent errors must match recomputed Euclidean RMS within absolute 1e-8 meters',()=>{
  for(const key of ['fit_error_m','independent_check_error_m']) {
    const input=fixture(); input.calibration[key]=2e-8;
    input.approval.content_sha256=scene.canonicalDigest(input);
    assert.throws(()=>scene.validateFieldMap(input),new RegExp(`${key} disagrees with affine RMS`));
    input.calibration[key]=.5e-8;
    input.approval.content_sha256=scene.canonicalDigest(input);
    assert.equal(scene.validateFieldMap(input,layout).approved,true);
  }
  const input=fixture();
  input.calibration.independent_check_points=[{pixel:[500,250],field_m:[4.7,2.1]},
    {pixel:[750,250],field_m:[7.5,2.5]}];
  input.calibration.independent_check_error_m=.5/Math.sqrt(2);
  input.approval.content_sha256=scene.canonicalDigest(input);
  assert.equal(scene.validateFieldMap(input,layout).approved,true);
  // Per-axis RMS and maximum error differ from the agreed Euclidean RMS.
  for(const wrong of [.25,.5]) {
    input.calibration.independent_check_error_m=wrong;
    assert.throws(()=>scene.validateFieldMap(input),/independent_check_error_m disagrees/);
  }
});

test('independent approval needs a genuinely distinct image location and metric landmark',()=>{
  const cases=[
    {pixel:[0,0],field_m:[0,5]},
    {pixel:[0,0],field_m:[0,4.9]},
    {pixel:[1,0],field_m:[0,5]},
    {pixel:[1e-12,0],field_m:[1e-12,5]}
  ];
  for(const check of cases) {
    const input=fixture(); input.obstacles[0].vertical_range_m={min:0,max:1};
    input.calibration.independent_check_points=[check];
    input.calibration.independent_check_error_m=Math.hypot(.01*check.pixel[0]-check.field_m[0],5-.01*check.pixel[1]-check.field_m[1]);
    input.approval.content_sha256=scene.canonicalDigest(input);
    const view=scene.validateFieldMap(input,layout);
    assert.equal(view.approved,false); assert.equal(view.geometry.obstacles[0].height_m,null);
    assert.match(view.warnings.join(' '),/distinct from fitted image and metric controls/);
  }
  const input=fixture();
  input.calibration.independent_check_points.unshift(clone(input.calibration.control_points[0]));
  input.approval.content_sha256=scene.canonicalDigest(input);
  assert.equal(scene.validateFieldMap(input,layout).approved,true);
});

test('affine image landmarks round-trip through field coordinates and actual 2D/3D ground projections',()=>{
  const input=fixture(),view=scene.validateFieldMap(input,layout),matrix=input.calibration.image_to_field;
  const landmarks=[...input.calibration.control_points,...input.calibration.independent_check_points,
    {pixel:[125,62.5],field_m:[1.25,4.375]},{pixel:[875,437.5],field_m:[8.75,.625]}];
  const close=(a,b)=>a.forEach((value,i)=>assert.ok(Math.abs(value-b[i])<1e-10,`${a} != ${b}`));
  const determinant=matrix[0]*matrix[4]-matrix[1]*matrix[3];
  for(const landmark of landmarks) {
    const [u,v]=landmark.pixel;
    const fieldPoint=[matrix[0]*u+matrix[1]*v+matrix[2],matrix[3]*u+matrix[4]*v+matrix[5]];
    close(fieldPoint,landmark.field_m);
    const x=fieldPoint[0]-matrix[2],y=fieldPoint[1]-matrix[5];
    close([(matrix[4]*x-matrix[1]*y)/determinant,(-matrix[3]*x+matrix[0]*y)/determinant],landmark.pixel);
    // The renderer's planar texture interpolates imported corners at Z=0.
    const s=u/input.image.width_px,t=v/input.image.height_px,[a,b,c,d]=view.background.corners;
    const imagePlanePoint=[0,1].map(i=>(1-s)*(1-t)*a[i]+s*(1-t)*b[i]+s*t*c[i]+(1-s)*t*d[i]);
    close(imagePlanePoint,fieldPoint);
    for(const mode of ['2d','3d']) {
      const project=renderer.projection(renderer.homeView({field:view.field},mode),800,500);
      const imagePosition=project([...imagePlanePoint,0]),metricPosition=project([...landmark.field_m,0]);
      assert.equal(imagePosition.visible,true); assert.equal(metricPosition.visible,true);
      close([imagePosition.x,imagePosition.y,imagePosition.depth],[metricPosition.x,metricPosition.y,metricPosition.depth]);
    }
  }
});

test('physical rings and holes are copied unchanged without footprint inflation or planar-height substitution',()=>{
  const input=fixture(),original=clone(input);
  const view=scene.validateFieldMap(input,layout),polygons=renderer.polygonData(view.geometry);
  assert.deepEqual(view.map,original);
  assert.deepEqual(view.geometry.boundary.vertices_m,input.boundary.outer);
  assert.deepEqual(view.geometry.obstacles[0].vertices_m,input.obstacles[0].outer);
  assert.deepEqual(polygons[0].points,input.obstacles[0].outer.map(([x,y])=>[x,y,0]));
  assert.equal(polygons[0].height,null); assert.equal(polygons[0].base,null);
  const again=scene.validateFieldMap(view.map,layout);
  assert.deepEqual(again.geometry,view.geometry);
  assert.deepEqual(input,original);
});

test('suggestions stay draft and cannot infer height from a planar image',()=>{
  const input=fixture(); input.obstacles[0].review={state:'draft',reviewed_revision:null};
  input.obstacles[0].provenance.kind='suggested';
  input.approval={state:'draft',reviewed_revision:null,content_sha256:null};
  const view=scene.validateFieldMap(input,layout);
  assert.equal(view.approved,false); assert.equal(view.geometry.obstacles[0].reviewed,false);
  input.obstacles[0].vertical_range_m={min:0,max:1};
  assert.throws(()=>scene.validateFieldMap(input),/cannot establish vertical geometry/);
});

test('holes are retained and must be closed clockwise, simple and strictly contained',()=>{
  const input=fixture();
  input.boundary.holes=[[[6,1],[6,2],[7,2],[7,1],[6,1]]];
  input.obstacles[0].holes=[[[2.2,1.2],[2.2,1.5],[2.5,1.5],[2.5,1.2],[2.2,1.2]]];
  const view=scene.validateFieldMap(input,layout);
  assert.deepEqual(view.geometry.boundary.holes_m,input.boundary.holes);
  assert.deepEqual(view.geometry.obstacles[0].holes_m,input.obstacles[0].holes);
  const wrong=clone(input); wrong.boundary.holes[0].reverse();
  assert.throws(()=>scene.validateFieldMap(wrong),/clockwise/);
  const touch=clone(input); touch.boundary.holes[0]=[[0,1],[0,2],[1,2],[1,1],[0,1]];
  assert.throws(()=>scene.validateFieldMap(touch),/strictly inside/);
  const nested=clone(input); nested.boundary.holes.push([[6.2,1.2],[6.2,1.4],[6.4,1.4],[6.4,1.2],[6.2,1.2]]);
  assert.throws(()=>scene.validateFieldMap(nested),/overlap or nest/);
});

test('rejects nonfinite, out-of-bounds, duplicate, open, self-crossing and wrongly wound metric rings',()=>{
  const changes=[
    x=>x.map.width_m=NaN,
    x=>x.boundary.outer[1]=[11,0],
    x=>x.obstacles.push(clone(x.obstacles[0])),
    x=>x.boundary.outer.pop(),
    x=>x.boundary.outer=[[0,0],[10,5],[10,0],[0,5],[0,0]],
    x=>x.boundary.outer.reverse(),
    x=>x.obstacles[0].outer=[[2,1],[3,1],[2.5,1],[3,2],[2,2],[2,1]],
    x=>x.obstacles[0].outer=[[2,1],[3,1],[3,2],[2,1],[2,2],[2,1]]
  ];
  for(const change of changes) { const input=fixture(); change(input); assert.throws(()=>scene.validateFieldMap(input),/Field map/); }
});

test('obstacle edges cannot cross a concave field boundary or its holes',()=>{
  const input=fixture();
  input.boundary.outer=[[0,0],[10,0],[10,5],[6,5],[6,3],[4,3],[4,5],[0,5],[0,0]];
  input.obstacles[0].outer=[[3,4],[7,4],[7,4.5],[3,4.5],[3,4]];
  assert.throws(()=>scene.validateFieldMap(input),/outside the boundary/);
  const hole=fixture(); hole.boundary.holes=[[[2.2,1.2],[2.2,1.4],[2.4,1.4],[2.4,1.2],[2.2,1.2]]];
  assert.throws(()=>scene.validateFieldMap(hole),/boundary hole/);
});

test('affine calibration rejects singular/projective/collinear inputs and unsupported origins',()=>{
  const changes=[
    x=>x.calibration.image_to_field[4]=0,
    x=>x.calibration.image_to_field[6]=.001,
    x=>x.calibration.model='homography',
    x=>x.calibration.control_points=[{pixel:[0,0],field_m:[0,0]},{pixel:[1,1],field_m:[1,1]},{pixel:[2,2],field_m:[2,2]}],
    x=>x.map.frame='alliance_relative',
    x=>x.map.units='ft',
    x=>x.schema_version='frc-field-map/2'
  ];
  for(const change of changes) { const input=fixture(); change(input); assert.throws(()=>scene.validateFieldMap(input),/Field map/); }
});

test('only local PNG/JPEG/WebP asset basenames are accepted and MIME/dimensions are checked',()=>{
  for(const name of ['../field.png','https://example.invalid/field.png','field.svg','file\\field.png','field..png']) {
    const input=fixture(); input.image.file_name=name; assert.throws(()=>scene.validateFieldMap(input),/basename/);
  }
  for(const change of [x=>x.image.mime_type='image/jpeg',x=>x.image.width_px=1.5,x=>x.image.sha256='x'.repeat(64)]) {
    const input=fixture(); change(input); assert.throws(()=>scene.validateFieldMap(input),/Field map/);
  }
});

test('unknown fields and excessive geometry are rejected instead of silently replacing the contract',()=>{
  const input=fixture(); input.origin_flip=true;
  assert.throws(()=>scene.validateFieldMap(input),/exactly/);
  const large=fixture(); large.obstacles=Array.from({length:129},(_,i)=>({...clone(large.obstacles[0]),id:String(i)}));
  assert.throws(()=>scene.validateFieldMap(large),/bounded array/);
});

test('UMD browser API works without crypto or network globals',()=>{
  const code=fs.readFileSync(path.join(__dirname,'../custom_vision/static/field_scene.js'),'utf8');
  const context=vm.createContext({Uint8Array,DataView,ArrayBuffer,TextEncoder});
  vm.runInContext(code,context);
  assert.equal(typeof context.CVFieldScene.validateFieldMap,'function');
  assert.equal(context.CVFieldScene.sha256(new TextEncoder().encode('abc')),scene.sha256(new TextEncoder().encode('abc')));
});
