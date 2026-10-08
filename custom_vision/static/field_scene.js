/* Portable frc-field-map/1 import. Image calibration is planar; this module
 * does not change localization coordinates, infer heights, fetch assets or plan.
 */
(function (root, factory) {
  'use strict';
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.CVFieldScene = api;
}(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';
  const finite = value => typeof value === 'number' && Number.isFinite(value);
  const hex = value => typeof value === 'string' && /^[a-f0-9]{64}$/.test(value);
  const integer = value => Number.isSafeInteger(value) && value >= 1;
  const EPS = 1e-9;
  function fail(message) { throw new Error(`Field map: ${message}`); }
  function obj(value, name, fields) {
    if (!value || typeof value !== 'object' || Array.isArray(value)) fail(`${name} must be an object`);
    if (fields && (Object.keys(value).some(key => !fields.includes(key)) || fields.some(key => !Object.hasOwn(value, key))))
      fail(`${name} must contain exactly ${fields.join(', ')}`);
    return value;
  }
  function text(value, name, max = 256, nullable = false) {
    if (nullable && value === null) return;
    if (typeof value !== 'string' || value.length > max) fail(`${name} must be a bounded string${nullable ? ' or null' : ''}`);
  }
  function number(value, name, min, max) {
    if (!finite(value) || value < min || value > max) fail(`${name} must be finite between ${min} and ${max}`);
  }
  function pair(value, name) {
    if (!Array.isArray(value) || value.length !== 2 || !value.every(finite)) fail(`${name} must contain two finite numbers`);
  }

  // SHA-256 deliberately has no WebCrypto dependency: ordinary HTTP LAN pages
  // are not secure contexts. Input is bytes, never paths or network resources.
  function sha256(bytes) {
    if (!(bytes instanceof Uint8Array)) fail('SHA-256 input must be Uint8Array bytes');
    const constants = [0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
      0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
      0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
      0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
      0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
      0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
      0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
      0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2];
    const padded = new Uint8Array(Math.ceil((bytes.length + 9) / 64) * 64);
    padded.set(bytes); padded[bytes.length] = 0x80;
    const view = new DataView(padded.buffer), bitLength = bytes.length * 8;
    view.setUint32(padded.length - 8, Math.floor(bitLength / 4294967296));
    view.setUint32(padded.length - 4, bitLength >>> 0);
    const state = [0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19];
    const words = new Uint32Array(64), rotate = (x, n) => (x >>> n) | (x << (32 - n));
    for (let offset = 0; offset < padded.length; offset += 64) {
      for (let i = 0; i < 16; i++) words[i] = view.getUint32(offset + i * 4);
      for (let i = 16; i < 64; i++) {
        const x = words[i - 15], y = words[i - 2];
        words[i] = (words[i - 16] + (rotate(x, 7) ^ rotate(x, 18) ^ (x >>> 3)) + words[i - 7] +
          (rotate(y, 17) ^ rotate(y, 19) ^ (y >>> 10))) >>> 0;
      }
      let [a,b,c,d,e,f,g,h] = state;
      for (let i = 0; i < 64; i++) {
        const t1 = (h + (rotate(e, 6) ^ rotate(e, 11) ^ rotate(e, 25)) + ((e & f) ^ (~e & g)) + constants[i] + words[i]) >>> 0;
        const t2 = ((rotate(a, 2) ^ rotate(a, 13) ^ rotate(a, 22)) + ((a & b) ^ (a & c) ^ (b & c))) >>> 0;
        h=g; g=f; f=e; e=(d+t1)>>>0; d=c; c=b; b=a; a=(t1+t2)>>>0;
      }
      for (const [i, value] of [a,b,c,d,e,f,g,h].entries()) state[i] = (state[i] + value) >>> 0;
    }
    return state.map(value => value.toString(16).padStart(8, '0')).join('');
  }

  function codePointOrder(left, right) {
    // JS default sort compares UTF-16 code units; the shared Python/Java
    // contract compares Unicode codepoints, including supplementary planes.
    let i=0,j=0;
    while (i<left.length && j<right.length) {
      const a=left.codePointAt(i),b=right.codePointAt(j);
      if (a!==b) return a<b ? -1 : 1;
      i+=a>0xffff ? 2 : 1; j+=b>0xffff ? 2 : 1;
    }
    return i<left.length ? 1 : j<right.length ? -1 : 0;
  }

  function canonicalString(value, depth = 0) {
    if (depth > 24) fail('canonical content nesting is too deep');
    if (value === null || typeof value === 'boolean' || typeof value === 'string') return JSON.stringify(value);
    if (typeof value === 'number') {
      if (!finite(value)) fail('canonical numbers must be finite');
      const bytes = new ArrayBuffer(8), view = new DataView(bytes);
      view.setFloat64(0, Object.is(value, -0) ? 0 : value, false);
      return JSON.stringify(`f64:${Array.from(new Uint8Array(bytes), b => b.toString(16).padStart(2, '0')).join('')}`);
    }
    if (Array.isArray(value)) return `[${value.map(item => canonicalString(item, depth + 1)).join(',')}]`;
    obj(value, 'canonical content');
    return `{${Object.keys(value).sort(codePointOrder).map(key => `${JSON.stringify(key)}:${canonicalString(value[key], depth + 1)}`).join(',')}}`;
  }

  function canonicalDigest(map) {
    obj(map, 'canonical map');
    const content = {};
    for (const key of Object.keys(map)) if (key !== 'approval') Object.defineProperty(content, key, {value: map[key], enumerable: true});
    return sha256(new TextEncoder().encode(canonicalString(content)));
  }

  const equal = (a, b) => a[0] === b[0] && a[1] === b[1];
  const cross = (a, b, c) => (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]);
  function onSegment(a, b, p) {
    return Math.abs(cross(a,b,p)) <= EPS && p[0] >= Math.min(a[0],b[0])-EPS && p[0] <= Math.max(a[0],b[0])+EPS &&
      p[1] >= Math.min(a[1],b[1])-EPS && p[1] <= Math.max(a[1],b[1])+EPS;
  }
  function intersect(a,b,c,d) {
    const abC=cross(a,b,c), abD=cross(a,b,d), cdA=cross(c,d,a), cdB=cross(c,d,b);
    return ((abC > EPS && abD < -EPS || abC < -EPS && abD > EPS) &&
            (cdA > EPS && cdB < -EPS || cdA < -EPS && cdB > EPS)) ||
      onSegment(a,b,c) || onSegment(a,b,d) || onSegment(c,d,a) || onSegment(c,d,b);
  }
  function inside(point, ring) {
    let result = false;
    for (let i=0; i<ring.length-1; i++) {
      const a=ring[i], b=ring[i+1];
      if (onSegment(a,b,point)) return false; // Holes must be strictly contained.
      if ((a[1] > point[1]) !== (b[1] > point[1]) && point[0] < (b[0]-a[0])*(point[1]-a[1])/(b[1]-a[1])+a[0]) result = !result;
    }
    return result;
  }
  function ringsIntersect(a,b) {
    for (let i=0; i<a.length-1; i++) for (let j=0; j<b.length-1; j++)
      if (intersect(a[i],a[i+1],b[j],b[j+1])) return true;
    return false;
  }
  function insideOrOn(point, ring) {
    return inside(point,ring) || ring.some((a,i)=>i<ring.length-1 && onSegment(a,ring[i+1],point));
  }
  function segmentContained(a,b,ring) {
    const cuts=[0,1], dx=b[0]-a[0], dy=b[1]-a[1], length2=dx*dx+dy*dy;
    for (let i=0; i<ring.length-1; i++) {
      const c=ring[i],d=ring[i+1],ac=cross(a,b,c),ad=cross(a,b,d),ca=cross(c,d,a),cb=cross(c,d,b);
      if ((ac>EPS && ad<-EPS || ac<-EPS && ad>EPS) && (ca>EPS && cb<-EPS || ca<-EPS && cb>EPS)) return false;
      if (onSegment(a,b,c)) cuts.push(((c[0]-a[0])*dx+(c[1]-a[1])*dy)/length2);
    }
    cuts.sort((x,y)=>x-y);
    return cuts.slice(1).every((end,i)=>{
      const t=(cuts[i]+end)/2;
      return insideOrOn([a[0]+t*dx,a[1]+t*dy],ring);
    });
  }
  function ring(value, name, ccw, field, budget) {
    if (!Array.isArray(value) || value.length < 4 || value.length > 513) fail(`${name} must be a closed ring with 3 to 512 vertices`);
    budget.count += value.length - 1;
    if (budget.count > 4096) fail('geometry exceeds 4096 vertices');
    for (const point of value) {
      pair(point, name);
      number(point[0], `${name} X`, 0, field.width_m);
      number(point[1], `${name} Y`, 0, field.height_m);
    }
    if (!equal(value[0], value[value.length-1])) fail(`${name} must explicitly close`);
    const count=value.length-1, seen=new Set();
    for (let i=0; i<count; i++) {
      const key=`${value[i][0]},${value[i][1]}`;
      if (seen.has(key)) fail(`${name} has duplicate vertices`);
      seen.add(key);
      if (equal(value[i], value[i+1])) fail(`${name} has a zero-length edge`);
      for (let j=i+1; j<count; j++) {
        if (j === i+1 || i === 0 && j === count-1) continue;
        if (intersect(value[i],value[i+1],value[j],value[j+1])) fail(`${name} self-intersects`);
      }
      const previous=value[(i+count-1)%count], current=value[i], next=value[(i+1)%count];
      if (Math.abs(cross(previous,current,next)) <= EPS &&
          (current[0]-previous[0])*(next[0]-current[0])+(current[1]-previous[1])*(next[1]-current[1]) < 0)
        fail(`${name} has overlapping adjacent edges`);
    }
    let twiceArea=0;
    for (let i=0; i<count; i++) twiceArea += value[i][0]*value[i+1][1]-value[i+1][0]*value[i][1];
    if (ccw ? twiceArea <= EPS : twiceArea >= -EPS) fail(`${name} must be nondegenerate and ${ccw ? 'counterclockwise' : 'clockwise'}`);
  }
  function review(value, name, revision) {
    obj(value,name,['state','reviewed_revision']);
    if (!['draft','approved'].includes(value.state)) fail(`${name}.state must be draft or approved`);
    if (value.reviewed_revision !== null && !integer(value.reviewed_revision)) fail(`${name}.reviewed_revision must be a positive integer or null`);
    return value.state === 'approved' && value.reviewed_revision === revision;
  }
  function provenance(value, name) {
    obj(value,name,['kind','label','uri']);
    if (!['suggested','manual','verified_source'].includes(value.kind)) fail(`${name}.kind is unsupported`);
    text(value.label,`${name}.label`); text(value.uri,`${name}.uri`,2048,true);
  }
  function polygon(value, name, field, budget, obstacle) {
    obj(value,name,obstacle ? ['id','outer','holes','review','provenance','vertical_range_m'] : ['outer','holes','review','provenance']);
    if (obstacle) { text(value.id,`${name}.id`); if (!value.id) fail(`${name}.id must not be empty`); }
    ring(value.outer,`${name}.outer`,true,field,budget);
    if (!Array.isArray(value.holes) || value.holes.length > 32) fail(`${name}.holes must be an array of at most 32 rings`);
    value.holes.forEach((hole,index) => {
      ring(hole,`${name}.holes[${index}]`,false,field,budget);
      if (!hole.slice(0,-1).every(point => inside(point,value.outer)) || ringsIntersect(hole,value.outer)) fail(`${name} hole must be strictly inside its outer ring`);
      for (const earlier of value.holes.slice(0,index))
        if (ringsIntersect(hole,earlier) || inside(hole[0],earlier) || inside(earlier[0],hole)) fail(`${name} holes overlap or nest`);
    });
    const reviewed=review(value.review,`${name}.review`,field.revision);
    provenance(value.provenance,`${name}.provenance`);
    if (obstacle && value.vertical_range_m !== null) {
      obj(value.vertical_range_m,`${name}.vertical_range_m`,['min','max']);
      number(value.vertical_range_m.min,`${name} minimum Z`,-1000,1000);
      number(value.vertical_range_m.max,`${name} maximum Z`,-1000,1000);
      if (value.vertical_range_m.max < value.vertical_range_m.min) fail(`${name} maximum Z must not be below minimum Z`);
      if (value.provenance.kind === 'suggested') fail(`${name} image suggestions cannot establish vertical geometry`);
    }
    return reviewed;
  }

  function validateFieldMap(parsed, wpilibLayoutOrNull = null) {
    obj(parsed,'root',['schema_version','map','image','source','calibration','boundary','obstacles','approval']);
    if (parsed.schema_version !== 'frc-field-map/1') fail('unsupported schema_version');
    const field=obj(parsed.map,'map',['id','revision','season','variant','frame','units','width_m','height_m']);
    text(field.id,'map.id'); text(field.variant,'map.variant'); text(field.season,'map.season',256,true);
    if (!field.id || !integer(field.revision)) fail('map needs a nonempty ID and positive integer revision');
    if (field.frame !== 'wpilib_nwu' || field.units !== 'm') fail('map frame must be fixed wpilib_nwu in meters');
    number(field.width_m,'map.width_m',0.01,1000); number(field.height_m,'map.height_m',0.01,1000);
    const image=obj(parsed.image,'image',['file_name','sha256','width_px','height_px','mime_type','attribution','license']);
    text(image.file_name,'image.file_name',128);
    const extension=/^[A-Za-z0-9][A-Za-z0-9_.-]*\.(png|jpe?g|webp)$/i.exec(image.file_name);
    if (!extension || image.file_name.includes('..')) fail('image.file_name must be a safe PNG/JPEG/WebP basename');
    if (!hex(image.sha256)) fail('image.sha256 must be 64 lowercase hex digits');
    number(image.width_px,'image.width_px',1,16384); number(image.height_px,'image.height_px',1,16384);
    if (!Number.isInteger(image.width_px) || !Number.isInteger(image.height_px)) fail('image dimensions must be integers');
    const mime=extension[1].toLowerCase()==='png' ? 'image/png' : extension[1].toLowerCase()==='webp' ? 'image/webp' : 'image/jpeg';
    if (image.mime_type !== mime) fail('image MIME type must match its filename');
    text(image.attribution,'image.attribution',2048,true); text(image.license,'image.license',512,true);
    const source=obj(parsed.source,'source',['kind','label','uri']);
    if (!['synthetic','user_image','official_geometry'].includes(source.kind)) fail('source.kind is unsupported');
    text(source.label,'source.label'); text(source.uri,'source.uri',2048,true);
    const calibration=obj(parsed.calibration,'calibration',['model','image_to_field','control_points','distortion','fit_error_m','independent_check_error_m','independent_check_points']);
    if (calibration.model !== 'affine') fail('only affine planar calibration is supported');
    const matrix=calibration.image_to_field;
    if (!Array.isArray(matrix) || matrix.length !== 9 || !matrix.every(finite) || matrix[6] !== 0 || matrix[7] !== 0 || matrix[8] !== 1)
      fail('calibration.image_to_field must be a finite affine row-major 3x3 matrix');
    if (Math.abs(matrix[0]*matrix[4]-matrix[1]*matrix[3]) < 1e-15) fail('calibration transform must be invertible');
    if (!['uncorrected','corrected','not_applicable'].includes(calibration.distortion)) fail('calibration.distortion is unsupported');
    for (const key of ['fit_error_m','independent_check_error_m'])
      if (calibration[key] !== null) number(calibration[key],`calibration.${key}`,0,Number.MAX_VALUE);
    for (const key of ['control_points','independent_check_points']) {
      const points=calibration[key];
      if (!Array.isArray(points) || points.length > 128) fail(`calibration.${key} must be a bounded points array`);
      for (const point of points) {
        obj(point,`calibration.${key} point`,['pixel','field_m']); pair(point.pixel,'pixel'); pair(point.field_m,'field_m');
        number(point.pixel[0],'pixel X',0,image.width_px); number(point.pixel[1],'pixel Y',0,image.height_px);
        number(point.field_m[0],'control field X',0,field.width_m); number(point.field_m[1],'control field Y',0,field.height_m);
      }
    }
    if (calibration.control_points.length < 3) fail('affine calibration needs at least three control points');
    const pixels=calibration.control_points.map(point=>point.pixel);
    if (!pixels.some((point,i)=>pixels.slice(i+1).some((other,j)=>pixels.slice(i+j+2).some(third=>Math.abs(cross(point,other,third))>EPS))))
      fail('affine control points must not be collinear');
    const mapPoint=([x,y])=>[matrix[0]*x+matrix[1]*y+matrix[2],matrix[3]*x+matrix[4]*y+matrix[5]];
    // The shared contract defines RMS of Euclidean residuals, not per-axis RMS,
    // maximum residual, image pixels, or an accuracy acceptance threshold.
    const residualRms=points=>{
      if (!points.length) return null;
      const residuals=points.map(point=>{
        const projected=mapPoint(point.pixel);
        return Math.hypot(projected[0]-point.field_m[0],projected[1]-point.field_m[1]);
      });
      const rms=Math.hypot(...residuals)/Math.sqrt(points.length);
      if (!finite(rms)) fail('calibration has unusable affine projected residuals');
      return rms;
    };
    const fitRms=residualRms(calibration.control_points),checkRms=residualRms(calibration.independent_check_points);
    for (const [key,actual] of [['fit_error_m',fitRms],['independent_check_error_m',checkRms]])
      if (calibration[key] !== null && actual !== null && Math.abs(calibration[key]-actual)>1e-8)
        fail(`calibration.${key} disagrees with affine RMS residual by more than 1e-8 m`);
    const sameLocation=(a,b)=>Math.abs(a[0]-b[0])<=EPS && Math.abs(a[1]-b[1])<=EPS;
    // Reusing either a fitted image location or its metric landmark is not an
    // independent check, even if the other half of its correspondence changes.
    const hasDistinctCheck=calibration.independent_check_points.some(check=>calibration.control_points.every(control=>
      !sameLocation(check.pixel,control.pixel) && !sameLocation(check.field_m,control.field_m)));
    const corners=[[0,0],[image.width_px,0],[image.width_px,image.height_px],[0,image.height_px]].map(mapPoint);
    if (!corners.every(point=>point.every(finite) && point.every(value=>Math.abs(value)<=10000))) fail('image calibration corners are unusable');
    const budget={count:0}, boundaryReviewed=polygon(parsed.boundary,'boundary',field,budget,false);
    if (!Array.isArray(parsed.obstacles) || parsed.obstacles.length > 128) fail('obstacles must be a bounded array');
    const ids=new Set(), reviewed=parsed.obstacles.map((obstacle,index)=>{
      const okay=polygon(obstacle,`obstacles[${index}]`,field,budget,true);
      if (ids.has(obstacle.id)) fail(`duplicate obstacle ID ${obstacle.id}`);
      ids.add(obstacle.id);
      // Obstacles must be inside the physical field boundary, outside its holes.
      if (!obstacle.outer.slice(0,-1).every((point,i)=>insideOrOn(point,parsed.boundary.outer) &&
          segmentContained(point,obstacle.outer[i+1],parsed.boundary.outer))) fail(`obstacles[${index}] lies outside the boundary`);
      for (const hole of parsed.boundary.holes)
        if (ringsIntersect(obstacle.outer,hole) || inside(obstacle.outer[0],hole) || inside(hole[0],obstacle.outer)) fail(`obstacles[${index}] overlaps a boundary hole`);
      return okay;
    });
    const approval=obj(parsed.approval,'approval',['state','reviewed_revision','content_sha256']);
    const revisionApproved=review({state:approval.state,reviewed_revision:approval.reviewed_revision},'approval',field.revision);
    if (approval.content_sha256 !== null && !hex(approval.content_sha256)) fail('approval.content_sha256 must be lowercase SHA-256 or null');
    const warnings=[];
    const calibrationComplete=calibration.fit_error_m !== null && calibration.independent_check_error_m !== null &&
      hasDistinctCheck;
    const approved=revisionApproved && approval.content_sha256 === canonicalDigest(parsed) && boundaryReviewed && reviewed.every(Boolean) && calibrationComplete;
    if (!approved) warnings.push(approval.state === 'approved' ? 'Approval is stale or incomplete; geometry remains draft and heights are withheld.' : 'Geometry is draft; reviewed physical dimensions and explicit approval are required.');
    if (!calibrationComplete)
      warnings.push('Approval requires provided finite fit error, independent check error and a check point distinct from fitted image and metric controls; image calibration is not physical verification.');
    warnings.push(`Provided image fit error: ${calibration.fit_error_m === null ? 'unknown' : calibration.fit_error_m + ' m'}; independent check error: ${calibration.independent_check_error_m === null ? 'unknown' : calibration.independent_check_error_m + ' m'}. These values are configuration metadata, not hardware qualification.`);
    warnings.push(`Computed affine RMS fit: ${fitRms} m; independent check: ${checkRms === null ? 'unavailable' : checkRms + ' m'}. Declared nonnull values with points must agree within 1e-8 m; no accuracy acceptance threshold is inferred.`);
    if (source.kind === 'synthetic') warnings.push('Synthetic map; not a competition field or hardware measurement.');
    let bindingValid=true;
    if (wpilibLayoutOrNull !== null) {
      const dimensions=wpilibLayoutOrNull?.field;
      bindingValid=!!dimensions && finite(dimensions.length) && finite(dimensions.width) &&
        Math.abs(dimensions.length-field.width_m) <= 1e-6 && Math.abs(dimensions.width-field.height_m) <= 1e-6;
      if (!bindingValid) warnings.push('Map dimensions do not match the loaded WPILib layout; no field overlay is permitted.');
    } else warnings.push('No WPILib layout is loaded; map uses its declared fixed NWU origin without tag binding.');
    const copied=JSON.parse(JSON.stringify(parsed));
    return {map:copied,field:{length:field.width_m,width:field.height_m},
      geometry:{boundary:{vertices_m:copied.boundary.outer,holes_m:copied.boundary.holes,reviewed:boundaryReviewed},
        obstacles:copied.obstacles.map((obstacle,index)=>({id:obstacle.id,vertices_m:obstacle.outer,holes_m:obstacle.holes,reviewed:reviewed[index],
          base_z_m:approved && obstacle.vertical_range_m !== null ? obstacle.vertical_range_m.min : null,
          height_m:approved && obstacle.vertical_range_m !== null ? obstacle.vertical_range_m.max-obstacle.vertical_range_m.min : null}))},
      background:{file:image.file_name,width:image.width_px,height:image.height_px,sha256:image.sha256,corners},
      status:!bindingValid ? 'layout_mismatch' : approved ? 'approved' : 'draft',warnings,bindingValid,approved};
  }

  return {validateFieldMap,sha256,canonicalDigest};
}));
