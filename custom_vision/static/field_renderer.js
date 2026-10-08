/* Local field visualization. Coordinates stay WPILib NWU meters throughout.
 * Glyph sizes and pose axes are display aids, not measured physical footprints.
 * No network requests, inferred obstacle heights, odometry, or pose solving.
 */
(function (root, factory) {
  'use strict';
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.CVFieldRenderer = api;
})(typeof globalThis === 'object' ? globalThis : this, function () {
  'use strict';
  const FRAME_MS = 1000 / 15;
  const COLORS = {background:'#101a21', grid:'#263741', border:'#57717f', text:'#dbe6ed', muted:'#8ba1ae',
    robot:'#65e7b1', camera:'#f1bd84', observed:'#85c7ff', tag:'#8295a1', x:'#e39870', y:'#65e7b1', z:'#85c7ff'};
  const finite = n => typeof n === 'number' && Number.isFinite(n);
  const clamp = (n, lo, hi) => Math.max(lo, Math.min(hi, n));
  const dot = (a, b) => a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
  const add = (a, b) => a.map((v, i) => v+b[i]);
  const sub = (a, b) => a.map((v, i) => v-b[i]);
  function point(value) {
    return Array.isArray(value) && value.length >= 2 && value.slice(0, 2).every(finite) &&
      (value.length < 3 || finite(value[2])) ? [value[0], value[1], value[2] || 0] : null;
  }
  function normalizePose(pose) {
    if (!pose || (pose.frame && pose.frame !== 'wpilib_nwu')) return null;
    const t = pose.translation_m, q = pose.rotation_quaternion_wxyz;
    if (!Array.isArray(t) || t.length !== 3 || !t.every(finite) ||
        !Array.isArray(q) || q.length !== 4 || !q.every(finite)) return null;
    const norm = Math.hypot(...q);
    if (!finite(norm) || norm < 1e-12) return null;
    return {translation_m:t.slice(), rotation_quaternion_wxyz:q.map(v => v/norm), frame:'wpilib_nwu'};
  }
  function rotateVector(quaternion, vector) {
    // WXYZ quaternion; no conversion through display Euler angles.
    const [w,x,y,z] = quaternion, [vx,vy,vz] = vector;
    const tx = 2*(y*vz-z*vy), ty = 2*(z*vx-x*vz), tz = 2*(x*vy-y*vx);
    return [vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz, vz+w*tz+x*ty-y*tx];
  }
  function poseAxes(pose, length = .45) {
    const valid = normalizePose(pose);
    if (!valid || !finite(length) || length <= 0) return null;
    return {origin:valid.translation_m, x:add(valid.translation_m, rotateVector(valid.rotation_quaternion_wxyz,[length,0,0])),
      y:add(valid.translation_m, rotateVector(valid.rotation_quaternion_wxyz,[0,length,0])),
      z:add(valid.translation_m, rotateVector(valid.rotation_quaternion_wxyz,[0,0,length]))};
  }
  function projection(view, width, height) {
    const center = [width/2,height/2], focal = Math.max(1, Math.min(width,height)*.92);
    const target = view.target || [0,0,0];
    if (view.mode === '2d') {
      const scale = focal / view.distance * view.zoom;
      return p => {
        const x=center[0]+(p[0]-target[0])*scale,y=center[1]-(p[1]-target[1])*scale;
        return {x,y,depth:0,scale,visible:finite(x) && finite(y)};
      };
    }
    const a=view.azimuth, e=view.elevation, ce=Math.cos(e), se=Math.sin(e);
    const forward=[-Math.cos(a)*ce,-Math.sin(a)*ce,-se];
    const right=[-Math.sin(a),Math.cos(a),0], up=[-Math.cos(a)*se,-Math.sin(a)*se,ce];
    const distance = view.distance / view.zoom;
    return p => {
      const relative=sub(p,target), depth=distance+dot(relative,forward);
      const x=center[0]+focal*dot(relative,right)/depth,y=center[1]-focal*dot(relative,up)/depth;
      return {x,y,depth,scale:focal/depth,visible:finite(x) && finite(y) && finite(depth) && depth > Math.max(.025,distance*.015)};
    };
  }
  function fieldBounds(field) {
    return field && finite(field.length) && finite(field.width) && field.length > 0 && field.width > 0 &&
      field.length <= 1000 && field.width <= 1000 ? {length:field.length,width:field.width} : null;
  }
  function homeView(scene, mode) {
    const field=fieldBounds(scene.field);
    // This default defines the viewport only, never an invented field boundary.
    return {mode, target:field ? [field.length/2,field.width/2,0] : [0,0,0],
      distance:field ? Math.max(field.length,field.width)*.85+1 : 6,
      azimuth:-2.2,elevation:.83,zoom:1};
  }
  function polygonData(geometry) {
    if (!geometry) return [];
    const polygons = geometry.obstacles || [];
    if (!Array.isArray(polygons)) return [];
    let budget=8192;
    return polygons.slice(0,128).map(p => {
      const vertices = p.vertices_m;
      if (!Array.isArray(vertices) || vertices.length < 3 || vertices.length > 513) return null;
      const points=vertices.map(point);
      if (points.some(v => !v)) return null;
      const holes=Array.isArray(p.holes_m)?p.holes_m.slice(0,32):[];
      if (holes.some(h => !Array.isArray(h) || h.length < 3 || h.length > 513)) return null;
      const rings=holes.map(h => h.map(point));
      if (rings.some(h => h.some(v => !v))) return null;
      budget-=points.length+rings.reduce((n,h) => n+h.length,0);
      if (budget < 0) return null;
      const reviewed=p.reviewed === true;
      const measuredInterval=reviewed && finite(p.base_z_m) && finite(p.height_m) && p.height_m >= 0;
      return {points,holes:rings,reviewed,base:measuredInterval?p.base_z_m:null,height:measuredInterval?p.height_m:null,
        label:String(p.name || p.id || 'Obstacle').slice(0,48)};
    }).filter(Boolean);
  }
  function trailSegments(points) {
    if (!Array.isArray(points)) return [];
    const paths=[];let path=[];
    points.slice(-160).forEach(p => {
      if (!p || p.break === true) {if (path.length) paths.push(path);path=[];}
      const position=p && point(p.translation_m);
      if (position) path.push(position);
      else {if (path.length) paths.push(path);path=[];}
    });
    if (path.length) paths.push(path);
    return paths;
  }
  function createRenderer(canvas, options = {}) {
    if (!canvas || typeof canvas.getContext !== 'function') throw new TypeError('A canvas is required');
    const ctx=canvas.getContext('2d');
    if (!ctx) throw new Error('Canvas2D is unavailable');
    const host=typeof globalThis === 'object' ? globalThis : {};
    const now=options.now || (() => host.performance ? host.performance.now() : Date.now());
    const setTimer=options.setTimer || ((fn,delay) => host.setTimeout(fn,delay));
    const clearTimer=options.clearTimer || (id => host.clearTimeout(id));
    let scene={}, view=homeView(scene,'2d'), width=640,height=400, dirty=true,destroyed=false;
    let pending=null,lastDraw=-Infinity,drag=null,drawCount=0;
    const listeners=[];
    const on=(name,fn) => {canvas.addEventListener(name,fn); listeners.push([name,fn]);};
    function requestDraw() {
      if (destroyed) return;
      dirty=true;
      if (pending !== null) return;
      pending=setTimer(() => {
        pending=null;
        if (!destroyed && dirty) {
          if (now()-lastDraw < FRAME_MS-.01) {requestDraw(); return;}
          dirty=false; lastDraw=now(); render(); drawCount++;
        }
      },Math.max(0,FRAME_MS-(now()-lastDraw)));
    }
    function resize() {
      const rect=canvas.getBoundingClientRect ? canvas.getBoundingClientRect() : {width:canvas.width,height:canvas.height};
      width=Math.max(1,finite(rect.width) && rect.width > 0 ? rect.width : 640);
      height=Math.max(1,finite(rect.height) && rect.height > 0 ? rect.height : 400);
      const ratio=Math.min(clamp(finite(host.devicePixelRatio) ? host.devicePixelRatio : 1,1,2),
        4096/width,4096/height,Math.sqrt(8_000_000)/Math.sqrt(width)/Math.sqrt(height));
      if (canvas.width !== Math.max(1,Math.round(width*ratio)) || canvas.height !== Math.max(1,Math.round(height*ratio))) {
        canvas.width=Math.max(1,Math.round(width*ratio)); canvas.height=Math.max(1,Math.round(height*ratio));
      }
      ctx.setTransform(ratio,0,0,ratio,0,0);
    }
    function line(points,color,lineWidth=1,dashed=false,fill=null) {
      if (!points.length || points.some(p => !p.visible || !finite(p.x) || !finite(p.y))) return;
      ctx.beginPath();ctx.moveTo(points[0].x,points[0].y);
      points.slice(1).forEach(p => ctx.lineTo(p.x,p.y));
      ctx.strokeStyle=color;ctx.lineWidth=lineWidth;ctx.setLineDash(dashed?[5,5]:[]);
      if (fill) {ctx.closePath();ctx.fillStyle=fill;ctx.fill();}
      ctx.stroke();ctx.setLineDash([]);
    }
    function text(label,p,color=COLORS.muted,align='left') {
      if (!p.visible || !finite(p.x) || !finite(p.y)) return;
      ctx.font='11px ui-monospace, SFMono-Regular, Consolas, monospace';ctx.textAlign=align;
      ctx.fillStyle=color;ctx.fillText(String(label).slice(0,96),p.x,p.y);
    }
    function axes(pose,project,length=.45,labels=false,alpha=1) {
      const a=poseAxes(pose,length);
      if (!a) return;
      ctx.save();ctx.globalAlpha*=alpha;
      ['x','y','z'].forEach(key => {
        if (view.mode === '2d' && key === 'z') return;
        const p=project(a[key]);line([project(a.origin),p],COLORS[key],1.5);
        if (labels) text('+'+key.toUpperCase(),{...p,x:p.x+4,y:p.y-4},COLORS[key]);
      });ctx.restore();
    }
    function grid(project) {
      const field=fieldBounds(scene.field), extent=view.distance/view.zoom*1.6;
      const bounds=field ? [0,field.length,0,field.width] : [view.target[0]-extent,view.target[0]+extent,view.target[1]-extent,view.target[1]+extent];
      const span=Math.max(bounds[1]-bounds[0],bounds[3]-bounds[2]);
      const step=span > 50 ? 10**Math.floor(Math.log10(span/20)) : 1;
      for (let x=Math.ceil(bounds[0]/step)*step,n=0;x<=bounds[1]+1e-9 && n<100;x+=step,n++)
        line([project([x,bounds[2],0]),project([x,bounds[3],0])],COLORS.grid);
      for (let y=Math.ceil(bounds[2]/step)*step,n=0;y<=bounds[3]+1e-9 && n<100;y+=step,n++)
        line([project([bounds[0],y,0]),project([bounds[1],y,0])],COLORS.grid);
      if (field) {
        const boundary=[[0,0,0],[field.length,0,0],[field.length,field.width,0],[0,field.width,0],[0,0,0]];
        line(boundary.map(project),COLORS.border,1.6);
        text(`${field.length.toFixed(2)} m`,{...project([field.length/2,0,0]),y:project([field.length/2,0,0]).y+18},COLORS.muted,'center');
      }
      axes({translation_m:[0,0,0],rotation_quaternion_wxyz:[1,0,0,0]},project,Math.max(.5,Math.min(1.5,view.distance*.1)),true);
      text('Origin',{...project([0,0,0]),x:project([0,0,0]).x+6,y:project([0,0,0]).y+16});
    }
    function imageTriangle(image,a,b,c,pa,pb,pc) {
      if (![pa,pb,pc].every(p => p.visible)) return;
      const det=a[0]*(b[1]-c[1])+b[0]*(c[1]-a[1])+c[0]*(a[1]-b[1]);
      if (Math.abs(det)<1e-12) return;
      const coefficients=(u,v,w) => [(u*(b[1]-c[1])+v*(c[1]-a[1])+w*(a[1]-b[1]))/det,
        (u*(c[0]-b[0])+v*(a[0]-c[0])+w*(b[0]-a[0]))/det,
        (u*(b[0]*c[1]-c[0]*b[1])+v*(c[0]*a[1]-a[0]*c[1])+w*(a[0]*b[1]-b[0]*a[1]))/det];
      const tx=coefficients(pa.x,pb.x,pc.x),ty=coefficients(pa.y,pb.y,pc.y);
      ctx.save();ctx.beginPath();ctx.moveTo(pa.x,pa.y);ctx.lineTo(pb.x,pb.y);ctx.lineTo(pc.x,pc.y);ctx.closePath();ctx.clip();
      ctx.transform(tx[0],ty[0],tx[1],ty[1],tx[2],ty[2]);ctx.drawImage(image,0,0);ctx.restore();
    }
    function background(project) {
      const bg=scene.backgroundImage, image=bg && bg.image;
      if (!image || !image.width || !image.height || !Array.isArray(bg.corners) || bg.corners.length !== 4) return;
      const corners=bg.corners.map(point).map(p => p && [p[0],p[1],0]);
      if (corners.some(p => !p)) return;
      const world=(u,v) => corners[0].map((_,i) => corners[0][i]*(1-u)*(1-v)+corners[1][i]*u*(1-v)+corners[2][i]*u*v+corners[3][i]*(1-u)*v);
      const n=view.mode==='3d'?8:1;
      ctx.save();ctx.globalAlpha=.65;
      for (let y=0;y<n;y++) for (let x=0;x<n;x++) {
        const uv=[[x/n,y/n],[(x+1)/n,y/n],[(x+1)/n,(y+1)/n],[x/n,(y+1)/n]];
        const px=uv.map(([u,v]) => [u*image.width,v*image.height]), p=uv.map(([u,v]) => project(world(u,v)));
        imageTriangle(image,px[0],px[1],px[2],p[0],p[1],p[2]);imageTriangle(image,px[0],px[2],px[3],p[0],p[2],p[3]);
      }ctx.restore();
    }
    function polygons(project) {
      const boundary=scene.geometry && scene.geometry.boundary;
      if (boundary && Array.isArray(boundary.vertices_m)) {
        [boundary.vertices_m,...(Array.isArray(boundary.holes_m)?boundary.holes_m.slice(0,32):[])].forEach(r => {
          if (!Array.isArray(r) || r.length < 3 || r.length > 512) return;
          const ring=r.map(point);if (ring.some(p => !p)) return;
          line([...ring,ring[0]].map(p => project([p[0],p[1],0])),COLORS.border,2,boundary.reviewed!==true);
        });
      }
      polygonData(scene.geometry).forEach(p => {
        // An uncertain planar footprint is shown on the map plane, never as a solid.
        const rings=[p.points,...p.holes].map(r => r.map(v => [v[0],v[1],p.base===null?0:p.base]));
        const ring=rings[0],color=p.reviewed?COLORS.border:COLORS.observed;
        function surface(paths,fill) {
          const projected=paths.map(r => r.map(project));
          if (projected.some(r => r.some(v => !v.visible))) return;
          ctx.beginPath();
          projected.forEach(r => {ctx.moveTo(r[0].x,r[0].y);r.slice(1).forEach(v => ctx.lineTo(v.x,v.y));ctx.closePath();});
          if (fill) {ctx.fillStyle=fill;ctx.fill('evenodd');}
          ctx.strokeStyle=color;ctx.lineWidth=1.6;ctx.setLineDash(!p.reviewed || p.height===null?[5,5]:[]);ctx.stroke();ctx.setLineDash([]);
        }
        surface(rings,p.reviewed && p.height!==null?'#34495933':null);
        if (view.mode==='3d' && p.height !== null && p.height > 0) {
          const tops=rings.map(r => r.map(v => [v[0],v[1],p.base+p.height]));
          rings.forEach((r,index) => {
            for (let i=0;i<r.length;i++) {
              const j=(i+1)%r.length;
              line([r[i],r[j],tops[index][j],tops[index][i]].map(project),color,1,false,'#34495955');
            }
          });
          surface(tops,'#57717f44');
        }
        const center=ring.reduce((out,p) => out.map((v,i) => v+p[i]/ring.length),[0,0,0]);
        center[2]=view.mode==='3d' && p.height !== null ? p.base+p.height : 0;
        text(`${p.label} · ${!p.reviewed?'draft':p.height===null?'z / height unknown':p.height+' m'}`,project(center),color,'center');
      });
    }
    function trail(points,color,project,current) {
      ctx.save();ctx.globalAlpha=current?.65:.28;
      trailSegments(points).forEach(path => line(path.map(project),color,1.5,!current));ctx.restore();
    }
    function marker(kind,source,project) {
      const state=source[kind] || {}, pose=normalizePose(state.pose || source[kind+'Pose']);
      if (!pose || !['valid','stale'].includes(state.status)) return;
      const p=project(pose.translation_m), color=COLORS[kind], stale=state.status==='stale';
      if (!p.visible) return;
      ctx.save();ctx.globalAlpha=stale?.4:1;ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=1.8;
      ctx.setLineDash(stale?[3,3]:[]);ctx.beginPath();
      if (kind==='robot') ctx.arc(p.x,p.y,6,0,Math.PI*2);
      else ctx.rect(p.x-5,p.y-4,10,8);
      ctx.stroke();ctx.setLineDash([]);
      axes(pose,project,.45,false,1);
      text(`${String(source.pipeline || source.sourceId || '').slice(0,24)} · vision ${kind}${stale?' · stale':''}`,
        {...p,x:p.x+10,y:p.y+(kind==='camera'?-9:13)},color);
      if (view.mode==='2d') text(`z ${pose.translation_m[2].toFixed(2)} m`,{...p,x:p.x+10,y:p.y+(kind==='camera'?-23:27)},COLORS.muted);
      ctx.restore();
    }
    function mountLink(source,project) {
      const camera=source.camera,robot=source.robot;
      if (!camera || !robot || camera.status!==robot.status || !['valid','stale'].includes(camera.status)) return;
      const cameraPose=normalizePose(camera.pose),robotPose=normalizePose(robot.pose);
      if (!cameraPose || !robotPose) return;
      ctx.save();ctx.globalAlpha=camera.status==='stale'?.2:.6;
      line([project(cameraPose.translation_m),project(robotPose.translation_m)],COLORS.muted,1,true);
      ctx.restore();
    }
    function tags(project,sources) {
      const observed=new Set(),used=new Set();
      sources.forEach(s => {
        // A rejected localization can still contain observed tag detections.
        // The display model has already cleared IDs on source invalidation.
        if (s.status==='stale') return;
        (s.observedTagIds || []).forEach(id => observed.add(id));(s.usedTagIds || []).forEach(id => used.add(id));
      });
      const tags=Array.isArray(scene.tags)?scene.tags:(scene.field && Array.isArray(scene.field.tags)?scene.field.tags:[]);
      tags.slice(0,512).map(tag => ({tag,pose:normalizePose(tag.pose)})).filter(t => t.pose)
        .sort((a,b) => project(b.pose.translation_m).depth-project(a.pose.translation_m).depth).forEach(({tag,pose}) => {
          const p=project(pose.translation_m),color=used.has(tag.id)?COLORS.robot:observed.has(tag.id)?COLORS.observed:COLORS.tag;
          if (!p.visible) return;
          ctx.save();ctx.strokeStyle=color;ctx.lineWidth=used.has(tag.id)?2:1;ctx.strokeRect(p.x-5,p.y-5,10,10);
          axes(pose,project,.25,false,observed.has(tag.id)?1:.5);text(String(tag.id),{...p,x:p.x+8,y:p.y+4},color);ctx.restore();
        });
    }
    function render() {
      resize();ctx.clearRect(0,0,width,height);ctx.fillStyle=COLORS.background;ctx.fillRect(0,0,width,height);
      const project=projection(view,width,height),sources=(Array.isArray(scene.sources)?scene.sources:[]).slice(0,32);
      background(project);grid(project);polygons(project);
      sources.forEach(s => {trail(s.cameraTrail,COLORS.camera,project,s.camera?.status==='valid');trail(s.robotTrail,COLORS.robot,project,s.robot?.status==='valid');});
      tags(project,sources);
      sources.forEach(s => mountLink(s,project));
      const markers=sources.flatMap(source => ['robot','camera'].map(kind => ({source,kind,pose:normalizePose(source[kind]?.pose || source[kind+'Pose'])})))
        .filter(m => m.pose).sort((a,b) => project(b.pose.translation_m).depth-project(a.pose.translation_m).depth);
      markers.forEach(m => marker(m.kind,m.source,project));
      if(options.decorations!==false){text(`${view.mode.toUpperCase()}  /  NWU METERS`,{x:16,y:24,visible:true},COLORS.text);
      if (!fieldBounds(scene.field)) text('Field geometry unavailable · metric grid only',{x:16,y:44,visible:true},COLORS.observed);
      text('Camera  •  Robot vision estimate  •  Tag: mapped / observed / used',{x:16,y:height-34,visible:true});
      text(view.mode==='3d'?'Drag orbit · Shift-drag pan · Wheel zoom · Home reset':'Drag pan · Wheel zoom · Home reset',{x:16,y:height-16,visible:true});}
    }
    function changed() {requestDraw();if (typeof options.onViewChange==='function') options.onViewChange({...view,target:view.target.slice()});}
    function pan(dx,dy) {
      const scale=Math.max(1,Math.min(width,height)*.92)/view.distance*view.zoom;
      if (view.mode==='2d') {view.target[0]-=dx/scale;view.target[1]+=dy/scale;}
      else {
        const right=[-Math.sin(view.azimuth),Math.cos(view.azimuth)],forward=[Math.cos(view.azimuth),Math.sin(view.azimuth)];
        view.target[0]-=(right[0]*dx+forward[0]*dy)/scale;
        view.target[1]-=(right[1]*dx+forward[1]*dy)/scale;
      }
    }
    on('pointerdown',e => {
      if (e.button!==0 && e.button!==2) return;
      drag={id:e.pointerId,x:e.clientX,y:e.clientY,pan:e.shiftKey || e.button===2 || view.mode==='2d'};
      if (canvas.setPointerCapture) canvas.setPointerCapture(e.pointerId);
      if (canvas.focus) canvas.focus();e.preventDefault();
    });
    on('pointermove',e => {
      if (!drag || drag.id!==e.pointerId) return;
      const dx=e.clientX-drag.x,dy=e.clientY-drag.y;drag.x=e.clientX;drag.y=e.clientY;
      if (drag.pan) pan(dx,dy);else {view.azimuth-=dx*.008;view.elevation=clamp(view.elevation+dy*.006,.12,1.5);}
      changed();
    });
    const release=e => {if (drag && drag.id===e.pointerId) {drag=null;if (canvas.releasePointerCapture && canvas.hasPointerCapture?.(e.pointerId)) canvas.releasePointerCapture(e.pointerId);}};
    on('pointerup',release);on('pointercancel',release);on('contextmenu',e => e.preventDefault());
    on('wheel',e => {e.preventDefault();view.zoom=clamp(view.zoom*Math.exp(-clamp(e.deltaY,-200,200)*.002),.25,20);changed();});
    on('keydown',e => {
      const arrows={ArrowLeft:[-20,0],ArrowRight:[20,0],ArrowUp:[0,-20],ArrowDown:[0,20]};
      if (e.key==='Home') view=homeView(scene,view.mode);
      else if (e.key==='+' || e.key==='=') view.zoom=clamp(view.zoom*1.15,.25,20);
      else if (e.key==='-') view.zoom=clamp(view.zoom/1.15,.25,20);
      else if (arrows[e.key]) {
        const [dx,dy]=arrows[e.key];
        if (view.mode==='2d' || e.shiftKey) pan(dx,dy);
        else {view.azimuth-=dx*.006;view.elevation=clamp(view.elevation+dy*.006,.12,1.5);}
      } else return;
      e.preventDefault();changed();
    });
    const observer=typeof host.ResizeObserver==='function'?new host.ResizeObserver(requestDraw):null;
    if (observer) observer.observe(canvas);
    requestDraw();
    return {setScene(next) {scene=next && typeof next==='object'?next:{};requestDraw();},
      setMode(mode) {if (!['2d','3d'].includes(mode)) throw new RangeError('Mode must be 2d or 3d');view.mode=mode;changed();},
      home() {view=homeView(scene,view.mode);changed();},draw:requestDraw,
      getView() {return {...view,target:view.target.slice()};},
      getStats() {return {drawCount,pending:pending!==null,mode:view.mode,maxFps:15};},
      destroy() {destroyed=true;if (pending!==null) clearTimer(pending);pending=null;
        if (observer) observer.disconnect();listeners.forEach(([name,fn]) => canvas.removeEventListener(name,fn));}};
  }
  return {createRenderer,normalizePose,rotateVector,poseAxes,projection,fieldBounds,homeView,polygonData,trailSegments};
});
