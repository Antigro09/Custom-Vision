/* Field display only: imports and replay stay in this browser session. */
(function(root) {
  'use strict';
  if (!root.CVFieldModel || !root.CVFieldRenderer || !root.CVFieldScene) return;
  const el=id=>document.getElementById(id), now=()=>performance.now();
  const live=root.CVFieldModel.createStore(), replay=root.CVFieldModel.createStore();
  const renderer=root.CVFieldRenderer.createRenderer(el('field-canvas'),{decorations:false});
  let selected=null, liveSelected=null, unavailable=false, context={field_layout:null,mounts:{}}, replayContext=null, replayPackets=null, replayIndex=0;
  let imported=null, mapCache=null, layoutCache={input:null,value:null}, localImage=null, imageURL=null, busy=false, mode='2d', timer=null;
  const finite=Number.isFinite, text=(id,value)=>{el(id).textContent=value;};
  function message(value) {text('field-message',value);el('field-message').hidden=!value;}
  function currentContext(){return replayPackets ? (replayContext || context) : context;}
  function activeStore(){return replayPackets ? replay : live;}
  function selectedSource(snapshot) {return snapshot.sources.find(s=>s.pipeline===selected)||snapshot.sources[0]||null;}
  function validateMap() {
    if (!imported) return null;
    const layout=currentContext().field_layout, key=layout?`${layout.field?.length}:${layout.field?.width}`:'unbound';
    if(!mapCache||mapCache.input!==imported||mapCache.key!==key)mapCache={input:imported,key,value:root.CVFieldScene.validateFieldMap(imported,layout)};
    return mapCache.value;
  }
  function poseText(pose) {
    if (!pose) return 'No accepted field pose';
    const p=pose.translation_m, q=pose.rotation_quaternion_wxyz;
    const yaw=Math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]))*180/Math.PI;
    return `X ${p[0].toFixed(2)} · Y ${p[1].toFixed(2)} · Z ${p[2].toFixed(2)} m / yaw ${yaw.toFixed(1)}°`;
  }
  function update() {
    const t=now(), snapshot=activeStore().snapshot(t);
    if(!replayPackets && unavailable) for(const item of snapshot.sources) {
      item.status='stale';item.reason='Dashboard connection unavailable';item.observedTagIds=[];item.usedTagIds=[];
      for(const family of ['camera','robot'])if(item[family].pose){item[family].status='stale';item[family].actionable=false;item[family].reason=item.reason;}
    }
    const source=selectedSource(snapshot);
    const layoutInput=currentContext().field_layout;
    if(layoutCache.input!==layoutInput)layoutCache={input:layoutInput,value:root.CVFieldModel.layoutScene(layoutInput)};
    const layout=layoutCache.value;let sceneMap=null;
    try {sceneMap=validateMap();}catch(error){message(error.message);}
    const binding=sceneMap?.bindingValid===true;
    const field=layout?.field || (binding ? sceneMap.field : null);
    let backgroundImage=null;
    if (binding && localImage && localImage.verified && sceneMap.background) {
      backgroundImage={image:localImage.image,corners:sceneMap.background.corners};
    }
    renderer.setScene({field,tags:layout?.tags||[],sources:el('field-show-all').checked?snapshot.sources:(source?[source]:[]),
      geometry:binding?sceneMap.geometry:null,backgroundImage});
    text('field-geometry-note',sceneMap && !binding ? 'Display map does not match runtime field dimensions' :
      field ? `${field.length.toFixed(2)} × ${field.width.toFixed(2)} m · ${layout?.tags?.length||0} configured tags${sceneMap?' · '+sceneMap.status:''}` : 'Field geometry unavailable · axes and metric grid only');
    if (sceneMap) text('field-map-status',`${sceneMap.map.map.id} · revision ${sceneMap.map.map.revision} · ${sceneMap.status}. Affine fit ${sceneMap.map.calibration.fit_error_m??'unknown'} m; independent check ${sceneMap.map.calibration.independent_check_error_m??'unknown'} m; distortion ${sceneMap.map.calibration.distortion}. ${sceneMap.warnings.join(' ')} ${localImage?.verified?'Matching local picture verified.':'Choose the matching local picture to display it.'}`);
    const replayLabel=replayPackets ? `REPLAY ${replayIndex+1}/${replayPackets.length} · ` : '';
    text('field-source-label',source ? `${replayLabel}${source.pipeline} · ${source.inputKind==='synthetic'?'synthetic fixture':source.inputKind==='camera'?'camera vision':'source '+source.inputKind} · separate estimate` : replayLabel+'Waiting for a vision observation');
    text('field-state',source?.status||'Awaiting data');el('field-state').className='badge '+(source?.status==='valid'?'live':source?.status==='stale'?'error':'');
    text('field-age',source&&finite(source.receiptAgeMs)?`${Math.round(source.receiptAgeMs)} ms receipt age`:'—');
    for(const family of ['camera','robot']) {
      const item=source?.[family];
      text('field-'+family+'-state',item?.status==='valid'?'Estimated':item?.status==='stale'?'Stale estimate': 'Unavailable');
      text('field-'+family+'-pose',item?.pose ? poseText(item.pose)+` · ${Math.round(item.receiptAgeMs)} ms measurement age`+(item.status==='stale'?' · historical only':'') : (item?.reason||`No field-to-${family} pose`));
    }
    const q=source?.quality;
    text('field-method',q?.method||'—');
    text('field-tags',source?`${source.usedTagIds.join(', ')||'none'} / ${source.observedTagIds.join(', ')||'none'}`:'—');
    text('field-quality',source?`${finite(q?.reprojectionErrorPx)?q.reprojectionErrorPx.toFixed(2)+' px':'unknown'} / ${finite(q?.ambiguity)?q.ambiguity.toFixed(3):'unknown'}`:'—');
    text('field-sync',source?.timing.timeSyncValid?'NT capture synchronized':'Capture synchronization unavailable');
    const mount=currentContext().mounts?.[source?.pipeline];
    text('field-mount',mount?.translation_m?mount.translation_m.map(x=>x.toFixed(2)).join(', ')+' m':'Missing / unavailable');
    el('field-canvas').setAttribute('aria-label',`${mode==='3d'?'3D perspective':'2D plan'} field visualization. ${replayPackets?'Replay. ':''}${source?.pipeline||'No source'}. Robot ${source?.robot.status||'unavailable'}, camera ${source?.camera.status||'unavailable'}. ${el('field-geometry-note').textContent}`);
    el('field-replay-next').disabled=!replayPackets||replayIndex>=replayPackets.length-1;
    el('field-live').disabled=!replayPackets;
    text('field-replay-status',replayPackets?`REPLAY · ${replayIndex+1} of ${replayPackets.length} packets. Step manually; stationary old packets expire independently.`:'Live dashboard observations. Replay is isolated from camera controls and NetworkTables.');
  }
  function schedule(){update();timer=setTimeout(schedule,document.hidden?1000:200);}
  function setMode(value){mode=value;text('field-controls-note',value==='3d'?'Drag orbit · Shift-drag pan · Wheel zoom':'Drag pan · Wheel zoom');renderer.setMode(value);for(const m of ['2d','3d']){el('field-mode-'+m).classList.toggle('active',m===value);el('field-mode-'+m).setAttribute('aria-pressed',String(m===value));}update();}
  async function verifyImage(map) {
    if (!localImage || !map?.background) return;
    const bg=map.background;
    localImage.verified=false;
    if(localImage.file.name!==bg.file)throw new Error('Picture filename does not match the imported field map.');
    if(localImage.image.naturalWidth!==bg.width||localImage.image.naturalHeight!==bg.height)throw new Error('Picture pixel dimensions do not match the imported field map.');
    if(root.CVFieldScene.sha256(localImage.bytes)!==bg.sha256)throw new Error('Picture content hash does not match the imported field map.');
    localImage.verified=true;
  }
  async function importMap(file) {
    if(!file)return;if(file.size>1024*1024)throw new Error('Field-map JSON must be no larger than 1 MiB.');
    const parsed=JSON.parse(await file.text()), checked=root.CVFieldScene.validateFieldMap(parsed,currentContext().field_layout);
    imported=parsed;if(localImage)localImage.verified=false;
    await verifyImage(checked);update();renderer.home();
  }
  async function importImage(file) {
    if(!file)return;if(file.size>12*1024*1024)throw new Error('Field picture must be no larger than 12 MiB.');
    if(!['image/png','image/jpeg','image/webp'].includes(file.type))throw new Error('Choose a local PNG, JPEG or WebP picture.');
    const bytes=new Uint8Array(await file.arrayBuffer()), url=URL.createObjectURL(file), image=new Image();
    try{image.src=url;await image.decode();if(image.naturalWidth*image.naturalHeight>16000000||image.naturalWidth>8192||image.naturalHeight>8192)throw new Error('Picture exceeds the 16-megapixel / 8192-pixel limit.');}
    catch(e){URL.revokeObjectURL(url);throw e;}
    if(imageURL)URL.revokeObjectURL(imageURL);imageURL=url;localImage={file,image,bytes,verified:false};
    const map=validateMap();await verifyImage(map);
    if(!map)text('field-map-status','Picture selected. Import a calibrated field-map JSON before displaying the picture.');
    update();renderer.home();
  }
  function stepReplay() {
    if(!replayPackets)return;
    const packet=replayPackets[replayIndex];
    if(!replay.ingest(packet,now(),'replay'))throw new Error('Replay packet is malformed, duplicated, out of order or from a retired session.');
    selected=packet.pipeline;update();
  }
  async function importReplay(file) {
    if(!file)return;if(file.size>2*1024*1024)throw new Error('Replay JSON must be no larger than 2 MiB.');
    const parsed=JSON.parse(await file.text()), packets=Array.isArray(parsed)?parsed:(Array.isArray(parsed.packets)?parsed.packets:[parsed]);
    if(!packets.length||packets.length>512)throw new Error('Replay requires 1–512 packets.');
    const trial=root.CVFieldModel.createStore();if(!trial.ingest(packets[0],now(),'replay'))throw new Error('Replay needs a valid source boot, pipeline, frame identity and observation payload.');
    const context=parsed.field_layout?{field_layout:parsed.field_layout,mounts:parsed.mounts||{}}:null;
    if(context&&!root.CVFieldModel.layoutScene(context.field_layout))throw new Error('Replay field layout is invalid.');
    replayPackets=packets;replayIndex=0;replay.clear();replayContext=context;
    stepReplay();renderer.home();
  }
  function input(id,action){el(id).addEventListener('change',async event=>{if(busy)return;busy=true;message('');try{await action(event.target.files[0]);}catch(error){message(error.message);update();}finally{event.target.value='';busy=false;}});}
  input('field-map-import',importMap);input('field-image-import',importImage);input('field-replay-import',importReplay);
  el('field-mode-2d').onclick=()=>setMode('2d');el('field-mode-3d').onclick=()=>setMode('3d');el('field-home').onclick=()=>renderer.home();
  el('field-show-all').onchange=update;el('field-clear-trail').onclick=()=>{activeStore().clearTrails();update();};
  el('field-clear-map').onclick=()=>{imported=null;localImage=null;if(imageURL)URL.revokeObjectURL(imageURL);imageURL=null;text('field-map-status','No display map imported.');message('');update();renderer.home();};
  el('field-replay-next').onclick=()=>{if(replayPackets&&replayIndex<replayPackets.length-1){replayIndex++;try{stepReplay();}catch(error){message(error.message);}}};
  el('field-live').onclick=()=>{replayPackets=null;replayContext=null;selected=liveSelected;replay.clear();message('');update();renderer.home();};
  root.CVFieldDashboard={selectPipeline(name){liveSelected=name;if(!replayPackets)selected=name;update();},
    updateLive(view){if(view?.version!==1)return;unavailable=false;const hadLayout=!!context.field_layout;context={field_layout:view.field_layout||null,mounts:view.mounts||{}};
      for(const [name,packet]of Object.entries(view.results||{})){const age=view.receipt_age_ms?.[name];if(finite(age)&&age>=0)live.ingest(packet,now()-age,'dashboard');}
      if(!selected&&!replayPackets)selected=Object.keys(view.results||{})[0]||null;
      update();if(!hadLayout&&context.field_layout)renderer.home();},
    markDisconnected(){unavailable=true;update();},
    destroy(){clearTimeout(timer);renderer.destroy();if(imageURL)URL.revokeObjectURL(imageURL);}};
  root.addEventListener('pagehide',()=>root.CVFieldDashboard.destroy(),{once:true});
  schedule();
})(globalThis);
