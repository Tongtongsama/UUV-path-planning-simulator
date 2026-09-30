/* Generic offline player. No simulation, network access, or control computation. */
(function () {
  'use strict';
  function upperBound(rows, t) {
    let lo = 0, hi = rows.length;
    while (lo < hi) { const m = (lo + hi) >>> 1; if (rows[m][0] <= t) lo = m + 1; else hi = m; }
    return lo;
  }
  function snapshot(arm, time) {
    const t = Math.max(arm.samples[0][0], Math.min(time, arm.end));
    const i = Math.max(0, upperBound(arm.samples, t) - 1);
    const raw = arm.samples[i], next = arm.samples[Math.min(i + 1, arm.samples.length - 1)];
    const f = next[0] > raw[0] ? (t - raw[0]) / (next[0] - raw[0]) : 0;
    const drawn = raw.map((v, k) => k === 0 ? t : v + f * (next[k] - v));
    const ci = Math.max(0, upperBound(arm.controls, t) - 1), control = arm.controls[ci];
    const ended = time >= arm.end;
    return {i, raw, drawn, control, ended, phase: ended ? arm.status : (control ? control[3] : 'INITIAL'),
      collisionDetected: Boolean(arm.collision && time >= arm.collision.detected_at)};
  }
  function sampleStep(times, t, direction) {
    if (direction > 0) return times[Math.min(times.length - 1, upperBound(times.map(x => [x]), t + 1e-9))];
    return times[Math.max(0, upperBound(times.map(x => [x]), t - 1e-9) - 1)];
  }
  const math = {upperBound, snapshot, sampleStep};
  if (typeof module !== 'undefined') module.exports = math;
  if (typeof document === 'undefined') return;

  const data = JSON.parse(document.getElementById('payload').textContent);
  const $ = id => document.getElementById(id);
  const canvas = $('view'), ctx = canvas.getContext('2d');
  const C = {bg:'#101722',panel:'#152130',text:'#e5edf6',muted:'#aebed1',grid:'#304258',
    p1:'#66bfff',approach:'#ffbd69',path:'#bac7d8',obstacle:'#4b596b',danger:'#ff7281',safe:'#85d7b0'};
  let scene = data.scenes[0], time = scene.start_time, playing = false, speed = 1, last = null;
  let times = [], recording = false, stopRecording = null;
  const labels={double_baffle_detour:'double_baffle · 末段对照',horizontal:'horizontal · 基本闭环',
    detour:'detour · 路径跟踪',edge_forced_detour:'edge_forced · 碰撞边界',zigzag_maze_detour:'zigzag · 碰撞边界'};
  data.scenes.forEach(s => {const o=document.createElement('option');o.value=s.name;o.textContent=labels[s.name]||s.name;$('scene').append(o);});
  function txt(s,x,y,color=C.text,size=16,align='left') {ctx.fillStyle=color;ctx.font=`${size}px "Segoe UI","Microsoft YaHei",sans-serif`;ctx.textAlign=align;ctx.fillText(s,x,y);}
  function line(points,color,width=1,dash=[]) {
    if (!points.length) return; ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);
    points.forEach((p,i)=>i?ctx.lineTo(...p):ctx.moveTo(...p));ctx.stroke();ctx.setLineDash([]);
  }
  function dot(x,y,r,color) {ctx.beginPath();ctx.fillStyle=color;ctx.arc(x,y,r,0,Math.PI*2);ctx.fill();}
  const num=(v,n=3)=>Number(v).toFixed(n);
  function sceneSelect(name) {
    scene=data.scenes.find(s=>s.name===name)||data.scenes[0];playing=false;time=scene.start_time;
    times=[...new Set(Object.values(scene.arms).flatMap(a=>a.samples.map(s=>s[0])))].sort((a,b)=>a-b);
    $('scene').value=scene.name;$('time').min=scene.start_time;$('time').max=scene.end_time;
    location.hash=scene.name; render();
  }
  function mapPanel(arm,key,color,x,y,w,h) {
    const s=snapshot(arm,time), b=scene.boundary;
    ctx.fillStyle=C.panel;ctx.fillRect(x,y,w,h);
    txt(key==='P1'?'P1 · baseline':'P1 + approach',x+14,y+25,color,19);
    const stateLabel=s.ended?`${arm.status} · 已结束 / frozen`:`${s.phase}`;
    txt(stateLabel,x+14,y+49,s.collisionDetected?C.danger:C.text,14);
    const top=y+68, availableH=h-189, scale=Math.min((w-82)/(b.x_max-b.x_min),availableH/(b.z_max-b.z_min));
    const mw=scale*(b.x_max-b.x_min),mh=scale*(b.z_max-b.z_min),left=x+(w-mw)/2;
    const point=(px,pz)=>[left+(px-b.x_min)*scale,top+(b.z_max-pz)*scale];
    ctx.fillStyle=C.bg;ctx.fillRect(left,top,mw,mh);
    for(let v=Math.ceil(b.x_min/2)*2;v<=b.x_max;v+=2){const [px]=point(v,0);line([[px,top],[px,top+mh]],C.grid);txt(v,px,top+mh+18,C.muted,12,'center');}
    for(let v=Math.ceil(b.z_min/2)*2;v<=b.z_max;v+=2){const [,py]=point(0,v);line([[left,py],[left+mw,py]],C.grid);txt(v,left-8,py+4,C.muted,12,'right');}
    txt('x East [m]',left+mw/2,top+mh+38,C.muted,13,'center');
    ctx.save();ctx.translate(left-37,top+mh/2);ctx.rotate(-Math.PI/2);txt('z Up [m]',0,0,C.muted,13,'center');ctx.restore();
    ctx.save();ctx.beginPath();ctx.rect(left,top,mw,mh);ctx.clip();
    for(const o of scene.obstacles){const [ox,oz]=point(o.x_min,o.z_max);ctx.fillStyle=C.obstacle;ctx.fillRect(ox,oz,(o.x_max-o.x_min)*scale,(o.z_max-o.z_min)*scale);}
    const inset=scene.constraints.vehicle_radius+scene.constraints.safety_margin;
    const a=point(b.x_min+inset,b.z_max-inset),z=point(b.x_max-inset,b.z_min+inset);
    line([a,[z[0],a[1]],z,[a[0],z[1]],a],C.muted,1,[3,5]);
    line(scene.path.map(p=>point(...p)),C.path,1.5,[6,5]);
    line(arm.samples.slice(0,s.i+1).map(r=>point(r[1],r[2])).concat([point(s.drawn[1],s.drawn[2])]),color,2.5);
    const start=point(scene.start.x,scene.start.z),goal=point(scene.goal.x,scene.goal.z);
    dot(...start,4,C.safe);txt('S',start[0]-11,start[1]-8,C.safe,12);
    ctx.beginPath();ctx.strokeStyle=C.safe;ctx.lineWidth=1;ctx.arc(...goal,scene.constraints.goal_tolerance*scale,0,Math.PI*2);ctx.stroke();
    line([[goal[0]-6,goal[1]],[goal[0]+6,goal[1]]],C.safe,2);line([[goal[0],goal[1]-6],[goal[0],goal[1]+6]],C.safe,2);txt('G',goal[0]+9,goal[1]-8,C.safe,12);
    const p=point(s.drawn[1],s.drawn[2]);
    ctx.beginPath();ctx.setLineDash([2,3]);ctx.strokeStyle=color;ctx.arc(...p,inset*scale,0,Math.PI*2);ctx.stroke();ctx.setLineDash([]);
    ctx.save();ctx.translate(...p);ctx.rotate(-s.drawn[3]);ctx.fillStyle=color;ctx.beginPath();
    ctx.moveTo(15,0);ctx.lineTo(5,-6);ctx.lineTo(-13,-6);ctx.lineTo(-10,0);ctx.lineTo(-13,6);ctx.lineTo(5,6);ctx.closePath();ctx.fill();dot(11,0,2.5,C.bg);ctx.restore();
    if(s.collisionDetected){line([[p[0]-11,p[1]-11],[p[0]+11,p[1]+11]],C.danger,3);line([[p[0]-11,p[1]+11],[p[0]+11,p[1]-11]],C.danger,3);}
    ctx.restore();ctx.strokeStyle=C.muted;ctx.strokeRect(left,top,mw,mh);
    const sy=y+h-68,raw=s.raw,distance=Math.hypot(raw[1]-scene.goal.x,raw[2]-scene.goal.z);
    txt(`样本 t=${num(raw[0],2)} s   速度 ${num(Math.hypot(raw[4],raw[5]))} m/s   目标距 ${num(distance)} m`,x+12,sy,C.text,13);
    txt(`pitch ${num(raw[3])} rad   q ${num(raw[6])} rad/s   ${s.control?s.control[4]:''}`,x+12,sy+21,C.text,13);
    const clearance=arm.metrics.minimum_executed_safety_margin;
    txt(`日志全程最小净余量：${clearance===null?'无障碍物':num(clearance*1000,2)+' mm'} · 艇形为姿态示意`,x+12,sy+43,C.muted,12);
    if(s.collisionDetected)txt(`碰撞检测区间 [${num(arm.collision.interval[0],2)}, ${num(arm.collision.interval[1],2)}] s`,x+12,y+65,C.danger,12);
  }
  function plot(title,column,controls,x,y,w,h) {
    const left=x+64,top=y+30,pw=w-85,ph=h-64;
    const all=Object.values(scene.arms).flatMap(a=>(controls?a.controls:a.samples).map(s=>s[column]));
    let low=all.length?Math.min(...all):0,high=all.length?Math.max(...all):0,pad=Math.max((high-low)*.12,.05);low-=pad;high+=pad;
    const px=t=>left+(t-scene.start_time)/Math.max(1e-9,scene.end_time-scene.start_time)*pw,py=v=>top+ph-(v-low)/(high-low)*ph;
    txt(title,x+10,y+17,C.text,15);
    for(let i=0;i<=2;i++){const v=low+(high-low)*i/2,p=py(v);line([[left,p],[left+pw,p]],C.grid);txt(num(v,2),left-8,p+4,C.muted,12,'right');}
    for(let i=0;i<=4;i++){const t=scene.start_time+(scene.end_time-scene.start_time)*i/4;txt(num(t,0),px(t),top+ph+18,C.muted,12,'center');}
    txt('physical time [s]',left+pw/2,top+ph+34,C.muted,12,'center');
    for(const [key,color] of [['P1',C.p1],['APPROACH',C.approach]]){
      const arm=scene.arms[key],rows=controls?arm.controls:arm.samples,points=[];
      rows.forEach((r,i)=>{if(controls&&i)points.push([px(r[0]),py(rows[i-1][column])]);points.push([px(r[0]),py(r[column])]);});
      if(controls&&rows.length)points.push([px(arm.end),py(rows[rows.length-1][column])]);
      line(points,color,1.3);
      const s=snapshot(arm,time),value=controls?(s.control?s.control[column]:0):s.drawn[column];dot(px(Math.min(time,arm.end)),py(value),3,color);
    }
    line([[px(time),top],[px(time),top+ph]],C.text,1,[3,3]);
  }
  function render() {
    const narrow=canvas.clientWidth<650,W=narrow?640:1280,H=narrow?1580:1020;
    if(canvas.width!==W||canvas.height!==H){canvas.width=W;canvas.height=H;}
    ctx.fillStyle=C.bg;ctx.fillRect(0,0,W,H);
    txt(`${scene.name} · 离线历史回放`,20,28,C.text,21);
    txt(`共同物理时间 ${num(time,2)} / ${num(scene.end_time,2)} s`,20,53,C.muted,16);
    if(recording)txt('视频 4× · 非实时策略执行',W-20,53,C.muted,14,'right');
    const width=narrow?600:610,mapHeight=530;
    mapPanel(scene.arms.P1,'P1',C.p1,20,72,width,mapHeight);
    mapPanel(scene.arms.APPROACH,'APPROACH',C.approach,narrow?20:650,narrow?614:72,width,mapHeight);
    const row=narrow?1155:620,chartW=narrow?300:610;
    plot('pitch [rad] · 原始不折叠',3,false,20,row,chartW,175);
    plot('q [rad/s]',6,false,narrow?320:650,row,chartW,175);
    plot('applied surge [N]',1,true,20,row+185,chartW,175);
    plot('applied pitch moment [Nm]',2,true,narrow?320:650,row+185,chartW,175);
    txt('虚线：固定 P1 路径 / 侵蚀边界   实线：实际轨迹   S 起点 / G 目标   艇首：深色圆点',20,H-20,C.muted,12);
    $('clock').value=`${num(time,2)} / ${num(scene.end_time,2)} s`;$('time').value=time;$('play').textContent=playing?'暂停':'播放';
    $('accessible').textContent=`${scene.name} 时间 ${num(time,2)} 秒。P1 ${snapshot(scene.arms.P1,time).phase}，approach ${snapshot(scene.arms.APPROACH,time).phase}。`;
  }
  function setTime(t){time=Math.min(scene.end_time,Math.max(scene.start_time,Number(t)));render();}
  $('scene').onchange=()=>sceneSelect($('scene').value);
  $('play').onclick=()=>{if(time>=scene.end_time)time=scene.start_time;playing=!playing;last=null;render();};
  $('restart').onclick=()=>{time=scene.start_time;playing=true;last=null;render();};
  $('time').oninput=()=>{playing=false;setTime($('time').value);};
  $('back').onclick=()=>{playing=false;setTime(sampleStep(times,time,-1));};
  $('step').onclick=()=>{playing=false;setTime(sampleStep(times,time,1));};
  $('speed').onchange=()=>{speed=Number($('speed').value);};
  new ResizeObserver(render).observe(canvas);
  function frame(now){if(playing&&last!==null){setTime(time+(now-last)/1000*speed);if(time>=scene.end_time){playing=false;render();if(stopRecording)stopRecording();}}last=now;requestAnimationFrame(frame);}
  requestAnimationFrame(frame);
  async function recordVideo() {
    if(recording)throw new Error('recording already active');
    sceneSelect('double_baffle_detour');speed=4;$('speed').value='4';
    recording=true;render();
    const stream=canvas.captureStream(30),mime=['video/webm;codecs=vp9','video/webm;codecs=vp8'].find(t=>MediaRecorder.isTypeSupported(t));
    if(!mime)throw new Error('WebM recording not supported');
    const rec=new MediaRecorder(stream,{mimeType:mime,videoBitsPerSecond:6000000}),chunks=[];
    for(const e of document.querySelectorAll('button,select,input'))e.disabled=true;
    return new Promise((resolve,reject)=>{
      rec.ondataavailable=e=>{if(e.data.size)chunks.push(e.data);};rec.onerror=e=>reject(e.error);
      rec.onstop=()=>{
        const blob=new Blob(chunks,{type:mime}),url=URL.createObjectURL(blob),a=document.createElement('a');
        a.href=url;a.download='double_baffle_comparison_4x.webm';a.click();
        setTimeout(()=>URL.revokeObjectURL(url),10000);stream.getTracks().forEach(t=>t.stop());recording=false;
        for(const e of document.querySelectorAll('button,select,input'))e.disabled=false;
        $('notice').textContent='视频来自同一冻结历史，共同时间 4× 播放；未重跑仿真。';render();resolve({bytes:blob.size,mime});
      };
      stopRecording=()=>{stopRecording=null;setTimeout(()=>rec.stop(),1500);};
      rec.start(1000);playing=true;last=null;
    });
  }
  window.replay={setTime,select:sceneSelect,recordVideo,snapshot:()=>({time,scene:scene.name,
    arms:Object.fromEntries(Object.entries(scene.arms).map(([k,a])=>[k,snapshot(a,time)]))})};
  sceneSelect(location.hash.slice(1));
})();
