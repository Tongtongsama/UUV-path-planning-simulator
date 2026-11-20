"""Build a self-contained, no-computation HTML trajectory viewer from one joint run."""
from pathlib import Path
import json, sys
ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT/'artifacts/s4_joint18_20261120_01/F2/P0'
ACC=json.loads((SRC/'acceptance.json').read_text())
rows=[json.loads(s) for s in (SRC/'navigation.jsonl').read_text().splitlines() if s]
plan=json.loads((SRC/'plan.json').read_text())
# Downsample to <= ~700 points, retain first/last and min-clearance point.
step=max(1,len(rows)//700); sample=rows[::step]
if sample[-1] is not rows[-1]: sample.append(rows[-1])
minrow=min(rows,key=lambda r:r['swept_clearance_m'])
if minrow not in sample:
    sample.append(minrow); sample=sorted(sample,key=lambda r:r['elapsed'])
pts=[[round(r['elapsed'],3),round(r['position_xz'][0],4),round(r['position_xz'][1],4),round(r['swept_clearance_m'],4),round(r['path_error'],4),round(r['current_xz'][0],5),round(r['current_xz'][1],5),r['phase']] for r in sample]
# Keep plan simplified; the viewer draws every segment.
planpts=[[round(x,4),round(z,4)] for x,z in plan['path']]
data={'points':pts,'plan':planpts,'obstacles':[[8,10,-11,-9],[4,6,-18,-14],[13,15,-6,-2],[13,15,-18,-14],[3,5,-6,-2]],'arrival':ACC['arrival_time_s'],'clearance':ACC['minimum_clearance_m'],'rms':ACC['path_rms_m'],'max':ACC['path_max_m'],'status':ACC['passed']}
html=r'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>S4/F2/P0 joint trajectory</title>
<style>
:root{--bg:#0b1220;--panel:#111d31;--ink:#e8f0ff;--muted:#8aa0bd;--cyan:#46d9ff;--green:#54e39a;--orange:#ff9f43;--red:#ff5c5c}
*{box-sizing:border-box} body{margin:0;background:radial-gradient(1200px 700px at 20% 0%,#182a46 0%,var(--bg) 55%);color:var(--ink);font:14px/1.45 ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif}
.wrap{max-width:1240px;margin:0 auto;padding:22px 20px 30px}
h1{font-size:22px;margin:0 0 2px;letter-spacing:.2px}.sub{color:var(--muted);font-size:12px;margin-bottom:14px}
.grid{display:grid;grid-template-columns:minmax(0,1fr) 310px;gap:16px}
.card{background:rgba(17,29,49,.92);border:1px solid #223654;border-radius:14px;padding:12px;box-shadow:0 10px 30px rgba(0,0,0,.18)}
canvas{width:100%;height:auto;display:block;border-radius:10px;background:#09101d}
.controls{display:flex;align-items:center;gap:8px;flex-wrap:wrap;margin-top:10px}
button{background:#1b3353;color:var(--ink);border:1px solid #2c4d78;border-radius:8px;padding:7px 11px;cursor:pointer;font:inherit}
button:hover{background:#24466d}input[type=range]{flex:1;min-width:140px;accent-color:var(--cyan)}
.badge{display:inline-block;padding:3px 8px;border-radius:999px;background:#173b2c;color:var(--green);font-size:11px;font-weight:700}
.metric{display:flex;justify-content:space-between;gap:10px;border-bottom:1px solid #1c2c45;padding:7px 0;font-size:12px}.metric b{font-variant-numeric:tabular-nums}
.k{color:var(--muted)}.v{font-size:13px}.flow{display:flex;flex-wrap:wrap;gap:5px;margin:8px 0 10px}.flow span{font-size:10px;color:#bcd2ee;background:#132641;border:1px solid #274566;padding:4px 6px;border-radius:6px}
.legend{display:flex;gap:12px;flex-wrap:wrap;color:var(--muted);font-size:11px;margin-top:8px}.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:4px;vertical-align:middle}
.note{font-size:11px;color:var(--muted);margin-top:10px}.warn{color:#ffb4a8}
@media(max-width:900px){.grid{grid-template-columns:1fr}}
</style></head><body><div class="wrap">
<h1>S4 / F2 / P0 — measured-current joint trajectory</h1>
<div class="sub">Self-contained replay from the recorded real trial. No solver, no planner, no simulation is started by this page.</div>
<div class="grid"><div class="card">
<canvas id="cv" width="1000" height="620"></canvas>
<div class="controls">
<button id="play">Pause</button><button id="reset">Reset</button><button id="speed">1×</button>
<input id="scrub" type="range" min="0" max="1000" value="0">
<span id="clock" style="font-variant-numeric:tabular-nums;color:var(--muted);min-width:76px;text-align:right">0.0 s</span>
</div>
<div class="legend"><span><i class="dot" style="background:#46d9ff"></i>actual UUV</span><span><i class="dot" style="background:#8aa0bd"></i>planned D path</span><span><i class="dot" style="background:#ff9f43"></i>current vector</span><span><i class="dot" style="background:#ff5c5c"></i>0.9 m clearance</span></div>
<div class="note">Yellow ring = current measured point at the UUV position. Current arrow length is scaled ×10 for visibility; values are m/s.</div>
</div>
<div class="card">
<div style="display:flex;justify-content:space-between;align-items:center"><b>Run status</b><span id="status" class="badge">RUNNING</span></div>
<div class="flow"><span>G12 guide</span><span>PID + allocator</span><span>xdyn</span><span>Gazebo</span><span>state feedback</span></div>
<div class="metric"><span class="k">elapsed</span><b id="m-t">0.0 s</b></div>
<div class="metric"><span class="k">clearance</span><b id="m-c">–</b></div>
<div class="metric"><span class="k">path error</span><b id="m-e">–</b></div>
<div class="metric"><span class="k">current x</span><b id="m-cur">–</b></div>
<div class="metric"><span class="k">arrival</span><b id="m-arr">–</b></div>
<div class="metric"><span class="k">min clearance</span><b id="m-min">–</b></div>
<div class="metric"><span class="k">RMS / max</span><b id="m-rms">–</b></div>
<div class="note" id="live">The display advances through recorded poses; it does not compute navigation.</div>
<div class="note warn">Boundary: simulator-only recorded joint run; not a sea trial and not a full 18-case video.</div>
</div></div>
<div class="note">Controls: play/pause, reset, speed (1×/2×/4×), and scrub. Everything is embedded in this file; opening it requires no Python, server, or network.</div>
</div>
<script>
const D=__DATA__;
const cv=document.getElementById('cv'), ctx=cv.getContext('2d');
const X0=0,X1=20,Z0=-20,Z1=0, M=48;
const sx=x=>M+(x-X0)/(X1-X0)*(cv.width-2*M);
const sy=z=>cv.height-M-(z-Z0)/(Z1-Z0)*(cv.height-2*M);
let idx=0,playing=true,speed=1,last=0;
const pts=D.points, maxT=D.arrival;
function draw(){
  ctx.clearRect(0,0,cv.width,cv.height);
  // grid
  ctx.strokeStyle='#14243b';ctx.lineWidth=1;
  for(let x=0;x<=20;x+=2){ctx.beginPath();ctx.moveTo(sx(x),sy(0));ctx.lineTo(sx(x),sy(-20));ctx.stroke();}
  for(let z=0;z>=-20;z-=2){ctx.beginPath();ctx.moveTo(sx(0),sy(z));ctx.lineTo(sx(20),sy(z));ctx.stroke();}
  // axes labels
  ctx.fillStyle='#6680a3';ctx.font='12px ui-sans-serif';
  for(let x=0;x<=20;x+=5)ctx.fillText(String(x),sx(x)-4,cv.height-18);
  for(let z=-20;z<=0;z+=5)ctx.fillText(String(z),8,sy(z)+4);
  // obstacles
  ctx.fillStyle='#34465f';ctx.strokeStyle='#526987';ctx.lineWidth=1;
  for(const [a,b,c,d] of D.obstacles){ctx.fillRect(sx(a),sy(d),sx(b)-sx(a),sy(c)-sy(d));ctx.strokeRect(sx(a),sy(d),sx(b)-sx(a),sy(c)-sy(d));}
  // planned path
  ctx.strokeStyle='#8aa0bd';ctx.lineWidth=2;ctx.setLineDash([7,6]);ctx.beginPath();
  D.plan.forEach(([x,z],i)=>i?ctx.lineTo(sx(x),sy(z)):ctx.moveTo(sx(x),sy(z)));ctx.stroke();ctx.setLineDash([]);
  // current point/trail
  const p=pts[idx];
  // traveled trail
  ctx.strokeStyle='#46d9ff';ctx.lineWidth=2.4;ctx.beginPath();
  for(let i=0;i<=idx;i++){const q=pts[i];i?ctx.lineTo(sx(q[1]),sy(q[2])):ctx.moveTo(sx(q[1]),sy(q[2]));}
  ctx.stroke();
  // start/goal
  ctx.fillStyle='#54e39a';ctx.beginPath();ctx.arc(sx(2),sy(-10),5,0,7);ctx.fill();
  ctx.fillStyle='#ffd166';ctx.beginPath();ctx.arc(sx(18),sy(-10),7,0,7);ctx.fill();ctx.strokeStyle='#0b1220';ctx.lineWidth=2;ctx.stroke();
  // current arrow at UUV
  const cx=p[5],cz=p[6], scale=10; const ax=sx(p[1]), ay=sy(p[2]);
  const dx=cx*scale*(sx(1)-sx(0)), dz=cz*scale*(sy(0)-sy(1));
  if(Math.hypot(dx,dz)>2){ctx.strokeStyle='#ff9f43';ctx.fillStyle='#ff9f43';ctx.lineWidth=2.4;ctx.beginPath();ctx.moveTo(ax,ay);ctx.lineTo(ax+dx,ay+dz);ctx.stroke();
    const ang=Math.atan2(dz,dx);ctx.beginPath();ctx.moveTo(ax+dx,ay+dz);ctx.lineTo(ax+dx-9*Math.cos(ang-.45),ay+dz-9*Math.sin(ang-.45));ctx.lineTo(ax+dx-9*Math.cos(ang+.45),ay+dz-9*Math.sin(ang+.45));ctx.closePath();ctx.fill();}
  // current marker ring
  ctx.strokeStyle='#ffd166';ctx.lineWidth=2;ctx.beginPath();ctx.arc(ax,ay,9,0,7);ctx.stroke();
  // UUV triangle oriented by recent motion
  let hx=1,hz=0; if(idx>0){const q=pts[idx-1];hx=p[1]-q[1];hz=p[2]-q[2];if(Math.hypot(hx,hz)<1e-8){hx=1;hz=0;}}
  const ang=Math.atan2(hz*(sy(0)-sy(1)),hx*(sx(1)-sx(0)));
  ctx.save();ctx.translate(ax,ay);ctx.rotate(ang);ctx.fillStyle='#46d9ff';ctx.strokeStyle='#d8f7ff';ctx.lineWidth=1.5;
  ctx.beginPath();ctx.moveTo(13,0);ctx.lineTo(-8,7);ctx.lineTo(-4,0);ctx.lineTo(-8,-7);ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();
  // clearance threshold annotation
  ctx.fillStyle='#ff5c5c';ctx.font='11px ui-sans-serif';ctx.fillText('threshold 0.9 m',cv.width-150,22);
}
function fmt(v,d=3){return v.toFixed(d)}
function update(){
  const p=pts[idx];
  document.getElementById('m-t').textContent=fmt(p[0],2)+' s';
  document.getElementById('m-c').textContent=fmt(p[3],3)+' m';
  document.getElementById('m-e').textContent=fmt(p[4],3)+' m';
  document.getElementById('m-cur').textContent=fmt(p[5],4)+' m/s';
  document.getElementById('m-arr').textContent=fmt(D.arrival,2)+' s';
  document.getElementById('m-min').textContent=fmt(D.clearance,6)+' m';
  document.getElementById('m-rms').textContent=fmt(D.rms,3)+' / '+fmt(D.max,3)+' m';
  document.getElementById('clock').textContent=fmt(p[0],1)+' s';
  document.getElementById('scrub').value=String(Math.round(idx/(pts.length-1)*1000));
  document.getElementById('status').textContent=p[7]==='ARRIVED'?'ARRIVED':'RUNNING';
  document.getElementById('status').style.background=p[7]==='ARRIVED'?'#173b2c':'#3b2d17';
  document.getElementById('status').style.color=p[7]==='ARRIVED'?'#54e39a':'#ffcf80';
}
function step(ts){if(!last)last=ts;const dt=Math.min(.1,(ts-last)/1000);last=ts;
  if(playing){const inc=dt*speed*Math.max(1,(pts.length-1)/Math.max(1,D.arrival));idx=Math.min(pts.length-1,idx+inc);}
  draw();update();if(idx<pts.length-1)requestAnimationFrame(step);
}
document.getElementById('play').onclick=e=>{playing=!playing;e.target.textContent=playing?'Pause':'Play';last=0;if(playing)requestAnimationFrame(step);};
document.getElementById('reset').onclick=()=>{idx=0;playing=false;document.getElementById('play').textContent='Play';draw();update();};
document.getElementById('speed').onclick=e=>{speed=speed===1?2:speed===2?4:1;e.target.textContent=speed+'×';};
document.getElementById('scrub').oninput=e=>{idx=Math.round(parseInt(e.target.value)/1000*(pts.length-1));draw();update();};
draw();update();requestAnimationFrame(step);
</script></body></html>'''
html=html.replace('__DATA__',json.dumps(data,ensure_ascii=False,separators=(',',':')))
out=ROOT/'artifacts/s4_trajectory_viewer_20261120_01/index.html'
out.write_text(html,encoding='utf-8')
print(out, len(html), 'bytes', len(pts),'points')
