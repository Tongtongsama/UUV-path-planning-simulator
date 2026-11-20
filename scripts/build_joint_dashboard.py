"""Build a self-contained joint dashboard: 18-cell heatmap, repeatability, evidence, GUI still."""
from pathlib import Path
import json, base64, hashlib
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'artifacts/s4_joint_dashboard_20261120_01'; OUT.mkdir(exist_ok=True)
BASE=ROOT/'artifacts/s4_joint18_20261120_01'
FLOWS=('F0','F1','F2'); CASES=('P0','P1','P2','P3','P4','P0_repeat')
plan=json.loads((BASE/'F2/P0/plan.json').read_text())
PATHS={
 'A':json.loads((ROOT/'artifacts/twin_axial_s4_planning_20261116_04/A_raw_astar/plan.json').read_text())['path'],
 'B':json.loads((ROOT/'artifacts/twin_axial_s4_planning_20261116_04/B_safe_shortcut/plan.json').read_text())['path'],
 'C':json.loads((ROOT/'artifacts/twin_axial_s4_planning_20261116_04/C_safe_smoothing/plan.json').read_text())['path'],
 'D':plan['path']}
obstacles=[[8,10,-11,-9],[4,6,-18,-14],[13,15,-6,-2],[13,15,-18,-14],[3,5,-6,-2]]
def read_points(p):
 rows=[json.loads(s) for s in (p/'navigation.jsonl').read_text().splitlines() if s]
 step=max(1,len(rows)//220); s=rows[::step]
 if s[-1] is not rows[-1]: s.append(rows[-1])
 return [[round(r['elapsed'],2),round(r['position_xz'][0],3),round(r['position_xz'][1],3),round(r['swept_clearance_m'],3),round(r['path_error'],3),round(r['current_xz'][0],5),round(r['current_xz'][1],5),r['phase']] for r in s]
cases={}
for f in FLOWS:
 for c in CASES:
  p=BASE/f/c
  if not (p/'acceptance.json').is_file(): continue
  a=json.loads((p/'acceptance.json').read_text())
  cases[f'{f}/{c}']={'flow':f,'case':c,'passed':bool(a['passed']),'arrival':a['arrival_time_s'],'clearance':a['minimum_clearance_m'],'margin':a['minimum_safety_margin_m'],'rms':a['path_rms_m'],'max':a['path_max_m'],'points':read_points(p)}
# Repeatability: all completed runs named for one initial.
repeats={}
for f in FLOWS:
 for c in ('P0','P1','P2','P3','P4'):
  runs=[]
  for label,dirn in (('initial',c),('repeat',c+'_repeat'),('repeat2',c+'_repeat2')):
   p=BASE/f/dirn
   if (p/'acceptance.json').is_file():
    a=json.loads((p/'acceptance.json').read_text())
    runs.append({'label':label,'arrival':a['arrival_time_s'],'clearance':a['minimum_clearance_m'],'rms':a['path_rms_m'],'points':read_points(p)})
  if runs: repeats[f'{f}/{c}']=runs
# heatmap html
heat=['<table class="heat"><thead><tr><th>flow</th>'+''.join(f'<th>{c}</th>' for c in CASES)+'</tr></thead><tbody>']
for f in FLOWS:
 heat.append(f'<tr><th class="flowh">{f}</th>')
 for c in CASES:
  k=f'{f}/{c}'; v=cases.get(k)
  if not v: heat.append('<td class="miss">missing</td>'); continue
  heat.append(f'<td><button class="cell" data-key="{k}" onclick="loadCase(\'{k}\')"><b>{"PASS" if v["passed"] else "FAIL"}</b><span>c {v["clearance"]:.3f} m</span><span>RMS {v["rms"]:.3f} m</span></button></td>')
 heat.append('</tr>')
heat.append('</tbody></table>'); heat=''.join(heat)
imgb64=base64.b64encode((ROOT/'artifacts/s4_trajectory_viewer_20261120_01/gui_demo_still.png').read_bytes()).decode()
counts={'matrix':len(cases),'matrix_pass':sum(1 for v in cases.values() if v['passed']),'repeat_runs':sum(len(v) for v in repeats.values()),'joint_total':len(list(BASE.glob('*/**/acceptance.json')))}
data={'plan':[[round(x,3),round(z,3)] for x,z in plan['path']],'paths':{k:[[round(x,3),round(z,3)] for x,z in v] for k,v in PATHS.items()},'obstacles':obstacles,'cases':cases,'repeats':repeats,'counts':counts}
html=r'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>S4 joint dashboard</title>
<style>
:root{--bg:#08101c;--panel:#0f1d30;--line:#223654;--ink:#e8f0ff;--muted:#8aa0bd;--cyan:#46d9ff;--green:#54e39a;--orange:#ff9f43;--red:#ff5c5c;--gold:#ffd166}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(1200px 700px at 20% 0%,#172b47 0%,var(--bg) 55%);color:var(--ink);font:14px/1.45 ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif}
.wrap{max-width:1320px;margin:0 auto;padding:20px 18px 32px}h1{font-size:23px;margin:0 0 2px}.sub{color:var(--muted);font-size:12px;margin-bottom:12px}
.tabs{display:flex;gap:8px;flex-wrap:wrap;margin:12px 0}.tab{background:#132641;border:1px solid var(--line);color:var(--ink);border-radius:999px;padding:7px 12px;cursor:pointer;font:inherit}.tab.active{background:#1d4f70;border-color:#46d9ff}
.panel{display:none}.panel.active{display:block}.card{background:rgba(15,29,48,.94);border:1px solid var(--line);border-radius:14px;padding:12px;margin-bottom:12px;box-shadow:0 10px 30px rgba(0,0,0,.18)}
.grid2{display:grid;grid-template-columns:minmax(0,1.38fr) minmax(320px,.62fr);gap:14px}@media(max-width:950px){.grid2{grid-template-columns:1fr}}
table.heat{width:100%;border-collapse:separate;border-spacing:5px;font-size:11px}table.heat th{color:var(--muted);font-weight:600;text-align:left;padding:2px 4px}table.heat th.flowh{color:#bfe6ff;font-size:13px;width:46px}
.cell{width:100%;min-height:74px;text-align:left;background:#12253d;border:1px solid #284568;border-radius:9px;color:var(--ink);padding:7px;cursor:pointer;display:flex;flex-direction:column;gap:2px;font:inherit}.cell b{color:var(--green);font-size:11px}.cell span{color:var(--muted);font-size:10px}.cell:hover,.cell.active{border-color:var(--cyan);background:#183a55}.miss{color:#ff8f8f;font-size:10px;padding:6px}
canvas{width:100%;height:auto;display:block;border-radius:9px;background:#070d17}.controls{display:flex;align-items:center;gap:8px;flex-wrap:wrap;margin-top:9px}button.ctl{background:#1b3353;color:var(--ink);border:1px solid #2c4d78;border-radius:8px;padding:6px 10px;cursor:pointer;font:inherit}button.ctl:hover{background:#24466d}input[type=range]{flex:1;min-width:130px;accent-color:var(--cyan)}
.metric{display:flex;justify-content:space-between;gap:10px;border-bottom:1px solid #1c2c45;padding:6px 0;font-size:12px}.metric b{font-variant-numeric:tabular-nums}.k{color:var(--muted)}.badge{display:inline-block;padding:3px 8px;border-radius:999px;background:#173b2c;color:var(--green);font-size:11px;font-weight:700}
.flow{display:flex;flex-wrap:wrap;gap:5px;margin:8px 0}.flow span{font-size:10px;color:#bcd2ee;background:#132641;border:1px solid #274566;padding:4px 6px;border-radius:6px}.legend{display:flex;gap:12px;flex-wrap:wrap;color:var(--muted);font-size:11px;margin-top:8px}.pathlegend{align-items:center;background:#0b192b;border:1px solid #1c2f49;border-radius:8px;padding:6px 8px}.pathlegend label{display:flex;align-items:center;gap:4px;color:#c9d6e8;font-size:10px;cursor:pointer}.pathlegend input{accent-color:var(--cyan)}.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:4px;vertical-align:middle}.note{font-size:11px;color:var(--muted);margin-top:9px}.warn{color:#ffb4a8}.cards{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}@media(max-width:700px){.cards{grid-template-columns:1fr}}.big{font-size:25px;font-weight:800;color:var(--gold)}.small{font-size:11px;color:var(--muted)}.evidence table{width:100%;border-collapse:collapse;font-size:12px}.evidence td,.evidence th{border-bottom:1px solid #1b2d46;padding:6px 4px;text-align:left}.evidence th{color:var(--muted)}img.still{width:100%;border-radius:10px;border:1px solid #223654}
</style></head><body><div class="wrap">
<h1>S4 joint dashboard — 18 matrix, repeatability, evidence, GUI demo</h1>
<div class="sub">Self-contained replay. No solver, planner, simulation, server or network is started by this page.</div>
<div class="tabs"><button class="tab active" data-tab="matrix">Joint 18 matrix</button><button class="tab" data-tab="repeat">Repeatability</button><button class="tab" data-tab="evidence">Evidence layers</button><button class="tab" data-tab="gui">GUI demo</button></div>

<div id="matrix" class="panel active"><div class="card"><div class="grid2">
<div><div class="card" style="margin:0"><div id="heatwrap">__HEATMAP__</div><div class="note">Click any cell to load its recorded trajectory. F0/F1 are control-only; F2 is v10 measured-current.</div></div></div>
<div><div class="card" style="margin:0"><div style="display:flex;justify-content:space-between;align-items:center"><b id="caseTitle">S4 / F2 / P0</b><span id="status" class="badge">RUNNING</span></div>
<div class="flow"><span>G12 guide</span><span>PID + allocator</span><span>xdyn</span><span>Gazebo</span><span>state feedback</span></div>
<canvas id="cv" width="900" height="560"></canvas>
<div class="legend pathlegend"><b style="color:#bfe6ff">Planning paths:</b><label><input type="checkbox" data-path="A" checked> <i class="dot" style="background:#7f7f7f"></i>A raw A*</label><label><input type="checkbox" data-path="B" checked> <i class="dot" style="background:#2f80ed"></i>B safe shortcut</label><label><input type="checkbox" data-path="C" checked> <i class="dot" style="background:#2ca02c"></i>C rounded/smoothing</label><label><input type="checkbox" data-path="D" checked> <i class="dot" style="background:#d62728"></i>D 4 m terminal</label></div>
<div class="controls"><button class="ctl" id="play">Pause</button><button class="ctl" id="reset">Reset</button><button class="ctl" id="speed">1×</button><input id="scrub" type="range" min="0" max="1000" value="0"><span id="clock" class="small">0.0 s</span></div>
<div class="metric"><span class="k">elapsed</span><b id="m-t">–</b></div><div class="metric"><span class="k">clearance</span><b id="m-c">–</b></div><div class="metric"><span class="k">path error</span><b id="m-e">–</b></div><div class="metric"><span class="k">arrival</span><b id="m-arr">–</b></div><div class="metric"><span class="k">min clearance</span><b id="m-min">–</b></div><div class="metric"><span class="k">RMS / max</span><b id="m-rms">–</b></div>
<div class="note" id="live">Recorded joint trajectory; no computation.</div></div></div>
</div></div></div>

<div id="repeat" class="panel"><div class="card"><div style="display:flex;gap:10px;align-items:center;flex-wrap:wrap"><b>Initial</b><select id="rsel" class="ctl"></select><span class="small">overlay of all completed runs for that initial</span></div><div class="grid2" style="margin-top:10px"><div><canvas id="rv" width="900" height="560"></canvas><div class="legend"><span><i class="dot" style="background:#46d9ff"></i>initial</span><span><i class="dot" style="background:#ff9f43"></i>repeat</span><span><i class="dot" style="background:#54e39a"></i>repeat 2/3</span></div></div><div><div class="metric"><span class="k">runs</span><b id="r-runs">–</b></div><div class="metric"><span class="k">arrival spread</span><b id="r-a">–</b></div><div class="metric"><span class="k">clearance spread</span><b id="r-c">–</b></div><div class="metric"><span class="k">max position diff</span><b id="r-p">–</b></div><div class="note" id="r-note"></div><div class="note warn">Historical P2 pair (separate 10-record set): 15.1 mm. Matrix P2 three-run spread: 0.11 mm. Both are retained; they are not the same dataset.</div></div></div></div></div>

<div id="evidence" class="panel"><div class="card"><div class="cards"><div class="card"><div class="big">__MATRIX__/18</div><div>joint matrix passed</div><div class="small">F0/F1 control + F2 measured</div></div><div class="card"><div class="big">__REPEATS__</div><div>joint repeat runs</div><div class="small">P0–P4 in all flows</div></div><div class="card"><div class="big">__JOINT__</div><div>joint records total</div><div class="small">33 milestone + 12 third repeats</div></div></div>
<div class="evidence"><table><tr><th>layer</th><th>count</th><th>what it is</th><th>not</th></tr><tr><td>Joint S4</td><td>__JOINT__ records</td><td>real xdyn + Gazebo joint trials, simulator-only</td><td>not sea trial</td></tr><tr><td>Five real pilots</td><td>5</td><td>F0/F1/F2 P0/P2/P4 real S4 pilots</td><td>not 18 real matrix</td></tr><tr><td>S4 Python</td><td>18</td><td>constant-flow Python evidence</td><td>not measured-current joint</td></tr></table></div>
<div class="note warn">These three layers are not interchangeable. The dashboard does not merge them into one success count.</div><div class="note">F0 = no-current control; F1 = constant-current control; F2 = v10 measured-current. Only F2 is measured-current.</div><div class="note">The 33-record milestone was 18 matrix + 15 second repeats. The 12 third repeats bring the current total to 45; both counts are retained.</div></div></div>

<div id="gui" class="panel"><div class="card"><img class="still" src="data:image/png;base64,__GUI__" alt="GUI demo still"><div class="note"><b>Visualization-only GUI demo.</b> Static S4 map, D path, UUV start marker, and v10 current reference balls. No xdyn, controller, navigation, gate or dynamics are started. Balls are sampled every 10 s; spacing/direction show current speed/direction. This is not a camera video and the black-frame camera MP4 is not shown.</div></div></div>
<div class="note">Files: 18 matrix and repeats under <code>artifacts/s4_joint18_20261120_01</code>; evidence review in <code>docs/operational_readiness_20261120.md</code>.</div>
</div>
<script>
const D=__DATA__;
const CASES=D.cases, REPS=D.repeats, PATHS=D.paths;
const obs=D.obstacles, plan=D.plan;
const cv=document.getElementById('cv'), ctx=cv.getContext('2d');
let currentKey='F2/P0', pts=CASES[currentKey].points, idx=0, playing=true,speed=1,last=0;
const M=46,W=cv.width,H=cv.height;const sx=x=>M+x/20*(W-2*M);const sy=z=>H-M-(z+20)/20*(H-2*M);
function base(g){
 g.clearRect(0,0,W,H);g.strokeStyle='#14243b';g.lineWidth=1;
 for(let x=0;x<=20;x+=2){g.beginPath();g.moveTo(sx(x),sy(0));g.lineTo(sx(x),sy(-20));g.stroke();}
 for(let z=-20;z<=0;z+=2){g.beginPath();g.moveTo(sx(0),sy(z));g.lineTo(sx(20),sy(z));g.stroke();}
 g.fillStyle='#34465f';g.strokeStyle='#526987';for(const [a,b,c,d] of obs){g.fillRect(sx(a),sy(d),sx(b)-sx(a),sy(c)-sy(d));g.strokeRect(sx(a),sy(d),sx(b)-sx(a),sy(c)-sy(d));}
 drawPaths(g);
 g.fillStyle='#54e39a';g.beginPath();g.arc(sx(2),sy(-10),5,0,7);g.fill();g.fillStyle='#ffd166';g.beginPath();g.arc(sx(18),sy(-10),7,0,7);g.fill();
}
function drawPaths(g){
 const colors={A:'#7f7f7f',B:'#2f80ed',C:'#2ca02c',D:'#d62728'};
 for(const k of ['A','B','C','D']){
  const el=document.querySelector('[data-path="'+k+'"]'); if(el && !el.checked) continue;
  const pp=PATHS[k]; if(!pp) continue;
  g.strokeStyle=colors[k];g.lineWidth=(k==='D'?2.6:2.0);g.setLineDash(k==='A'?[5,4]:[]);
  g.beginPath();pp.forEach(([x,z],i)=>i?g.lineTo(sx(x),sy(z)):g.moveTo(sx(x),sy(z)));g.stroke();g.setLineDash([]);
 }
}
function drawCase(){
 base(ctx);const p=pts[idx];
 ctx.strokeStyle='#46d9ff';ctx.lineWidth=2.5;ctx.beginPath();for(let i=0;i<=idx;i++){const q=pts[i];i?ctx.lineTo(sx(q[1]),sy(q[2])):ctx.moveTo(sx(q[1]),sy(q[2]));}ctx.stroke();
 const ax=sx(p[1]),ay=sy(p[2]);ctx.strokeStyle='#ffd166';ctx.lineWidth=2;ctx.beginPath();ctx.arc(ax,ay,9,0,7);ctx.stroke();
 const cx=p[5]*10*(sx(1)-sx(0)),cz=p[6]*10*(sy(0)-sy(1));if(Math.hypot(cx,cz)>2){ctx.strokeStyle='#ff9f43';ctx.lineWidth=2.5;ctx.beginPath();ctx.moveTo(ax,ay);ctx.lineTo(ax+cx,ay+cz);ctx.stroke();}
 let hx=1,hz=0;if(idx>0){const q=pts[idx-1];hx=p[1]-q[1];hz=p[2]-q[2];if(Math.hypot(hx,hz)<1e-8){hx=1;hz=0;}}
 const ang=Math.atan2(hz*(sy(0)-sy(1)),hx*(sx(1)-sx(0)));ctx.save();ctx.translate(ax,ay);ctx.rotate(ang);ctx.fillStyle='#46d9ff';ctx.strokeStyle='#d8f7ff';ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(13,0);ctx.lineTo(-8,7);ctx.lineTo(-4,0);ctx.lineTo(-8,-7);ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();
}
function updateCase(){const v=CASES[currentKey],p=pts[idx];
 document.getElementById('caseTitle').textContent='S4 / '+currentKey;
 document.getElementById('m-t').textContent=p[0].toFixed(2)+' s';document.getElementById('m-c').textContent=p[3].toFixed(3)+' m';document.getElementById('m-e').textContent=p[4].toFixed(3)+' m';
 document.getElementById('m-arr').textContent=v.arrival.toFixed(2)+' s';document.getElementById('m-min').textContent=v.clearance.toFixed(6)+' m';document.getElementById('m-rms').textContent=v.rms.toFixed(3)+' / '+v.max.toFixed(3)+' m';
 document.getElementById('clock').textContent=p[0].toFixed(1)+' s';document.getElementById('scrub').value=Math.round(idx/(pts.length-1)*1000);
 const st=document.getElementById('status');st.textContent=p[7]==='ARRIVED'?'ARRIVED':'RUNNING';st.style.background=p[7]==='ARRIVED'?'#173b2c':'#3b2d17';st.style.color=p[7]==='ARRIVED'?'#54e39a':'#ffcf80';
}
function tick(ts){if(!last)last=ts;const dt=Math.min(.1,(ts-last)/1000);last=ts;if(playing){idx=Math.min(pts.length-1,idx+dt*speed*Math.max(1,(pts.length-1)/Math.max(1,CASES[currentKey].arrival)));}drawCase();updateCase();if(idx<pts.length-1)requestAnimationFrame(tick);}
function loadCase(k){if(!CASES[k])return;currentKey=k;pts=CASES[k].points;idx=0;playing=false;document.getElementById('play').textContent='Play';document.querySelectorAll('.cell').forEach(e=>e.classList.toggle('active',e.dataset.key===k));drawCase();updateCase();}
document.getElementById('play').onclick=e=>{playing=!playing;e.target.textContent=playing?'Pause':'Play';last=0;if(playing)requestAnimationFrame(tick);};
document.getElementById('reset').onclick=()=>{idx=0;playing=false;document.getElementById('play').textContent='Play';drawCase();updateCase();};
document.getElementById('speed').onclick=e=>{speed=speed===1?2:speed===2?4:1;e.target.textContent=speed+'×';};
document.getElementById('scrub').oninput=e=>{idx=Math.round(parseInt(e.target.value)/1000*(pts.length-1));drawCase();updateCase();};
// repeatability
const rv=document.getElementById('rv'),rctx=rv.getContext('2d');const rsel=document.getElementById('rsel');
Object.keys(REPS).sort().forEach(k=>{const o=document.createElement('option');o.value=k;o.textContent=k;rsel.appendChild(o);});rsel.value='F2/P2';
function drawRepeat(){
 base(rctx);const runs=REPS[rsel.value]||[];const cols=['#46d9ff','#ff9f43','#54e39a'];
 runs.forEach((r,i)=>{rctx.strokeStyle=cols[i%3];rctx.lineWidth=2;rctx.beginPath();r.points.forEach((p,j)=>j?rctx.lineTo(sx(p[1]),sy(p[2])):rctx.moveTo(sx(p[1]),sy(p[2])));rctx.stroke();});
 let maxd=0;for(let i=0;i<runs.length;i++)for(let j=i+1;j<runs.length;j++){const a=runs[i].points,b=runs[j].points,n=Math.min(a.length,b.length);for(let k=0;k<n;k++)maxd=Math.max(maxd,Math.hypot(a[k][1]-b[k][1],a[k][2]-b[k][2]));}
 const ar=runs.map(r=>r.arrival),cl=runs.map(r=>r.clearance);
 document.getElementById('r-runs').textContent=runs.length;
 document.getElementById('r-a').textContent=runs.length?Math.max(...ar)-Math.min(...ar):'–';
 document.getElementById('r-c').textContent=runs.length?((Math.max(...cl)-Math.min(...cl))*1000).toFixed(2)+' mm':'–';
 document.getElementById('r-p').textContent=(maxd*1000).toFixed(2)+' mm';
 document.getElementById('r-note').textContent='Selected '+rsel.value+'; each color is one completed run.';
}
rsel.onchange=drawRepeat;
document.querySelectorAll('.tab').forEach(b=>b.onclick=()=>{document.querySelectorAll('.tab').forEach(x=>x.classList.remove('active'));document.querySelectorAll('.panel').forEach(x=>x.classList.remove('active'));b.classList.add('active');document.getElementById(b.dataset.tab).classList.add('active');if(b.dataset.tab==='repeat')drawRepeat();});
base(ctx);drawCase();updateCase();requestAnimationFrame(tick);drawRepeat();
document.querySelectorAll('[data-path]').forEach(cb=>cb.onchange=()=>{drawCase();drawRepeat();});
</script></body></html>'''
html=html.replace('__HEATMAP__',heat).replace('__DATA__',json.dumps(data,ensure_ascii=False,separators=(',',':'))).replace('__GUI__',imgb64).replace('__MATRIX__',str(counts['matrix_pass'])).replace('__REPEATS__',str(counts['repeat_runs'])).replace('__JOINT__',str(counts['joint_total']))
(OUT/'index.html').write_text(html,encoding='utf-8')
print('html',len(html),'bytes','matrix',counts,'repeat keys',len(repeats))
