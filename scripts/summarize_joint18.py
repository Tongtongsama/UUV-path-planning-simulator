"""Summarize the S4 joint 18-case matrix; never invent missing records."""
from pathlib import Path
import json, hashlib
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'artifacts/s4_joint18_20261120_01'
FLOWS=('F0','F1','F2'); CASES=('P0','P1','P2','P3','P4','P0_repeat')
rows={}; missing=[]
for f in FLOWS:
 for c in CASES:
  p=BASE/f/c/'acceptance.json'
  if not p.is_file(): missing.append(f'{f}/{c}'); continue
  a=json.loads(p.read_text()); rows[f'{f}/{c}']={'passed':a['passed'],'arrival_s':a['arrival_time_s'],'clearance_m':a['minimum_clearance_m'],'margin_m':a['minimum_safety_margin_m'],'rms_m':a['path_rms_m'],'max_m':a['path_max_m'],'checks_all_true':all(a['checks'].values()),'acceptance_sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
out={'matrix':'S4 3 flows x (P0-P4 + P0_repeat)','flows':{'F0':'no current control','F1':'constant current control','F2':'v10 measured-current'},'expected':18,'records':len(rows),'all_passed':all(v['passed'] and v['checks_all_true'] for v in rows.values()) and not missing,'missing':missing,'rows':rows,'scope':'simulator joint trials; five real S4 pilots and 18 S4 Python remain separate'}
(BASE/'joint18_summary.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
md=['# S4 joint 18 matrix','',f'Records: {len(rows)}/18; missing: {", ".join(missing) if missing else "none"}','', '| case | arrival s | clearance m | margin m | RMS m | max m | passed |','|---|---:|---:|---:|---:|---:|---|']
for k,v in rows.items(): md.append(f"| {k} | {v['arrival_s']:.2f} | {v['clearance_m']:.6f} | {v['margin_m']:.6f} | {v['rms_m']:.6f} | {v['max_m']:.6f} | {v['passed']} |")
(BASE/'JOINT18.md').write_text('\n'.join(md)+'\n',encoding='utf-8')
print(json.dumps({'records':len(rows),'missing':missing,'all_passed':out['all_passed']},indent=2))
