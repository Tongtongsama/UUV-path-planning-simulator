"""Read-only diagnosis of frozen double-baffle handoff; no policy changes."""
import json
from collections import Counter
from math import hypot, pi
from pathlib import Path
from validation.guidance_matched_comparison import ROOT, digest, fingerprint
from validation.planner_paired import write


def analyze(data,path,entry_distance):
    """Evaluate actual LOS readiness predicates separately from hypothetical guards."""
    length=sum(hypot(b['x']-a['x'],b['z']-a['z']) for a,b in zip(path,path[1:]))
    rows=[]
    for i,step in enumerate(data['history']['controls']):
        command=step['guidance_command']
        if command is None:continue
        state=data['history']['states'][i];pose=state['pose'];twist=state['twist']
        distance=hypot(pose['x']-path[-1]['x'],pose['z']-path[-1]['z'])
        remaining=length-step['path_progress_s']
        predicates=dict(final_segment=command['projection']['segment_index']==len(path)-2,
                        remaining_arc=remaining<=entry_distance,goal_distance=distance<=entry_distance)
        if all(predicates.values())!=command['terminal_ready']:raise ValueError('readiness reconstruction mismatch')
        rows.append(dict(step=i,time=step['time'],phase=step['guidance_phase'],distance=distance,
            remaining_arc=remaining,predicates=predicates,terminal_ready=command['terminal_ready'],
            ground_speed=hypot(twist['u'],twist['w']),q=twist['q'],pitch=pose['pitch'],
            state_before=state,controller_diagnostics=step['controller_diagnostics']))
    return dict(status=data['summary']['status'],entry_distance=entry_distance,
        phases=dict(Counter(s['guidance_phase'] for s in data['history']['controls'])),
        terminal_ready_count=sum(r['terminal_ready'] for r in rows),
        closest=min(rows,key=lambda r:r['distance']),last=rows[-1],
        first_final_segment=next((r for r in rows if r['predicates']['final_segment']),None),
        first_arc_qualified=next((r for r in rows if r['predicates']['final_segment'] and r['predicates']['remaining_arc']),None),
        caveat='Capture guards were never evaluated in this history; hypothetical guard checks are not the cause of non-handoff.')


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();source=ROOT/'artifacts/planner_paired_v1'
    manifest={k.replace('\\','/'):v for k,v in json.loads((source/'manifest.json').read_text()).items()}
    names=['geometry.json','freeze.json']+[f'runs/double_baffle_detour_P1_{i}.json' for i in (1,2)]
    for name in names:
        if digest(source/name)!=manifest[name]:raise ValueError('archived input mismatch')
    geometry=json.loads((source/'geometry.json').read_text())
    path=next(r for r in geometry if r['scene']=='double_baffle_detour')['arms']['P1']['path']
    freeze=json.loads((source/'freeze.json').read_text())
    tolerance=freeze['phase_b']['los']['terminal_entry_distance']
    results=[analyze(json.loads((source/f'runs/double_baffle_detour_P1_{i}.json').read_text()),path,tolerance) for i in (1,2)]
    if fingerprint(results[0])!=fingerprint(results[1]):raise ValueError('diagnostic repeats differ')
    args.output.mkdir(parents=True,exist_ok=False)
    write(args.output/'results.json',dict(repeats=results,repeats_equal=True,
        source_sha256={name:digest(source/name) for name in names},runner_sha256=digest(Path(__file__))))
    write(args.output/'manifest.json',{'results.json':digest(args.output/'results.json')})
    print(json.dumps({k:results[0][k] for k in ('status','phases','terminal_ready_count')},indent=2))


if __name__=='__main__':main()
