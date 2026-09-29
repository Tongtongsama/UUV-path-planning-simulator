"""Frozen-path, preregistered three-scene approach comparison, two repeats."""
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
from controller import CascadedPID3DOFController
from core import Path as GeometricPath, Pose
from guidance import LOSConfig, ProjectionConfig
from navigation import Navigator, LOSNavigationAdapter, ControlledStartupPolicy, TerminalCapturePolicy
from navigation.terminal_approach import TerminalApproachConfig, TerminalApproachPolicy
from physics import Python3DOFBackend
from planner import EnvironmentPlanningSpace, PlanningRequest
from trajectory import generate_trajectory
from simulation.vehicle_config import load_synthetic_parameters
from validation.planner_paired import inputs, write
from validation.guidance_matched_comparison import ROOT, FrozenPlanner, compact_control, digest, fingerprint, RUNTIME_FOLDERS


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--full', action='store_true')
    parser.add_argument('--gate',type=Path,default=ROOT/'artifacts/terminal_approach_v1')
    args=parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'runs').mkdir()
    config=json.loads((ROOT/'config/terminal_approach_v1.json').read_text())
    paired,phase,parameters,prepared=inputs()
    if args.full:
        gate=json.loads((args.gate/'summary.json').read_text())
        gate_manifest={k.replace('\\','/'):v for k,v in json.loads((args.gate/'manifest.json').read_text()).items()}
        for name,sha in gate_manifest.items():
            if digest(args.gate/name)!=sha:raise ValueError('pilot evidence mismatch')
        pilot=json.loads((args.gate/'freeze.json').read_text())
        if not gate['three_scene_success_gate'] or pilot['config']['policy']!=config['policy']:
            raise ValueError('full run requires passed unchanged-parameter pilot')
        config['scenes']=paired['scenes']
        config['pilot_manifest_sha256']=digest(args.gate/'manifest.json')
    old=ROOT/'artifacts/planner_paired_v1'
    manifest={k.replace('\\','/'):v for k,v in json.loads((old/'manifest.json').read_text()).items()}
    for name in ['geometry.json']+[f'runs/{scene}_P1_1.json' for scene in config['scenes']]:
        if digest(old/name)!=manifest[name]:raise ValueError('frozen source mismatch: '+name)
    geometry=json.loads((old/'geometry.json').read_text())
    sources={str(p.relative_to(ROOT)):digest(p) for folder in RUNTIME_FOLDERS|{'validation','config'}
             for p in (ROOT/folder).rglob('*') if p.is_file() and p.suffix in ('.py','.json','.yaml')}
    write(args.output/'freeze.json',dict(config=config,phase=phase,source_sha256=sources,
        geometry_sha256=digest(old/'geometry.json')))
    rows=[]
    for scene in config['scenes']:
        _,req,cfg,tcfg,plan,_,_=next(p for p in prepared if p[0]==scene)
        path=GeometricPath.from_list([Pose(**p) for p in next(r for r in geometry if r['scene']==scene)['arms']['P1']['path']])
        plan=replace(plan,path=path)
        generated=generate_trajectory(path,EnvironmentPlanningSpace(req.environment),req.planning_constraints,tcfg,
            start_time=req.initial_state.timestamp,request=PlanningRequest(req.initial_state,req.goal,req.planning_constraints))
        for arm in ('P1','APPROACH'):
            signatures=[]
            for repeat in (1,2):
                policy=TerminalApproachPolicy(TerminalApproachConfig(**config['policy'])) if arm=='APPROACH' else None
                nav=Navigator(FrozenPlanner(plan),CascadedPID3DOFController(parameters),Python3DOFBackend(load_synthetic_parameters()),
                    cfg,tcfg,generator=lambda *a,**k:generated,startup_policy=ControlledStartupPolicy(),
                    capture_policy=TerminalCapturePolicy(),guidance_adapter=LOSNavigationAdapter(
                        ProjectionConfig(**phase['projection']),LOSConfig(**{**phase['los'],'nominal_speed':tcfg.nominal_speed})),
                    approach_policy=policy)
                result=nav.run(req)
                history=dict(states=[asdict(s) for s in result.state_history],references=[asdict(s) for s in result.reference_history],
                             controls=[compact_control(s) for s in result.control_history])
                archived_match=None
                if arm=='P1':
                    archived=json.loads((old/f'runs/{scene}_P1_1.json').read_text())
                    archived_match=fingerprint(history)==fingerprint(archived['history'])
                    if not archived_match:raise ValueError('disabled behavior changed: '+scene)
                diagnostics=None if policy is None else dict(evaluations=policy.events,handoff=policy.handoff,
                    prediction_steps=policy.prediction_steps)
                signature=fingerprint(dict(history=history,diagnostics=diagnostics,status=result.status.name,metrics=result.metrics))
                signatures.append(signature)
                first=next((i for i,s in enumerate(result.control_history) if s.guidance_phase=='TERMINAL_APPROACH'),None)
                row=dict(scene=scene,arm=arm,repeat=repeat,status=result.status.name,message=result.message,
                    metrics=result.metrics,archived_history_match=archived_match,
                    first_approach_state=asdict(result.state_history[first]) if first is not None else None,
                    prediction_wall_seconds=policy.wall_seconds if policy else 0.,deterministic_sha256=signature)
                write(args.output/'runs'/f'{scene}_{arm}_{repeat}.json',dict(summary=row,history=history,diagnostics=diagnostics))
                rows.append(row)
                print(scene,arm,repeat,result.status.name, 'predicted steps',policy.prediction_steps if policy else 0,flush=True)
            if signatures[0]!=signatures[1]:raise ValueError('repeat mismatch')
    for name,expected in sources.items():
        if digest(ROOT/name)!=expected:raise ValueError('source changed during run: '+name)
    passed=all(r['status']=='SUCCESS' for r in rows if r['arm']=='APPROACH')
    write(args.output/'summary.json',dict(rows=rows,deterministic=True,all_approach_success=passed,
        three_scene_success_gate=passed if not args.full else None,
        full_12_authorized_by_gate=True if args.full else passed,
        interpretation='Finite candidate failure is UNKNOWN, not physical infeasibility.'))
    write(args.output/'manifest.json',{str(p.relative_to(args.output)):digest(p) for p in args.output.rglob('*') if p.is_file()})


if __name__=='__main__':main()
