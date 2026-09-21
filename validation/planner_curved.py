"""Frozen P1 versus conservative circular-fillet P2; no production control edits."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
from time import perf_counter

from core import Path as GeometricPath, Pose
from controller import CascadedPID3DOFController
from guidance import ProjectionConfig, LOSConfig
from navigation import Navigator, LOSNavigationAdapter, ControlledStartupPolicy, TerminalCapturePolicy
from physics import Python3DOFBackend
from planner import EnvironmentPlanningSpace, PlanningRequest, PlanningResult, PlanningStatus
from trajectory import generate_trajectory
from trajectory.circular_fillet import circular_fillet, FilletConfig, FilletStatus
from simulation.vehicle_config import load_synthetic_parameters
from validation.planner_paired import inputs, write
from validation.guidance_matched_comparison import (ROOT, FrozenPlanner, compact_control, common_metrics,
    failure_evidence, fingerprint, digest, RUNTIME_FOLDERS)
from validation.navigation_acceptance import check_execution


def prepare_curves(config, prepared):
    """Never tune on closed-loop results; use only already frozen P1 candidates."""
    source = ROOT/config['source_dataset']
    if digest(source/'manifest.json') != config['source_manifest_sha256']:
        raise ValueError('original P0/P1 manifest changed')
    for name,expected in json.loads((source/'manifest.json').read_text()).items():
        if digest(source/name)!=expected:
            raise ValueError('original evidence changed: '+name)
    original = json.loads((source/'geometry.json').read_text())
    pairs, audits = [], []
    fc = FilletConfig(**config['fillet'])
    for item in prepared:
        name,req,cfg,tcfg,_,_,common = item
        old = next(g for g in original if g['scene']==name)
        p1 = GeometricPath(tuple(Pose(**p) for p in old['arms']['P1']['path']))
        candidates = [(None,p1)]
        seen = {tuple((p.x,p.z) for p in p1)}
        for c in sorted([c for c in old['candidates'] if 'geometry' in c],
                        key=lambda c:(c['geometry']['total_cost'],c['index'])):
            key = tuple(tuple(p) for p in c['path'])
            if key not in seen:
                points = [Pose(x=x,z=z) for x,z in key]; points[0] = req.initial_state.pose
                candidates.append((c['index'],GeometricPath(tuple(points))))
                seen.add(key)
        space = EnvironmentPlanningSpace(req.environment)
        preq = PlanningRequest(req.initial_state,req.goal,req.planning_constraints)
        trials, accepted = [], None
        begun = perf_counter()
        for index,path in candidates:
            result = circular_fillet(path,preq,space,fc)
            trials.append(dict(candidate_index=index,status=result.status.value,reason=result.reason,
                corner_index=result.corner_index,source_path=[asdict(p) for p in path],
                arcs=[asdict(a) for a in result.arcs],
                curved_path=[asdict(p) for p in result.path] if result.path else None))
            if result.status is FilletStatus.ACCEPTED:
                accepted = result.path
                break
        audit = dict(scene=name,trials=trials,accepted=accepted is not None,
            source_candidate_changed=bool(accepted and trials[-1]['candidate_index'] is not None),
            shaping_time=perf_counter()-begun,scene_definition=common,required_clearance=preq.constraints.required_clearance,
            evidence_status='UNKNOWN',reason='Geometric sweep certificate is not dynamic execution evidence')
        audits.append(audit)
        for arm,path in [('P1',p1),('P2',accepted)]:
            pairs.append((name,arm,req,cfg,tcfg,path,audit))
    return pairs,audits


def audit_hash(audits):
    return fingerprint([{k:v for k,v in a.items() if k!='shaping_time'} for a in audits])


def execute(pair,parameters,phase,repeat,source):
    name,arm,req,cfg,tcfg,path,audit = pair
    if path is None:
        return dict(summary=dict(scene=name,arm=arm,repeat=repeat,status='UNSMOOTHABLE',
            evidence_status='UNKNOWN',message=audit['trials'][-1]['reason']),history=None)
    plan = PlanningResult(PlanningStatus.SUCCESS,'frozen_'+arm,path=path,path_cost=path.length())
    space = EnvironmentPlanningSpace(req.environment)
    generated = generate_trajectory(path,space,req.planning_constraints,tcfg,
        start_time=req.initial_state.timestamp,request=PlanningRequest(req.initial_state,req.goal,req.planning_constraints))
    # No shortcut: the exact sampled arc vertices must survive trajectory preparation.
    if generated.shortcut_path != path:
        raise ValueError('trajectory preparation changed fillet vertices')
    nav = Navigator(FrozenPlanner(plan),CascadedPID3DOFController(parameters),
        Python3DOFBackend(load_synthetic_parameters()),cfg,tcfg,
        generator=lambda *a,**k:generated,startup_policy=ControlledStartupPolicy(),
        capture_policy=TerminalCapturePolicy(),guidance_adapter=LOSNavigationAdapter(
            ProjectionConfig(**phase['projection']),LOSConfig(**{**phase['los'],'nominal_speed':tcfg.nominal_speed})))
    result = nav.run(req)
    history = dict(states=[asdict(s) for s in result.state_history],
        references=[asdict(s) for s in result.reference_history],
        controls=[compact_control(s) for s in result.control_history])
    metrics = common_metrics(result,path,req.environment,req.planning_constraints.required_clearance,cfg.dt,'los')
    following = [s for s in result.control_history if s.guidance_phase=='PATH_FOLLOWING']
    checks = check_execution(result,req,cfg)
    if arm=='P1':
        archived = json.loads((source/'runs'/f'{name}_P1_{repeat}.json').read_text())
        checks['frozen_P1_history_equal'] = fingerprint(history)==fingerprint(archived['history'])
        if not checks['frozen_P1_history_equal']:
            raise ValueError('P1 physical baseline drift: '+name)
    return dict(summary=dict(scene=name,arm=arm,repeat=repeat,status=result.status.name,
        message=result.message,path_length=path.length(),metrics=metrics,checks=checks,
        following_collision=any(s.collision for s in following),
        following_out_of_bounds=any(not s.in_bounds for s in following),
        failure=failure_evidence(result,path,req.environment,req.planning_constraints.required_clearance,cfg.dt,'los')
            if result.status.name!='SUCCESS' else None),history=history)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--stage',choices=['geometry','collision','remaining'],required=True)
    args = p.parse_args()
    config = json.loads((ROOT/'config/planner_curved_v1.json').read_text())
    _,phase,parameters,prepared = inputs()
    if config['fillet']['lookahead']!=phase['los']['lookahead']:
        raise ValueError('LOS lookahead mismatch')
    pairs,audits = prepare_curves(config,prepared)
    if args.stage=='geometry':
        args.output.mkdir(parents=True,exist_ok=False)
        write(args.output/'geometry.json',audits)
        sources = {str(p.relative_to(ROOT)).replace('\\','/'):digest(p)
                   for folder in RUNTIME_FOLDERS | {'validation'} for p in (ROOT/folder).rglob('*')
                   if p.is_file() and p.suffix in ('.py','.json','.yaml')}
        write(args.output/'freeze.json',dict(config=config,source_sha256=sources,geometry_sha256=audit_hash(audits)))
        for a in audits:
            print(a['scene'],'ACCEPTED' if a['accepted'] else 'UNSMOOTHABLE',a['trials'][-1]['reason'],flush=True)
        return
    freeze = json.loads((args.output/'freeze.json').read_text())
    if audit_hash(audits)!=freeze['geometry_sha256']:
        raise ValueError('geometry repeat mismatch')
    for name,expected in freeze['source_sha256'].items():
        if digest(ROOT/name)!=expected:
            raise ValueError('source changed after freeze: '+name)
    if args.stage=='remaining' and not (args.output/'collision_summary.json').exists():
        raise ValueError('collision-scene review must precede remaining scenes')
    run_dir = args.output/args.stage
    run_dir.mkdir(exist_ok=False)
    rows = []
    for pair in pairs:
        name,arm = pair[:2]
        if (name in config['collision_scenes_first']) != (args.stage=='collision'):
            continue
        signatures=[]
        for repeat in (1,2):
            record=execute(pair,parameters,phase,repeat,ROOT/config['source_dataset'])
            signature=fingerprint(dict(history=record['history'],
                summary={k:v for k,v in record['summary'].items() if k!='repeat'}))
            signatures.append(signature)
            record['deterministic_sha256']=signature
            write(run_dir/f'{name}_{arm}_{repeat}.json',record)
            rows.append(record['summary'])
            print(name,arm,repeat,record['summary']['status'],flush=True)
        if signatures[0]!=signatures[1]:
            raise ValueError('repeat mismatch')
    write(args.output/(args.stage+'_summary.json'),dict(rows=rows,deterministic=True))
    if args.stage=='remaining':
        all_rows=json.loads((args.output/'collision_summary.json').read_text())['rows']+rows
        write(args.output/'summary.json',dict(rows=all_rows,deterministic=True,
            counts={arm:dict(Counter(r['status'] for r in all_rows if r['arm']==arm and r['repeat']==1)) for arm in ('P1','P2')}))
        write(args.output/'manifest.json',{str(p.relative_to(args.output)):digest(p) for p in args.output.rglob('*') if p.is_file()})


if __name__=='__main__':
    main()
