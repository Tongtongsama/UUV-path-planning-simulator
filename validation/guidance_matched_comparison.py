"""Preregistered LOS-then-timed 16-case comparison; diagnostics never affect control."""
import argparse
from collections import Counter
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
from math import cos, sin, hypot, pi, sqrt
from pathlib import Path
import sys

import numpy as np
from core import Pose, Twist, VehicleState
from controller import CascadedPID3DOFController
from environment import Boundary, WorldModel, Obstacle, ConstantCurrent
from guidance import LOSConfig, ProjectionConfig
from navigation import (Navigator, NavigationConfig, NavigationRequest, ControlledStartupPolicy,
    StartupConfig, TerminalCapturePolicy, TerminalCaptureConfig, LOSNavigationAdapter)
from physics import Python3DOFBackend
from planner import AStarPlanner, AStarConfig, EnvironmentPlanningSpace, PlanningRequest, PlanningConstraints
from trajectory import TrajectoryConfig, generate_trajectory
from simulation.controller_config import load_baseline_controller_parameters
from simulation.vehicle_config import load_synthetic_parameters
from validation.navigation_acceptance import ROOT, build_cases, check_execution, clean
from validation.frozen_candidate_regression import signed_rectangle_margin
from validation.tracking_diagnostics import polyline_distance

RUNTIME_FOLDERS={'core','guidance','navigation','controller','physics','environment','planner','trajectory','simulation','config'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fingerprint(data):
    return hashlib.sha256(json.dumps(clean(data),sort_keys=True,allow_nan=False).encode()).hexdigest()


def wrapped_step(values):
    """Maximum shortest-angle step, including transitions when supplied."""
    return max((abs((b-a+pi)%(2*pi)-pi) for a,b in zip(values,values[1:])),default=0.)


def phase(step,trajectory):
    """Method-independent physical phase; direct guidance alone is not capture."""
    if step.guidance_phase!='TIMED_REFERENCE':return step.guidance_phase
    if step.startup_phase in ('BRAKE','ALIGN'):return 'STARTUP'
    time=step.reference_time if step.reference_time is not None else step.time
    return 'TERMINAL_CAPTURE' if time>=trajectory.end_time else 'PATH_FOLLOWING'


def compact_control(step):
    record=asdict(step)
    if record['guidance_command'] is not None:
        record['guidance_command']['projection'].pop('next_state')
    for key in ('position_error','pitch_error','velocity_error'):
        record['archived_timed_'+key]=record.pop(key)
    return record


def nearest_projection(point,path):
    """Global nearest finite segment FOR DIAGNOSTICS ONLY; ties use lower index."""
    best=None
    arc=0.
    for i,(a,b) in enumerate(zip(path,path.poses[1:])):
        dx,dz=b.x-a.x,b.z-a.z
        length=hypot(dx,dz)
        if not length:continue
        tx,tz=dx/length,dz/length
        along=(point.x-a.x)*tx+(point.z-a.z)*tz
        mu=max(0.,min(1.,along/length))
        x,z=a.x+mu*dx,a.z+mu*dz
        distance=hypot(point.x-x,point.z-z)
        value=(distance,i,arc+mu*length,-(point.x-x)*tz+(point.z-z)*tx)
        if best is None or value[:2]<best[:2]:best=value
        arc+=length
    return best or (hypot(point.x-path[0].x,point.z-path[0].z),0,0.,0.)


def common_metrics(result,path,world,required,dt,arm):
    """Same spatial/error/envelope definitions for both methods; no stop guards."""
    steps,states=result.control_history,result.state_history
    xy=np.array([[s.pose.x,s.pose.z] for s in states])
    vertices=np.array([[p.x,p.z] for p in path])
    distances=polyline_distance(xy,vertices) if len(path)>1 else np.linalg.norm(xy-vertices[0],axis=1)
    phases=[phase(s,result.trajectory) for s in steps]
    ids=sorted({j for i,p in enumerate(phases) if p=='PATH_FOLLOWING' for j in (i,i+1)})
    following=distances[ids]
    effective=[s.controller_diagnostics.pitch_target if s.controller_diagnostics is not None
               else s.effective_guidance.pitch if s.effective_guidance is not None
               else result.reference_history[i].pose.pitch for i,s in enumerate(steps)]
    raw=[s.guidance_command.pitch_ref if s.guidance_command is not None else result.reference_history[i].pose.pitch
         for i,s in enumerate(steps) if phases[i]=='PATH_FOLLOWING']
    commands=[s.guidance_command for s in steps if s.guidance_command is not None]
    rates=[abs(s.twist.q) for s in states]
    exceed=next((i for i,q in enumerate(rates) if q>.35),None)
    per_phase={p:wrapped_step([a for a,k in zip(effective,phases) if k==p])
               for p in ('STARTUP','PATH_FOLLOWING','TERMINAL_CAPTURE')}
    progress=[s.path_progress_s for s in steps] if arm=='los' else []
    following_steps=[s for s,p in zip(steps,phases) if p=='PATH_FOLLOWING']
    stall=False
    if arm=='los' and result.status.name=='TIMEOUT' and following_steps and phases[-1]=='PATH_FOLLOWING':
        last=following_steps[-1]
        tail=[s for s in following_steps if s.time>=last.time-5.-1e-9]
        stall=last.time-tail[0].time>=5.-1e-9 and last.path_progress_s-tail[0].path_progress_s<=.01
    metrics=dict(result.metrics)
    metrics.update(finite_polyline_rmse=float(np.sqrt(np.mean(distances**2))),
        finite_polyline_max=float(max(distances)),
        following_polyline_rmse=float(np.sqrt(np.mean(following**2))) if ids else None,
        following_polyline_max=float(max(following)) if ids else None,
        minimum_boundary_margin=min(signed_rectangle_margin(s.pose,world.boundary.bounds,required) for s in states),
        maximum_abs_q=max(rates),rate_envelope_violation=max(rates)>.35,
        first_rate_exceedance_time=states[exceed].timestamp if exceed is not None else None,
        maximum_effective_pitch_step=wrapped_step(effective),maximum_raw_following_pitch_step=wrapped_step(raw),
        maximum_effective_pitch_step_by_phase=per_phase,
        progress_clamp_count=sum(c.projection.progress_clamped for c in commands) if arm=='los' else None,
        switch_limit_count=sum(c.projection.switch_limit_reached for c in commands) if arm=='los' else None,
        committed_switch_count=sum(c.switch_count for c in commands) if arm=='los' else None,
        progress_monotonicity_violations=sum(b<a for a,b in zip(progress,progress[1:])) if arm=='los' else None,
        progress_stall=stall if arm=='los' else None,
        capture_phase_duration=sum(p=='TERMINAL_CAPTURE' for p in phases)*dt,
        censored=result.status.name!='SUCCESS')
    return metrics


def failure_evidence(result,path,world,required,dt,arm):
    """First safety event, otherwise last observed transition, without causal claims."""
    steps=result.control_history
    index=next((i for i,s in enumerate(steps) if s.collision or not s.in_bounds),None)
    safety=index is not None
    if index is None:index=len(steps)-1 if steps else None
    if index is None:
        return dict(phase='STARTUP' if result.status.name=='STARTUP_FAILED' else 'NO_TRANSITION',
                    time=result.final_state.timestamp,message=result.message)
    step=steps[index];before,after=result.state_history[index:index+2]
    ref=result.reference_history[index]
    current_phase=phase(step,result.trajectory)
    diagnostic=nearest_projection(before.pose,path)
    guide=step.guidance_command
    raw_point=guide.lookahead_point if guide is not None else ref.pose
    raw_pitch=guide.pitch_ref if guide is not None else ref.pose.pitch
    dx,dz=ref.pose.x-before.pose.x,ref.pose.z-before.pose.z
    target_arc=guide.progress_s if guide else nearest_projection(ref.pose,path)[2]
    corner_arcs=[];arc=0.
    for a,b in zip(path,path.poses[1:]):
        arc+=hypot(b.x-a.x,b.z-a.z);corner_arcs.append(arc)
    near_corner=any(abs(target_arc-s)<=1. for s in corner_arcs[:-1])
    return dict(first_safety_event=safety,transition=index,phase=current_phase,
        time_start=before.timestamp,time_end=after.timestamp,near_corner_one_metre=near_corner,
        actual_before=asdict(before),actual_after=asdict(after),raw_reference_position=asdict(raw_point),
        raw_pitch=raw_pitch,effective_guidance=asdict(step.effective_guidance) if step.effective_guidance else None,
        active_segment=guide.segment_index if guide else None,
        diagnostic_nearest_actual_segment=diagnostic[1],path_distance=diagnostic[0],
        cross_track=guide.cross_track_error if guide else diagnostic[3],
        projected_s=guide.projected_s if guide else None,committed_s=step.path_progress_s,
        reference_lead=dx*cos(ref.pose.pitch)+dz*sin(ref.pose.pitch) if arm=='timed' else None,
        commanded=asdict(step.commanded),applied=asdict(step.applied),at_limit=step.at_limit,
        controller_diagnostics=asdict(step.controller_diagnostics) if step.controller_diagnostics else None,
        raw_reference_point_clearance=world.clearance(raw_point),
        executed_clearance=step.segment_clearance,net_clearance=step.segment_clearance-required,
        boundary_margin=min(signed_rectangle_margin(p.pose,world.boundary.bounds,required) for p in (before,after)),
        collision=step.collision,out_of_bounds=not step.in_bounds,
        note='Nearest-segment/phase diagnostics are observations, not a causal proof.')


class FrozenPlanner:
    def __init__(self,plan):self.plan_result=plan
    def plan(self,request,space):return self.plan_result


def prepare(settings,parameters):
    """Build each A* and postprocessed path once, shared by all four executions."""
    prepared=[]
    for suite,case,current in build_cases('all',settings):
        cfg=NavigationConfig(**settings['navigation'])
        if case.get('force_limit'):cfg=replace(cfg,maximum_tau_x=case['force_limit'])
        world=WorldModel(Boundary.rectangle(**suite['boundary']),
            [Obstacle.rectangle(**o) for o in case['obstacles']],ConstantCurrent(current))
        initial=VehicleState(Pose(**suite['start'],pitch=case.get('pitch',0)),Twist(u=case.get('speed',0)))
        goal=Pose(x=case['goal'][0],z=case['goal'][1]) if 'goal' in case else Pose(**suite['goal'])
        req=NavigationRequest(initial,goal,world,PlanningConstraints(**suite['constraints']))
        preq=PlanningRequest(initial,goal,req.planning_constraints)
        space=EnvironmentPlanningSpace(world)
        plan=AStarPlanner(AStarConfig(**suite['astar'])).plan(preq,space)
        if not plan.success:raise ValueError(f"preflight planning failed: {case['name']}")
        tcfg=TrajectoryConfig(**settings['trajectory'])
        generated=generate_trajectory(plan.path,space,req.planning_constraints,tcfg,start_time=initial.timestamp,request=preq)
        common=dict(scene=case,boundary=suite['boundary'],current=current,initial=asdict(initial),goal=asdict(goal),
            constraints=asdict(req.planning_constraints),navigation=asdict(cfg),trajectory=asdict(tcfg),
            astar=suite['astar'],raw_path=[asdict(p) for p in plan.path],processed_path=[asdict(p) for p in generated.shortcut_path],
            sampled_path=[asdict(p) for p in generated.sampled_path],
            timed_trajectory=[asdict(s) for s in generated.trajectory],controller=asdict(parameters),
            startup=asdict(StartupConfig()),capture=asdict(TerminalCaptureConfig()),
            physics_yaml=(ROOT/'config/vehicles/uuv_3dof_synthetic_v1.yaml').read_text())
        prepared.append((case['name'],req,cfg,tcfg,plan,generated,common))
    if len(prepared)!=16 or len({p[0] for p in prepared})!=16:raise ValueError('expected sixteen unique fixtures')
    return prepared


def run_case(prepared,parameters,phase_b,arm):
    name,req,cfg,tcfg,plan,generated,common=prepared
    adapter=LOSNavigationAdapter(ProjectionConfig(**phase_b['projection']),
        LOSConfig(**{**phase_b['los'],'nominal_speed':tcfg.nominal_speed})) if arm=='los' else None
    nav=Navigator(FrozenPlanner(plan),CascadedPID3DOFController(parameters),
        Python3DOFBackend(load_synthetic_parameters()),cfg,tcfg,
        generator=lambda *args,**kwargs:generated,capture_policy=TerminalCapturePolicy(),
        startup_policy=ControlledStartupPolicy(),guidance_adapter=adapter)
    first=nav.run(req);second=nav.run(req)
    deterministic=(first.status==second.status and first.state_history==second.state_history
        and first.reference_history==second.reference_history and first.control_history==second.control_history
        and first.metrics==second.metrics and first.message==second.message)
    required=req.planning_constraints.required_clearance
    metrics=common_metrics(first,generated.shortcut_path,req.environment,required,cfg.dt,arm)
    checks=check_execution(first,req,cfg)
    checks.update(deterministic=deterministic,no_secondary_clip=first.metrics['navigation_clip_count']==0,
        shared_generated_trajectory=first.trajectory==generated.trajectory,
        shared_raw_path=first.planning_result.path==plan.path)
    if arm=='los':
        steps=first.control_history
        checks.update(progress_monotonic=metrics['progress_monotonicity_violations']==0,
            progress_frozen_outside_following=all(s.guidance_command is None and s.path_progress_s==
                (steps[i-1].path_progress_s if i else 0.) for i,s in enumerate(steps) if s.guidance_phase!='PATH_FOLLOWING'))
    row=dict(name=name,arm=arm,status=first.status.name,message=first.message,checks=checks,metrics=metrics,
        shared_input_sha256=fingerprint(common),
        failure_evidence=failure_evidence(first,generated.shortcut_path,req.environment,required,cfg.dt,arm)
            if first.status.name!='SUCCESS' else None)
    history=dict(states=[asdict(s) for s in first.state_history],
        archived_timed_references=[asdict(s) for s in first.reference_history],
        controls=[compact_control(s) for s in first.control_history])
    return row,history


def verify_candidate(candidate):
    for name,expected in candidate['source_sha256'].items():
        if name.split('/')[0] in RUNTIME_FOLDERS and digest(ROOT/name)!=expected:
            raise ValueError(f'frozen runtime mismatch: {name}')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--candidate',type=Path,default=ROOT/'artifacts/guidance-v1.0-phase-b-candidate')
    args=parser.parse_args()
    candidate=json.loads((args.candidate/'candidate.json').read_text())
    for name,expected in json.loads((args.candidate/'manifest.json').read_text()).items():
        if digest(args.candidate/name)!=expected:raise ValueError(f'candidate archive changed: {name}')
    verify_candidate(candidate)
    settings=json.loads((ROOT/'config/navigation_v08.json').read_text())
    phase_b=json.loads((ROOT/'config/guidance_phase_b.json').read_text())
    parameters=load_baseline_controller_parameters(ROOT/phase_b['controller'])
    args.output.mkdir(parents=True,exist_ok=False)
    contract=ROOT/'docs/guidance_matched_16_contract.md'
    (args.output/'contract.md').write_text(contract.read_text(encoding='utf-8'),encoding='utf-8')
    experiment=dict(candidate_sha256=digest(args.candidate/'candidate.json'),git_head=candidate['git_head'],
        created_utc=datetime.now(timezone.utc).isoformat(),phase_b=phase_b,scene_settings=settings,
        contract_sha256=digest(contract),runner_sha256=digest(Path(__file__)),executable=sys.executable,
        arm_order=['los','timed'],repeats=2,slowdown=False,rate_diagnostic_threshold=.35)
    (args.output/'experiment.json').write_text(json.dumps(experiment,indent=2),encoding='utf-8')
    prepared=prepare(settings,parameters)
    common_dir=args.output/'common';common_dir.mkdir()
    for name,*_,common in prepared:
        (common_dir/(name+'.json')).write_text(json.dumps(clean(common),allow_nan=False),encoding='utf-8')
    reports=[]
    for arm in ('los','timed'):
        folder=args.output/arm;folder.mkdir()
        for item in prepared:
            verify_candidate(candidate)
            row,history=run_case(item,parameters,phase_b,arm)
            reports.append(row)
            (folder/(row['name']+'.json')).write_text(json.dumps(clean(dict(summary=row,history=history)),allow_nan=False),encoding='utf-8')
            print(arm,row['name'],row['status'],'rate=',row['metrics']['rate_envelope_violation'],flush=True)
        (folder/'summary.json').write_text(json.dumps(clean([r for r in reports if r['arm']==arm]),indent=2,allow_nan=False),encoding='utf-8')
    verify_candidate(candidate)
    pairs=[dict(name=p[0],**{arm:next(r for r in reports if r['arm']==arm and r['name']==p[0]) for arm in ('los','timed')}) for p in prepared]
    matched=all(p['los']['shared_input_sha256']==p['timed']['shared_input_sha256'] for p in pairs)
    summary=dict(experiment=experiment,matched_inputs=matched,pairs=pairs,
        evidence_valid=matched and all(all(r['checks'].values()) for r in reports),
        outcomes={arm:dict(Counter(r['status'] for r in reports if r['arm']==arm)) for arm in ('los','timed')},
        rate_violations={arm:sum(r['metrics']['rate_envelope_violation'] for r in reports if r['arm']==arm) for arm in ('los','timed')})
    (args.output/'comparison.json').write_text(json.dumps(clean(summary),indent=2,allow_nan=False),encoding='utf-8')
    (args.output/'manifest.json').write_text(json.dumps({p.relative_to(args.output).as_posix():digest(p)
        for p in args.output.rglob('*') if p.is_file()},indent=2),encoding='utf-8')
    allowed={'SUCCESS','COLLISION','OUT_OF_BOUNDS','STARTUP_FAILED','CAPTURE_FAILED','TIMEOUT'}
    if not summary['evidence_valid'] or any(r['status'] not in allowed for r in reports):raise SystemExit(1)


if __name__=='__main__':main()
