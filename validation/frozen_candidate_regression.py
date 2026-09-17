"""Frozen 16-scenario regression; classify evidence without tuning any policy."""
import argparse
from bisect import bisect_right
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
from math import cos, sin, hypot
from pathlib import Path
import subprocess
import sys
from core import Pose, Twist, VehicleState
from controller import CascadedPID3DOFController
from environment import Boundary, WorldModel, Obstacle, ConstantCurrent
from navigation import Navigator, NavigationRequest, NavigationConfig, ControlledStartupPolicy, TerminalCapturePolicy
from physics import Python3DOFBackend
from planner import AStarPlanner, AStarConfig, PlanningConstraints
from trajectory import TrajectoryConfig, sample_reference
from simulation.controller_config import load_baseline_controller_parameters
from simulation.vehicle_config import load_synthetic_parameters
from validation.navigation_acceptance import ROOT, build_cases, check_execution, clean
from validation.controlled_startup_experiments import corner_timing


def signed_rectangle_margin(pose, bounds, clearance=0.):
    """Signed distance to rectangular edges minus required footprint clearance."""
    xmin,zmin,xmax,zmax=bounds
    return min(pose.x-xmin,xmax-pose.x,pose.z-zmin,zmax-pose.z)-clearance


def reference_segment(trajectory, time):
    """Zero-based outgoing sampled segment; clamp terminal queries to last segment."""
    return max(0,min(len(trajectory)-2,bisect_right([s.timestamp for s in trajectory],time)-1))


def first_violation(result, world, required, scene_name):
    """Observed phase and diagnostics, not a causal verdict. Safety wins over labels."""
    index=next((i for i,s in enumerate(result.control_history) if s.collision or not s.in_bounds),None)
    if index is None:return None
    step=result.control_history[index]; a,b=result.state_history[index:index+2]
    ref=result.reference_history[index]; t=step.reference_time if step.reference_time is not None else step.time
    traj=result.trajectory; segment=reference_segment(traj,t)
    corners=[i for i in range(1,len(traj)-1) if abs(traj[i].pose.pitch-traj[i-1].pose.pitch)>1e-8]
    leg=sum(i<=segment for i in corners)
    arc=[0.]
    for u,v in zip(traj,traj.states[1:]):arc.append(arc[-1]+hypot(v.pose.x-u.pose.x,v.pose.z-u.pose.z))
    progress=arc[segment]+hypot(ref.pose.x-traj[segment].pose.x,ref.pose.z-traj[segment].pose.z)
    near=any(abs(progress-arc[i])<=1. for i in corners)
    if step.startup_phase in ('BRAKE','ALIGN'):phase='startup'
    elif step.effective_guidance is not None or t>=traj.end_time:phase='terminal_capture_or_hold'
    elif near:phase='corner_zone'
    elif leg==0:phase='first_leg'
    elif leg==len(corners):phase='final_leg'
    else:phase='after_corner'
    dx,dz=a.pose.x-ref.pose.x,a.pose.z-ref.pose.z
    diag=step.controller_diagnostics
    next_ref=sample_reference(traj,t+.05)
    return dict(transition_index=index,time_start=a.timestamp,time_end=b.timestamp,
        phase=phase,narrow_passage_context=('narrow_passage' in scene_name or scene_name.startswith('current_')),
        sampled_trajectory_segment=segment,geometric_leg=leg,reference_time=t,
        reference=asdict(ref),actual_before=asdict(a),actual_after=asdict(b),
        cross_track=-dx*sin(ref.pose.pitch)+dz*cos(ref.pose.pitch),
        reference_lead=-dx*cos(ref.pose.pitch)-dz*sin(ref.pose.pitch),
        reference_pitch_error=step.pitch_error,effective_pitch_error=diag.pitch_error if diag else None,
        controller_diagnostics=asdict(diag) if diag else None,commanded=asdict(step.commanded),applied=asdict(step.applied),
        pitch_limit_fraction=abs(step.applied.tau_m)/diag.effective_limits.maximum_tau_m if diag else None,
        surge_limit_fraction=abs(step.applied.tau_x)/diag.effective_limits.maximum_tau_x if diag else None,
        reference_segment_clearance=world.segment_clearance(ref.pose,next_ref.pose),
        executed_segment_clearance=step.segment_clearance,executed_net_margin=step.segment_clearance-required,
        boundary_distance_before=world.boundary.distance_to_boundary(a.pose),
        boundary_distance_after=world.boundary.distance_to_boundary(b.pose),
        signed_eroded_boundary_margin=min(signed_rectangle_margin(s.pose,world.boundary.bounds,required) for s in (a,b)),
        collision=step.collision,out_of_bounds=not step.in_bounds,
        note='Phase is indexed by frozen reference progress, not proof of physical cause. Corner zone = +/-1 m arc.')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    out=parser.parse_args().output;out.mkdir(parents=True,exist_ok=False)
    settings=json.loads((ROOT/'config/navigation_v08.json').read_text())
    parameters=load_baseline_controller_parameters(ROOT/'config/controllers/cascaded_pid_3dof_restoring_candidate.yaml')
    reports=[]
    for suite,case,current in build_cases('all',settings):
        config=NavigationConfig(**settings['navigation'])
        if case.get('force_limit'):config=replace(config,maximum_tau_x=case['force_limit'])
        world=WorldModel(Boundary.rectangle(**suite['boundary']),[Obstacle.rectangle(**o) for o in case['obstacles']],ConstantCurrent(current))
        initial=VehicleState(Pose(**suite['start'],pitch=case.get('pitch',0)),Twist(u=case.get('speed',0)))
        goal=Pose(x=case['goal'][0],z=case['goal'][1]) if 'goal' in case else Pose(**suite['goal'])
        request=NavigationRequest(initial,goal,world,PlanningConstraints(**suite['constraints']))
        nav=Navigator(AStarPlanner(AStarConfig(**suite['astar'])),CascadedPID3DOFController(parameters),
            Python3DOFBackend(load_synthetic_parameters()),config,TrajectoryConfig(**settings['trajectory']),
            generator=corner_timing,capture_policy=TerminalCapturePolicy(),startup_policy=ControlledStartupPolicy())
        a=nav.run(request);b=nav.run(request)
        deterministic=(a.status==b.status and a.state_history==b.state_history and a.reference_history==b.reference_history
            and a.control_history==b.control_history and a.metrics==b.metrics and a.message==b.message)
        checks=check_execution(a,request,config)
        checks['no_secondary_clip']=a.metrics['navigation_clip_count']==0
        row=dict(name=case['name'],status=a.status.name,message=a.message,deterministic=deterministic,
            checks=checks,must_succeed=case['must_succeed'],baseline_requirement_met=not case['must_succeed'] or a.status.name=='SUCCESS',
            metrics=a.metrics,first_safety_violation=first_violation(a,world,request.planning_constraints.required_clearance,case['name']))
        reports.append(row)
        evidence=dict(summary=row,scene=case,boundary=suite['boundary'],current=current,initial_state=asdict(initial),goal=asdict(goal),
            navigation=asdict(config),trajectory_config=settings['trajectory'],astar=suite['astar'],constraints=suite['constraints'],
            controller=asdict(parameters),startup=asdict(nav.startup_policy.config),capture=asdict(nav.capture_policy.config),
            local_slowdown=dict(speed=.2,half_width=1.,selection='sample interval midpoint; internal knots'),
            physics=(ROOT/'config/vehicles/uuv_3dof_synthetic_v1.yaml').read_text(),
            states=[asdict(s) for s in a.state_history],references=[asdict(s) for s in a.reference_history],
            controls=[asdict(s) for s in a.control_history],trajectory=[asdict(s) for s in a.trajectory] if a.trajectory else None,
            raw_path=[asdict(p) for p in a.planning_result.path] if a.planning_result and a.planning_result.path else None)
        (out/(case['name']+'.json')).write_text(json.dumps(clean(evidence),allow_nan=False),encoding='utf-8')
        print(case['name'],a.status.name,a.message,flush=True)
    tests=subprocess.run([sys.executable,'-m','pytest','tests','-q','-p','no:cacheprovider'],cwd=ROOT,capture_output=True,text=True)
    (out/'pytest.log').write_text(tests.stdout+tests.stderr,encoding='utf-8')
    summary=dict(date=datetime.now(timezone.utc).isoformat(),cases=reports,tests_exit_code=tests.returncode,
        evidence_valid=all(r['deterministic'] and all(r['checks'].values()) for r in reports),
        baseline_requirements_met=all(r['baseline_requirement_met'] for r in reports),
        installed=subprocess.check_output([sys.executable,'-m','pip','freeze'],text=True).splitlines(),
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
            for folder in ('core','controller','navigation','physics','planner','trajectory','simulation','environment','validation','tests','config')
            for p in (ROOT/folder).rglob('*') if p.is_file() and p.suffix in ('.py','.yaml','.json')})
    (out/'summary.json').write_text(json.dumps(clean(summary),indent=2,allow_nan=False),encoding='utf-8')
    (out/'manifest.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file()},indent=2),encoding='utf-8')
    if tests.returncode or not summary['evidence_valid']:raise SystemExit(1)


if __name__=='__main__':main()
