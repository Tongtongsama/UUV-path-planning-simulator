"""Ordered LOS adapter diagnostics, preserving failures and fixed inner control."""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
from math import pi, cos, sin
from pathlib import Path as FilePath

from core import Path, Pose, Twist, VehicleState
from controller import CascadedPID3DOFController
from environment import Boundary, WorldModel, ConstantCurrent
from guidance import LOSConfig, ProjectionConfig
from navigation import (Navigator, NavigationConfig, NavigationRequest, NavigationStatus,
    LOSNavigationAdapter, TerminalCapturePolicy, ControlledStartupPolicy)
from physics import Python3DOFBackend
from planner import PlanningConstraints
from trajectory import TrajectoryConfig
from simulation.controller_config import load_baseline_controller_parameters
from simulation.vehicle_config import load_synthetic_parameters
from validation.tracking_diagnostics import ROOT, PrescribedPathPlanner, cases
from validation.navigation_acceptance import check_execution, clean


def fixtures(settings):
    """Three minimum lines, isolated 30-degree corner, frozen detour, current."""
    world=WorldModel(Boundary.rectangle(x_min=-10,x_max=20,z_min=-20,z_max=0))
    initial=VehicleState(Pose(z=-5),Twist.zero())
    constraints=PlanningConstraints(.1,goal_tolerance=.25)
    speed=settings['los']['nominal_speed']
    for name,z in [('horizontal',-5),('ascent',-4),('descent',-6)]:
        yield name,Path((initial.pose,Pose(x=3,z=z))),world,initial,constraints,speed
    path=Path((initial.pose,Pose(x=5,z=-5),Pose(x=5+5*cos(pi/6),z=-5+5*sin(pi/6))))
    yield 'isolated_corner_30',path,world,replace(initial,twist=Twist(u=speed)),constraints,speed
    _,path,env,start,limits,*_=next(c for c in cases() if c[0]=='detour')
    yield 'detour',path,env,start,limits,settings['detour_nominal_speed']
    yield 'constant_vertical_current',Path((initial.pose,Pose(x=3,z=-5))),WorldModel(
        world.boundary,current=ConstantCurrent(settings['constant_current'])),initial,constraints,speed


def execute(inputs,settings):
    """Run twice on the same navigator to verify lifecycle reset and determinism."""
    name,path,world,initial,constraints,speed=inputs
    params=load_baseline_controller_parameters(ROOT/settings['controller'])
    config=NavigationConfig(**settings['navigation'])
    adapter=LOSNavigationAdapter(ProjectionConfig(**settings['projection']),
        LOSConfig(**{**settings['los'],'nominal_speed':speed}))
    nav=Navigator(PrescribedPathPlanner(path),CascadedPID3DOFController(params),
        Python3DOFBackend(load_synthetic_parameters()),config,
        TrajectoryConfig(nominal_speed=speed,shortcut=False),capture_policy=TerminalCapturePolicy(),
        startup_policy=ControlledStartupPolicy(),guidance_adapter=adapter)
    request=NavigationRequest(initial,path[-1],world,constraints)
    result=nav.run(request)
    again=nav.run(request)
    deterministic=(result.status==again.status and result.state_history==again.state_history
        and result.reference_history==again.reference_history and result.control_history==again.control_history
        and result.metrics==again.metrics and result.message==again.message)
    checks=check_execution(result,request,config)
    steps=result.control_history
    checks.update(deterministic=deterministic,
        no_secondary_clip=result.metrics['navigation_clip_count']==0,
        progress_monotonic=result.metrics.get('progress_monotonicity_violations')==0,
        spatial_metrics_not_timed=result.metrics['position_rmse'] is None and result.metrics['reference_duration'] is None,
        explicit_inner_loop=all(s.controller_diagnostics is not None and
            s.controller_diagnostics.guidance_mode=='direct_body_guidance' for s in steps),
        progress_frozen_outside_following=all(s.guidance_command is None and
            s.path_progress_s==(steps[i-1].path_progress_s if i else 0.)
            for i,s in enumerate(steps) if s.guidance_phase!='PATH_FOLLOWING'))
    controls=[]
    for s in steps:
        record=asdict(s)
        # Avoid repeating the entire geometry in every next_state. All evidence
        # needed to reconstruct projection is retained, and geometry is saved once.
        if record['guidance_command'] is not None:
            record['guidance_command']['projection'].pop('next_state')
        for key in ('position_error','pitch_error','velocity_error'):
            record['archived_timed_'+key]=record.pop(key)
        controls.append(record)
    report=dict(name=name,status=result.status.name,message=result.message,metrics=result.metrics,
        checks=checks,must_succeed=name in ('horizontal','ascent','descent'),
        initial=asdict(initial),path=[asdict(p) for p in path],constraints=asdict(constraints),
        current=world.current_at(initial.pose,initial.timestamp).tolist(),
        navigation=asdict(config),projection=asdict(adapter.projection_config),los=asdict(adapter.los_config),
        controller=asdict(params),startup=asdict(nav.startup_policy.config),capture=asdict(nav.capture_policy.config))
    history=dict(states=[asdict(s) for s in result.state_history],
        archived_timed_references=[asdict(s) for s in result.reference_history],controls=controls)
    return report,history


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage',choices=('straight','remaining','all'),default='all')
    parser.add_argument('--output',type=FilePath,required=True)
    args=parser.parse_args()
    settings=json.loads((ROOT/'config/guidance_phase_b.json').read_text(encoding='utf-8'))
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'config.json').write_text(json.dumps(settings,indent=2),encoding='utf-8')
    reports=[]
    for i,inputs in enumerate(fixtures(settings)):
        if args.stage=='straight' and i>=3 or args.stage=='remaining' and i<3:continue
        report,history=execute(inputs,settings)
        reports.append(report)
        (args.output/(inputs[0]+'.json')).write_text(json.dumps(clean(dict(summary=report,history=history)),allow_nan=False),encoding='utf-8')
        print(inputs[0],report['status'],report['metrics']['terminal_position_error'],
              report['metrics'].get('terminal_handoff_time'),flush=True)
    summary=dict(stage=args.stage,cases=reports,
        diagnostic_checks_passed=all(all(r['checks'].values()) for r in reports),
        required_straights_passed=all(r['status']=='SUCCESS' for r in reports if r['must_succeed']),
        mission_success_count=sum(r['status']=='SUCCESS' for r in reports),
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
            for folder in ('guidance','navigation','controller','physics','core','environment','planner','trajectory','simulation','validation','config')
            for p in (ROOT/folder).rglob('*') if p.is_file() and p.suffix in ('.py','.json','.yaml')})
    (args.output/'summary.json').write_text(json.dumps(clean(summary),indent=2,allow_nan=False),encoding='utf-8')
    (args.output/'manifest.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest()
        for p in args.output.iterdir() if p.is_file()},indent=2),encoding='utf-8')
    allowed={'SUCCESS','COLLISION','OUT_OF_BOUNDS','TIMEOUT','CAPTURE_FAILED','STARTUP_FAILED'}
    if not summary['diagnostic_checks_passed'] or not summary['required_straights_passed'] or any(r['status'] not in allowed for r in reports):
        raise SystemExit(1)


if __name__=='__main__':main()
