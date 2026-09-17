"""Single-factor detour execution-budget experiments with consistent FF/PID caps."""
import argparse
from dataclasses import asdict,replace
from datetime import datetime,timezone
import hashlib
import json
from math import atan2
from pathlib import Path
import subprocess
import sys
import numpy as np
from core import Pose,Twist,VehicleState
from environment import Boundary,WorldModel,Obstacle
from navigation import Navigator,NavigationConfig,NavigationRequest
from planner import AStarPlanner,AStarConfig,PlanningConstraints,PlanningRequest,EnvironmentPlanningSpace
from trajectory import generate_trajectory,TrajectoryConfig
from physics import Python3DOFBackend
from simulation.controller_config import load_baseline_controller_parameters
from simulation.vehicle_config import load_synthetic_parameters
from validation.tracking_diagnostics import ROOT,PrescribedPathPlanner,polyline_distance
from validation.feedforward_pid_prototype import FeedforwardPIDPrototype
from validation.navigation_acceptance import clean,check_execution


def run_case(name,speed=.5,aligned=False,extra_margin=0.):
    suite=json.loads((ROOT/'config/scenarios/planner_acceptance_v1.json').read_text())
    physical=load_synthetic_parameters()
    world=WorldModel(Boundary.rectangle(**suite['boundary']),[Obstacle.rectangle(**o) for o in suite['cases'][0]['obstacles']])
    # Execution policy remains radius .2 + safety .1 across ALL cases.
    execution=PlanningConstraints(**suite['constraints'])
    planning=replace(execution,safety_margin=execution.safety_margin+extra_margin)
    start=VehicleState(Pose(**suite['start']),Twist.zero()); goal=Pose(**suite['goal'])
    space=EnvironmentPlanningSpace(world)
    request=PlanningRequest(start,goal,planning)
    plan=AStarPlanner(AStarConfig(**suite['astar'])).plan(request,space)
    if not plan.success:
        return dict(name=name,status='PLANNING_FAILED',planning_status=plan.status.name,
                    speed=speed,aligned=aligned,extra_margin=extra_margin),{}
    processed=generate_trajectory(plan.path,space,planning,TrajectoryConfig(nominal_speed=speed),request=request)
    path=processed.shortcut_path
    if aligned:
        start=replace(start,pose=replace(start.pose,pitch=atan2(path[1].z-path[0].z,path[1].x-path[0].x)))
    cfg=NavigationConfig()
    controller=FeedforwardPIDPrototype(load_baseline_controller_parameters(),physical.restoring_pitch_coefficient,
                                      (cfg.maximum_tau_x,cfg.maximum_tau_m))
    mission=NavigationRequest(start,goal,world,execution)
    result=Navigator(PrescribedPathPlanner(path),controller,Python3DOFBackend(physical),cfg,
                     TrajectoryConfig(nominal_speed=speed,shortcut=False)).run(mission)
    checks=check_execution(result,mission,cfg)
    if not all(checks.values()) or len(controller.records)!=len(result.control_history):
        raise AssertionError(f'{name}: {result.message} {checks}')
    if result.metrics['navigation_clip_count']!=0:
        raise AssertionError('Effective limit contract violated')
    points=np.array([[s.pose.x,s.pose.z] for s in result.state_history])
    vertices=np.array([[p.x,p.z] for p in path])
    distances=polyline_distance(points,vertices)
    reference_clearance=result.metrics['minimum_reference_clearance']
    report=dict(name=name,status=result.status.name,speed=speed,aligned=aligned,extra_margin=extra_margin,
        checks=checks,config=asdict(cfg),controller=asdict(controller.parameters),coefficient=controller.coefficient,
        effective_limits=controller.limits,execution_constraints=asdict(execution),planning_constraints=asdict(planning),
        metrics=result.metrics,path=[asdict(p) for p in path],raw_path=[asdict(p) for p in plan.path],
        path_length=processed.trajectory.duration*speed,
        max_executed_path_distance=float(max(distances)),
        reference_execution_budget=reference_clearance-execution.required_clearance,
        conservative_chord_deviation_upper_bound=float(max(np.maximum(distances[:-1],distances[1:])+
            np.linalg.norm(np.diff(points,axis=0),axis=1)/2)),
        first_reference_corner_time=float(np.linalg.norm(vertices[1]-vertices[0])/speed) if len(vertices)>2 else None,
        expanded_nodes=plan.expanded_nodes,
        note='Extra margin affects planning/postprocessing ONLY. Actual-motion safety remains 0.3 m.')
    history=dict(states=[asdict(s) for s in result.state_history],references=[asdict(s) for s in result.reference_history],
        controls=[asdict(s) for s in result.control_history],controller_diagnostics=controller.records)
    return report,history


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    design=[('reference',{}),('initial_pitch_aligned',dict(aligned=True))]
    design += [(f'speed_{v:.1f}',dict(speed=v)) for v in (.4,.3,.2)]
    design += [(f'planning_extra_{m:.1f}',dict(extra_margin=m)) for m in (.1,.2,.3,.5)]
    reports=[];histories={}
    for name,kwargs in design:
        report,history=run_case(name,**kwargs)
        again,second=run_case(name,**kwargs)
        if report!=again or history!=second:
            raise AssertionError('Non-deterministic result')
        report['deterministic']=True
        reports.append(report);histories[name]=history
        (args.output/(name+'.json')).write_text(json.dumps(clean(dict(summary=report,history=history)),allow_nan=False),encoding='utf-8')
        print(name,report['status'],report.get('metrics',{}).get('simulation_duration'),flush=True)
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.patches import Rectangle
    fig=Figure(figsize=(13,5),layout='constrained');FigureCanvasAgg(fig)
    for ax,title,subset in zip(fig.subplots(1,3),('Initial alignment','Uniform speed','Planning-only extra margin'),
         ([reports[0],reports[1]],[reports[0]]+reports[2:5],[reports[0]]+reports[5:])):
        ax.add_patch(Rectangle((4.9,-7),.2,4,color='gray'))
        for r in subset:
            if not histories[r['name']]:continue
            line,=ax.plot([s['pose']['x'] for s in histories[r['name']]['states']],
                         [s['pose']['z'] for s in histories[r['name']]['states']],label=r['name']+' '+r['status'])
            ax.plot([p['x'] for p in r['path']],[p['z'] for p in r['path']],ls='--',color=line.get_color(),alpha=.45)
        ax.set(title=title,xlabel='x (m)',ylabel='z (m)',xlim=(0,10),ylim=(-10,0),aspect='equal')
        ax.legend(fontsize=6);ax.grid(alpha=.2)
    fig.suptitle('Single-factor detour trials: consistent feedforward, fixed executed clearance 0.3 m; dashed=reference')
    fig.savefig(args.output/'comparison.png',dpi=140);fig.savefig(args.output/'comparison.svg');fig.clear()
    summary=dict(date=datetime.now(timezone.utc).isoformat(),executable=sys.executable,cases=reports,
        installed=subprocess.check_output([sys.executable,'-m','pip','freeze'],text=True).splitlines(),
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
            for folder in ('core','controller','physics','navigation','trajectory','environment','planner','simulation','validation','tests','config')
            for p in (ROOT/folder).rglob('*') if p.is_file() and p.suffix in ('.py','.json','.yaml')})
    (args.output/'summary.json').write_text(json.dumps(clean(summary),indent=2,allow_nan=False),encoding='utf-8')
    manifest={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in args.output.iterdir() if p.is_file()}
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')


if __name__=='__main__':main()
