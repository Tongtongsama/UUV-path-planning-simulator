"""Single-factor startup then local corner timing experiments; no smoothing."""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
from math import hypot
from pathlib import Path as FilePath
import subprocess
import sys
import numpy as np
from core import Trajectory, Twist
from controller import CascadedPID3DOFController
from navigation import Navigator, NavigationConfig, NavigationRequest, TerminalCapturePolicy, ControlledStartupPolicy
from physics import Python3DOFBackend
from trajectory import generate_trajectory, TrajectoryConfig, sample_reference
from simulation.controller_config import load_baseline_controller_parameters
from simulation.vehicle_config import load_synthetic_parameters
from validation.tracking_diagnostics import ROOT, PrescribedPathPlanner, polyline_distance
from validation.startup_corner_diagnosis import fixtures
from validation.navigation_acceptance import clean


def corner_timing(path, space, constraints, config, **kwargs):
    """Experimental +/-1 m corner zone at min(nominal,.2) m/s.

    Retains baseline sampled geometry and pitch jumps. Outgoing interval speed
    and its timestamp duration are changed together. No acceleration constraint.
    """
    result = generate_trajectory(path, space, constraints, config, **kwargs)
    vertices = list(result.shortcut_path)
    corner_arcs = np.cumsum([hypot(b.x-a.x,b.z-a.z) for a,b in zip(vertices, vertices[1:])])[:-1]
    states = list(result.trajectory)
    output = []
    arc = 0.
    time = states[0].timestamp
    for i, state in enumerate(states[:-1]):
        length = hypot(states[i+1].pose.x-state.pose.x, states[i+1].pose.z-state.pose.z)
        middle = arc+length/2
        slow = any(abs(middle-c)<=1. for c in corner_arcs)
        speed = min(config.nominal_speed,.2) if slow else config.nominal_speed
        output.append(replace(state, timestamp=time, twist=Twist(u=speed)))
        arc += length
        time += length/speed
    output.append(replace(states[-1], timestamp=time, twist=Twist.zero()))
    return replace(result, trajectory=Trajectory.from_list(output))


def execute(inputs, startup, slow):
    name, path, world, initial, constraints, speed = inputs
    controller = CascadedPID3DOFController(load_baseline_controller_parameters(
        ROOT/'config/controllers/cascaded_pid_3dof_restoring_candidate.yaml'))
    config = NavigationConfig()
    nav = Navigator(PrescribedPathPlanner(path), controller, Python3DOFBackend(load_synthetic_parameters()),
        config, TrajectoryConfig(nominal_speed=speed, shortcut=False),
        generator=corner_timing if slow else generate_trajectory,
        capture_policy=TerminalCapturePolicy(), startup_policy=ControlledStartupPolicy() if startup else None)
    result = nav.run(NavigationRequest(initial,path[-1],world,constraints))
    history = dict(states=[asdict(s) for s in result.state_history],
        references=[asdict(s) for s in result.reference_history], controls=[asdict(s) for s in result.control_history],
        trajectory=[asdict(s) for s in result.trajectory])
    clocks = [s.reference_time if s.reference_time is not None else s.time for s in result.control_history]
    final_clock=result.state_history[-1].timestamp-result.metrics.get('startup_duration',0.)
    moving = [i for i, s in enumerate(result.control_history)
        if s.startup_phase not in ('BRAKE','ALIGN') and clocks[i]<result.trajectory.end_time]
    xy = np.array([[s.pose.x,s.pose.z] for s in result.state_history])
    distance = polyline_distance(xy, np.array([[p.x,p.z] for p in path]))
    checks = dict(history_aligned=len(result.state_history)==len(result.control_history)+1,
        initial_unchanged=result.state_history[0]==initial,
        no_secondary_clip=result.metrics['navigation_clip_count']==0,
        clock_reference_matches=all(replace(sample_reference(result.trajectory,t),timestamp=s.timestamp)==r
            for t,s,r in zip(clocks+[final_clock],result.state_history,result.reference_history)),
        startup_outer_loop_bypass=all(s.controller_diagnostics.guidance_mode=='direct_body_guidance'
            for s in result.control_history if s.startup_phase in ('ALIGN','BRAKE')),
        applied_limits=all(abs(s.applied.tau_x)<=20 and abs(s.applied.tau_m)<=8 for s in result.control_history))
    report = dict(name=name,startup=startup,slow=slow,status=result.status.name,message=result.message,
        metrics=result.metrics,checks=checks,config=asdict(config),controller=asdict(controller.parameters),
        startup_config=asdict(nav.startup_policy.config) if startup else None,
        corner_schedule=dict(speed=.2,half_width=1.,selection='sample interval midpoint') if slow else None,
        constraints=asdict(constraints),path=[asdict(p) for p in path],
        moving_max_path_distance=float(max(distance[moving])) if moving else None)
    return report,history


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage',choices=('startup','corner'),required=True)
    parser.add_argument('--output',type=FilePath,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    design=[]
    for inputs,turn,family in fixtures():
        if args.stage=='startup' and family in ('detour','launch') and inputs[0].endswith('_original'):
            for startup in (False,True):design.append((inputs,startup,False))
        if args.stage=='corner' and ((family=='detour' and inputs[0].endswith('_original')) or family=='corner'):
            for slow in (False,True):design.append((inputs,family=='detour',slow))
    reports=[];histories={}
    for inputs,startup,slow in design:
        report,history=execute(inputs,startup,slow)
        again,repeat=execute(inputs,startup,slow)
        report['deterministic']=report==again and history==repeat
        key=f'{inputs[0]}_startup{int(startup)}_slow{int(slow)}'
        reports.append(report);histories[key]=history
        (args.output/(key+'.json')).write_text(json.dumps(clean(dict(summary=report,history=history)),allow_nan=False),encoding='utf-8')
        print(key,report['status'],report['metrics'].get('startup_duration'),report['metrics']['minimum_executed_safety_margin'],flush=True)
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    fig=Figure(figsize=(11,8),layout='constrained');FigureCanvasAgg(fig)
    for ax,speed in zip(fig.subplots(2,2).flat,(.2,.3,.4,.5)):
        for r in reports:
            if r['name']!=f'detour_{speed}_original':continue
            key=f"{r['name']}_startup{int(r['startup'])}_slow{int(r['slow'])}"
            h=histories[key]['states']
            ax.plot([s['pose']['x'] for s in h],[s['pose']['z'] for s in h],label=f"startup={r['startup']}, slow={r['slow']}: {r['status']}")
        ax.fill([4.9,5.1,5.1,4.9],[-7,-7,-3,-3],color='gray',alpha=.5)
        ax.set(title=f'{speed} m/s',xlabel='x (m)',ylabel='z (m)',aspect='equal');ax.legend(fontsize=7);ax.grid(alpha=.2)
    fig.savefig(args.output/'comparison.png',dpi=140)
    test=subprocess.run([sys.executable,'-m','pytest','tests','-q','-p','no:cacheprovider'],cwd=ROOT,capture_output=True,text=True)
    (args.output/'pytest.log').write_text(test.stdout+test.stderr,encoding='utf-8')
    summary=dict(stage=args.stage,cases=reports,tests_exit_code=test.returncode,
        diagnostics_passed=all(r['deterministic'] and all(r['checks'].values()) for r in reports),
        installed=subprocess.check_output([sys.executable,'-m','pip','freeze'],text=True).splitlines(),
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
            for folder in ('core','controller','navigation','physics','planner','trajectory','simulation','environment','validation','tests','config')
            for p in (ROOT/folder).rglob('*') if p.is_file() and p.suffix in ('.py','.yaml','.json')})
    (args.output/'summary.json').write_text(json.dumps(clean(summary),indent=2,allow_nan=False),encoding='utf-8')
    (args.output/'manifest.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in args.output.iterdir() if p.is_file()},indent=2),encoding='utf-8')
    if test.returncode or not summary['diagnostics_passed']:raise SystemExit(1)


if __name__=='__main__':main()
