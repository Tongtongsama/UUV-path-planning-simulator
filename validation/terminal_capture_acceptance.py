"""Isolated capture first, then unchanged timed missions; no retiming or smoothing."""
import argparse
from dataclasses import asdict
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path as FilePath
import subprocess
import sys
from core import Path,Pose,Twist,VehicleState,Trajectory
from environment import WorldModel,Boundary
from controller import CascadedPID3DOFController
from navigation import Navigator,NavigationConfig,NavigationRequest,TerminalCapturePolicy
from planner import PlanningConstraints
from physics import Python3DOFBackend
from trajectory import TrajectoryConfig,TrajectoryGenerationResult,sample_reference
from simulation.controller_config import load_baseline_controller_parameters
from simulation.vehicle_config import load_synthetic_parameters
from validation.tracking_diagnostics import ROOT,PrescribedPathPlanner,cases
from validation.navigation_acceptance import clean,check_execution


def terminal_fixture(path,space,constraints,config,*,start_time,request):
    """Test-only one stationary tick, then terminal goal; no cruise is tested."""
    a=VehicleState(path[0],Twist.zero(),start_time)
    b=VehicleState(Pose(x=path[-1].x,z=path[-1].z,pitch=path[0].pitch),Twist.zero(),start_time+.05)
    return TrajectoryGenerationResult(path,path,path,path,Trajectory.from_list([a,b]))


def run(inputs,enabled=True,isolated=False):
    name,path,world,initial,constraints,speed,*_=inputs
    controller=CascadedPID3DOFController(load_baseline_controller_parameters(
        ROOT/'config/controllers/cascaded_pid_3dof_restoring_candidate.yaml'))
    cfg=NavigationConfig()
    options=dict(capture_policy=TerminalCapturePolicy() if enabled else None)
    if isolated:options['generator']=terminal_fixture
    nav=Navigator(PrescribedPathPlanner(path),controller,Python3DOFBackend(load_synthetic_parameters()),
        cfg,TrajectoryConfig(nominal_speed=speed,shortcut=False),**options)
    request=NavigationRequest(initial,path[-1],world,constraints)
    result=nav.run(request)
    checks=check_execution(result,request,cfg)
    checks['no_secondary_clip']=result.metrics['navigation_clip_count']==0
    checks['original_reference_preserved']=all(r==sample_reference(result.trajectory,s.timestamp)
        for r,s in zip(result.reference_history,result.state_history))
    targets=[s for s in result.control_history if s.effective_guidance is not None]
    checks['explicit_outer_loop_bypass']=all(s.controller_diagnostics.guidance_mode=='direct_body_guidance' and
        s.controller_diagnostics.surge_target==s.effective_guidance.u and s.controller_diagnostics.depth_correction==0 for s in targets)
    checks['bounded_guidance']=all(abs(s.effective_guidance.pitch)<=nav.capture_policy.config.maximum_pitch+1e-12 and
        abs(s.effective_guidance.q)<=nav.capture_policy.config.maximum_reference_pitch_rate+1e-12 for s in targets) if enabled else True
    checks['bounded_guidance_slew']=True
    if enabled:
        previous=None
        for index,step in enumerate(result.control_history):
            target=step.effective_guidance
            if target is not None:
                prior=previous.pitch if previous is not None else result.reference_history[index].pose.pitch
                checks['bounded_guidance_slew'] &= abs(target.pitch-prior)<=nav.capture_policy.config.maximum_reference_pitch_rate*cfg.dt+1e-12
            previous=target
        checks['capture_budget']=result.metrics['capture_duration']<=nav.capture_policy.config.budget+1e-9
    history=dict(states=[asdict(s) for s in result.state_history],original_references=[asdict(s) for s in result.reference_history],
                 controls=[asdict(s) for s in result.control_history])
    return dict(name=name,enabled=enabled,isolated=isolated,status=result.status.name,message=result.message,
        checks=checks,metrics=result.metrics,controller=asdict(controller.parameters),navigation=asdict(cfg),
        capture_config=asdict(nav.capture_policy.config) if enabled else None,
        constraints=asdict(constraints),path=[asdict(p) for p in path]),history


def fixtures():
    world=WorldModel(Boundary.rectangle(x_min=-3,x_max=3,z_min=-8,z_max=-2))
    goal=Pose(x=0,z=-5)
    for name,x,z,u in [('lateral_up',0,-5.4,0),('lateral_down',0,-4.6,0),
                        ('overshoot',.4,-5,0),('mixed',.2,-5.4,.02),('already_arrived',.1,-5,0)]:
        start=Pose(x=x,z=z)
        yield (name,Path.from_list([start,goal]),world,VehicleState(start,Twist(u=u)),PlanningConstraints(.2,.1,goal_tolerance=.25),.2),True
    selected={c[0]:c for c in cases() if c[0] in ('ascending','descending','detour')}
    base=selected['ascending']
    selected['horizontal']=('horizontal',Path.from_list([Pose(x=1,z=-5),Pose(x=9,z=-5)]),*base[2:])
    for scene in ('horizontal','ascending','descending','detour'):
        for speed in ((.2,.3) if scene=='detour' else (.2,.5)):
            c=selected[scene]
            yield (f'{scene}_{speed}',*c[1:5],speed),False


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=FilePath,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    reports=[];histories={}
    for inputs,isolated in fixtures():
        for enabled in (False,True):
            r,h=run(inputs,enabled,isolated);again,second=run(inputs,enabled,isolated)
            if enabled:
                # A capture policy must not alter any pre-terminal execution.
                before=[s for s in h['states'] if s['timestamp']<=r['metrics']['reference_duration']]
                old=[s for s in histories[inputs[0]+'_hold']['states'] if s['timestamp']<=r['metrics']['reference_duration']]
                r['checks']['pre_terminal_execution_unchanged']=before==old
                again['checks']['pre_terminal_execution_unchanged']=before==old
            r['deterministic']=r==again and h==second
            key=inputs[0]+('_capture' if enabled else '_hold')
            # Disabled detour and isolated residual cases are diagnostic controls.
            required=enabled or (not isolated and not inputs[0].startswith('detour')) or inputs[0]=='already_arrived'
            r['accepted']=all(r['checks'].values()) and r['deterministic'] and (r['status']=='SUCCESS' if required else r['status'] in ('SUCCESS','TIMEOUT'))
            reports.append(r);histories[key]=h
            (args.output/(key+'.json')).write_text(json.dumps(clean(dict(summary=r,history=h)),allow_nan=False),encoding='utf-8')
            print(key,r['status'],f"error={r['metrics']['terminal_position_error']:.4f}",r['message'],flush=True)
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    fig=Figure(figsize=(11,7),layout='constrained');FigureCanvasAgg(fig)
    for ax,scene in zip(fig.subplots(2,2).flat,('lateral_up','overshoot','detour_0.2','detour_0.3')):
        for suffix in ('hold','capture'):
            states=histories[scene+'_'+suffix]['states']
            ax.plot([s['pose']['x'] for s in states],[s['pose']['z'] for s in states],label=suffix)
        ax.set(title=scene,xlabel='x (m)',ylabel='z (m)',aspect='equal');ax.legend();ax.grid(alpha=.2)
    fig.savefig(args.output/'comparison.png',dpi=140);fig.savefig(args.output/'comparison.svg');fig.clear()
    command=[sys.executable,'-m','pytest','tests','-q','-rs','-p','no:cacheprovider']
    tests=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,encoding='utf-8',errors='replace')
    (args.output/'pytest.log').write_text(tests.stdout+tests.stderr,encoding='utf-8')
    summary=dict(date=datetime.now(timezone.utc).isoformat(),cases=reports,executable=sys.executable,
        tests_exit_code=tests.returncode,automated_acceptance=tests.returncode==0 and all(r['accepted'] for r in reports),
        installed=subprocess.check_output([sys.executable,'-m','pip','freeze'],text=True).splitlines(),
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
            for folder in ('core','controller','navigation','physics','planner','trajectory','simulation','environment','validation','tests','config')
            for p in (ROOT/folder).rglob('*') if p.is_file() and p.suffix in ('.py','.yaml','.json')})
    (args.output/'summary.json').write_text(json.dumps(clean(summary),indent=2,allow_nan=False),encoding='utf-8')
    manifest={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in args.output.iterdir() if p.is_file()}
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    raise SystemExit(0 if summary['automated_acceptance'] else 1)


if __name__=='__main__':main()
