"""Formal Control v0.5.1 acceptance and unchanged low-speed terminal diagnosis."""
import argparse
from dataclasses import asdict,replace
from datetime import datetime,timezone
import hashlib
import json
from math import cos,sin,hypot
from pathlib import Path as FilePath
import subprocess
import sys
import numpy as np
from core import Path,Pose
from controller import CascadedPID3DOFController
from navigation import Navigator,NavigationConfig,NavigationRequest
from physics import Python3DOFBackend
from trajectory import TrajectoryConfig
from simulation.controller_config import load_baseline_controller_parameters
from simulation.vehicle_config import load_synthetic_parameters
from validation.tracking_diagnostics import ROOT,PrescribedPathPlanner,cases
from validation.navigation_acceptance import clean,check_execution

CANDIDATE=ROOT/'config/controllers/cascaded_pid_3dof_restoring_candidate.yaml'


def terminal_summary(result):
    """Aligned step diagnostics at reference end and final completed interval."""
    states=result.state_history;steps=result.control_history
    times=np.array([s.timestamp for s in states])
    end=result.trajectory.end_time
    first=min(int(np.searchsorted(times,end)),len(states)-1)
    theta=result.trajectory[-1].pose.pitch
    def snapshot(index):
        s=states[index];ref=result.reference_history[index]
        dx,dz=s.pose.x-ref.pose.x,s.pose.z-ref.pose.z
        diag=steps[index].controller_diagnostics if index<len(steps) else None
        return dict(state=asdict(s),reference=asdict(ref),
            along_error=dx*cos(theta)+dz*sin(theta),cross_error=-dx*sin(theta)+dz*cos(theta),
            forward_error=(ref.pose.x-s.pose.x)*cos(s.pose.pitch)+(ref.pose.z-s.pose.z)*sin(s.pose.pitch),
            controller_diagnostics=asdict(diag) if diag else None)
    terminal=steps[first:]
    signs=np.array([np.sign(s.controller_diagnostics.surge_target) for s in terminal])
    targets=np.array([s.controller_diagnostics.surge_target for s in terminal])
    significant=signs[abs(targets)>1e-4]
    positions=np.array([[s.pose.x,s.pose.z] for s in states[first:]])
    last_speeds=[hypot(s.twist.u,s.twist.w) for s in states if s.timestamp>=states[-1].timestamp-10]
    return dict(reference_end_time=end,reference_end_reached=times[-1]>=end,
        at_or_after_reference_end=snapshot(first),last_control_sample=snapshot(len(steps)-1),final=snapshot(len(states)-1),
        post_reference_net_displacement=float(np.linalg.norm(positions[-1]-positions[0])),
        post_reference_travel_distance=float(np.sum(np.linalg.norm(np.diff(positions,axis=0),axis=1))),
        terminal_target_sign_changes=int(np.sum(signs[1:]!=signs[:-1])),
        significant_target_sign_changes=int(np.sum(significant[1:]!=significant[:-1])),
        sign_noise_threshold=1e-4,last_10s_max_speed=max(last_speeds),
        active_surge_braking_steps=sum(s.applied.tau_x*states[i].twist.u< -1e-8 for i,s in enumerate(steps)),
        post_reference_negative_surge_work_j=float(sum(min(0.,s.applied.tau_x*states[i].twist.u)*
            (states[i+1].timestamp-states[i].timestamp) for i,s in enumerate(steps) if i>=first)))


def run(inputs,speed,coefficient,caps=None):
    name,path,world,initial,constraints,_,_,_=inputs
    p=replace(load_baseline_controller_parameters(CANDIDATE),restoring_pitch_coefficient=coefficient)
    cfg=NavigationConfig() if caps is None else NavigationConfig(maximum_tau_x=caps[0],maximum_tau_m=caps[1])
    controller=CascadedPID3DOFController(p)
    req=NavigationRequest(initial,path[-1],world,constraints)
    result=Navigator(PrescribedPathPlanner(path),controller,Python3DOFBackend(load_synthetic_parameters()),
                     cfg,TrajectoryConfig(nominal_speed=speed,shortcut=False)).run(req)
    checks=check_execution(result,req,cfg)
    checks['no_second_clipping']=result.metrics['navigation_clip_count']==0
    checks['diagnostics_aligned']=all(s.limits_aware and s.controller_diagnostics is not None and
        s.commanded==s.applied==s.controller_diagnostics.output and s.time==s.controller_diagnostics.timestamp for s in result.control_history)
    report=dict(scenario=name,status=result.status.name,speed=speed,controller=asdict(p),navigation=asdict(cfg),
                path=[asdict(p) for p in path],checks=checks,metrics=result.metrics,terminal=terminal_summary(result))
    history=dict(states=[asdict(s) for s in result.state_history],references=[asdict(s) for s in result.reference_history],
                 controls=[asdict(s) for s in result.control_history])
    return report,history


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=FilePath,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    selected={c[0]:c for c in cases() if c[0] in ('ascending','descending','detour')}
    base=selected['ascending']
    selected['horizontal']=('horizontal',Path.from_list([Pose(x=1,z=-5),Pose(x=9,z=-5)]),*base[2:])
    design=[]
    for scene in ('horizontal','ascending','descending'):
        design.append((scene+'_off',scene,.5,0,None,'SUCCESS' if scene=='horizontal' else 'TIMEOUT'))
        for speed in (.2,.5):design.append((f'{scene}_candidate_{speed:.1f}',scene,speed,6,None,'SUCCESS'))
    for coefficient in (4.8,7.2):
        for scene in ('ascending','descending'):
            design.append((f'{scene}_mismatch_{coefficient}',scene,.5,coefficient,None,None))
    design += [('saturation_surge','horizontal',.5,6,(2,8),None),('saturation_pitch','ascending',.5,6,(20,.5),None)]
    design += [(f'detour_{v:.1f}','detour',v,6,None,None) for v in (.2,.3)]
    reports=[];histories={}
    for name,scene,speed,coefficient,caps,expected in design:
        report,history=run(selected[scene],speed,coefficient,caps)
        again,second=run(selected[scene],speed,coefficient,caps)
        report.update(name=name,deterministic=report==again and history==second,expected_status=expected)
        report['accepted']=all(report['checks'].values()) and report['deterministic'] and (expected is None or report['status']==expected)
        reports.append(report);histories[name]=history
        (args.output/(name+'.json')).write_text(json.dumps(clean(dict(summary=report,history=history)),allow_nan=False),encoding='utf-8')
        print(name,report['status'],f"error={report['metrics']['terminal_position_error']:.4f}",f"accepted={report['accepted']}",flush=True)
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    fig=Figure(figsize=(12,8),layout='constrained');FigureCanvasAgg(fig)
    axes=fig.subplots(2,2)
    for name in ('detour_0.2','detour_0.3'):
        h=histories[name];r=next(r for r in reports if r['name']==name)
        t=[s['timestamp'] for s in h['states']];end=r['terminal']['reference_end_time']
        xy=np.array([[s['pose']['x'],s['pose']['z']] for s in h['states']])
        ref=np.array([[s['pose']['x'],s['pose']['z']] for s in h['references']])
        theta=h['references'][-1]['pose']['pitch'];delta=xy-ref
        axes[0,0].plot(t,delta[:,0]*cos(theta)+delta[:,1]*sin(theta),label=name)
        axes[0,1].plot(t,-delta[:,0]*sin(theta)+delta[:,1]*cos(theta),label=name)
        axes[1,0].plot(t,[hypot(s['twist']['u'],s['twist']['w']) for s in h['states']],label=name)
        axes[1,1].plot(t[:-1],[s['applied']['tau_x'] for s in h['controls']],label=name)
        for ax in axes.flat:ax.axvline(end,alpha=.2,ls='--')
    for ax,label in zip(axes.flat,('Along error (m), final-segment frame','Cross error (m), final-segment frame','Body speed (m/s)','Applied surge force (N)')):
        ax.set(xlabel='Simulation time (s)',ylabel=label);ax.legend();ax.grid(alpha=.2)
    fig.suptitle('Unchanged low-speed detour; dashed lines mark reference end times')
    fig.savefig(args.output/'terminal_diagnosis.png',dpi=140);fig.savefig(args.output/'terminal_diagnosis.svg');fig.clear()
    cmd=[sys.executable,'-m','pytest','tests','-q','-rs','-p','no:cacheprovider']
    test=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,encoding='utf-8',errors='replace')
    (args.output/'pytest.log').write_text(test.stdout+test.stderr,encoding='utf-8')
    summary=dict(date=datetime.now(timezone.utc).isoformat(),cases=reports,executable=sys.executable,tests_command=cmd,
        tests_exit_code=test.returncode,automated_acceptance=test.returncode==0 and all(r['accepted'] for r in reports),
        installed=subprocess.check_output([sys.executable,'-m','pip','freeze'],text=True).splitlines(),
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
            for folder in ('core','controller','navigation','physics','planner','trajectory','simulation','environment','validation','tests','config')
            for p in (ROOT/folder).rglob('*') if p.is_file() and p.suffix in ('.py','.yaml','.json')})
    (args.output/'summary.json').write_text(json.dumps(clean(summary),indent=2,allow_nan=False),encoding='utf-8')
    manifest={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in args.output.iterdir() if p.is_file()}
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    raise SystemExit(0 if summary['automated_acceptance'] else 1)


if __name__=='__main__':main()
