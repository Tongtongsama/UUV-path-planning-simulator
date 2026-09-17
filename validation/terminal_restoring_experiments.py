"""Opt-in terminal-guidance/restoring ablations; production defaults unchanged."""
import argparse
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
from math import atan2, cos, hypot, pi, sin
from pathlib import Path as FilePath
import subprocess
import sys

import numpy as np
from core import ControlInput, Path, Pose, Twist, VehicleState
from controller import CascadedPID3DOFController, wrap_to_pi
from environment import Boundary, WorldModel
from navigation import NavigationConfig, NavigationRequest, Navigator
from physics import Python3DOFBackend
from planner import PlanningConstraints
from simulation.controller_config import load_baseline_controller_parameters
from simulation.vehicle_config import load_synthetic_parameters
from trajectory import TrajectoryConfig, generate_trajectory
from planner import EnvironmentPlanningSpace
from validation.tracking_diagnostics import ROOT, PrescribedPathPlanner, cases
from validation.navigation_acceptance import clean, check_execution


class ExperimentalController:
    """Audit wrapper, not a replacement baseline controller.

    FF is added to the already clipped PID output then clipped again. The PID
    receives no feedback about this additional saturation: an explicit study
    limitation, with both clipping stages recorded. Guidance changes only the
    reference sent to PID after the original trajectory ends.
    """
    def __init__(self, parameters, restoring_coefficient=0., terminal_time=None,
                 goal=None, capture_radius=.15, release_radius=.25,
                 approach_gain=.35, approach_speed=.5):
        if not 0 < capture_radius < release_radius:
            raise ValueError('Need 0 < capture radius < release radius')
        self.pid=CascadedPID3DOFController(parameters)
        self.coefficient=restoring_coefficient
        self.terminal_time=terminal_time
        self.goal=goal
        self.capture_radius=capture_radius
        self.release_radius=release_radius
        self.approach_gain=approach_gain
        self.approach_speed=approach_speed
        self.reset()

    def reset(self):
        self.pid.reset()
        self.records=[]
        self.captured=False
        self.hold_pitch=None

    def step(self, state, reference, dt):
        effective=reference
        phase='TIMED_REFERENCE'
        if self.terminal_time is not None and state.timestamp >= self.terminal_time:
            dx,dz=self.goal.x-state.pose.x,self.goal.z-state.pose.z
            distance=hypot(dx,dz)
            if self.captured and distance > self.release_radius:
                self.captured=False
            if not self.captured and distance <= self.capture_radius:
                self.captured=True
                self.hold_pitch=state.pose.pitch
            if self.captured:
                pitch=self.hold_pitch
                speed=0.
                phase='TERMINAL_BRAKE'
            else:
                pitch=atan2(dz,dx)
                error=wrap_to_pi(pitch-state.pose.pitch)
                speed=min(self.approach_speed,self.approach_gain*distance)*max(0.,cos(error))
                phase='TERMINAL_APPROACH'
            # Explicitly bypass the old outer position/depth corrections, while
            # retaining the same inner PID, memory, gains and force limits.
            effective=VehicleState(replace(state.pose,pitch=pitch),Twist(u=speed),reference.timestamp)
        base=self.pid.step(state,effective,dt)
        ff=self.coefficient*sin(state.pose.pitch)
        combined=base.tau_m+ff
        limit=self.pid.parameters.maximum_tau_m
        command=replace(base,tau_m=float(np.clip(combined,-limit,limit)))
        self.records.append(dict(phase=phase,effective_reference=asdict(effective),
            base_command=asdict(base),feedforward_tau_m=ff,pre_final_clip_tau_m=combined,
            post_final_clip_tau_m=command.tau_m,extra_clipped=abs(combined)>limit,
            memory=asdict(self.pid.memory)))
        return command


def run_navigation(inputs, variant):
    name,path,world,initial,constraints,speed,horizon,_=inputs
    generated=generate_trajectory(path,EnvironmentPlanningSpace(world),constraints,
                                  TrajectoryConfig(nominal_speed=speed,shortcut=False))
    ff=variant in ('restoring_only','combined')
    guidance=variant in ('terminal_only','combined')
    physical=load_synthetic_parameters()
    wrapper=ExperimentalController(load_baseline_controller_parameters(),
        restoring_coefficient=physical.restoring_pitch_coefficient if ff else 0.,
        terminal_time=generated.trajectory.end_time if guidance else None,goal=path[-1])
    config=NavigationConfig(max_duration=horizon)
    request=NavigationRequest(initial,path[-1],world,constraints)
    navigator=Navigator(PrescribedPathPlanner(path),wrapper,Python3DOFBackend(physical),config,
                        TrajectoryConfig(nominal_speed=speed,shortcut=False))
    result=navigator.run(request)
    checks=check_execution(result,request,config)
    if not all(checks.values()) or len(wrapper.records)!=len(result.control_history):
        raise AssertionError(f'{name}/{variant}: invalid evidence {result.message} {checks}')
    times=np.array([s.time for s in result.control_history])
    moving=times<generated.trajectory.end_time
    errors=np.array([s.pitch_error for s in result.control_history])
    report=dict(name=name,variant=variant,status=result.status.name,checks=checks,
        metrics=result.metrics,config=asdict(config),constraints=asdict(constraints),
        path=[asdict(p) for p in path],controller=asdict(wrapper.pid.parameters),
        restoring_coefficient=wrapper.coefficient,
        terminal_policy=dict(enabled=guidance,activation_time=wrapper.terminal_time,
            capture_radius=wrapper.capture_radius,release_radius=wrapper.release_radius,
            approach_gain=wrapper.approach_gain,approach_speed=wrapper.approach_speed),
        extra_clip_count=sum(r['extra_clipped'] for r in wrapper.records),
        terminal_active_steps=sum(r['phase']!='TIMED_REFERENCE' for r in wrapper.records),
        moving_pitch_rmse=float(np.sqrt(np.mean(errors[moving]**2))),
        note='Navigation errors remain against ORIGINAL timed reference; effective PID reference recorded separately')
    history=dict(states=[asdict(s) for s in result.state_history],
        original_references=[asdict(s) for s in result.reference_history],
        controls=[asdict(s) for s in result.control_history],experimental_controller=wrapper.records)
    return report,history


def pitch_hold(angle, variant):
    """Fixed-attitude, zero-translation experiment; no terminal policy."""
    physical=load_synthetic_parameters()
    parameters=load_baseline_controller_parameters()
    if variant=='integral_capacity':
        parameters=replace(parameters,pitch_integral_limit=parameters.maximum_tau_m/parameters.pitch_ki)
    wrapper=ExperimentalController(parameters,
        restoring_coefficient=physical.restoring_pitch_coefficient if variant=='restoring_only' else 0.)
    physics=Python3DOFBackend(physical)
    state=VehicleState(Pose(z=-20),Twist.zero())
    history=[]
    dt=.05
    for _ in range(1200):
        reference=VehicleState(Pose(z=-20,pitch=angle),Twist.zero(),state.timestamp)
        command=wrapper.step(state,reference,dt)
        history.append(dict(state=asdict(state),error=wrap_to_pi(angle-state.pose.pitch),
                            command=asdict(command),diagnostics=wrapper.records[-1]))
        state=physics.step(state,command,np.zeros(2),dt)
    last=history[-200:]
    report=dict(angle_deg=angle*180/pi,variant=variant,duration=60.,dt=dt,
        pitch_integral_limit=parameters.pitch_integral_limit,
        final_pitch=state.pose.pitch,final_error_deg=wrap_to_pi(angle-state.pose.pitch)*180/pi,
        last_10s_rmse_deg=float(np.sqrt(np.mean([h['error']**2 for h in last]))*180/pi),
        max_abs_tau_m=max(abs(h['command']['tau_m']) for h in history),
        saturation_fraction=float(np.mean([abs(h['command']['tau_m'])>=parameters.maximum_tau_m-1e-12 for h in history])),
        extra_clip_count=sum(r['extra_clipped'] for r in wrapper.records),
        final_state=asdict(state))
    return report,history


def plot_results(reports,histories,holds,output):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.patches import Rectangle
    fig=Figure(figsize=(12,8),layout='constrained'); FigureCanvasAgg(fig)
    for ax,name in zip(fig.subplots(2,2).flat,('horizontal','ascending','descending','detour')):
        group=[r for r in reports if r['name']==name]
        path=group[0]['path']
        ax.plot([p['x'] for p in path],[p['z'] for p in path],'k--',label='Original path')
        if name=='detour':
            ax.add_patch(Rectangle((4.9,-7),.2,4,color='gray'))
        for report in group:
            states=histories[name+'_'+report['variant']]['states']
            ax.plot([s['pose']['x'] for s in states],[s['pose']['z'] for s in states],
                label=report['variant']+': '+report['status'])
        ax.set(title=name,xlabel='x (m)',ylabel='z (m)',aspect='equal',xlim=(0,10),ylim=(-10,0))
        ax.legend(fontsize=7); ax.grid(alpha=.2)
    fig.suptitle('Experimental 2 x 2 ablation: same PID gains, same timed path, same arrival tolerances')
    fig.savefig(output/'navigation_comparison.png',dpi=140)
    fig.savefig(output/'navigation_comparison.svg'); fig.clear()
    fig=Figure(figsize=(8,4),layout='constrained'); FigureCanvasAgg(fig)
    ax=fig.subplots()
    for variant in ('baseline','integral_capacity','restoring_only'):
        group=sorted([r for r in holds if r['variant']==variant],key=lambda r:r['angle_deg'])
        ax.plot([r['angle_deg'] for r in group],[r['last_10s_rmse_deg'] for r in group],'o-',label=variant)
    ax.set(xlabel='Hold target pitch (degrees)',ylabel='Last 10 s pitch RMSE (degrees)')
    ax.legend(); ax.grid(alpha=.2)
    fig.savefig(output/'pitch_hold.png',dpi=140); fig.savefig(output/'pitch_hold.svg'); fig.clear()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=FilePath,required=True)
    args=parser.parse_args(); args.output.mkdir(parents=True,exist_ok=False)
    selected=[c for c in cases() if c[0] in ('ascending','descending','detour')]
    base=selected[0]
    horizontal=('horizontal',Path.from_list([Pose(x=1,z=-5),Pose(x=9,z=-5)]),base[2],base[3],base[4],.5,120.,None)
    selected.insert(0,horizontal)
    reports=[]; histories={}; holds=[]
    def save(name,payload):
        (args.output/(name+'.json')).write_text(json.dumps(clean(payload),allow_nan=False),encoding='utf-8')
    for inputs in selected:
        for variant in ('baseline','restoring_only','terminal_only','combined'):
            report,history=run_navigation(inputs,variant)
            again,second=run_navigation(inputs,variant)
            if report!=again or history!=second:
                raise AssertionError('Navigation not deterministic')
            report['deterministic']=True
            name=inputs[0]+'_'+variant
            reports.append(report); histories[name]=history
            save(name,dict(summary=report,history=history))
            print(name,report['status'],f"error={report['metrics']['terminal_position_error']:.4f}",flush=True)
    for angle in (-60,-32,-atan2(2,8)*180/pi,atan2(2,8)*180/pi,32,60):
        for variant in ('baseline','integral_capacity','restoring_only'):
            report,history=pitch_hold(angle*pi/180,variant)
            again,second=pitch_hold(angle*pi/180,variant)
            if report!=again or history!=second:
                raise AssertionError('Pitch hold not deterministic')
            report['deterministic']=True; holds.append(report)
            save(f'hold_{angle:+.3f}_{variant}',dict(summary=report,history=history))
            print('hold',angle,variant,f"last10s_RMSE={report['last_10s_rmse_deg']:.4f} deg",flush=True)
    plot_results(reports,histories,holds,args.output)
    report=dict(date=datetime.now(timezone.utc).isoformat(),executable=sys.executable,python=sys.version,
        navigation=reports,pitch_hold=holds,
        installed=subprocess.check_output([sys.executable,'-m','pip','freeze'],text=True).splitlines(),
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
            for folder in ('core','controller','physics','navigation','trajectory','environment','planner','simulation','validation','tests','config')
            for p in (ROOT/folder).rglob('*') if p.is_file() and p.suffix in ('.py','.json','.yaml')},
        limitations=['Exact known synthetic restoring coefficient, no model mismatch',
            'FF added after PID clipping; no final-actuator anti-windup feedback',
            'Terminal guidance is experimental, not obstacle-aware or replanning',
            'Integral-capacity variant changes ONLY integral limit in isolated attitude tests'])
    save('summary',report)
    manifest={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in args.output.iterdir() if p.is_file()}
    save('manifest',manifest)


if __name__=='__main__':
    main()
