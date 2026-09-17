"""Diagnostic experiments only: unchanged PID, physics and timed references.

No gain tuning, reference retiming or production-policy changes are performed.
"""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
from math import atan2, cos, sin, pi, ceil
from pathlib import Path as FilePath
import subprocess
import sys

import numpy as np
from core import Path, Pose, Twist, VehicleState
from controller import CascadedPID3DOFController, wrap_to_pi
from environment import Boundary, WorldModel, Obstacle
from navigation import Navigator, NavigationConfig, NavigationRequest
from planner import AStarPlanner, AStarConfig, EnvironmentPlanningSpace, PlanningRequest, PlanningResult, PlanningStatus, PlanningConstraints
from simulation.controller_config import load_baseline_controller_parameters
from simulation.vehicle_config import load_synthetic_parameters
from physics import Python3DOFBackend
from trajectory import TrajectoryConfig, generate_trajectory
from validation.navigation_acceptance import clean, check_execution

ROOT = FilePath(__file__).resolve().parents[1]


def controller_terms(state, reference, parameters, memory):
    """Audit the baseline equations using actual post-step integral memory.

    This is diagnostic reconstruction, never the applied controller. Every
    reconstructed output is checked against the real controller API output.
    """
    p = parameters
    dx, dz = reference.pose.x-state.pose.x, reference.pose.z-state.pose.z
    forward = cos(state.pose.pitch)*dx + sin(state.pose.pitch)*dz
    ur = float(np.clip(reference.twist.u+p.position_to_surge_gain*forward,
                       -p.maximum_surge_reference, p.maximum_surge_reference))
    correction = float(np.clip((1 if ur >= 0 else -1)*p.depth_to_pitch_gain*dz,
                               -p.maximum_pitch_correction, p.maximum_pitch_correction))
    error = wrap_to_pi(reference.pose.pitch+correction-state.pose.pitch)
    proportional = p.pitch_kp*error
    derivative = p.pitch_rate_kd*(reference.twist.q-state.twist.q)
    integral = p.pitch_ki*memory.pitch_error_integral
    raw = proportional+derivative+integral
    return dict(forward_position_error=forward, depth_error=dz,
                surge_target=ur, effective_pitch_target=reference.pose.pitch+correction,
                effective_pitch_error=error, pitch_p=proportional, pitch_d=derivative,
                pitch_i=integral, pitch_integral=memory.pitch_error_integral,
                raw_tau_m=raw, reconstructed_tau_m=float(np.clip(raw,-p.maximum_tau_m,p.maximum_tau_m)),
                reconstructed_tau_x=float(np.clip(p.surge_kp*(ur-state.twist.u)+p.surge_ki*memory.surge_error_integral,
                                                   -p.maximum_tau_x,p.maximum_tau_x)))


class ObservedController:
    """Record real PID memory; do not alter the reference or command."""
    def __init__(self, parameters):
        self.delegate = CascadedPID3DOFController(parameters)
        self.records = []

    def reset(self):
        self.delegate.reset()
        self.records.clear()

    def step(self, state, reference, dt):
        command = self.delegate.step(state, reference, dt)
        record = controller_terms(state, reference, self.delegate.parameters, self.delegate.memory)
        if not np.allclose([command.tau_x,command.tau_m],
                           [record['reconstructed_tau_x'],record['reconstructed_tau_m']],atol=1e-12,rtol=0):
            raise AssertionError('Diagnostic decomposition differs from actual PID')
        self.records.append(record)
        return command


class PrescribedPathPlanner:
    """Experiment fixture, not a new planning algorithm; Navigator validates it."""
    name = 'diagnostic_prescribed_path'

    def __init__(self, path):
        self.path = path

    def plan(self, request, space):
        return PlanningResult(PlanningStatus.SUCCESS,self.name,self.path)


def polyline_distance(points, vertices):
    """Nearest finite-segment distance, not distance to infinite supporting lines."""
    points, vertices = np.asarray(points,float), np.asarray(vertices,float)
    if len(vertices) < 2 or np.any(np.linalg.norm(np.diff(vertices,axis=0),axis=1)==0):
        raise ValueError('Need nondegenerate polyline segments')
    a, delta = vertices[:-1], np.diff(vertices,axis=0)
    fractions = np.clip(np.sum((points[:,None,:]-a)*delta,axis=2)/np.sum(delta*delta,axis=1),0,1)
    nearest = a+fractions[:,:,None]*delta
    return np.min(np.linalg.norm(points[:,None,:]-nearest,axis=2),axis=1)


def summarize(result, records, path, config, turn_time=None):
    """Separate reference timing, geometric deviation, inner PID error and limits."""
    states, refs, controls = result.state_history, result.reference_history, result.control_history
    xy=np.array([[s.pose.x,s.pose.z] for s in states])
    rxy=np.array([[s.pose.x,s.pose.z] for s in refs])
    times=np.array([s.timestamp for s in states])
    vertices=np.array([[s.x,s.z] for s in path])
    theta=np.array([s.pose.pitch for s in refs])
    tangent=np.column_stack((np.cos(theta),np.sin(theta)))
    offset=xy-rxy
    # Positive means the timed reference is ahead in its active segment direction.
    lag=-np.sum(offset*tangent,axis=1)
    cross=offset[:,0]*(-tangent[:,1])+offset[:,1]*tangent[:,0]
    distance=polyline_distance(xy,vertices)
    torques=np.array([s.applied.tau_m for s in controls])
    effective=np.array([s['effective_pitch_error'] for s in records])
    geometric=np.array([wrap_to_pi(s.pose.pitch-r.pose.pitch) for s,r in zip(states,refs)])
    diagnostics={}
    for phase, mask in (
        ('moving',times[:-1]<result.trajectory.end_time),
        ('terminal',times[:-1]>=result.trajectory.end_time),
        ('turn', (times[:-1]>=turn_time)&(times[:-1]<result.trajectory.end_time) if turn_time is not None else np.zeros(len(controls),bool))):
        if not np.any(mask):
            diagnostics[phase]=None
            continue
        ids=np.flatnonzero(mask)
        diagnostics[phase]=dict(samples=len(ids),
            max_path_distance=float(max(distance[ids])),
            max_abs_timed_cross_track=float(max(abs(cross[ids]))),
            max_reference_lead=float(max(lag[ids])), min_reference_lead=float(min(lag[ids])),
            max_tangent_pitch_error_deg=float(max(abs(geometric[ids]))*180/pi),
            max_effective_pitch_error_deg=float(max(abs(effective[ids]))*180/pi),
            max_abs_tau_m=float(max(abs(torques[ids]))),
            torque_saturation_fraction=float(np.mean(abs(torques[ids])>=config.maximum_tau_m-1e-12)),
            torque_above_90_percent_fraction=float(np.mean(abs(torques[ids])>=.9*config.maximum_tau_m)),
            pitch_integral_at_limit_fraction=float(np.mean([abs(records[i]['pitch_integral'])>=1-1e-12 for i in ids])))
    if turn_time is not None:
        # Include one pre-corner numerical sample and the first at/after reference end.
        lo=max(0,np.searchsorted(times,turn_time)-1)
        hi=min(len(times),np.searchsorted(times,result.trajectory.end_time)+1)
        envelope=xy[lo:hi]
        lengths=np.linalg.norm(np.diff(envelope,axis=0),axis=1)
        # Distance to a closed set is 1-Lipschitz: bound any point of each chord.
        upper=max(distance[lo:hi])
        if len(lengths):
            upper=max(upper,max(np.maximum(distance[lo:hi-1],distance[lo+1:hi])+lengths/2))
        diagnostics['turn_envelope']=dict(x_min=float(min(envelope[:,0])),x_max=float(max(envelope[:,0])),
            z_min=float(min(envelope[:,1])),z_max=float(max(envelope[:,1])),
            chord_path_distance_upper_bound=float(upper),
            note='Centre trace during scheduled turn through reference end, not certified vehicle feasibility')
    return dict(metrics=result.metrics,phases=diagnostics,
                last_controller_terms=records[-1],
                last_state=asdict(states[-1]), last_reference=asdict(refs[-1]),
                turn_time=turn_time,reference_end=result.trajectory.end_time,
                final_reference_lead=float(lag[-1]),final_timed_cross_track=float(cross[-1]))


def run_case(name, path, world, initial, constraints, speed, horizon, turn_time=None):
    config=NavigationConfig(max_duration=horizon)
    params=load_baseline_controller_parameters()
    observer=ObservedController(params)
    request=NavigationRequest(initial,path[-1],world,constraints)
    navigator=Navigator(PrescribedPathPlanner(path),observer,Python3DOFBackend(load_synthetic_parameters()),
                        config,TrajectoryConfig(nominal_speed=speed,shortcut=False))
    result=navigator.run(request)
    if result.trajectory is None or len(observer.records)!=len(result.control_history) or not observer.records:
        raise RuntimeError(f'{name}: diagnostic pipeline failed: {result.status} {result.message}')
    checks=check_execution(result,request,config)
    if not all(checks.values()):
        raise AssertionError(checks)
    report=summarize(result,observer.records,path,config,turn_time)
    report.update(name=name,status=result.status.name,checks=checks,path=[asdict(p) for p in path],
                  speed=speed,config=asdict(config),constraints=asdict(constraints),
                  controller_parameters=asdict(params))
    history=dict(state=[asdict(s) for s in result.state_history],reference=[asdict(s) for s in result.reference_history],
                 controls=[asdict(s) for s in result.control_history],controller_terms=observer.records)
    return report,history


def cases():
    """Freeze original geometry for controlled obstacle/alignment comparisons."""
    suite=json.loads((ROOT/'config/scenarios/planner_acceptance_v1.json').read_text())
    constraints=PlanningConstraints(**suite['constraints'])
    for name,goal in [('ascending',Pose(x=9,z=-3)),('descending',Pose(x=9,z=-7)),('detour',Pose(x=9,z=-5))]:
        obstacles=[Obstacle.rectangle(**o) for o in suite['cases'][0]['obstacles']] if name=='detour' else []
        world=WorldModel(Boundary.rectangle(**suite['boundary']),obstacles)
        initial=VehicleState(Pose(**suite['start']),Twist.zero())
        req=PlanningRequest(initial,goal,constraints)
        space=EnvironmentPlanningSpace(world)
        planned=AStarPlanner(AStarConfig(**suite['astar'])).plan(req,space)
        generated=generate_trajectory(planned.path,space,constraints,request=req)
        path=generated.shortcut_path
        yield name,path,world,initial,constraints,.5,120.,None
        if name=='detour':
            aligned=replace(initial,pose=replace(initial.pose,pitch=atan2(path[1].z-path[0].z,path[1].x-path[0].x)))
            turn_time=np.hypot(path[1].x-path[0].x,path[1].z-path[0].z)/.5
            for variant,env,state in [('aligned',world,aligned),
                                       ('open',WorldModel(world.boundary),initial),
                                       ('open_aligned',WorldModel(world.boundary),aligned)]:
                yield 'detour_'+variant,path,env,state,constraints,.5,120.,float(turn_time)
    world=WorldModel(Boundary.rectangle(x_min=-30,x_max=30,z_min=-50,z_max=0))
    for angle in (15,30,45,60,90,120):
        for speed in (.2,.3,.4,.5):
            theta=angle*pi/180
            path=Path.from_list([Pose(x=0,z=-20),Pose(x=5,z=-20),Pose(x=5+5*cos(theta),z=-20+5*sin(theta))])
            # Matched initial surge removes a rest-to-cruise step as the main confound.
            initial=VehicleState(path[0],Twist(u=speed))
            horizon=float(ceil(10/speed+60))
            yield f'corner_{angle:03d}_{speed:.1f}',path,world,initial,constraints,speed,horizon,5/speed


def plot_corners(reports, histories, output):
    """One fixed-axis panel per angle; retain actual sampled turn envelopes."""
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    fig=Figure(figsize=(13,8),layout='constrained'); FigureCanvasAgg(fig)
    for ax,angle in zip(fig.subplots(2,3).flat,(15,30,45,60,90,120)):
        selected=[r for r in reports if r['name'].startswith(f'corner_{angle:03d}_')]
        path=selected[0]['path']
        ax.plot([p['x'] for p in path],[p['z']+20 for p in path],'k--',label='Reference')
        for report in selected:
            # Show moving phase only so terminal settling does not obscure the turn.
            states=[s for s in histories[report['name']]['state'] if s['timestamp']<=report['reference_end']]
            ax.plot([s['pose']['x'] for s in states],[s['pose']['z']+20 for s in states],label=f"{report['speed']:.1f} m/s")
        ax.set(title=f'{angle} degrees',xlabel='East x (m)',ylabel='z relative to start (m)',
               xlim=(-1,11),ylim=(-2,7),aspect='equal')
        ax.legend(fontsize=7); ax.grid(alpha=.2)
    fig.suptitle('Unchanged baseline: scheduled moving phase only; not a reliability classification')
    fig.savefig(output/'corner_envelopes.png',dpi=140)
    fig.savefig(output/'corner_envelopes.svg'); fig.clear()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=FilePath,required=True)
    args=parser.parse_args(); args.output.mkdir(parents=True,exist_ok=False)
    reports=[]; histories={}
    for inputs in cases():
        report,history=run_case(*inputs)
        repeat,second=run_case(*inputs)
        if report!=repeat or history!=second:
            raise AssertionError('Nondeterministic diagnostic run')
        report['deterministic']=True
        reports.append(report); histories[report['name']]=history
        (args.output/(report['name']+'.json')).write_text(json.dumps(clean(dict(summary=report,history=history)),allow_nan=False),encoding='utf-8')
        print(report['name'],report['status'],f"goal error {report['metrics']['terminal_position_error']:.3f}",flush=True)
    plot_corners(reports,histories,args.output)
    summary=dict(executable=sys.executable,python=sys.version,cases=reports,
                 unchanged_controller=True,unchanged_physics=True,shortcut=False,
                 note='Deterministic diagnostic measurements; no reliability thresholds or retiming policy fitted')
    summary['source_sha256']={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
        for folder in ('controller','physics','navigation','trajectory','environment','planner','core','simulation','validation','tests','config')
        for p in (ROOT/folder).rglob('*') if p.is_file() and p.suffix in ('.py','.json','.yaml')}
    summary['installed']=subprocess.check_output([sys.executable,'-m','pip','freeze'],text=True).splitlines()
    (args.output/'summary.json').write_text(json.dumps(clean(summary),indent=2,allow_nan=False),encoding='utf-8')
    manifest={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in args.output.iterdir() if p.is_file()}
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')


if __name__=='__main__':
    main()
