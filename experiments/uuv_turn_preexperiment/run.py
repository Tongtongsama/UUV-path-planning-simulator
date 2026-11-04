"""Opt-in vertical-plane local-reference shape experiment using production dynamics.

Run: .venv/Scripts/python -m experiments.uuv_turn_preexperiment.run --output artifacts/uuv_turn_preexperiment_YYYYMMDD
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import json
from math import atan2, hypot, pi, sin, cos
from pathlib import Path
import platform
import subprocess
import sys
import time

import numpy as np

from controller import CascadedPID3DOFController
from core import ControlInput, Pose, Twist, VehicleState
from navigation import NavigationConfig, NavigationRequest, Navigator, TerminalCapturePolicy
from physics import Python3DOFBackend
from simulation.controller_config import load_baseline_controller_parameters
from simulation.vehicle_config import load_synthetic_parameters
from trajectory import TrajectoryConfig
from validation.tracking_diagnostics import ROOT, PrescribedPathPlanner, cases


HERE = Path(__file__).resolve().parent


class ReferenceAdapter:
    """Wrap the actual PID without resetting it at either local-reference event."""

    def __init__(self, delegate, path, speed, amplitude, start_s, end_s, zero_after_s=None):
        self.delegate = delegate
        self.path = path
        self.speed = speed
        self.amplitude = amplitude
        self.start_s = start_s
        self.end_s = end_s
        self.zero_after_s = zero_after_s
        self.zero_active = False
        self.records = []
        a, b, c = path
        self.leg1 = hypot(b.x-a.x, b.z-a.z)
        self.leg2 = hypot(c.x-b.x, c.z-b.z)
        self.u1 = np.array([(b.x-a.x)/self.leg1, (b.z-a.z)/self.leg1])
        self.u2 = np.array([(c.x-b.x)/self.leg2, (c.z-b.z)/self.leg2])
        self.vertex = np.array([b.x,b.z])

    @property
    def last_diagnostics(self):
        return None if self.zero_active else self.delegate.last_diagnostics

    def reset(self):
        self.delegate.reset()
        self.records.clear()
        self.zero_active = False

    def _reference(self, nominal):
        if self.amplitude == 0:
            return nominal
        s = self.speed * nominal.timestamp
        s0 = self.start_s
        s1 = self.end_s
        if not s0 < s < s1:
            return nominal
        # A local outward depth bulge begins at a fixed global-path arc-length
        # event and returns to the unchanged vertex. Position/tangent/speed
        # match the nominal path at both ends. q follows actual curvature.
        span = s1-s0
        y = (s-s0)/span
        outward = np.array([0.,-1.])
        pos = self.vertex+(s-self.leg1)*self.u1+self.amplitude*sin(pi*y)**2*outward
        tangent = self.u1+self.amplitude*pi/span*sin(2*pi*y)*outward
        second = self.amplitude*2*pi*pi/span**2*cos(2*pi*y)*outward
        norm = float(np.linalg.norm(tangent))
        if norm < 1e-8:
            raise ValueError('local reference has zero tangent')
        pitch = atan2(tangent[1], tangent[0])
        q = self.speed*float(np.linalg.det(np.stack([tangent, second])))/norm**2
        return VehicleState(Pose(x=float(pos[0]), z=float(pos[1]), pitch=pitch),
                            Twist(u=self.speed*norm, q=q), nominal.timestamp)

    def step_with_limits(self, current, reference, dt, limits):
        effective = self._reference(reference)
        if self.zero_after_s is not None and self.speed*current.timestamp >= self.zero_after_s:
            self.zero_active = True
            self.records.append(effective)
            return ControlInput.zero()
        command = self.delegate.step_with_limits(current, effective, dt, limits)
        self.records.append(effective)
        return command

    def step(self, current, reference, dt):
        effective = self._reference(reference)
        if self.zero_after_s is not None and self.speed*current.timestamp >= self.zero_after_s:
            self.zero_active = True
            self.records.append(effective)
            return ControlInput.zero()
        command = self.delegate.step(current, effective, dt)
        self.records.append(effective)
        return command

    def step_guidance(self, current, target, dt, limits):
        self.zero_active = False
        command = self.delegate.step_guidance(current, target, dt, limits)
        self.records.append(VehicleState(Pose(x=self.path[-1].x,z=self.path[-1].z,pitch=target.pitch),
                                         Twist(u=target.u,q=target.q),current.timestamp))
        return command


def fixture():
    scene = next(c for c in cases() if c[0] == 'detour')
    return scene[1], scene[2], scene[3], scene[4]


def run_case(alpha, shift, config, dt=None, wrapped=True, capture=False, zero_after_reference=False):
    path, world, initial, constraints = fixture()
    speed = config['speed_m_s']
    amplitude = config['maximum_outward_depth_bump_m']*(1-alpha)
    nav_config = NavigationConfig(dt=dt or config['dt_s'], max_duration=config['timeout_s'])
    pid = CascadedPID3DOFController(load_baseline_controller_parameters())
    start_s = config['nominal_start_event_s_m']+shift
    end_s = config['nominal_end_event_s_m']+shift
    leg1=hypot(path[1].x-path[0].x,path[1].z-path[0].z)
    if not 0 <= start_s < end_s <= leg1:raise ValueError('local program leaves first path leg')
    length=sum(hypot(b.x-a.x,b.z-a.z) for a,b in zip(path,path.poses[1:]))
    controller = ReferenceAdapter(pid, path, speed, amplitude, start_s, end_s,
        zero_after_s=length if zero_after_reference else None) if wrapped else pid
    nav = Navigator(PrescribedPathPlanner(path), controller,
                    Python3DOFBackend(load_synthetic_parameters()), nav_config,
                    TrajectoryConfig(nominal_speed=speed, shortcut=False),
                    capture_policy=TerminalCapturePolicy() if capture else None)
    result = nav.run(NavigationRequest(initial, path[-1], world, constraints))
    effective = controller.records if wrapped else result.reference_history[:-1]
    if len(effective) != len(result.control_history):
        raise AssertionError('reference/control history length mismatch')
    rows = []
    for i, step in enumerate(result.control_history):
        state, ref = result.state_history[i], effective[i]
        diag = step.controller_diagnostics
        rows.append(dict(time_s=step.time, x_m=state.pose.x, y_m=state.pose.y, z_m=state.pose.z,
            roll_rad=state.pose.roll, pitch_rad=state.pose.pitch, yaw_rad=state.pose.yaw,
            u_m_s=state.twist.u, v_m_s=state.twist.v, w_m_s=state.twist.w,
            p_rad_s=state.twist.p, q_rad_s=state.twist.q, r_rad_s=state.twist.r,
            ref_x_m=ref.pose.x, ref_z_m=ref.pose.z, ref_pitch_rad=ref.pose.pitch,
            ref_u_m_s=ref.twist.u, ref_q_rad_s=ref.twist.q,
            tau_x_request_N=diag.requested.tau_x if diag else None,
            tau_m_request_Nm=diag.requested.tau_m if diag else None,
            tau_x_limited_N=step.commanded.tau_x, tau_m_limited_Nm=step.commanded.tau_m,
            tau_x_applied_N=step.applied.tau_x, tau_m_applied_Nm=step.applied.tau_m,
            tau_y_applied_N=step.applied.tau_y, tau_z_applied_N=step.applied.tau_z,
            tau_k_applied_Nm=step.applied.tau_k, tau_n_applied_Nm=step.applied.tau_n,
            mode='OPEN_LOOP_ZERO' if zero_after_reference and speed*step.time>=length else step.capture_phase if capture and step.capture_phase!='DISABLED' else step.guidance_phase,
            guidance_u_m_s=step.effective_guidance.u if step.effective_guidance else None,
            guidance_pitch_rad=step.effective_guidance.pitch if step.effective_guidance else None,
            guidance_q_rad_s=step.effective_guidance.q if step.effective_guidance else None,
            center_segment_distance_m=step.segment_clearance,
            proxy_hull_clearance_m=step.segment_clearance-constraints.vehicle_radius,
            signed_safety_margin_m=step.segment_clearance-constraints.required_clearance,
            proxy_hull_overlap=step.segment_clearance<=constraints.vehicle_radius,
            safety_violation=step.collision, out_of_bounds=not step.in_bounds,
            at_limit=step.at_limit, clipped=step.clipped))
    final = result.final_state
    exit_cfg = config['exit']
    speed_final = hypot(final.twist.u, final.twist.w)
    exit_ok = bool(final.pose.x-constraints.vehicle_radius >= exit_cfg['x_min_m']
        and abs(final.pose.z-path[-1].z) <= exit_cfg['z_error_max_m']
        and abs(final.pose.pitch) <= exit_cfg['pitch_error_max_rad']
        and speed_final <= exit_cfg['speed_max_m_s']
        and abs(final.twist.q) <= exit_cfg['q_max_rad_s'])
    success = result.status.name == 'SUCCESS' and exit_ok
    angles = np.unwrap([s.pose.pitch for s in result.state_history])
    turn = np.diff(angles)
    lengths = [hypot(b.pose.x-a.pose.x,b.pose.z-a.pose.z) for a,b in zip(result.state_history,result.state_history[1:])]
    min_index = min(range(len(rows)),key=lambda i: rows[i]['center_segment_distance_m']) if rows else None
    end_index = next((i for i,s in enumerate(result.state_history) if speed*s.timestamp >= end_s),len(result.state_history)-1)
    signed_residual = float(angles[-1]-angles[end_index]) if speed*final.timestamp >= end_s else None
    request_peak = max((abs(r['tau_m_request_Nm']) for r in rows if r['tau_m_request_Nm'] is not None), default=0.)
    applied_peak = max((abs(r['tau_m_applied_Nm']) for r in rows), default=0.)
    request_peak_x = max((abs(r['tau_x_request_N']) for r in rows if r['tau_x_request_N'] is not None), default=0.)
    applied_peak_x = max((abs(r['tau_x_applied_N']) for r in rows), default=0.)
    summary = dict(alpha=alpha, outward_depth_bump_m=amplitude, start_shift_m=shift,
        termination_policy='open_loop_zero' if zero_after_reference else 'existing_capture' if capture else 'terminal_hold',
        start_event_s_m=start_s,
        end_event_s_m=end_s, status=result.status.name, success=success,
        exit_ok=exit_ok, termination_reason=result.message or result.status.name,
        steps=len(rows), duration_s=result.metrics['simulation_duration'],
        d_min_m=min((r['signed_safety_margin_m'] for r in rows),default=None),
        proxy_hull_d_min_m=min((r['proxy_hull_clearance_m'] for r in rows),default=None),
        center_d_min_m=min((r['center_segment_distance_m'] for r in rows),default=None),
        d_safe_m=0., proxy_hull_d_safe_m=constraints.safety_margin,
        proxy_radius_m=constraints.vehicle_radius,
        first_min_segment_start_x_m=rows[min_index]['x_m'] if min_index is not None else None,
        first_min_segment_start_z_m=rows[min_index]['z_m'] if min_index is not None else None,
        exit_x_error_m=final.pose.x-path[-1].x, exit_z_error_m=final.pose.z-path[-1].z,
        exit_pitch_error_rad=final.pose.pitch, exit_speed_m_s=speed_final,
        exit_q_rad_s=final.twist.q, final_r_rad_s=final.twist.r,
        signed_turn_rad=float(sum(turn)), absolute_turn_rad=float(sum(abs(turn))),
        residual_turn_after_program_rad=signed_residual,
        command_strength_peak_ref_q_rad_s=max((abs(r['ref_q_rad_s']) for r in rows),default=0.),
        peak_tau_m_request_ratio=request_peak/nav_config.maximum_tau_m,
        peak_tau_m_applied_ratio=applied_peak/nav_config.maximum_tau_m,
        peak_tau_x_request_ratio=request_peak_x/nav_config.maximum_tau_x,
        peak_tau_x_applied_ratio=applied_peak_x/nav_config.maximum_tau_x,
        saturation_fraction=result.metrics['saturation_fraction'],
        path_length_m=sum(lengths), tracking_rmse_m=float(np.sqrt(np.mean([
            (r['x_m']-r['ref_x_m'])**2+(r['z_m']-r['ref_z_m'])**2 for r in rows]))) if rows else None,
        collision=result.metrics['collision'], out_of_bounds=result.metrics['out_of_bounds'],
        reference_pitch_jump_max_rad=max((abs(b['ref_pitch_rad']-a['ref_pitch_rad']) for a,b in zip(rows,rows[1:])),default=0.),
        final_state=asdict(final),d_min_over_L=None,
        navigation_metrics=result.metrics)
    return summary, rows, result


def write_case(output, name, summary, rows):
    (output/f'{name}.json').write_text(json.dumps(summary, indent=2, default=float),encoding='utf-8')
    with (output/f'{name}.csv').open('w',newline='',encoding='utf-8') as handle:
        writer=csv.DictWriter(handle,fieldnames=rows[0].keys())
        writer.writeheader(); writer.writerows(rows)


def make_plots(output, groups, histories, config):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle
    for group, parameter, identity in [('E1','alpha',1.),('E2','start_shift_m',0.)]:
        data = groups[group]
        fig, axes = plt.subplots(2,2,figsize=(10,7),layout='constrained')
        for ax, metric, ylabel in zip(axes.flat,
            ['d_min_m','signed_turn_rad','saturation_fraction','duration_s'],
            ['signed safety-envelope d_min (m)','actual signed turn (rad)','saturation fraction','run time (s)']):
            for label, marker, color in [('success','o','tab:green'),('failure','x','tab:red')]:
                subset=[r for r in data if r['success']==(label=='success')]
                ax.scatter([r[parameter] for r in subset],[r[metric] for r in subset],marker=marker,color=color,label=label)
            ax.axvline(identity,color='black',ls=':',label='identity' if group=='E1' else 'zero shift')
            if metric=='d_min_m': ax.axhline(data[0]['d_safe_m'],color='gray',ls='--',label='d_safe')
            ax.set(xlabel=parameter,ylabel=ylabel);ax.grid(alpha=.2);ax.legend(fontsize=8)
        fig.savefig(output/f'{group}_scan.png',dpi=140);plt.close(fig)
    fig, axes = plt.subplots(1,2,figsize=(13,5),layout='constrained')
    path,world,_,_=fixture()
    for name, rows in histories.items():
        if name not in ('identity','E1_00','E1_08','E2_00','E2_20'):continue
        axes[0].plot([r['x_m'] for r in rows],[r['z_m'] for r in rows],label=name)
        axes[1].plot([r['time_s'] for r in rows],[r['pitch_rad'] for r in rows],label=name+' pitch')
    axes[0].plot([p.x for p in path],[p.z for p in path],'k--',label='global path')
    axes[0].add_patch(Rectangle((4.9,-7),.2,4,color='gray',alpha=.5))
    axes[0].set(xlabel='x (m)',ylabel='z (m)',aspect='equal');axes[0].legend(fontsize=8);axes[0].grid(alpha=.2)
    axes[1].set(xlabel='time (s)',ylabel='pitch (rad)');axes[1].legend(fontsize=7);axes[1].grid(alpha=.2)
    fig.savefig(output/'trajectory_pitch.png',dpi=140);plt.close(fig)
    fig, axes = plt.subplots(2,1,figsize=(10,7),sharex=True,layout='constrained')
    for name in ('identity','E1_00','E1_08'):
        rows=histories[name]
        axes[0].plot([r['time_s'] for r in rows],[r['q_rad_s'] for r in rows],label=name)
        axes[1].plot([r['time_s'] for r in rows],[r['tau_m_applied_Nm'] for r in rows],label=name)
    axes[0].set(ylabel='q (rad/s)');axes[1].set(xlabel='time (s)',ylabel='applied tau_m (Nm)')
    for ax in axes:ax.legend();ax.grid(alpha=.2)
    fig.savefig(output/'q_tau.png',dpi=140);plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,6),layout='constrained')
    ax.add_patch(Rectangle((4.9,-7),.2,4,color='gray',alpha=.5,label='wall'))
    ax.plot([p.x for p in path],[p.z for p in path],'k--',lw=1,label='fixed global path')
    for name,color in [('identity','tab:red'),('E1_00','tab:blue')]:
        rows=histories[name]
        summary=json.loads((output/f'{name}.json').read_text(encoding='utf-8'))
        ax.plot([r['x_m'] for r in rows],[r['z_m'] for r in rows],color=color,label=name)
        start_s=summary['start_event_s_m']; leg=hypot(path[1].x-path[0].x,path[1].z-path[0].z)
        event=(path[0].x+start_s/leg*(path[1].x-path[0].x),path[0].z+start_s/leg*(path[1].z-path[0].z))
        ax.scatter(*event,marker='^',color=color,s=45)
        ax.scatter(summary['first_min_segment_start_x_m'],summary['first_min_segment_start_z_m'],
                   marker='x',color=color,s=55)
        for index in sorted(set([0,min(range(len(rows)),key=lambda i:abs(rows[i]['time_s']-start_s/config['speed_m_s'])),
                                  min(range(len(rows)),key=lambda i:abs(rows[i]['time_s']-summary['end_event_s_m']/config['speed_m_s'])),
                                  min(range(len(rows)),key=lambda i:abs(rows[i]['x_m']-summary['first_min_segment_start_x_m']))])):
            r=rows[index]
            ax.add_patch(Circle((r['x_m'],r['z_m']),summary['proxy_radius_m'],fill=False,
                                edgecolor=color,alpha=.7))
            ax.arrow(r['x_m'],r['z_m'],.3*cos(r['pitch_rad']),.3*sin(r['pitch_rad']),
                     color=color,head_width=.07,length_includes_head=True)
    ax.scatter(path[1].x,path[1].z,marker='s',color='black',s=30,label='global path vertex')
    ax.set(xlabel='x (m)',ylabel='z (m)',aspect='equal',xlim=(.5,9.8),ylim=(-9.7,-3.8))
    ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.savefig(output/'key_poses.png',dpi=140);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--smoke',action='store_true',help='only baseline and one transformed point')
    args=parser.parse_args()
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    config=json.loads((HERE/'config.json').read_text(encoding='utf-8'))
    started=time.perf_counter()
    groups={'E1':[],'E2':[]};histories={}
    def execute(name,alpha,shift,group=None,dt=None,wrapped=True):
        tic=time.perf_counter()
        summary,rows,result=run_case(alpha,shift,config,dt,wrapped)
        summary['wall_time_s']=time.perf_counter()-tic
        write_case(output,name,summary,rows)
        histories[name]=rows
        if group:groups[group].append(summary)
        print(name,summary['status'],'d_min=',round(summary['d_min_m'],4),'wall=',round(summary['wall_time_s'],2),flush=True)
        return summary,rows,result
    raw,raw_rows,raw_result=execute('unwrapped_baseline',1.,0.,wrapped=False)
    identity,identity_rows,identity_result=execute('identity',1.,0.)
    same=(raw_rows==identity_rows and raw_result.state_history==identity_result.state_history
          and raw_result.control_history==identity_result.control_history)
    repeat,repeat_rows,_=run_case(1.,0.,config)
    deterministic=identity_rows==repeat_rows and identity['navigation_metrics']==repeat['navigation_metrics']
    if not same or not deterministic:raise AssertionError('baseline identity or determinism failed')
    if args.smoke:
        execute('smoke_shape',0.,0.)
    else:
        for i,alpha in enumerate(config['e1_alpha']):execute(f'E1_{i:02d}',alpha,0.,'E1')
        for i,shift in enumerate(config['e2_shift_m']):execute(f'E2_{i:02d}',config['e2_fixed_alpha'],shift,'E2')
        if not any(abs(r['d_min_m']-identity['d_min_m'])>1e-5 for r in groups['E1']):
            raise AssertionError('reference parameter had no measurable effect')
        if not any(abs(r['d_min_m']-groups['E2'][10]['d_min_m'])>1e-5 for r in groups['E2']):
            raise AssertionError('start-position parameter had no measurable effect')
        make_plots(output,groups,histories,config)
    config_snapshot=dict(config=config,git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        git_status=subprocess.check_output(['git','status','--short'],cwd=ROOT,text=True).splitlines(),
        python=sys.version,platform=platform.platform(),numpy=np.__version__,seed=config['seed'],
        started_utc=datetime.now(timezone.utc).isoformat(),elapsed_wall_s=time.perf_counter()-started,
        run_count=3+(1 if args.smoke else len(config['e1_alpha'])+len(config['e2_shift_m'])),
        identity_equal=same,baseline_deterministic=deterministic,
        groups=groups)
    (output/'run_summary.json').write_text(json.dumps(config_snapshot,indent=2,default=float),encoding='utf-8')


if __name__=='__main__':main()
