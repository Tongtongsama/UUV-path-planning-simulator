"""Saved-state terminal diagnostics; no planning or complete-path Physics replay."""
import argparse
from dataclasses import asdict
import hashlib
import json
from math import atan2, hypot
from pathlib import Path

import numpy as np
from core import Pose, Twist, VehicleState
from controller import BodyGuidance3DOF, CascadedPID3DOFController, ControlLimits3DOF
from environment import Boundary, WorldModel, Obstacle, ConstantCurrent
from navigation import NavigationConfig, TerminalCaptureConfig, TerminalCapturePolicy
from physics import Python3DOFBackend
from physics.frames import body_to_world_velocity_matrix, world_current_to_body
from physics.uuv_3dof import Python3DOFDynamics
from planner import EnvironmentPlanningSpace
from simulation.controller_config import load_baseline_controller_parameters
from simulation.vehicle_config import load_synthetic_parameters
from validation.navigation_acceptance import ROOT, clean


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def state_from_dict(data):
    return VehicleState(Pose(**data['pose']),Twist(**data['twist']),data['timestamp'])


def speed_diagnostics(state,current):
    """Distinguish frame rotation from water-relative velocity subtraction."""
    absolute=np.array([state.twist.u,state.twist.w])
    water=world_current_to_body(np.asarray(current),state.pose.pitch)
    world=body_to_world_velocity_matrix(state.pose.pitch)[:2,:2]@absolute
    return dict(body_absolute_speed=float(np.linalg.norm(absolute)),world_ground_speed=float(np.linalg.norm(world)),
        water_relative_speed=float(np.linalg.norm(absolute-water)),world_velocity=world.tolist(),
        body_current=water.tolist())


def restore_controller(common,history,entry):
    """Replay controller calls only, validating every output and final integral state.

    No private memory writes, Physics calls, time reset or fresh-integrator shortcut.
    """
    controller=CascadedPID3DOFController(load_baseline_controller_parameters(
        ROOT/'config/controllers/cascaded_pid_3dof_restoring_candidate.yaml'))
    if asdict(controller.parameters)!=common['controller']:raise ValueError('controller config changed')
    cfg=common['navigation'];limits=ControlLimits3DOF(cfg['maximum_tau_x'],cfg['maximum_tau_m'])
    controller.reset()
    for i,record in enumerate(history['controls'][:entry]):
        target=record['effective_guidance']
        if target is None:raise ValueError('expected saved LOS direct guidance')
        output=controller.step_guidance(state_from_dict(history['states'][i]),BodyGuidance3DOF(**target),cfg['dt'],limits)
        if asdict(output)!=record['commanded']:raise ValueError(f'controller replay differs at {i}')
        if asdict(controller.memory)!=record['controller_diagnostics']['integral_after']:
            raise ValueError(f'integral replay differs at {i}')
    return controller


def load_entry(evidence,name):
    common=json.loads((evidence/'common'/(name+'.json')).read_text())
    history=json.loads((evidence/'los'/(name+'.json')).read_text())['history']
    entry=next((i for i,s in enumerate(history['controls']) if s['guidance_phase']=='TERMINAL_CAPTURE'),len(history['controls']))
    state=state_from_dict(history['states'][entry])
    return common,history,entry,state


def theoretical_brake_case(current=(0.,.05)):
    """At held pitch=0 and u=q=0, no heave actuator can reject vertical drag."""
    dynamics=Python3DOFDynamics(load_synthetic_parameters())
    at_rest=dynamics.derivatives(np.zeros(6),np.zeros(3),np.asarray(current))
    drift=np.array([0.,0.,0.,0.,-current[1],0.])
    equilibrium=dynamics.derivatives(drift,np.zeros(3),np.asarray(current))
    return dict(rest_nu_dot=at_rest.nu_dot.tolist(),drift_nu_dot=equilibrium.nu_dot.tolist(),
        drift_world_rates=equilibrium.eta_dot.tolist(),brake_speed=TerminalCaptureConfig().brake_speed)


def replay(evidence,name,mode,settings):
    """Direct frozen capture, zero-current counterfactual, or bounded pre-capture probe."""
    if mode not in ('original','zero_current','stabilize'):raise ValueError('unknown replay mode')
    common,history,entry,state=load_entry(evidence,name)
    initial=state
    controller=restore_controller(common,history,entry)
    initial_memory=asdict(controller.memory)
    cfg=NavigationConfig(**common['navigation'])
    capture=TerminalCapturePolicy(TerminalCaptureConfig(**common['capture']))
    current=[0.,0.] if mode=='zero_current' else common['current']
    world=WorldModel(Boundary.rectangle(**common['boundary']),
        [Obstacle.rectangle(**o) for o in common['scene']['obstacles']],ConstantCurrent(current))
    space=EnvironmentPlanningSpace(world)
    required=common['constraints']['vehicle_radius']+common['constraints']['safety_margin']
    limits=ControlLimits3DOF(cfg.maximum_tau_x,cfg.maximum_tau_m)
    physics=Python3DOFBackend(load_synthetic_parameters())
    goal=Pose(**common['goal'])
    a,b=[Pose(**p) for p in common['processed_path'][-2:]]
    final_pitch=atan2(b.z-a.z,b.x-a.x)
    held=0.;states=[asdict(state)];steps=[];handoff=None;failure=None;status='MISSION_HORIZON'
    stabilizing=mode=='stabilize'
    stabilization_started=state.timestamp
    initial_rate=abs(state.twist.q)

    def qualified(s):
        return bool(hypot(s.pose.x-goal.x,s.pose.z-goal.z)<=cfg.goal_position_tolerance
            and hypot(s.twist.u,s.twist.w)<=cfg.goal_speed_tolerance and abs(s.twist.q)<=cfg.goal_pitch_rate_tolerance)

    while state.timestamp+cfg.dt<=common['initial']['timestamp']+cfg.max_duration+1e-9:
        if stabilizing:
            if abs(state.pose.pitch)>capture.config.maximum_actual_pitch or abs(state.twist.q)>initial_rate+1e-9:
                status='STABILIZATION_ENVELOPE_FAILED';break
            if hypot(state.pose.x-initial.pose.x,state.pose.z-initial.pose.z)>settings['stabilization_maximum_displacement']:
                status='STABILIZATION_DISPLACEMENT_FAILED';break
            if hypot(state.twist.u,state.twist.w)<=capture.config.brake_speed and abs(state.twist.q)<=capture.config.brake_pitch_rate:
                stabilizing=False
            elif state.timestamp-stabilization_started+cfg.dt>settings['stabilization_budget']+1e-9:
                status='STABILIZATION_BUDGET_FAILED';break
        if stabilizing:
            phase='DIAGNOSTIC_STABILIZATION'
            target=BodyGuidance3DOF(-max(-capture.config.brake_reverse_target,
                min(capture.config.brake_reverse_target,state.twist.u)),initial.pose.pitch,0.)
        else:
            if handoff is None:handoff=state.timestamp
            reference=VehicleState(Pose(x=goal.x,z=goal.z,pitch=final_pitch),Twist.zero(),state.timestamp)
            decision=capture.update_active(state,reference,goal,cfg.dt,qualified(state),space,required)
            if decision.failure:
                status='CAPTURE_FAILED';failure=decision.failure;break
            phase=decision.phase
            target=decision.target or BodyGuidance3DOF(0.,final_pitch,0.)
        output=controller.step_guidance(state,target,cfg.dt,limits)
        if abs(output.tau_x)>cfg.maximum_tau_x or abs(output.tau_m)>cfg.maximum_tau_m:raise AssertionError('limit exceeded')
        nxt=physics.step(state,output,np.asarray(current),cfg.dt)
        clearance=world.segment_clearance(state.pose,nxt.pose)
        in_bounds=world.boundary.contains_segment(state.pose,nxt.pose,required)
        steps.append(dict(time=state.timestamp,phase=phase,target=asdict(target),control=asdict(output),
            controller_diagnostics=asdict(controller.last_diagnostics),speeds=speed_diagnostics(state,current),
            clearance=clearance,net_clearance=clearance-required,in_bounds=in_bounds))
        states.append(asdict(nxt))
        held=held+cfg.dt if qualified(state) and qualified(nxt) else 0.
        state=nxt
        if clearance<=required:status='COLLISION';break
        if not in_bounds:status='OUT_OF_BOUNDS';break
        if qualified(state) and held+1e-12>=cfg.settle_time:status='SUCCESS';break
    exact_suffix=None
    if mode=='original':
        exact_suffix=states==history['states'][entry:]
        if not exact_suffix:raise AssertionError('original replay differs from saved physics suffix')
    summary=dict(name=name,mode=mode,status=status,message=failure,entry_index=entry,
        entry_state=asdict(initial),initial_controller_memory=initial_memory,final_state=asdict(state),
        elapsed=state.timestamp-initial.timestamp,handoff_time=handoff,
        capture_budget=capture.config.budget,capture_attempts=capture.attempts,goal_held_seconds=held,
        final_error=hypot(state.pose.x-goal.x,state.pose.z-goal.z),current=current,
        final_speeds=speed_diagnostics(state,current),exact_original_suffix=exact_suffix,
        all_limits_satisfied=all(abs(s['control']['tau_x'])<=cfg.maximum_tau_x and abs(s['control']['tau_m'])<=cfg.maximum_tau_m for s in steps),
        phase_counts={phase:sum(s['phase']==phase for s in steps) for phase in sorted({s['phase'] for s in steps})},
        capture_config=asdict(capture.config))
    return dict(summary=summary,states=states,steps=steps)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence',type=Path,default=ROOT/'artifacts/guidance_matched_16_v1')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    manifest=json.loads((args.evidence/'manifest.json').read_text())
    for name,expected in manifest.items():
        if digest(args.evidence/name)!=expected:raise ValueError(f'evidence mismatch: {name}')
    settings=json.loads((ROOT/'config/terminal_replay_diagnostics.json').read_text())
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'config.json').write_text(json.dumps(settings,indent=2),encoding='utf-8')
    cases=[]
    for name,modes in [('detour',('original','stabilize')),('current_+0.00_+0.05',('original','zero_current'))]:
        for mode in modes:
            result=replay(args.evidence,name,mode,settings)
            again=replay(args.evidence,name,mode,settings)
            if result!=again:raise AssertionError('replay is not deterministic')
            result['summary']['deterministic']=True
            cases.append(result['summary'])
            (args.output/(name+'_'+mode+'.json')).write_text(json.dumps(clean(result),allow_nan=False),encoding='utf-8')
            print(name,mode,result['summary']['status'],result['summary']['elapsed'],flush=True)
    summary=dict(cases=cases,theoretical_brake=theoretical_brake_case(),
        input_manifest_sha256=digest(args.evidence/'manifest.json'),
        diagnostic_source_sha256=digest(Path(__file__)),config_sha256=digest(ROOT/'config/terminal_replay_diagnostics.json'))
    (args.output/'summary.json').write_text(json.dumps(clean(summary),indent=2,allow_nan=False),encoding='utf-8')
    (args.output/'manifest.json').write_text(json.dumps({p.name:digest(p) for p in args.output.iterdir() if p.is_file()},indent=2),encoding='utf-8')


if __name__=='__main__':main()
