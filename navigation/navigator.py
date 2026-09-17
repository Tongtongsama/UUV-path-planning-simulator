"""Compose existing algorithms; no planning, dynamics or PID implementation."""
from math import hypot, isclose, isfinite, sqrt, pi
from dataclasses import replace
from typing import Callable
import numpy as np
from core import ControlInput, Path, VehicleState
from controller.backend import Controller3DOF
from controller import LimitAwareController3DOF, ControlLimits3DOF, ControllerDiagnostics3DOF
from controller import GuidanceController3DOF
from .terminal_capture import TerminalCapturePolicy
from .startup import ControlledStartupPolicy
from physics.backend import PhysicsBackend, extract_reduced_state
from planner import GlobalPlanner, PlanningRequest, PlanningResult, EnvironmentPlanningSpace, validate_path
from trajectory import TrajectoryConfig, TrajectoryGenerationResult, generate_trajectory, sample_reference
from .models import NavigationConfig, NavigationRequest, NavigationResult, NavigationStatus as S, NavigationStep


def angle_error(a: float,b: float) -> float:
    """Signed shortest pitch difference."""
    return (a-b+pi)%(2*pi)-pi


class Navigator:
    """Run one static mission; reset controller each run, use stateless Physics.step."""
    def __init__(self, planner: GlobalPlanner, controller: Controller3DOF,
                 physics: PhysicsBackend, config: NavigationConfig = NavigationConfig(),
                 trajectory_config: TrajectoryConfig = TrajectoryConfig(),
                 generator: Callable[...,TrajectoryGenerationResult] = generate_trajectory,
                 capture_policy: TerminalCapturePolicy | None = None,
                 startup_policy: ControlledStartupPolicy | None = None) -> None:
        if not isinstance(config,NavigationConfig) or not isinstance(trajectory_config,TrajectoryConfig):
            raise TypeError("invalid navigation or trajectory configuration")
        for obj, methods in ((planner,("plan",)),(controller,("step","reset")),(physics,("step",))):
            if not all(callable(getattr(obj,m,None)) for m in methods):
                raise TypeError("backend does not provide required methods")
        if not callable(generator):
            raise TypeError("generator must be callable")
        self.planner,self.controller,self.physics=planner,controller,physics
        self.config,self.trajectory_config,self.generator=config,trajectory_config,generator
        if capture_policy is not None:
            if not isinstance(capture_policy,TerminalCapturePolicy):raise TypeError('invalid capture policy')
            if not isinstance(controller,GuidanceController3DOF):raise TypeError('capture requires explicit body-guidance capability')
        self.capture_policy=capture_policy
        if startup_policy is not None:
            if not isinstance(startup_policy,ControlledStartupPolicy):raise TypeError('invalid startup policy')
            if not isinstance(controller,GuidanceController3DOF):raise TypeError('startup requires explicit body-guidance capability')
        self.startup_policy=startup_policy

    def run(self, request: NavigationRequest) -> NavigationResult:
        """Plan once, generate once, then execute until safety/goal/horizon termination."""
        if not isinstance(request,NavigationRequest):
            raise TypeError("request must be NavigationRequest")
        c=self.config
        if self.capture_policy is not None:self.capture_policy.reset()
        if self.startup_policy is not None:self.startup_policy.reset()
        startup_delay=0.
        if request.planning_constraints.goal_tolerance>c.goal_position_tolerance:
            raise ValueError("planning tolerance must not exceed navigation position tolerance")
        world=request.environment
        required=request.planning_constraints.required_clearance
        space=EnvironmentPlanningSpace(world)
        planning_request=PlanningRequest(request.initial_state,request.goal,request.planning_constraints)
        states=[request.initial_state]; refs=[]; steps=[]
        plan=None; trajectory=None; held=0.0
        applied=None; current=None
        ref_clearance=None
        executed_clearance=world.segment_clearance(states[0].pose,states[0].pose)

        def arrived(state):
            return bool(hypot(state.pose.x-request.goal.x,state.pose.z-request.goal.z)<=c.goal_position_tolerance
                and hypot(state.twist.u,state.twist.w)<=c.goal_speed_tolerance
                and abs(state.twist.q)<=c.goal_pitch_rate_tolerance)

        def finish(status,message=None,failed_command=None):
            errors=[hypot(s.pose.x-r.pose.x,s.pose.z-r.pose.z) for s,r in zip(states,refs)]
            pitches=[angle_error(s.pose.pitch,r.pose.pitch) for s,r in zip(states,refs)]
            n=len(steps); final=states[-1]
            metric={"success":status is S.SUCCESS,"collision":any(s.collision for s in steps),
                "out_of_bounds":any(not s.in_bounds for s in steps),
                "simulation_duration":final.timestamp-states[0].timestamp,
                "reference_duration":None if trajectory is None else trajectory.duration,
                "position_rmse":sqrt(sum(e*e for e in errors)/len(errors)) if errors else None,
                "maximum_position_error":max(errors,default=None),
                "pitch_rmse":sqrt(sum(e*e for e in pitches)/len(pitches)) if pitches else None,
                "maximum_pitch_error":max(map(abs,pitches),default=None),
                "terminal_position_error":hypot(final.pose.x-request.goal.x,final.pose.z-request.goal.z),
                "terminal_speed":hypot(final.twist.u,final.twist.w),"terminal_pitch_rate":final.twist.q,
                "goal_held_seconds":held,"settling_time_after_reference":
                    max(0.,final.timestamp-trajectory.end_time-startup_delay) if trajectory is not None and status is S.SUCCESS else None,
                "minimum_reference_clearance":ref_clearance if ref_clearance is not None and isfinite(ref_clearance) else None,
                "minimum_executed_clearance":executed_clearance if isfinite(executed_clearance) else None,
                "minimum_executed_safety_margin":executed_clearance-required if isfinite(executed_clearance) else None,
                "saturation_count":sum(s.at_limit for s in steps),
                "saturation_duration":sum(s.at_limit for s in steps)*c.dt,
                "saturation_fraction":sum(s.at_limit for s in steps)/n if n else 0.,
                "navigation_clip_count":sum(s.clipped for s in steps),
                "control_effort_proxy":sum(s.applied.tau_x**2+s.applied.tau_m**2 for s in steps)*c.dt}
            if self.capture_policy is not None:
                moving=[e for s,e in zip(states,errors) if trajectory is not None and states[0].timestamp+startup_delay<=s.timestamp<trajectory.end_time+startup_delay]
                metric.update(capture_duration=0. if self.capture_policy.started is None else final.timestamp-self.capture_policy.started,
                    capture_attempts=self.capture_policy.attempts,
                    moving_position_rmse=sqrt(sum(e*e for e in moving)/len(moving)) if moving else None)
            if self.startup_policy is not None:
                metric.update(startup_duration=startup_delay,startup_released=self.startup_policy.released)
            for axis in ("tau_x","tau_m"):
                values=[getattr(s.applied,axis) for s in steps]
                metric["maximum_"+axis]=max(map(abs,values),default=0.)
                metric["rms_"+axis]=sqrt(sum(v*v for v in values)/n) if n else 0.
                metric["variation_"+axis]=sum(abs(b-a) for a,b in zip(values,values[1:]))
            return NavigationResult(status,plan,trajectory,tuple(states),tuple(refs),tuple(steps),metric,message,failed_command,
                applied if failed_command is not None else None,
                tuple(float(v) for v in current) if failed_command is not None and current is not None and current.shape==(2,) else None)

        try:
            plan=self.planner.plan(planning_request,space)
            if not isinstance(plan,PlanningResult):
                raise TypeError("planner did not return PlanningResult")
            if not plan.success:
                return finish(S.PLANNING_FAILED,plan.status.name+": "+str(plan.message))
            check=validate_path(plan.path,space,request.planning_constraints,request=planning_request)
            if not check.valid:
                return finish(S.PLANNING_FAILED,check.message)
        except Exception as exc:
            return finish(S.PLANNING_FAILED,f"{type(exc).__name__}: {exc}")
        try:
            generated=self.generator(plan.path,space,request.planning_constraints,self.trajectory_config,
                start_time=states[0].timestamp,request=planning_request)
            if not isinstance(generated,TrajectoryGenerationResult):
                raise TypeError("generator did not return TrajectoryGenerationResult")
            trajectory=generated.trajectory
            if trajectory.start_time != states[0].timestamp:
                raise ValueError("trajectory clock must start at initial timestamp")
            for state in trajectory:
                extract_reduced_state(state)
            check=validate_path(Path.from_list([s.pose for s in trajectory]),space,request.planning_constraints,request=planning_request)
            if not check.valid: raise ValueError(check.message)
            ref_clearance=min((world.segment_clearance(a.pose,b.pose) for a,b in zip(trajectory,trajectory.states[1:])),
                              default=world.clearance(trajectory[0].pose))
            refs.append(sample_reference(trajectory,states[0].timestamp))
        except Exception as exc:
            return finish(S.TRAJECTORY_FAILED,f"{type(exc).__name__}: {exc}")
        try: self.controller.reset()
        except Exception as exc: return finish(S.CONTROLLER_FAILURE,f"reset: {exc}")
        if arrived(states[0]) and c.settle_time==0:
            return finish(S.SUCCESS)
        for _ in range(round(c.max_duration/c.dt)):
            state=states[-1]; reference=refs[-1]; command=None; applied=None; current=None
            try:
                decision=None
                startup_decision=None
                if self.startup_policy is not None and not arrived(state):
                    startup_decision=self.startup_policy.update(state,trajectory[0].pose.pitch,c.dt)
                    if startup_decision.failure:return finish(S.STARTUP_FAILED,startup_decision.failure)
                    if startup_decision.target is not None:decision=startup_decision
                startup_active=startup_decision is not None and startup_decision.target is not None
                if self.capture_policy is not None and not startup_active:
                    decision=self.capture_policy.update(state,reference,request.goal,c.dt,trajectory.end_time+startup_delay,
                        arrived(state),space,required)
                    if decision.failure:return finish(S.CAPTURE_FAILED,decision.failure)
                limits_aware=isinstance(self.controller,LimitAwareController3DOF)
                if decision is not None and decision.target is not None:
                    limits_aware=True
                    command=self.controller.step_guidance(state,decision.target,c.dt,
                        ControlLimits3DOF(c.maximum_tau_x,c.maximum_tau_m))
                elif limits_aware:
                    command=self.controller.step_with_limits(state,reference,c.dt,
                        ControlLimits3DOF(c.maximum_tau_x,c.maximum_tau_m))
                else:
                    command=self.controller.step(state,reference,c.dt)
                if not isinstance(command,ControlInput): raise TypeError("controller must return ControlInput")
                values=np.asarray(command.to_numpy(),dtype=float)
                if not np.all(np.isfinite(values)): raise ValueError("nonfinite commanded control")
                if any(abs(v)>1e-12 for v in (command.tau_y,command.tau_z,command.tau_k,command.tau_n)):
                    raise ValueError("inactive control components must be zero")
                applied=ControlInput(tau_x=float(np.clip(command.tau_x,-c.maximum_tau_x,c.maximum_tau_x)),
                    tau_m=float(np.clip(command.tau_m,-c.maximum_tau_m,c.maximum_tau_m)))
                if limits_aware and applied!=command:
                    raise ValueError('limits-aware controller exceeded supplied limits')
                diagnostics=getattr(self.controller,'last_diagnostics',None)
                if not isinstance(diagnostics,ControllerDiagnostics3DOF):
                    diagnostics=None
                if diagnostics is not None and (diagnostics.output!=command or diagnostics.timestamp!=state.timestamp):
                    raise ValueError('controller diagnostics do not match this step')
            except Exception as exc:
                return finish(S.CONTROLLER_FAILURE,f"{type(exc).__name__}: {exc}",command if isinstance(command,ControlInput) else None)
            try:
                current=np.asarray(world.current_at(state.pose,state.timestamp),dtype=float)
                if current.shape!=(2,) or not np.all(np.isfinite(current)): raise ValueError("invalid world current")
            except Exception as exc: return finish(S.ENVIRONMENT_FAILURE,str(exc),command)
            try:
                nxt=self.physics.step(state,applied,current,c.dt)
                extract_reduced_state(nxt)
                if nxt.timestamp<=state.timestamp or not isclose(nxt.timestamp,state.timestamp+c.dt,rel_tol=0,abs_tol=1e-9):
                    raise ValueError("Physics timestamp does not advance by dt")
            except Exception as exc: return finish(S.NUMERICAL_FAILURE,f"{type(exc).__name__}: {exc}",command)
            try:
                clearance=world.segment_clearance(state.pose,nxt.pose)
                in_bounds=world.boundary.contains_segment(state.pose,nxt.pose,required)
                collision=clearance<=required
            except Exception as exc: return finish(S.ENVIRONMENT_FAILURE,str(exc),command)
            executed_clearance=min(executed_clearance,clearance)
            steps.append(NavigationStep(state.timestamp,command,applied,tuple(float(v) for v in current),
                hypot(state.pose.x-reference.pose.x,state.pose.z-reference.pose.z),
                angle_error(state.pose.pitch,reference.pose.pitch),
                hypot(state.twist.u-reference.twist.u,state.twist.w-reference.twist.w),
                clearance,in_bounds,collision,command!=applied,
                abs(applied.tau_x)>=(diagnostics.effective_limits.maximum_tau_x if diagnostics else c.maximum_tau_x)-1e-12
                or abs(applied.tau_m)>=(diagnostics.effective_limits.maximum_tau_m if diagnostics else c.maximum_tau_m)-1e-12,
                limits_aware,diagnostics,
                decision.phase if decision is not None and not startup_active else 'DISABLED',decision.target if decision is not None else None,
                startup_decision.phase if startup_decision is not None else 'DISABLED',
                state.timestamp-startup_delay if self.startup_policy is not None else None))
            if startup_active:startup_delay+=c.dt
            states.append(nxt)
            refs.append(replace(sample_reference(trajectory,nxt.timestamp-startup_delay),timestamp=nxt.timestamp))
            if collision: return finish(S.COLLISION,"Executed segment violates required obstacle clearance")
            if not in_bounds: return finish(S.OUT_OF_BOUNDS,"Executed footprint leaves permitted boundary")
            held=held+c.dt if arrived(nxt) and arrived(state) else 0.0
            if arrived(nxt) and held+1e-12>=c.settle_time:
                return finish(S.SUCCESS)
        return finish(S.TIMEOUT,"Fixed mission horizon exhausted; terminal hold remained active")
