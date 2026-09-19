"""Phase B adapter contracts, using unchanged inner control and safety checks."""
from dataclasses import replace
import pytest

from core import Path, Pose, Twist, VehicleState
from controller import CascadedPID3DOFController
from environment import Boundary, WorldModel, Obstacle
from guidance import LOSConfig, ProjectionConfig
from navigation import (Navigator, NavigationConfig, NavigationRequest, NavigationStatus,
    LOSNavigationAdapter, TerminalCapturePolicy, TerminalCaptureConfig, ControlledStartupPolicy)
from physics import Python3DOFBackend
from planner import PlanningConstraints
from simulation.controller_config import load_baseline_controller_parameters
from simulation.vehicle_config import load_synthetic_parameters
from validation.tracking_diagnostics import ROOT, PrescribedPathPlanner
from trajectory import TrajectoryConfig, generate_trajectory


def adapter():
    return LOSNavigationAdapter(ProjectionConfig(.2, 2., .05, 3),LOSConfig(.5,.3,.25))


def controller():
    return CascadedPID3DOFController(load_baseline_controller_parameters(
        ROOT/'config/controllers/cascaded_pid_3dof_restoring_candidate.yaml'))


class Stay:
    def step(self,state,command,current,dt):return replace(state,timestamp=state.timestamp+dt)


def setup(goal=Pose(x=2,z=-5),initial=None,physics=None,config=None,capture=None,startup=None):
    initial=initial or VehicleState(Pose(z=-5),Twist.zero())
    path=Path((initial.pose,goal))
    request=NavigationRequest(initial,goal,WorldModel(Boundary.rectangle(
        x_min=-10,x_max=10,z_min=-10,z_max=0)),PlanningConstraints(.1,goal_tolerance=.25))
    nav=Navigator(PrescribedPathPlanner(path),controller(),physics or Stay(),
        config or NavigationConfig(max_duration=1),TrajectoryConfig(nominal_speed=.3,shortcut=False),
        capture_policy=capture or TerminalCapturePolicy(),startup_policy=startup,guidance_adapter=adapter())
    return nav,request


def test_body_conversion_and_no_outer_loop_no_secondary_clipping():
    nav,request=setup(config=NavigationConfig(max_duration=.2,maximum_tau_x=.1,maximum_tau_m=.2))
    result=nav.run(request)
    for step in result.control_history:
        command=step.guidance_command
        assert step.guidance_phase=='PATH_FOLLOWING'
        assert step.effective_guidance==nav.guidance_adapter.to_body(command)
        d=step.controller_diagnostics
        assert d.guidance_mode=='direct_body_guidance' and d.depth_correction==0
        assert d.surge_target==command.surge_speed_ref
        assert d.pitch_target==command.pitch_ref
        assert d.output==step.applied==step.commanded
        assert abs(step.applied.tau_x)<=.1 and abs(step.applied.tau_m)<=.2
    assert result.metrics['reference_duration'] is None
    assert result.metrics['position_rmse'] is None
    assert result.metrics['navigation_clip_count']==0


def test_progress_frozen_through_startup():
    nav,request=setup(goal=Pose(x=2,z=-4),startup=ControlledStartupPolicy())
    result=nav.run(request)
    assert result.control_history
    assert all(s.guidance_phase=='STARTUP' and s.path_progress_s==0
               and s.guidance_command is None for s in result.control_history)


def test_explicit_capture_ignores_timed_end_and_freezes_progress():
    class ExplicitOnly(TerminalCapturePolicy):
        def update(self,*args,**kwargs):raise AssertionError('timed entry must not be used')
    class MoveThenStay:
        def step(self,s,command,current,dt):
            return replace(s,pose=replace(s.pose,x=min(2.,s.pose.x+.2)),timestamp=s.timestamp+dt)
    policy=ExplicitOnly()
    nav,request=setup(physics=MoveThenStay(),config=NavigationConfig(max_duration=3),capture=policy)
    result=nav.run(request)
    assert result.status is NavigationStatus.SUCCESS
    following=[s for s in result.control_history if s.guidance_phase=='PATH_FOLLOWING']
    capture=[s for s in result.control_history if s.guidance_phase=='TERMINAL_CAPTURE']
    assert following[-1].guidance_command.terminal_ready and capture
    assert capture[0].time==pytest.approx(following[-1].time+nav.config.dt)
    assert capture[0].time<result.trajectory.end_time
    assert all(s.guidance_command is None and s.path_progress_s==following[-1].path_progress_s for s in capture)
    assert all(s.effective_guidance.u<=0 for s in capture)


def test_explicit_capture_and_timed_capture_share_exact_policy():
    from tests.integration.test_terminal_capture import args
    timed,explicit=TerminalCapturePolicy(),TerminalCapturePolicy()
    for i in range(5):
        a=args(u=.5,time=i*.05)
        expected=timed.update(*a)
        actual=explicit.update_active(a[0],a[1],a[2],a[3],a[5],a[6],a[7])
        assert actual==expected
        assert explicit.started==timed.started and explicit.attempts==timed.attempts


def test_singleton_bypasses_startup_and_retains_settling():
    nav,request=setup(goal=Pose(z=-5),startup=ControlledStartupPolicy())
    result=nav.run(request)
    assert result.status is NavigationStatus.SUCCESS
    assert all(s.guidance_phase=='TERMINAL_CAPTURE' and s.path_progress_s==0 for s in result.control_history)
    assert result.metrics['goal_held_seconds']>=nav.config.settle_time-1e-12
    assert nav.capture_policy.attempts==0


def test_capture_failure_is_latched_without_resume():
    initial=VehicleState(Pose(z=-5),Twist(u=.5))
    nav,request=setup(goal=Pose(z=-5),initial=initial,
        capture=TerminalCapturePolicy(TerminalCaptureConfig(budget=.1)))
    result=nav.run(request)
    assert result.status is NavigationStatus.CAPTURE_FAILED
    assert all(s.guidance_phase=='TERMINAL_CAPTURE' for s in result.control_history)
    assert result.metrics['capture_duration']<=.1+1e-12


def test_actual_swept_collision_check_still_active():
    class Jump:
        def step(self,s,*args):return replace(s,pose=replace(s.pose,z=-4),timestamp=s.timestamp+args[-1])
    nav,request=setup(physics=Jump())
    world=WorldModel(request.environment.boundary,[Obstacle.rectangle(
        'crossing',x_min=-.2,x_max=.2,z_min=-4.6,z_max=-4.4)])
    result=nav.run(replace(request,environment=world))
    assert result.status is NavigationStatus.COLLISION
    assert world.clearance(result.final_state.pose)>.1


def test_invalid_adapter_capabilities_and_tolerance():
    nav,request=setup()
    with pytest.raises(ValueError,match='capture'):
        Navigator(nav.planner,nav.controller,nav.physics,guidance_adapter=adapter())
    with pytest.raises(ValueError,match='entry distance'):
        Navigator(nav.planner,nav.controller,nav.physics,NavigationConfig(goal_position_tolerance=.5),
            capture_policy=TerminalCapturePolicy(),guidance_adapter=adapter())


@pytest.mark.parametrize('z',[ -5, -4, -6 ])
def test_real_straights_signs_reset_and_limits(z):
    nav,request=setup(goal=Pose(x=3,z=z),physics=Python3DOFBackend(load_synthetic_parameters()),
                      config=NavigationConfig(),startup=ControlledStartupPolicy())
    result=nav.run(request)
    assert result.status is NavigationStatus.SUCCESS,result.message
    again=nav.run(request)
    assert result.state_history==again.state_history and result.control_history==again.control_history
    assert result.metrics==again.metrics
    los=[s for s in result.control_history if s.guidance_command is not None]
    assert los
    if z!=-5:assert los[0].guidance_command.pitch_ref*(z+5)>0
    assert result.metrics['progress_monotonicity_violations']==0
    assert result.metrics['navigation_clip_count']==0
    assert all(s.in_bounds and not s.collision for s in result.control_history)
