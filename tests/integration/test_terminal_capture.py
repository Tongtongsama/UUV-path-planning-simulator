"""Capture contract: explicit guidance, bounded actions and actual-motion safety."""
from dataclasses import replace
from math import pi
import pytest
from core import Pose,Twist,VehicleState,Path
from controller import BodyGuidance3DOF,CascadedPID3DOFController,ControlLimits3DOF
from environment import WorldModel,Boundary,Obstacle
from planner import EnvironmentPlanningSpace,PlanningConstraints
from navigation import TerminalCapturePolicy,TerminalCaptureConfig,Navigator,NavigationConfig,NavigationRequest,NavigationStatus
from simulation.controller_config import load_baseline_controller_parameters
from validation.terminal_capture_acceptance import terminal_fixture
from validation.tracking_diagnostics import PrescribedPathPlanner


def args(u=0.,q=0.,pitch=0.,time=0.,goal=Pose(z=.4)):
    state=VehicleState(Pose(pitch=pitch),Twist(u=u,q=q),time)
    space=EnvironmentPlanningSpace(WorldModel(Boundary.rectangle(x_min=-3,x_max=3,z_min=-3,z_max=3)))
    return [state,VehicleState(goal,Twist.zero(),time),goal,.05,0.,False,space,.1]


def test_no_override_before_end_or_when_already_qualified():
    p=TerminalCapturePolicy();a=args();a[4]=1.
    assert p.update(*a).target is None and p.started is None
    a[4]=0.;a[5]=True
    assert p.update(*a).phase=='ORIGINAL_SETTLE' and p.started is None


def test_braking_precedes_reorientation_when_moving():
    p=TerminalCapturePolicy();d=p.update(*args(u=.5))
    assert d.phase=='BRAKE' and d.target.u<0 and d.target.pitch==0 and p.attempts==0


def test_side_target_slews_and_rear_target_uses_reverse_not_pi():
    p=TerminalCapturePolicy();d=p.update(*args())
    assert d.phase=='ALIGN' and abs(d.target.pitch)<=.15*.05
    p.reset();d=p.update(*args(goal=Pose(x=-.4)))
    assert d.phase=='MOVE' and d.target.u<0 and d.target.pitch==0


def test_no_unsafe_segment_returns_failure():
    class Blocked:
        def is_segment_valid(self,*a):return False
    p=TerminalCapturePolicy();a=args();a[6]=Blocked()
    assert 'no admissible' in p.update(*a).failure


def test_time_budget_stops_before_nonintegral_budget_overrun():
    p=TerminalCapturePolicy(TerminalCaptureConfig(budget=.07))
    assert p.update(*args(u=.5)).failure is None
    assert 'time budget' in p.update(*args(u=.5,time=.05)).failure


def test_attempt_budget_and_reset():
    p=TerminalCapturePolicy(TerminalCaptureConfig(max_attempts=1,move_duration=.05))
    assert p.update(*args(goal=Pose(x=.4))).phase=='MOVE'
    assert p.update(*args(goal=Pose(x=.4),time=.05)).phase=='BRAKE'
    assert 'attempt budget' in p.update(*args(goal=Pose(x=.4),time=.1)).failure
    p.reset();assert p.started is None and p.attempts==0


@pytest.mark.parametrize('pitch,q',[(71*pi/180,0),(0,.36)])
def test_actual_attitude_guards(pitch,q):
    assert 'guard' in TerminalCapturePolicy().update(*args(pitch=pitch,q=q)).failure


def test_guidance_has_no_hidden_outer_loop_and_respects_caps():
    c=CascadedPID3DOFController(load_baseline_controller_parameters())
    s=VehicleState(Pose(x=99,z=-20),Twist.zero())
    target=BodyGuidance3DOF(.08,.2,.03)
    command=c.step_guidance(s,target,.05,ControlLimits3DOF(.1,.2))
    d=c.last_diagnostics
    assert d.guidance_mode=='direct_body_guidance'
    assert d.surge_target==.08 and d.pitch_target==.2 and d.depth_correction==0
    assert command.tau_x==.1 and command.tau_m==.2


@pytest.mark.parametrize('kwargs',[{'budget':0},{'max_attempts':True},{'maximum_pitch':pi/2}])
def test_invalid_config(kwargs):
    with pytest.raises((TypeError,ValueError)):TerminalCaptureConfig(**kwargs)


def test_capture_checks_actual_segment_even_when_selected_correction_is_safe():
    class JumpPhysics:
        def step(self,s,control,current,dt):
            return replace(s,pose=replace(s.pose,x=.5) if s.timestamp>=.05 else s.pose,timestamp=s.timestamp+dt)
    start=Pose(z=-5.4);goal=Pose(z=-5)
    world=WorldModel(Boundary.rectangle(x_min=-2,x_max=2,z_min=-7,z_max=-3),
        [Obstacle.rectangle('wall',x_min=.1,x_max=.2,z_min=-6,z_max=-4)])
    path=Path.from_list([start,goal]);request=NavigationRequest(VehicleState(start,Twist.zero()),goal,world,PlanningConstraints(.02,goal_tolerance=.25))
    c=CascadedPID3DOFController(load_baseline_controller_parameters())
    nav=Navigator(PrescribedPathPlanner(path),c,JumpPhysics(),NavigationConfig(max_duration=1),
        generator=terminal_fixture,capture_policy=TerminalCapturePolicy())
    r=nav.run(request)
    assert r.status is NavigationStatus.COLLISION
    assert r.control_history[-1].effective_guidance is not None
    assert world.clearance(r.final_state.pose)>.02  # endpoints alone would miss crossing


def test_capture_failure_is_structured_navigation_outcome():
    class Stay:
        def step(self,s,control,current,dt):return replace(s,timestamp=s.timestamp+dt)
    start=Pose(z=-5.4);goal=Pose(z=-5);path=Path.from_list([start,goal])
    world=WorldModel(Boundary.rectangle(x_min=-2,x_max=2,z_min=-7,z_max=-3))
    req=NavigationRequest(VehicleState(start,Twist.zero()),goal,world,PlanningConstraints(.1,goal_tolerance=.25))
    nav=Navigator(PrescribedPathPlanner(path),CascadedPID3DOFController(load_baseline_controller_parameters()),Stay(),
        NavigationConfig(max_duration=1),generator=terminal_fixture,capture_policy=TerminalCapturePolicy(TerminalCaptureConfig(budget=.1)))
    r=nav.run(req)
    assert r.status is NavigationStatus.CAPTURE_FAILED and 'budget' in r.message
    assert r.metrics['capture_duration']<=.1+1e-12


def test_real_physics_goal_qualification_normalizes_numpy_boolean():
    from validation.terminal_capture_acceptance import fixtures,run
    inputs,isolated=next((i,s) for i,s in fixtures() if i[0]=='already_arrived')
    report,_=run(inputs,True,isolated)
    assert report['status']=='SUCCESS'
    assert report['metrics']['capture_attempts']==0
