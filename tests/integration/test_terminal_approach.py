"""Approach isolation, bounded search, actual gates and unchanged PID limits."""
from copy import deepcopy
from dataclasses import asdict, replace
import pytest
from core import Path, Pose, Twist, VehicleState
from controller import CascadedPID3DOFController, ControlLimits3DOF
from environment import WorldModel, Boundary
from guidance import LOSConfig, ProjectionConfig
from navigation import LOSNavigationAdapter, TerminalCaptureConfig
from navigation.terminal_approach import TerminalApproachConfig, TerminalApproachPolicy, ApproachCursor
from physics import Python3DOFBackend
from planner import EnvironmentPlanningSpace
from simulation.controller_config import load_baseline_controller_parameters
from simulation.vehicle_config import load_synthetic_parameters


def fixture():
    state=VehicleState(Pose(x=.8),Twist.zero())
    adapter=LOSNavigationAdapter(ProjectionConfig(.2,2.,.05,3),LOSConfig(.5,.2,.25))
    progress=adapter.initialize(Path.from_list([Pose(),Pose(x=1)]),state)
    progress=replace(progress,committed_s=.8)
    world=WorldModel(Boundary.rectangle(x_min=-5,x_max=5,z_min=-5,z_max=5))
    controller=CascadedPID3DOFController(load_baseline_controller_parameters())
    args=[state,progress,None,adapter,controller,Python3DOFBackend(load_synthetic_parameters()),
          world,EnvironmentPlanningSpace(world),.1,Pose(x=1),TerminalCaptureConfig(),ControlLimits3DOF(20,8),.05,120.]
    return args


@pytest.mark.parametrize('kwargs',[{'horizon':0},{'hold_steps':True},{'hold_steps':1.5},
                                  {'speeds':[]},{'prediction_margin':float('nan')}])
def test_configuration_rejects_invalid(kwargs):
    with pytest.raises(ValueError):TerminalApproachConfig(**kwargs)


def test_prediction_does_not_mutate_real_integrator_or_progress_or_physics():
    a=fixture();p=TerminalApproachPolicy()
    before=deepcopy(a[4].__dict__);progress=asdict(a[1])
    target,ready,failure=p.update(*a)
    assert p.active and target is not None and not ready and failure is None
    assert a[4].__dict__==before and asdict(a[1])==progress
    assert p.prediction_steps==2
    # Actual samples, not prediction alone, finish the hold counter.
    a[0]=replace(a[0],timestamp=.05)
    assert not p.update(*a)[1]
    a[0]=replace(a[0],timestamp=.1)
    assert p.update(*a)[1] and p.handoff is not None


def test_real_prediction_deviation_fails_without_handoff():
    a=fixture();p=TerminalApproachPolicy();p.update(*a)
    a[0]=replace(a[0],pose=Pose(x=.81),timestamp=.05)
    assert p.update(*a)==(None,False,'APPROACH_PREDICTION_DEVIATION')


def test_swept_safety_rejects_candidate_and_budget_is_unknown():
    class Blocked:
        def is_segment_valid(self,*args):return False
    a=fixture();a[7]=Blocked()
    p=TerminalApproachPolicy(TerminalApproachConfig(max_evaluations=1))
    assert p.update(*a)==(None,False,None)
    assert not p.active and all(r['outcome']=='PREDICTED_UNSAFE' for r in p.events[0]['candidates'])
    assert p.update(*a)[2]=='APPROACH_UNKNOWN_SEARCH_EXHAUSTED'


@pytest.mark.parametrize('twist,pitch',[(Twist(u=.04),0),(Twist(w=.04),0),(Twist(q=.04),0),(Twist.zero(),.2)])
def test_actual_speed_rate_pitch_gate_cannot_be_replaced_by_prediction(twist,pitch):
    a=fixture();p=TerminalApproachPolicy();cursor=ApproachCursor(a[1],None)
    for _ in range(4):
        _,ready=p.kernel(replace(a[0],pose=Pose(x=.8,pitch=pitch),twist=twist),cursor,a[3],a[9],a[10],.1,.35)
        assert not ready


def test_outside_region_does_not_brake_or_predict():
    a=fixture();p=TerminalApproachPolicy(TerminalApproachConfig(evaluation_arc=.1))
    assert p.update(*a)==(None,False,None)
    assert p.evaluations==0 and not p.active


def test_horizon_unknown_does_not_claim_infeasibility():
    a=fixture();p=TerminalApproachPolicy(TerminalApproachConfig(horizon=.05))
    assert p.update(*a)==(None,False,None)
    assert all(r['outcome']=='UNKNOWN_HORIZON' for r in p.events[0]['candidates'])


def test_hold_resets_after_actual_disqualification():
    a=fixture();p=TerminalApproachPolicy();cursor=ApproachCursor(a[1],None)
    p.kernel(a[0],cursor,a[3],a[9],a[10],.1,.35)
    assert cursor.held==1
    p.kernel(replace(a[0],twist=Twist(q=.1)),cursor,a[3],a[9],a[10],.1,.35)
    assert cursor.held==0


def test_nonzero_ground_current_is_out_of_domain_not_water_relative_braking():
    from environment import ConstantCurrent
    a=fixture();a[6]=WorldModel(a[6].boundary,current=ConstantCurrent([0,.05]))
    assert TerminalApproachPolicy().update(*a)[2]=='APPROACH_OUT_OF_DOMAIN_CURRENT'


def test_rejected_predictions_respect_interval_and_leave_real_memory_unchanged():
    a=fixture();p=TerminalApproachPolicy(TerminalApproachConfig(horizon=.05))
    memory=a[4].memory
    p.update(*a);count=p.prediction_steps
    a[0]=replace(a[0],timestamp=.1)
    p.update(*a)
    assert p.evaluations==1 and p.prediction_steps==count and a[4].memory==memory


def test_real_navigator_freezes_original_progress_during_approach():
    from navigation import Navigator, NavigationRequest, NavigationConfig, TerminalCapturePolicy
    from planner import PlanningConstraints
    from trajectory import TrajectoryConfig
    from validation.tracking_diagnostics import PrescribedPathPlanner
    a=fixture();path=Path.from_list([Pose(),Pose(x=1)])
    initial=VehicleState(Pose(),Twist.zero())
    request=NavigationRequest(initial,Pose(x=1),a[6],PlanningConstraints(.1,goal_tolerance=.25))
    policy=TerminalApproachPolicy()
    nav=Navigator(PrescribedPathPlanner(path),a[4],a[5],NavigationConfig(max_duration=30),
        TrajectoryConfig(nominal_speed=.2,shortcut=False),capture_policy=TerminalCapturePolicy(),
        guidance_adapter=a[3],approach_policy=policy)
    result=nav.run(request)
    steps=[s for s in result.control_history if s.guidance_phase=='TERMINAL_APPROACH']
    assert steps and len({s.path_progress_s for s in steps})==1
    assert all(s.guidance_command is None and s.effective_guidance is not None for s in steps)
    assert all(abs(s.applied.tau_x)<=20 and abs(s.applied.tau_m)<=8 for s in steps)
    assert policy.handoff is not None
