"""Startup clock, guards and experimental timing contracts."""
from dataclasses import replace
import pytest
from core import Pose, Twist, VehicleState, Path
from environment import Boundary, WorldModel
from planner import PlanningConstraints, EnvironmentPlanningSpace
from navigation import ControlledStartupPolicy, StartupConfig
from trajectory import TrajectoryConfig, sample_reference
from validation.controlled_startup_experiments import corner_timing, execute


def test_startup_brakes_before_slew_and_is_rate_limited():
    policy=ControlledStartupPolicy()
    moving=VehicleState(Pose(),Twist(u=.5))
    d=policy.update(moving,-.5,.05)
    assert d.phase=='BRAKE' and d.target.pitch==0 and d.target.u<0
    rest=replace(moving,twist=Twist.zero(),timestamp=.05)
    d=policy.update(rest,-.5,.05)
    assert d.phase=='ALIGN' and d.target.pitch==pytest.approx(-.0075)
    assert d.target.q==pytest.approx(-.15)


def test_startup_requires_continuous_settle_and_reset():
    policy=ControlledStartupPolicy()
    # A non-trivial heading change must qualify continuously after alignment.
    for i in range(11):
        pitch=-.2 if i==0 else -.5
        assert policy.update(VehicleState(Pose(pitch=pitch),Twist.zero(),i*.05),-.5,.05).target is not None
    assert policy.update(VehicleState(Pose(pitch=-.5),Twist.zero(),.55),-.5,.05).target is None
    assert policy.released
    policy.reset()
    assert not policy.released and policy.origin is None


@pytest.mark.parametrize('kind',['drift','budget','heading'])
def test_startup_fails_explicitly(kind):
    p=ControlledStartupPolicy(StartupConfig(budget=.1))
    p.update(VehicleState(Pose(),Twist.zero()),-.5,.05)
    s=VehicleState(Pose(x=.2 if kind=='drift' else 0),Twist.zero(),.1 if kind=='budget' else .05)
    assert p.update(s,2 if kind=='heading' else -.5,.05).failure


@pytest.mark.parametrize('surge',[0.,.5,.8,1.])
def test_already_aligned_forward_motion_releases_without_artificial_stop(surge):
    p=ControlledStartupPolicy()
    decision=p.update(VehicleState(Pose(pitch=-.2),Twist(u=surge),0),-.2,.05)
    assert decision.phase=='RELEASED_ALIGNED' and decision.target is None and p.released


def test_aligned_release_is_bounded_and_rejects_unbounded_speed_by_braking():
    p=ControlledStartupPolicy()
    decision=p.update(VehicleState(Pose(),Twist(u=1.01)),0,.05)
    assert decision.phase=='BRAKE' and decision.target.u<0 and not p.released


@pytest.mark.parametrize('degrees,accepted',[(66.8014,True),(70.,True),(81.8699,False)])
def test_near_vertical_heading_envelope_is_explicit(degrees,accepted):
    from math import radians
    p=ControlledStartupPolicy()
    decision=p.update(VehicleState(Pose(),Twist.zero()),-radians(degrees),.05)
    assert (decision.failure is None) is accepted


def test_corner_timing_changes_position_clock_and_u_together():
    world=WorldModel(Boundary.rectangle(x_min=-2,x_max=10,z_min=-5,z_max=5))
    path=Path.from_list([Pose(),Pose(x=3),Pose(x=6,z=2)])
    cfg=TrajectoryConfig(nominal_speed=.5,shortcut=False)
    result=corner_timing(path,EnvironmentPlanningSpace(world),PlanningConstraints(.1),cfg)
    speeds=set()
    for a,b in zip(result.trajectory,result.trajectory.states[1:]):
        length=((b.pose.x-a.pose.x)**2+(b.pose.z-a.pose.z)**2)**.5
        assert length/(b.timestamp-a.timestamp)==pytest.approx(a.twist.u)
        speeds.add(a.twist.u)
        mid=sample_reference(result.trajectory,(a.timestamp+b.timestamp)/2)
        assert mid.pose.x==pytest.approx((a.pose.x+b.pose.x)/2)
    assert speeds=={.2,.5}
    assert result.trajectory[-1].twist==Twist.zero()
    assert result.shortcut_path==path


@pytest.mark.parametrize('goal_z',[0.,1.,-1.])
@pytest.mark.parametrize('speed',[.2,.5])
def test_real_startup_pauses_clock_without_teleport_or_resetting_time(goal_z,speed):
    world=WorldModel(Boundary.rectangle(x_min=-3,x_max=5,z_min=-5,z_max=3))
    initial=VehicleState(Pose(),Twist.zero())
    inputs=('slope',Path.from_list([Pose(),Pose(x=2,z=goal_z)]),world,initial,PlanningConstraints(.2,.1),speed)
    r,h=execute(inputs,True,False)
    assert all(r['checks'].values())
    assert r['status']=='SUCCESS'
    startup=[s for s in h['controls'] if s['startup_phase'] in ('ALIGN','BRAKE')]
    if goal_z==0:
        assert not startup and h['controls'][0]['startup_phase']=='RELEASED_ALIGNED'
    else:
        assert startup and all(abs(s['reference_time'])<1e-10 for s in startup)
    assert h['states'][0]['pose']['pitch']==0
    assert (r['metrics']['startup_duration']==0) is (goal_z==0)
    assert all(b['timestamp']-a['timestamp']==pytest.approx(.05) for a,b in zip(h['states'],h['states'][1:]))
    if goal_z!=0:
        release=next(i for i,s in enumerate(h['controls']) if s['startup_phase']=='RELEASED')
        assert h['controls'][release]['controller_diagnostics']['integral_before']==h['controls'][release-1]['controller_diagnostics']['integral_after']


def test_startup_actual_segment_collision_is_not_bypassed():
    from environment import Obstacle
    from navigation import Navigator, NavigationRequest, NavigationStatus
    from controller import CascadedPID3DOFController
    from simulation.controller_config import load_baseline_controller_parameters
    from validation.tracking_diagnostics import PrescribedPathPlanner
    class JumpPhysics:
        def step(self,state,control,current,dt):
            return replace(state,pose=Pose(x=1),timestamp=state.timestamp+dt)
    world=WorldModel(Boundary.rectangle(x_min=-2,x_max=3,z_min=-3,z_max=3),
        [Obstacle.rectangle(obstacle_id='wall',x_min=.45,x_max=.55,z_min=-.4,z_max=.4)])
    path=Path.from_list([Pose(),Pose(x=1,z=-1.5)])
    nav=Navigator(PrescribedPathPlanner(path),CascadedPID3DOFController(load_baseline_controller_parameters()),
        JumpPhysics(),startup_policy=ControlledStartupPolicy())
    result=nav.run(NavigationRequest(VehicleState(Pose(),Twist.zero()),path[-1],world,PlanningConstraints(.05)))
    assert result.status is NavigationStatus.COLLISION
    assert result.control_history[0].startup_phase=='ALIGN'
