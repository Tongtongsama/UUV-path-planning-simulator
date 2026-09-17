"""Navigation terminal policies, true dynamics signs and pipeline integration."""
from dataclasses import replace
from math import atan2
import pytest
from core import ControlInput, Pose, Twist, VehicleState
from environment import Boundary, WorldModel, Obstacle, ConstantCurrent
from planner import AStarPlanner, PlanningConstraints, PlanningResult, PlanningStatus
from navigation import NavigationConfig, NavigationRequest, NavigationStatus as S, Navigator
from controller import CascadedPID3DOFController
from physics import Python3DOFBackend
from simulation.controller_config import load_baseline_controller_parameters
from simulation.vehicle_config import load_synthetic_parameters


def mission(goal=Pose(x=9,z=-5), initial=None, obstacles=(), current=None):
    world=WorldModel(Boundary.rectangle(x_min=0,x_max=10,z_min=-10,z_max=0),obstacles,current)
    return NavigationRequest(initial or VehicleState(Pose(x=1,z=-5),Twist.zero()),goal,world,PlanningConstraints(.2,goal_tolerance=.25))


def real(config=NavigationConfig()):
    return Navigator(AStarPlanner(),CascadedPID3DOFController(load_baseline_controller_parameters()),
                     Python3DOFBackend(load_synthetic_parameters()),config)


class FixedController:
    def __init__(self,command=ControlInput.zero()): self.command=command; self.resets=0
    def reset(self): self.resets+=1
    def step(self,state,reference,dt): return self.command


class StayPhysics:
    def step(self,state,control,current,dt): return replace(state,timestamp=state.timestamp+dt)


def test_timeout_clock_and_saturation_record():
    controller=FixedController(ControlInput(tau_x=100,tau_m=-100))
    n=Navigator(AStarPlanner(),controller,StayPhysics(),NavigationConfig(dt=.1,max_duration=.3))
    request=mission(initial=VehicleState(Pose(x=1,z=-5),Twist.zero(),10))
    r=n.run(request)
    assert r.status is S.TIMEOUT and len(r.control_history)==3
    assert len(r.state_history)==len(r.reference_history)==4
    assert r.final_state.timestamp==pytest.approx(10.3)
    assert controller.resets==1
    assert all(s.commanded.tau_x==100 and s.applied.tau_x==20 and s.applied.tau_m==-8 for s in r.control_history)
    assert r.metrics["navigation_clip_count"]==3
    assert r.metrics["control_effort_proxy"]==pytest.approx((20**2+8**2)*.3)
    assert [s.timestamp for s in r.state_history]==[s.timestamp for s in r.reference_history]


def test_goal_requires_speed_rate_and_hold():
    req=mission(goal=Pose(x=1,z=-5))
    cfg=NavigationConfig(dt=.1,max_duration=1,settle_time=.3)
    n=Navigator(AStarPlanner(),FixedController(),StayPhysics(),cfg)
    r=n.run(req)
    assert r.status is S.SUCCESS and len(r.control_history)==3
    for twist in (Twist(u=1),Twist(q=1)):
        r=n.run(replace(req,initial_state=replace(req.initial_state,twist=twist)))
        assert r.status is S.TIMEOUT


def test_planning_and_trajectory_failures():
    n=real()
    r=n.run(mission(goal=Pose(x=11,z=-5)))
    assert r.status is S.PLANNING_FAILED and r.trajectory is None and not r.control_history
    def fail(*args,**kwargs): raise ValueError("deliberate generation failure")
    n.generator=fail
    r=n.run(mission())
    assert r.status is S.TRAJECTORY_FAILED and not r.control_history


@pytest.mark.parametrize("command", [None,ControlInput(tau_x=float('nan')),ControlInput(tau_z=1)])
def test_controller_failures(command):
    r=Navigator(AStarPlanner(),FixedController(command),StayPhysics()).run(mission())
    assert r.status is S.CONTROLLER_FAILURE and len(r.state_history)==1


@pytest.mark.parametrize("mode", ["nan","clock","inactive"])
def test_numerical_failures_preserve_last_valid_state(mode):
    class BrokenPhysics:
        def step(self,state,control,current,dt):
            return replace(state,pose=Pose(x=float('nan')) if mode=='nan' else Pose(x=1,y=1 if mode=='inactive' else 0,z=-5),
                           timestamp=state.timestamp if mode=='clock' else state.timestamp+dt)
    r=Navigator(AStarPlanner(),FixedController(),BrokenPhysics()).run(mission())
    assert r.status is S.NUMERICAL_FAILURE and len(r.state_history)==1 and not r.control_history


def test_continuous_tunnelling_and_out_of_bounds():
    class Jump:
        def __init__(self,x): self.x=x
        def step(self,state,control,current,dt): return VehicleState(Pose(x=self.x,z=-5),Twist.zero(),state.timestamp+dt)
    wall=Obstacle.rectangle("wall",x_min=4.99,x_max=5.01,z_min=-6,z_max=-4)
    req=mission(obstacles=[wall])
    r=Navigator(AStarPlanner(),FixedController(),Jump(9)).run(req)
    assert r.status is S.COLLISION and r.metrics["minimum_executed_clearance"]==0
    assert req.environment.clearance(r.final_state.pose)>.2
    r=Navigator(AStarPlanner(),FixedController(),Jump(11)).run(mission())
    assert r.status is S.OUT_OF_BOUNDS


@pytest.mark.parametrize("dz", [-1,1])
def test_real_physics_vertical_sign(dz):
    physics=Python3DOFBackend(load_synthetic_parameters())
    initial=VehicleState(Pose(x=1,z=-5,pitch=atan2(dz,2)),Twist(u=.5))
    end=physics.step(initial,ControlInput.zero(),[0,0],.02)
    assert end.pose.x>initial.pose.x
    assert (end.pose.z-initial.pose.z)*dz>0


@pytest.mark.parametrize("current", [(0,0),(.1,0),(-.1,0)])
def test_real_horizontal_and_narrow_passage(current):
    obstacles=[Obstacle.rectangle("lower",x_min=4,x_max=6,z_min=-10,z_max=-5.4),
               Obstacle.rectangle("upper",x_min=4,x_max=6,z_min=-4.6,z_max=0)]
    req=mission(obstacles=obstacles,current=ConstantCurrent(current))
    navigator=real()
    a=navigator.run(req); b=navigator.run(req)
    assert a.status is S.SUCCESS
    assert not a.metrics["collision"] and not a.metrics["out_of_bounds"]
    assert a.metrics["terminal_speed"]<=.1
    assert a.metrics["terminal_position_error"]<=.25
    assert a.metrics["minimum_executed_safety_margin"]>0
    assert a.state_history==b.state_history and a.control_history==b.control_history
    assert a.metrics==b.metrics


@pytest.mark.parametrize("kwargs", [dict(dt=0),dict(max_duration=.12),dict(goal_speed_tolerance=-1),dict(dt=True)])
def test_navigation_config_rejects_invalid_data(kwargs):
    with pytest.raises((TypeError,ValueError)):
        NavigationConfig(**kwargs)


def test_acceptance_checker_rejects_false_success():
    from validation.navigation_acceptance import check_execution
    cfg=NavigationConfig(dt=.1,max_duration=.2)
    req=mission()
    timed_out=Navigator(AStarPlanner(),FixedController(),StayPhysics(),cfg).run(req)
    assert all(check_execution(timed_out,req,cfg).values())
    changed=replace(timed_out,status=S.SUCCESS)
    assert not check_execution(changed,req,cfg)["success_position_speed_rate"]


def test_environment_failure_and_reset_exception():
    class BadCurrent(ConstantCurrent):
        def velocity_at(self,position,time): return [float('nan'),0]
    req=mission(current=BadCurrent([0,0]))
    r=Navigator(AStarPlanner(),FixedController(),StayPhysics()).run(req)
    assert r.status is S.ENVIRONMENT_FAILURE
    class BadReset(FixedController):
        def reset(self): raise RuntimeError("reset failed")
    r=Navigator(AStarPlanner(),BadReset(),StayPhysics()).run(mission())
    assert r.status is S.CONTROLLER_FAILURE


def test_current_plot_names_preserve_decimal_identifiers(tmp_path):
    pytest.importorskip("matplotlib")
    from visualization.navigation import plot_navigation
    cfg=NavigationConfig(dt=.1,max_duration=.2)
    req=mission()
    result=Navigator(AStarPlanner(),FixedController(),StayPhysics(),cfg).run(req)
    for name in ("current_+0.00_+0.00", "current_+0.00_+0.05"):
        scene={"name":name,"obstacles":[],"boundary":{
            "x_min":0,"x_max":10,"z_min":-10,"z_max":0}}
        plot_navigation(result,req,scene,cfg,tmp_path/name)
        for extension in ("png","svg"):
            assert (tmp_path/(name+"."+extension)).is_file()
    assert len(list(tmp_path.iterdir()))==4
