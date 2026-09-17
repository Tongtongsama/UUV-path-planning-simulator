"""Planning robustness margin must not silently change execution acceptance."""
import pytest
from core import Pose,Twist,VehicleState
from environment import Boundary,WorldModel
from planner import AStarPlanner,PlanningConstraints
from navigation import NavigationConfig,NavigationRequest,Navigator
from physics import Python3DOFBackend
from simulation.controller_config import load_baseline_controller_parameters
from simulation.vehicle_config import load_synthetic_parameters
from validation.feedforward_pid_prototype import FeedforwardPIDPrototype
from validation.detour_margin_experiments import run_case


def test_planning_margin_is_separate_from_actual_safety_threshold():
    report,history=run_case('test_extra',extra_margin=.2)
    assert report['planning_constraints']['safety_margin']==pytest.approx(.3)
    assert report['execution_constraints']['safety_margin']==pytest.approx(.1)
    assert report['metrics']['minimum_reference_clearance']>=.5
    assert report['reference_execution_budget']==pytest.approx(report['metrics']['minimum_reference_clearance']-.3)
    assert history and report['metrics']['navigation_clip_count']==0


def test_known_tighter_navigation_caps_are_used_before_integration():
    cfg=NavigationConfig(dt=.05,max_duration=.1,maximum_tau_x=.1,maximum_tau_m=.1)
    c=FeedforwardPIDPrototype(load_baseline_controller_parameters(),6,(cfg.maximum_tau_x,cfg.maximum_tau_m))
    req=NavigationRequest(VehicleState(Pose(x=1,z=-5),Twist.zero()),Pose(x=9,z=-5),
        WorldModel(Boundary.rectangle(x_min=0,x_max=10,z_min=-10,z_max=0)),PlanningConstraints(.2))
    r=Navigator(AStarPlanner(),c,Python3DOFBackend(load_synthetic_parameters()),cfg).run(req)
    assert len(c.records)==2 and r.metrics['navigation_clip_count']==0
    assert c.memory.surge_error_integral==0
    assert all(s.applied.tau_x==pytest.approx(.1) for s in r.control_history)
