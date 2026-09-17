"""Real Navigation negotiates effective limits before updating controller memory."""
from core import Pose,Twist,VehicleState,ControlInput
from controller import CascadedPID3DOFController
from simulation.controller_config import load_baseline_controller_parameters
from simulation.vehicle_config import load_synthetic_parameters
from navigation import Navigator,NavigationConfig,NavigationRequest,NavigationStatus
from planner import AStarPlanner,PlanningConstraints
from physics import Python3DOFBackend
from environment import WorldModel,Boundary


def setup(controller):
    cfg=NavigationConfig(max_duration=.1,maximum_tau_x=.1,maximum_tau_m=.1)
    req=NavigationRequest(VehicleState(Pose(x=1,z=-5),Twist.zero()),Pose(x=9,z=-5),
        WorldModel(Boundary.rectangle(x_min=0,x_max=10,z_min=-10,z_max=0)),PlanningConstraints(.2))
    return Navigator(AStarPlanner(),controller,Python3DOFBackend(load_synthetic_parameters()),cfg).run(req)


def test_formal_controller_uses_navigation_limits_and_logs_all_three_signals():
    c=CascadedPID3DOFController(load_baseline_controller_parameters())
    r=setup(c)
    assert r.status is NavigationStatus.TIMEOUT
    assert c.memory.surge_error_integral==0
    for step in r.control_history:
        assert step.limits_aware and not step.clipped
        d=step.controller_diagnostics
        assert d.timestamp==step.time and d.output==step.commanded==step.applied
        assert d.effective_limits.maximum_tau_x==.1
        assert d.requested.tau_x>.1 and step.applied.tau_x==.1


def test_old_backend_still_works_and_is_explicitly_not_limit_aware():
    class Legacy:
        def reset(self):pass
        def step(self,state,reference,dt):return ControlInput(tau_x=1)
    r=setup(Legacy())
    assert r.status is NavigationStatus.TIMEOUT
    assert all(not s.limits_aware and s.clipped and s.controller_diagnostics is None for s in r.control_history)


def test_false_limit_capability_returns_controller_failure():
    class Broken:
        def reset(self):pass
        def step(self,state,reference,dt):return ControlInput.zero()
        def step_with_limits(self,state,reference,dt,limits):return ControlInput(tau_x=10)
    assert setup(Broken()).status is NavigationStatus.CONTROLLER_FAILURE
