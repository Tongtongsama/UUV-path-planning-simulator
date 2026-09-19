"""Saved-state replay semantics and analytic fixed-heading current equilibrium."""
from dataclasses import asdict
import numpy as np
import pytest

from core import Pose,Twist,VehicleState
from controller import BodyGuidance3DOF,CascadedPID3DOFController,ControlLimits3DOF
from navigation import NavigationConfig,TerminalCaptureConfig,TerminalCapturePolicy
from simulation.controller_config import load_baseline_controller_parameters
from validation.terminal_handoff_replays import speed_diagnostics,theoretical_brake_case,restore_controller,ROOT


@pytest.mark.parametrize('pitch',[-1.,0.,1.])
def test_absolute_body_speed_equals_ground_speed_not_water_relative(pitch):
    state=VehicleState(Pose(pitch=pitch),Twist(u=.2,w=-.05))
    speeds=speed_diagnostics(state,[0.,.05])
    assert speeds['body_absolute_speed']==pytest.approx(speeds['world_ground_speed'])
    state=VehicleState(Pose(),Twist(w=-.05))
    speeds=speed_diagnostics(state,[0.,.05])
    assert speeds['water_relative_speed']==pytest.approx(0.)
    assert speeds['world_ground_speed']==pytest.approx(.05)
    assert speeds['world_ground_speed']>TerminalCaptureConfig().brake_speed


def test_horizontal_vertical_current_equilibrium_is_drifting_not_rest():
    evidence=theoretical_brake_case()
    assert evidence['rest_nu_dot'][1]==pytest.approx(-(.05*6+1.5*.05**2)/28)
    np.testing.assert_allclose(evidence['drift_nu_dot'],0.,atol=1e-12)
    np.testing.assert_allclose(evidence['drift_world_rates'],[0.,.05,0.],atol=1e-12)
    np.testing.assert_allclose(theoretical_brake_case((0.,0.))['rest_nu_dot'],0.,atol=1e-12)


def test_restore_integrals_by_public_calls_and_reject_corrupt_evidence():
    parameters=load_baseline_controller_parameters(ROOT/'config/controllers/cascaded_pid_3dof_restoring_candidate.yaml')
    controller=CascadedPID3DOFController(parameters)
    config=NavigationConfig();limits=ControlLimits3DOF(config.maximum_tau_x,config.maximum_tau_m)
    states=[VehicleState(Pose(),Twist(),i*config.dt) for i in range(3)]
    target=BodyGuidance3DOF(.2,.1)
    controls=[]
    for state in states:
        command=controller.step_guidance(state,target,config.dt,limits)
        controls.append(dict(effective_guidance=asdict(target),commanded=asdict(command),
            controller_diagnostics=asdict(controller.last_diagnostics)))
    common=dict(controller=asdict(parameters),navigation=asdict(config))
    history=dict(states=[asdict(s) for s in states],controls=controls)
    restored=restore_controller(common,history,3)
    assert restored.memory==controller.memory
    assert restored.memory.surge_error_integral!=0
    history['controls'][1]['commanded']['tau_x']+=1
    with pytest.raises(ValueError,match='differs'):restore_controller(common,history,3)


def test_capture_rate_guard_is_not_relaxed_by_replay():
    from planner import EnvironmentPlanningSpace
    from environment import WorldModel,Boundary
    state=VehicleState(Pose(),Twist(u=.5,q=.536),30.55)
    space=EnvironmentPlanningSpace(WorldModel(Boundary.rectangle(x_min=-2,x_max=2,z_min=-2,z_max=2)))
    policy=TerminalCapturePolicy()
    decision=policy.update_active(state,state,Pose(x=.2),.05,False,space,.1)
    assert 'guard' in decision.failure
    assert policy.config.maximum_actual_pitch_rate==.35
