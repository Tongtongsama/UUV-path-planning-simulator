"""Experimental wrappers must preserve baseline and expose effective references."""
from math import pi, sin
import pytest
from core import Pose,Twist,VehicleState
from controller import CascadedPID3DOFController
from simulation.controller_config import load_baseline_controller_parameters
from validation.terminal_restoring_experiments import ExperimentalController


def test_disabled_wrapper_equals_baseline():
    p=load_baseline_controller_parameters()
    base=CascadedPID3DOFController(p)
    wrapper=ExperimentalController(p)
    state=VehicleState(Pose(z=-5,pitch=.2),Twist(u=.3))
    reference=VehicleState(Pose(x=2,z=-4,pitch=.4),Twist(u=.5))
    for _ in range(20):
        assert wrapper.step(state,reference,.05)==base.step(state,reference,.05)
    assert wrapper.pid.memory==base.memory


def test_restoring_uses_actual_pitch_and_final_limit():
    wrapper=ExperimentalController(load_baseline_controller_parameters(),restoring_coefficient=6)
    state=VehicleState(Pose(z=-5,pitch=pi/2),Twist.zero())
    ref=VehicleState(Pose(z=-5,pitch=pi),Twist.zero())
    command=wrapper.step(state,ref,.05)
    record=wrapper.records[-1]
    assert record['feedforward_tau_m']==pytest.approx(6*sin(state.pose.pitch))
    assert record['base_command']['tau_m']==8
    assert record['pre_final_clip_tau_m']==14
    assert command.tau_m==8 and record['extra_clipped']


def test_terminal_activation_reference_and_hysteresis():
    p=load_baseline_controller_parameters()
    wrapper=ExperimentalController(p,terminal_time=10,goal=Pose(x=1,z=-5))
    ref=VehicleState(Pose(x=1,z=-5),Twist.zero(),9.9)
    state=VehicleState(Pose(z=-5),Twist.zero(),9.9)
    wrapper.step(state,ref,.05)
    assert wrapper.records[-1]['phase']=='TIMED_REFERENCE'
    for x,phase in [(0,'TERMINAL_APPROACH'),(.9,'TERMINAL_BRAKE'),(.8,'TERMINAL_BRAKE'),(.7,'TERMINAL_APPROACH')]:
        state=VehicleState(Pose(x=x,z=-5),Twist.zero(),10)
        wrapper.step(state,ref,.05)
        record=wrapper.records[-1]
        assert record['phase']==phase
        assert record['effective_reference']['pose']['x']==state.pose.x
        assert record['effective_reference']['twist']['u']>=0
    wrapper.reset()
    assert not wrapper.captured and wrapper.records==[]


def test_terminal_does_not_request_forward_thrust_when_facing_away():
    wrapper=ExperimentalController(load_baseline_controller_parameters(),terminal_time=0,goal=Pose(x=1))
    state=VehicleState(Pose(pitch=pi),Twist.zero())
    wrapper.step(state,VehicleState(Pose(x=1),Twist.zero()),.05)
    assert wrapper.records[-1]['effective_reference']['twist']['u']==0


def test_capture_radius_validation():
    with pytest.raises(ValueError):
        ExperimentalController(load_baseline_controller_parameters(),capture_radius=.3,release_radius=.25)
