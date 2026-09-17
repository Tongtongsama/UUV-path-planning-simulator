"""Formal optional feedforward, changing limits, and observable anti-windup."""
from dataclasses import replace,FrozenInstanceError
from math import pi
import pytest
from core import Pose,Twist,VehicleState
from controller import CascadedPID3DOFController,ControlLimits3DOF,LimitAwareController3DOF
from simulation.controller_config import load_baseline_controller_parameters,DEFAULT_CONTROLLER_CONFIG
from validation.feedforward_pid_prototype import FeedforwardPIDPrototype


def test_legacy_yaml_and_zero_coefficient_match_independent_old_equations():
    p=load_baseline_controller_parameters()
    assert 'restoring_pitch_coefficient' not in DEFAULT_CONTROLLER_CONFIG.read_text()
    assert p.restoring_pitch_coefficient==0
    a=CascadedPID3DOFController(p)
    old=FeedforwardPIDPrototype(p,0)
    for pitch in [0,.3,-3.1,3.1]*40:
        state=VehicleState(Pose(z=-5,pitch=pitch),Twist(u=.3,q=.2))
        ref=VehicleState(Pose(x=3,z=-3,pitch=-pitch),Twist(u=.5))
        # Call the prototype's independent baseline equations, not capability dispatch.
        assert a.step(state,ref,.05)==old.step(state,ref,.05)
        assert a.memory==old.memory


@pytest.mark.parametrize('sign',[-1,1])
def test_saturation_unwinds_and_recovers_when_effective_limits_change(sign):
    p=replace(load_baseline_controller_parameters(),pitch_kp=0,pitch_rate_kd=0,restoring_pitch_coefficient=6)
    c=CascadedPID3DOFController(p)
    s=VehicleState(Pose(pitch=sign*pi/2),Twist.zero())
    outward=VehicleState(Pose(pitch=sign*(pi/2+.2)),Twist.zero())
    inward=VehicleState(Pose(pitch=sign*(pi/2-.2)),Twist.zero())
    low=ControlLimits3DOF(20,2); high=ControlLimits3DOF(20,8)
    c.step_with_limits(s,outward,.1,high)
    before=c.memory.pitch_error_integral
    for _ in range(10):
        assert c.step_with_limits(s,outward,.1,low).tau_m==sign*2
        assert c.memory.pitch_error_integral==before
    c.step_with_limits(s,inward,.1,low)
    assert abs(c.memory.pitch_error_integral)<abs(before)+1e-12
    c.step_with_limits(s,outward,.1,high)
    assert c.memory.pitch_error_integral==pytest.approx(before)
    assert abs(c.last_diagnostics.output.tau_m)<8


def test_feedforward_uses_actual_pitch_and_no_double_compensation_at_equilibrium():
    p=replace(load_baseline_controller_parameters(),restoring_pitch_coefficient=6)
    c=CascadedPID3DOFController(p)
    s=VehicleState(Pose(pitch=pi/6),Twist.zero())
    for _ in range(100):
        assert c.step(s,s,.05).tau_m==pytest.approx(3)
        assert c.memory.pitch_error_integral==0
    ref=replace(s,pose=Pose(pitch=pi/3))
    c.step(s,ref,.05)
    assert c.last_diagnostics.restoring_feedforward==pytest.approx(3)


def test_compose_before_clipping_and_immutable_consistent_diagnostics():
    p=replace(load_baseline_controller_parameters(),pitch_kp=20,pitch_ki=0,pitch_rate_kd=0,restoring_pitch_coefficient=6)
    c=CascadedPID3DOFController(p)
    assert isinstance(c,LimitAwareController3DOF)
    state=VehicleState(Pose(pitch=pi/2),Twist.zero(),12)
    ref=replace(state,pose=Pose(pitch=pi/2-.5))
    out=c.step(state,ref,.1)
    d=c.last_diagnostics
    assert out.tau_m==pytest.approx(-4)
    assert d.output==out and d.requested.tau_m==pytest.approx(-4) and d.timestamp==12
    assert d.pitch_pd+d.restoring_feedforward+p.pitch_ki*d.integral_after.pitch_error_integral==d.requested.tau_m
    with pytest.raises(FrozenInstanceError): d.pitch_pd=0
    c.reset()
    assert c.last_diagnostics is None


@pytest.mark.parametrize('bad',[-1,float('nan'),float('inf'),True,'6'])
def test_invalid_coefficient(bad):
    with pytest.raises((TypeError,ValueError)):
        replace(load_baseline_controller_parameters(),restoring_pitch_coefficient=bad)


@pytest.mark.parametrize('bad',[0,-1,float('nan'),True])
def test_invalid_effective_limit(bad):
    with pytest.raises((TypeError,ValueError)):ControlLimits3DOF(20,bad)


def test_failed_attempt_clears_diagnostics_without_mutating_integral():
    c=CascadedPID3DOFController(load_baseline_controller_parameters())
    s=VehicleState(Pose(),Twist.zero());c.step(s,s,.1);before=c.memory
    with pytest.raises(TypeError):c.step_with_limits(s,s,.1,None)
    assert c.last_diagnostics is None and c.memory==before
