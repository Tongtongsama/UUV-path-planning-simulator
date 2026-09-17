"""Executable feedforward/anti-windup design acceptance checks."""
from dataclasses import replace
from math import pi
import pytest
from core import Pose,Twist,VehicleState
from controller import CascadedPID3DOFController
from simulation.controller_config import load_baseline_controller_parameters
from validation.feedforward_pid_prototype import FeedforwardPIDPrototype


def test_zero_ff_matches_baseline_including_saturation_and_memory():
    p=load_baseline_controller_parameters()
    a=CascadedPID3DOFController(p); b=FeedforwardPIDPrototype(p)
    for pitch in [0,.2,-3.1,3.1]*30:
        s=VehicleState(Pose(z=-5,pitch=pitch),Twist(u=.2,q=.3))
        r=VehicleState(Pose(x=2,z=-3,pitch=-pitch),Twist(u=.5))
        assert a.step(s,r,.05)==b.step(s,r,.05)
        assert a.memory==b.memory


@pytest.mark.parametrize('sign',[-1,1])
def test_ff_saturation_freezes_outward_integral_but_allows_unwind(sign):
    p=replace(load_baseline_controller_parameters(),pitch_kp=0,pitch_rate_kd=0)
    c=FeedforwardPIDPrototype(p,restoring_coefficient=6,applied_limits=(20,2))
    s=VehicleState(Pose(pitch=sign*pi/2),Twist.zero())
    outward=VehicleState(Pose(pitch=sign*(pi/2+.1)),Twist.zero())
    assert c.step(s,outward,.1).tau_m==sign*2
    assert c.memory.pitch_error_integral==0
    inward=VehicleState(Pose(pitch=sign*(pi/2-.1)),Twist.zero())
    c.step(s,inward,.1)
    assert c.memory.pitch_error_integral==pytest.approx(-sign*.01)
    c.reset()
    assert c.memory.pitch_error_integral==0 and c.records==[]


def test_combined_sum_is_clipped_once_not_feedback_first():
    p=replace(load_baseline_controller_parameters(),pitch_kp=20,pitch_ki=0,pitch_rate_kd=0)
    c=FeedforwardPIDPrototype(p,restoring_coefficient=6)
    s=VehicleState(Pose(pitch=pi/2),Twist.zero())
    r=VehicleState(Pose(pitch=pi/2-.5),Twist.zero())
    # PD=-10, FF=+6 => -4. Clipping PD first would incorrectly produce -2.
    assert c.step(s,r,.1).tau_m==pytest.approx(-4)


def test_invalid_coefficient_and_caps():
    p=load_baseline_controller_parameters()
    for coefficient in (-1,float('nan'),True):
        with pytest.raises((ValueError,TypeError)):
            FeedforwardPIDPrototype(p,coefficient)
    with pytest.raises(ValueError):
        FeedforwardPIDPrototype(p,applied_limits=(20,0))
