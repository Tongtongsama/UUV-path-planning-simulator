"""Evidence helpers must measure geometry and the actual baseline PID."""
import numpy as np
import pytest
from core import Pose, Twist, VehicleState
from controller import CascadedPID3DOFController
from simulation.controller_config import load_baseline_controller_parameters
from validation.tracking_diagnostics import ObservedController, polyline_distance, cases, run_case


def test_polyline_distance_uses_finite_segments_and_corner():
    d=polyline_distance([[.5,.2],[1.2,.5],[2,2]],[[0,0],[1,0],[1,1]])
    np.testing.assert_allclose(d,[.2,.2,np.sqrt(2)])
    with pytest.raises(ValueError):
        polyline_distance([[0,0]],[[0,0],[0,0]])


def test_observer_does_not_change_pid_outputs_or_memory():
    p=load_baseline_controller_parameters()
    observed=ObservedController(p)
    baseline=CascadedPID3DOFController(p)
    for pitch in [0,.2,-3.1,3.1]*10:
        state=VehicleState(Pose(x=1,z=-5,pitch=pitch),Twist(u=.4,q=.1))
        reference=VehicleState(Pose(x=2,z=-4,pitch=-pitch),Twist(u=.5))
        assert observed.step(state,reference,.05)==baseline.step(state,reference,.05)
        assert observed.delegate.memory==baseline.memory
    observed.reset()
    assert observed.records==[]


def test_corner_design_and_shortcut_disabled_in_execution():
    corner_cases=[c for c in cases() if c[0].startswith('corner_')]
    assert len(corner_cases)==24
    case=corner_cases[0]
    name,path,world,initial,constraints,speed,horizon,turn=case
    assert len(path)==3 and initial.twist.u==speed
    assert turn==pytest.approx(5/speed)
    # A short horizon is sufficient to verify the corner is preserved in timing.
    report,history=run_case(name,path,world,initial,constraints,speed,.1,turn)
    assert report['reference_end']==pytest.approx(10/speed)
    assert report['status']=='TIMEOUT'
    assert len(history['state'])==3
