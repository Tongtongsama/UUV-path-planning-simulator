"""Offline fair-comparison diagnostics do not change runtime policy contracts."""
from math import pi
from types import SimpleNamespace
import pytest

from core import Pose, Twist, VehicleState, Path, Trajectory
from environment import Boundary, WorldModel
from navigation import NavigationStatus
from validation.guidance_matched_comparison import phase, wrapped_step, nearest_projection, common_metrics, fingerprint


def test_direct_guidance_does_not_imply_capture():
    trajectory=Trajectory((VehicleState(Pose(),Twist(),0.),VehicleState(Pose(x=1),Twist(),1.)))
    step=SimpleNamespace(guidance_phase='PATH_FOLLOWING',effective_guidance=object())
    assert phase(step,trajectory)=='PATH_FOLLOWING'
    step=SimpleNamespace(guidance_phase='TIMED_REFERENCE',startup_phase='RELEASED',reference_time=.5,time=3.)
    assert phase(step,trajectory)=='PATH_FOLLOWING'
    step.reference_time=1.
    assert phase(step,trajectory)=='TERMINAL_CAPTURE'


def test_wrapped_pitch_step_does_not_count_two_pi():
    assert wrapped_step([pi-.01,-pi+.01])==pytest.approx(.02)
    assert wrapped_step([])==0


def test_projection_metric_is_finite_not_infinite_line():
    path=Path((Pose(),Pose(x=1)))
    distance,index,arc,cross=nearest_projection(Pose(x=2,z=1),path)
    assert distance==pytest.approx(2**.5)
    assert index==0 and arc==1 and cross==1


@pytest.mark.parametrize('arm',['los','timed'])
def test_rate_violation_is_diagnostic_and_censored_failure_not_success(arm):
    states=(VehicleState(Pose(),Twist(q=.4),0.),VehicleState(Pose(x=.1),Twist(),.05))
    trajectory=Trajectory(states)
    step=SimpleNamespace(guidance_phase='PATH_FOLLOWING',controller_diagnostics=None,
        effective_guidance=None,guidance_command=None,path_progress_s=0.,time=0.)
    result=SimpleNamespace(state_history=states,control_history=(step,),trajectory=trajectory,
        reference_history=states,metrics={},status=NavigationStatus.TIMEOUT)
    world=WorldModel(Boundary.rectangle(x_min=-1,x_max=2,z_min=-1,z_max=1))
    metrics=common_metrics(result,Path((Pose(),Pose(x=1))),world,.2,.05,arm)
    assert metrics['rate_envelope_violation'] and metrics['maximum_abs_q']==.4
    assert result.status is NavigationStatus.TIMEOUT and metrics['censored']
    assert metrics['minimum_boundary_margin']==pytest.approx(.8)
    assert metrics['switch_limit_count']==(0 if arm=='los' else None)
    assert metrics['progress_stall']==(False if arm=='los' else None)


def test_common_fingerprint_changes_for_speed_or_safety():
    original={'speed':.5,'clearance':.2}
    assert fingerprint(original)==fingerprint({'clearance':.2,'speed':.5})
    assert fingerprint(original)!=fingerprint({**original,'speed':.3})
    assert fingerprint(original)!=fingerprint({**original,'clearance':.3})
