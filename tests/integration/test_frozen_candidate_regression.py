import json
from core import Pose, Twist, VehicleState, Trajectory
from validation.navigation_acceptance import ROOT, build_cases
from validation.frozen_candidate_regression import signed_rectangle_margin, reference_segment


def test_original_suite_is_sixteen_unique_scenarios():
    cases=build_cases('all',json.loads((ROOT/'config/navigation_v08.json').read_text()))
    assert len(cases)==16
    assert len({c['name'] for _,c,_ in cases})==16


def test_signed_rectangle_margin_does_not_hide_outside_point():
    assert signed_rectangle_margin(Pose(x=2,z=2),(0,0,4,4),.3)==1.7
    assert signed_rectangle_margin(Pose(x=5,z=2),(0,0,4,4),.3)==-1.3


def test_segment_uses_outgoing_knot_and_clamps_terminal():
    traj=Trajectory.from_list([VehicleState(Pose(x=i),Twist.zero(),float(i)) for i in range(3)])
    assert reference_segment(traj,.9)==0
    assert reference_segment(traj,1.)==1
    assert reference_segment(traj,4.)==1
