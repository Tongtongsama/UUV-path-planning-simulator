"""Ensure M2 checker detects corrupted reference evidence."""
from dataclasses import replace
import pytest
from core import Path, Pose, Trajectory, Twist, VehicleState
from environment import Boundary, WorldModel
from planner import EnvironmentPlanningSpace, PlanningConstraints, PlanningRequest
from trajectory import generate_trajectory, TrajectoryConfig
from validation.trajectory_acceptance import check_generation


@pytest.mark.parametrize("corruption,failed", [("speed","reference_speed_nominal"),
    ("pitch","pitch_matches_segment"),("terminal","terminal_twist_zero"),("time","duration_consistent")])
def test_corrupt_evidence_fails(corruption,failed):
    space=EnvironmentPlanningSpace(WorldModel(Boundary.rectangle(x_min=-1,x_max=3,z_min=-1,z_max=1)))
    req=PlanningRequest(VehicleState.zero(),Pose(x=2),PlanningConstraints(0))
    config=TrajectoryConfig()
    result=generate_trajectory(Path.from_list([Pose(),req.goal]),space,req.constraints,config,request=req)
    assert all(check_generation(result,space,req,config,result).values())
    states=list(result.trajectory.states)
    if corruption=="speed": states[0]=replace(states[0],twist=Twist(u=.2))
    if corruption=="pitch": states[0]=replace(states[0],pose=Pose(pitch=.4))
    if corruption=="terminal": states[-1]=replace(states[-1],twist=Twist(q=.1))
    if corruption=="time": states[-1]=replace(states[-1],timestamp=states[-1].timestamp+1)
    changed=replace(result,trajectory=Trajectory.from_list(states))
    checks=check_generation(changed,space,req,config,result)
    assert not checks[failed] and not checks["deterministic"]
