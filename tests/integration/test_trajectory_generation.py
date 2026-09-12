"""Analytical timing, geometry and planner-to-reference regression tests."""
import json
from math import cos, sin, pi, hypot
import pytest
from core import Path, Pose
from planner import PlanningConstraints, validate_path, AStarPlanner
from trajectory import TrajectoryConfig, generate_trajectory, sample_reference
from validation.planner_acceptance import ROOT, build_case
from environment import Boundary, WorldModel
from planner import EnvironmentPlanningSpace


def space():
    return EnvironmentPlanningSpace(WorldModel(Boundary.rectangle(x_min=-10,x_max=10,z_min=-10,z_max=10)))


@pytest.mark.parametrize("end,pitch", [(Pose(x=2),0),(Pose(z=2),pi/2),(Pose(z=-2),-pi/2),(Pose(x=-2),pi)])
def test_analytical_straight_reference(end,pitch):
    result=generate_trajectory(Path.from_list([Pose(),end]),space(),PlanningConstraints(0),
        TrajectoryConfig(nominal_speed=.5,sample_spacing=.3),start_time=2)
    t=result.trajectory
    assert t.duration == pytest.approx(4)
    assert t[0].pose.pitch == pytest.approx(pitch)
    assert t[-1].twist.u == 0
    for a,b in zip(t,t.states[1:]):
        dt=b.timestamp-a.timestamp
        assert b.pose.x-a.pose.x == pytest.approx(dt*a.twist.u*cos(a.pose.pitch))
        assert b.pose.z-a.pose.z == pytest.approx(dt*a.twist.u*sin(a.pose.pitch))
        assert hypot(b.pose.x-a.pose.x,b.pose.z-a.pose.z) <= .3+1e-12
    assert sample_reference(t,0).twist.u == 0
    assert sample_reference(t,100).pose == t[-1].pose
    assert sample_reference(t,3).twist.u == .5


def test_duplicate_singleton_and_corner_policy():
    p=Pose(x=1,pitch=.3)
    result=generate_trajectory(Path.from_list([p,p,p]),space(),PlanningConstraints(0))
    assert len(result.trajectory)==1 and result.trajectory[0].twist.u==0
    assert result.trajectory[0].pose==p
    path=Path.from_list([Pose(),Pose(x=1),Pose(x=1,z=1)])
    r=generate_trajectory(path,space(),PlanningConstraints(0),TrajectoryConfig(shortcut=False,sample_spacing=1,nominal_speed=1))
    assert sample_reference(r.trajectory,.5).pose.pitch==0
    assert sample_reference(r.trajectory,1).pose.pitch==pytest.approx(pi/2)
    assert r.raw_path==path
    assert len(generate_trajectory(path,space(),PlanningConstraints(0)).shortcut_path)==2


@pytest.mark.parametrize("kw", [dict(nominal_speed=0),dict(sample_spacing=-1),dict(nominal_speed=True),
    dict(sample_spacing=float('nan')),dict(shortcut=1),dict(max_samples=False)])
def test_config_rejection(kw):
    with pytest.raises((TypeError,ValueError)):
        TrajectoryConfig(**kw)


def test_limits_invalid_geometry_and_time():
    p=Path.from_list([Pose(),Pose(x=1)])
    with pytest.raises(ValueError):
        generate_trajectory(p,space(),PlanningConstraints(0),TrajectoryConfig(max_samples=1))
    with pytest.raises(ValueError):
        generate_trajectory(p,space(),PlanningConstraints(0),start_time=1e30)
    with pytest.raises(ValueError):
        generate_trajectory(Path.from_list([Pose(x=100)]),space(),PlanningConstraints(0))


@pytest.mark.parametrize("filename", ["planner_acceptance_v1.json","planner_acceptance_v2.json"])
def test_all_user_scenarios_after_postprocessing(filename):
    config=json.loads((ROOT/"config/scenarios"/filename).read_text(encoding="utf-8"))
    for case in config["cases"]:
        s,request,settings=build_case(config,case)
        planned=AStarPlanner(settings).plan(request,s)
        assert planned.status.name==case["expected_status"]
        if not planned.success:
            continue
        result=generate_trajectory(planned.path,s,request.constraints,request=request)
        assert result==generate_trajectory(planned.path,s,request.constraints,request=request)
        for path in (result.deduplicated_path,result.shortcut_path,result.sampled_path):
            assert validate_path(path,s,request.constraints,request=request).valid
        assert result.shortcut_path.length() <= planned.path.length()+1e-10
        assert result.trajectory[-1].twist.u==0
