"""Finite-candidate ranking and shared postprocessing regression checks."""
from dataclasses import replace
from math import pi
from types import SimpleNamespace
import pytest
from core import Path, Pose, VehicleState, Twist
from environment import Boundary, WorldModel, Obstacle
from planner import AStarConfig, EnvironmentPlanningSpace, PlanningRequest, PlanningConstraints, validate_path
from planner.candidates import CandidateConfig, CandidatePlanner, simplify_path, geometry_cost
from validation.frozen_candidate_regression import signed_rectangle_margin


@pytest.mark.parametrize('points,expected', [
    ([(0,0),(1,0),(2,0)], [(0,0),(2,0)]),
    ([(0,0),(1,0),(0,0)], [(0,0),(1,0),(0,0)]),
    ([(0,0),(1,1),(2,2)], [(0,0),(2,2)]),
    ([(0,0),(0,0),(1,0)], [(0,0),(1,0)]),
    ([(0,0)], [(0,0)]),
    ([(0,0),(1,0),(1,1)], [(0,0),(1,0),(1,1)]),
    ([(0,0),(1,0),(2,1e-8)], [(0,0),(1,0),(2,1e-8)])])
def test_simplification(points, expected):
    path = Path(tuple(Pose(x=x,z=z) for x,z in points))
    actual = simplify_path(path)
    assert [(p.x,p.z) for p in actual] == expected
    assert simplify_path(actual) == actual
    assert actual.length() == pytest.approx(path.length())


@pytest.mark.parametrize('kwargs', [dict(buffers=()),dict(buffers=(.1,)),dict(buffers=(-1,)),
    dict(sample_spacing=0),dict(turn_weight=-1),dict(spacing_weight=float('nan'))])
def test_config_rejects_invalid(kwargs):
    with pytest.raises((ValueError,TypeError)):
        CandidateConfig(**kwargs)


def fixture():
    world = WorldModel(Boundary.rectangle(x_min=0,x_max=10,z_min=-10,z_max=0),
        [Obstacle.rectangle(obstacle_id='wall',x_min=4.9,x_max=5.1,z_min=-7,z_max=-3)])
    space = EnvironmentPlanningSpace(world)
    request = PlanningRequest(VehicleState(Pose(x=1,z=-5),Twist()),Pose(x=9,z=-5),
                              PlanningConstraints(.2,.1,.25))
    margin = lambda p,r: signed_rectangle_margin(p, world.boundary.bounds,r)
    return request,space,margin


def test_candidate_minimum_and_determinism():
    req,space,margin = fixture()
    planner = CandidatePlanner(AStarConfig(resolution=.5),margin)
    first = planner.plan(req,space)
    rows = planner.diagnostics
    second = planner.plan(req,space)
    assert first.success and first.path == second.path
    assert first.path_cost == second.path_cost
    assert validate_path(first.path,space,req.constraints,request=req).valid
    assert first.path_cost == min(r['geometry']['total_cost'] for r in rows if 'geometry' in r)
    assert sum(r.get('selected',False) for r in rows) == 1
    assert all(r['geometry']['evidence_status']=='UNKNOWN' for r in rows if 'geometry' in r)
    assert req.constraints.required_clearance == pytest.approx(.3)


def test_reversal_has_pi_turn_cost():
    req,space,margin = fixture()
    path = Path((Pose(x=1,z=-5),Pose(x=2,z=-5),Pose(x=1,z=-5)))
    metrics = geometry_cost(simplify_path(path),space,.3,margin,CandidateConfig())
    assert metrics['turn_radians'] == [pi]
    assert metrics['unknown_turns'] == 1


def test_invalid_start_propagates():
    req,space,margin = fixture()
    req = replace(req,start=VehicleState(Pose(x=5,z=-5),Twist()))
    result = CandidatePlanner(AStarConfig(),margin).plan(req,space)
    assert result.status.name == 'INVALID_START'
    assert result.path is None


def test_explicit_boundary_capability_required():
    with pytest.raises(TypeError):
        CandidatePlanner(AStarConfig(),None)
