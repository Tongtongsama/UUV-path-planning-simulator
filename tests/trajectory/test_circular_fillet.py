"""Analytic arc geometry and conservative swept-circle certification."""
from dataclasses import replace
from math import sqrt, pi
import pytest
from core import Path, Pose, VehicleState, Twist
from environment import Boundary, WorldModel, Obstacle
from planner import PlanningRequest, PlanningConstraints, EnvironmentPlanningSpace, validate_path
from trajectory.circular_fillet import FilletConfig, FilletStatus, circular_fillet


def fixture(points=((0,0),(2,0),(2,2)), obstacles=()):
    path = Path(tuple(Pose(x=x,z=z) for x,z in points))
    req = PlanningRequest(VehicleState(path[0],Twist()),path[-1],PlanningConstraints(.01))
    world = WorldModel(Boundary.rectangle(x_min=-5,x_max=5,z_min=-5,z_max=5),list(obstacles))
    return path,req,EnvironmentPlanningSpace(world)


@pytest.mark.parametrize('sign',[1,-1])
def test_quarter_circle_geometry(sign):
    path,req,space = fixture(((0,0),(2,0),(2,2*sign)))
    result = circular_fillet(path,req,space)
    assert result.status is FilletStatus.ACCEPTED
    arc = result.arcs[0]
    assert arc.entry.x == pytest.approx(1)
    assert arc.entry.z == pytest.approx(0)
    assert arc.exit.x == pytest.approx(2)
    assert arc.exit.z == pytest.approx(sign)
    assert arc.center.x == pytest.approx(1)
    assert arc.center.z == pytest.approx(sign)
    assert arc.signed_angle == pytest.approx(sign*pi/2)
    assert arc.sagitta <= .001
    assert arc.sampled_length == pytest.approx(pi/2,abs=.001)
    assert validate_path(result.path,space,req.constraints,request=req).valid
    assert circular_fillet(path,req,space) == result


def test_obstacle_between_safe_arc_endpoints_rejected():
    x,z = 1+sqrt(.5),1-sqrt(.5)
    obstacle = Obstacle.rectangle(obstacle_id='interior',x_min=x-.005,x_max=x+.005,z_min=z-.005,z_max=z+.005)
    path,req,space = fixture(obstacles=[obstacle])
    assert validate_path(path,space,req.constraints,request=req).valid
    result = circular_fillet(path,req,space)
    assert result.status is FilletStatus.UNSMOOTHABLE
    assert result.reason == 'ARC_SWEPT_CAPSULE_UNSAFE'
    assert result.path is None


def test_capsule_checks_include_sagitta():
    path,req,space = fixture()
    class RecordingSpace:
        contains = space.contains
        clearance = space.clearance
        is_pose_valid = space.is_pose_valid
        calls = []
        def is_segment_valid(self,a,b,required_clearance=0):
            self.calls.append(required_clearance)
            return space.is_segment_valid(a,b,required_clearance)
    recording = RecordingSpace()
    result = circular_fillet(path,req,recording)
    assert result.path is not None
    assert any(r > req.constraints.required_clearance for r in recording.calls)


@pytest.mark.parametrize('points,reason',[
    (((0,0),(1,0),(0,0)),'REVERSAL'),
    (((0,0),(.5,0),(.5,.5)),'OVERLAP'),
    (((0,0),(2,0),(4,.1)),'LOOKAHEAD')])
def test_rejections(points,reason):
    path,req,space = fixture(points)
    result = circular_fillet(path,req,space)
    assert result.path is None
    assert reason in result.reason


def test_adjacent_arcs_compete_for_same_segment():
    path,req,space = fixture(((0,0),(2,0),(2,1),(4,1)))
    assert circular_fillet(path,req,space).reason == 'ADJACENT_TANGENT_REGIONS_OVERLAP'


@pytest.mark.parametrize('points',[((0,0),),((0,0),(1,0),(2,0)),((0,0),(0,0),(2,0))])
def test_degenerate_and_straight(points):
    path,req,space = fixture(points)
    result = circular_fillet(path,req,space)
    assert result.status is FilletStatus.ACCEPTED
    assert result.arcs == ()


@pytest.mark.parametrize('kwargs',[dict(radius=0),dict(max_chord=-1),dict(max_sagitta=2),
    dict(lookahead=float('nan')),dict(max_lookahead_fraction=1),dict(max_points=True)])
def test_config_validation(kwargs):
    with pytest.raises((TypeError,ValueError)):
        FilletConfig(**kwargs)


def test_point_budget():
    path,req,space = fixture()
    assert circular_fillet(path,req,space,FilletConfig(max_points=2)).reason == 'POINT_BUDGET'
