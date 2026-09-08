"""Planner contracts and continuous geometry integration regressions."""
from dataclasses import FrozenInstanceError
import pytest
from core import Path, Pose, Twist, VehicleState
from environment import Boundary, Obstacle, WorldModel
from planner import (EnvironmentPlanningSpace, GlobalPlanner, PlanningConstraints,
                     PlanningRequest, PlanningResult, PlanningSpace, PlanningStatus,
                     validate_path, validate_request)


def space(obstacles=()):
    return EnvironmentPlanningSpace(WorldModel(
        Boundary.rectangle(x_min=0, x_max=10, z_min=0, z_max=10), obstacles))


def request(start=Pose(x=1, z=5), goal=Pose(x=9, z=5), **kwargs):
    return PlanningRequest(VehicleState(start, Twist.zero()), goal,
                           PlanningConstraints(0, **kwargs))


def wall():
    return Obstacle.rectangle("thin", x_min=4.999, x_max=5.001, z_min=4, z_max=6)


class StraightLinePlanner(GlobalPlanner):
    """Test-only adapter exercise, not an obstacle-avoidance baseline."""
    @property
    def name(self) -> str:
        return "straight_line_test_planner"

    def plan(self, request, space):
        status = validate_request(request, space)
        if status is not None:
            return PlanningResult(status, self.name)
        path = Path.from_list([request.start.pose, request.goal])
        validation = validate_path(path, space, request.constraints, request=request)
        if not validation.valid:
            return PlanningResult(PlanningStatus.NO_PATH, self.name, message=validation.message)
        return PlanningResult(PlanningStatus.SUCCESS, self.name, path=path)


@pytest.mark.parametrize("field", ["vehicle_radius", "safety_margin", "goal_tolerance", "max_path_length"])
@pytest.mark.parametrize("value", [-1, float("nan"), float("inf"), True, "1"])
def test_constraints_reject_invalid_values(field, value):
    data = dict(vehicle_radius=0)
    data[field] = value
    with pytest.raises((ValueError, TypeError)):
        PlanningConstraints(**data)


def test_constraints_immutable_and_clearance():
    c = PlanningConstraints(1, safety_margin=0.5)
    assert c.required_clearance == 1.5
    with pytest.raises(FrozenInstanceError):
        c.vehicle_radius = 2
    with pytest.raises(ValueError):
        PlanningConstraints(1e308, 1e308)


@pytest.mark.parametrize("pose", [Pose(y=1), Pose(roll=1), Pose(yaw=1), Pose(x=float("nan"))])
def test_request_rejects_invalid_plane(pose):
    with pytest.raises(ValueError):
        request(goal=pose)


def test_request_preserves_active_start_state():
    start = VehicleState(Pose(x=1, z=2, pitch=.2), Twist(u=1, w=.2, q=.1), 3)
    assert PlanningRequest(start, Pose(), PlanningConstraints(0)).start is start
    with pytest.raises(ValueError):
        PlanningRequest(VehicleState(Pose(), Twist(v=1)), Pose(), PlanningConstraints(0))


@pytest.mark.parametrize("status", list(PlanningStatus))
def test_result_path_invariants(status):
    p = Path.from_list([Pose()])
    if status is PlanningStatus.SUCCESS:
        with pytest.raises(ValueError):
            PlanningResult(status, "test")
        assert PlanningResult(status, "test", p).success
    else:
        with pytest.raises(ValueError):
            PlanningResult(status, "test", p)
        assert PlanningResult(status, "test").path is None


@pytest.mark.parametrize("kw", [dict(planner_name=" "), dict(planning_time=-1),
    dict(path_cost=float("inf")), dict(expanded_nodes=-1), dict(expanded_nodes=True),
    dict(expanded_nodes=1.5), dict(message=5), dict(status="SUCCESS")])
def test_result_invalid_metadata(kw):
    data = dict(status=PlanningStatus.NO_PATH, planner_name="test")
    data.update(kw)
    with pytest.raises((ValueError, TypeError)):
        PlanningResult(**data)


def test_space_clearance_and_contact():
    s = space([wall()])
    assert isinstance(s, PlanningSpace)
    assert s.contains(Pose())
    assert s.is_pose_valid(Pose())
    assert not s.is_pose_valid(Pose(), .1)
    assert s.is_pose_valid(Pose(x=1, z=1), 1)
    assert not s.is_pose_valid(Pose(x=4.999, z=5))
    assert s.clearance(Pose(x=4, z=5)) == pytest.approx(.999)
    assert space().clearance(Pose()) == float("inf")


def test_complete_segment_detects_thin_obstacle_and_tangency():
    s = space([wall()])
    a, b = Pose(x=1, z=5), Pose(x=9, z=5)
    assert s.is_pose_valid(a) and s.is_pose_valid(b)
    assert not s.is_segment_valid(a, b)
    assert not s.is_segment_valid(Pose(x=1, z=3), Pose(x=9, z=3), 1)
    assert s.is_segment_valid(Pose(x=1, z=2), Pose(x=9, z=2), 1)


def test_concave_boundary_segment_and_zero_length():
    s = EnvironmentPlanningSpace(WorldModel(Boundary.polygon(
        [(0,0), (4,0), (4,1), (1,1), (1,4), (0,4)])))
    a, b = Pose(x=.5, z=3), Pose(x=3, z=.5)
    assert s.is_pose_valid(a) and s.is_pose_valid(b)
    assert not s.is_segment_valid(a, b)
    assert s.is_segment_valid(a, a)
    obstacle_space = space([wall()])
    assert not obstacle_space.is_segment_valid(Pose(x=5,z=5), Pose(x=5,z=5))


def test_path_validation_diagnostics():
    r = request()
    p = Path.from_list([r.start.pose, r.goal])
    assert validate_path(p, space(), r.constraints, request=r).valid
    assert validate_path(p, space([wall()]), r.constraints).invalid_segment_index == 0
    assert validate_path(Path.from_list([Pose(x=-1)]), space(), r.constraints).invalid_waypoint_index == 0
    assert not validate_path(p, space(), PlanningConstraints(0, max_path_length=7)).valid
    assert validate_path(p, space(), PlanningConstraints(0, max_path_length=8)).valid
    assert not validate_path(Path.from_list([Pose(x=2,z=5), r.goal]), space(), r.constraints, request=r).valid
    assert not validate_path(Path.from_list([r.start.pose, Pose(x=8,z=5)]), space(), r.constraints, request=r).valid
    assert validate_path(Path.from_list([r.start.pose, Pose(x=8.95,z=5)]), space(), r.constraints, request=r).valid
    with pytest.raises(ValueError):
        validate_path(p, space(), PlanningConstraints(1), request=r)


def test_dummy_planner_end_to_end():
    planner = StraightLinePlanner()
    assert planner.plan(request(), space()).success
    blocked = planner.plan(request(), space([wall()]))
    assert blocked.status is PlanningStatus.NO_PATH and blocked.path is None
    assert planner.plan(request(start=Pose(x=5,z=5)), space([wall()])).status is PlanningStatus.INVALID_START
    assert planner.plan(request(goal=Pose(x=11)), space()).status is PlanningStatus.INVALID_GOAL
    with pytest.raises(TypeError):
        GlobalPlanner()


def test_single_point_path_and_invalid_pose():
    r = request(goal=Pose(x=1,z=5))
    assert validate_path(Path.from_list([r.goal]), space(), r.constraints, request=r).valid
    assert not validate_path(Path.from_list([Pose(y=1)]), space(), r.constraints).valid
