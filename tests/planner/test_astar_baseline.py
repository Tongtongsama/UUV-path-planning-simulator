"""Grid coordinates, search correctness, limits and collision regressions."""
from math import sqrt
import pytest
from core import Path, Pose, Twist, VehicleState
from environment import Boundary, Obstacle, WorldModel
from planner import (AStarConfig, AStarPlanner, OccupancyGrid, PlanningConstraints,
                     PlanningRequest, PlanningStatus, EnvironmentPlanningSpace,
                     validate_path)


def world(obstacles=()):
    return EnvironmentPlanningSpace(WorldModel(Boundary.rectangle(
        x_min=0, x_max=10, z_min=-10, z_max=0), obstacles))


def request(start=Pose(x=1,z=-5), goal=Pose(x=9,z=-5), **kwargs):
    return PlanningRequest(VehicleState(start,Twist.zero()), goal, PlanningConstraints(**dict(vehicle_radius=0, **kwargs)))


def test_grid_coordinate_contract():
    grid = OccupancyGrid(world(), 3)
    assert grid.shape == (4,4)
    assert grid.index_to_pose((2,1)) == Pose(x=6,z=-7)
    assert grid.world_to_index(Pose(x=10,z=0)) == (3,3)
    assert grid.world_to_index(Pose(x=1.5,z=-8.5)) == (1,1)
    for i in range(4):
        for j in range(4):
            assert grid.world_to_index(grid.index_to_pose((i,j))) == (i,j)
    with pytest.raises(ValueError):
        grid.world_to_index(Pose(x=-.01))
    with pytest.raises(ValueError):
        grid.index_to_pose((4,0))
    with pytest.raises(TypeError):
        grid.index_to_pose((True,0))


@pytest.mark.parametrize("kwargs", [dict(resolution=0),dict(resolution=-1),dict(resolution=True),
    dict(resolution=float("nan")),dict(connectivity=6),dict(connectivity=8.0),
    dict(max_grid_nodes=0),dict(max_expanded_nodes=True)])
def test_config_validation(kwargs):
    with pytest.raises((TypeError,ValueError)):
        AStarConfig(**kwargs)


@pytest.mark.parametrize("connectivity,expected", [(4,8),(8,4*sqrt(2))])
def test_free_space_optimal_lattice_cost(connectivity,expected):
    r = request(Pose(x=1,z=-9),Pose(x=5,z=-5))
    p = AStarPlanner(AStarConfig(connectivity=connectivity))
    a, b = p.plan(r,world()), p.plan(r,world())
    assert a.success and a.path_cost == pytest.approx(expected)
    assert a.path == b.path and a.expanded_nodes == b.expanded_nodes
    assert a.planning_time >= 0


def test_detour_and_final_validation():
    obstacle = Obstacle.rectangle("wall",x_min=4.9,x_max=5.1,z_min=-7,z_max=-3)
    s, r = world([obstacle]), request(safety_margin=.2)
    result = AStarPlanner().plan(r,s)
    assert result.success and result.path_cost > 8
    assert validate_path(result.path,s,r.constraints,request=r).valid
    assert result.path.start == r.start.pose and result.path.goal == r.goal


def test_thin_wall_unreachable_despite_free_nodes():
    obstacle = Obstacle.rectangle("wall",x_min=4.49,x_max=4.51,z_min=-10,z_max=0)
    result = AStarPlanner().plan(request(),world([obstacle]))
    assert result.status is PlanningStatus.NO_PATH and result.path is None


def test_no_corner_cutting():
    obstacles = [Obstacle.circle("a",(2,-9),.1),Obstacle.circle("b",(1,-8),.1)]
    grid = OccupancyGrid(world(obstacles),1)
    assert (2,2) not in grid.neighbors((1,1),8)


def test_off_grid_endpoints_and_pitch_preservation():
    r = request(Pose(x=.3,z=-9.7,pitch=.2),Pose(x=9.7,z=-.2,pitch=-.1))
    result = AStarPlanner(AStarConfig(resolution=3)).plan(r,world())
    assert result.success
    assert result.path.start == r.start.pose and result.path.goal == r.goal
    assert validate_path(result.path,world(),r.constraints,request=r).valid


def test_endpoint_connector_collision():
    # Start is safe, but each of its four surrounding lattice nodes is blocked.
    obstacles = [Obstacle.circle(str(i),(x,z),.1) for i,(x,z) in enumerate(
        [(1,-9),(1,-8),(2,-9),(2,-8)])]
    result = AStarPlanner().plan(request(Pose(x=1.5,z=-8.5)),world(obstacles))
    assert result.status is PlanningStatus.NO_PATH
    assert "connector" in result.message


def test_invalid_endpoints_and_limits():
    assert AStarPlanner().plan(request(start=Pose(x=-1)),world()).status is PlanningStatus.INVALID_START
    assert AStarPlanner().plan(request(goal=Pose(x=11)),world()).status is PlanningStatus.INVALID_GOAL
    for config in (AStarConfig(max_grid_nodes=1),AStarConfig(max_expanded_nodes=1)):
        assert AStarPlanner(config).plan(request(),world()).status is PlanningStatus.LIMIT_REACHED
    assert AStarPlanner().plan(request(max_path_length=7),world()).status is PlanningStatus.NO_PATH
    assert AStarPlanner().plan(request(max_path_length=8),world()).success


def test_identical_positions_zero_length():
    r = request(goal=Pose(x=1,z=-5),max_path_length=0)
    result = AStarPlanner().plan(r,world())
    assert result.success and result.path_cost == 0 and len(result.path) == 1


def test_footprint_blocks_narrow_passage():
    obstacles = [Obstacle.rectangle("low",x_min=4,x_max=6,z_min=-10,z_max=-5.4),
                 Obstacle.rectangle("high",x_min=4,x_max=6,z_min=-4.6,z_max=0)]
    s = world(obstacles)
    assert AStarPlanner().plan(request(),s).success
    assert AStarPlanner().plan(request(safety_margin=.5),s).status is PlanningStatus.NO_PATH


@pytest.mark.parametrize("connectivity", [4,8])
@pytest.mark.parametrize("tolerance", [0.1,2.0])
def test_obstacle_route_cost_matches_independent_dijkstra(connectivity,tolerance):
    from heapq import heappop, heappush
    from math import hypot
    s = world([Obstacle.rectangle("box",x_min=4,x_max=6,z_min=-7,z_max=-3)])
    r = request(goal_tolerance=tolerance)
    # Independent integer lattice enumeration with continuous edge validation.
    source, target = (1,-5), (9,-5)
    queue, costs = [(0,source)], {source:0}
    while queue:
        cost, node = heappop(queue)
        if cost != costs[node]:
            continue
        if hypot(node[0]-target[0],node[1]-target[1]) <= tolerance:
            break
        for dx in (-1,0,1):
            for dz in (-1,0,1):
                if (dx,dz) == (0,0) or (connectivity == 4 and dx and dz):
                    continue
                nxt = (node[0]+dx,node[1]+dz)
                a, b = Pose(x=node[0],z=node[1]), Pose(x=nxt[0],z=nxt[1])
                if not s.is_pose_valid(b) or not s.is_segment_valid(a,b):
                    continue
                if dx and dz and not (s.is_pose_valid(Pose(x=nxt[0],z=node[1]))
                    and s.is_pose_valid(Pose(x=node[0],z=nxt[1]))):
                    continue
                candidate = cost+hypot(dx,dz)
                if candidate < costs.get(nxt,float("inf")):
                    costs[nxt] = candidate
                    heappush(queue,(candidate,nxt))
    result = AStarPlanner(AStarConfig(connectivity=connectivity)).plan(r,s)
    assert result.success and result.path_cost == pytest.approx(cost)


@pytest.mark.parametrize("offset", [0.0,0.25,0.5])
def test_start_in_goal_region_needs_no_grid(offset):
    r = request(goal=Pose(x=1+offset,z=-5),goal_tolerance=.5,max_path_length=0)
    result = AStarPlanner(AStarConfig(max_grid_nodes=1)).plan(r,world())
    assert result.success and result.path == Path.from_list([r.start.pose])
    assert result.path_cost == 0 and result.expanded_nodes == 0


def test_region_reachable_without_exact_goal_connector():
    obstacles = [Obstacle.circle(str(i),(x,z),.1) for i,(x,z) in enumerate(
        [(8,-5),(8,-6),(9,-5),(9,-6)])]
    s = world(obstacles)
    r = request(goal=Pose(x=8.5,z=-5.5),goal_tolerance=2)
    assert AStarPlanner().plan(request(goal=r.goal,goal_tolerance=0),s).status is PlanningStatus.NO_PATH
    result = AStarPlanner().plan(r,s)
    assert result.success and result.path.goal != r.goal
    assert validate_path(result.path,s,r.constraints,request=r).valid


def test_region_distance_drives_length_pruning():
    r = request(goal_tolerance=2,max_path_length=6)
    result = AStarPlanner().plan(r,world())
    assert result.success and result.path_cost == 6
    assert result.path.goal == Pose(x=7,z=-5)
    assert AStarPlanner().plan(request(goal_tolerance=2,max_path_length=5.99),world()).status is PlanningStatus.NO_PATH


def test_zero_tolerance_still_reaches_exact_off_grid_goal():
    r = request(goal=Pose(x=8.5,z=-5.5,pitch=.3),goal_tolerance=0)
    result = AStarPlanner().plan(r,world())
    assert result.success and result.path.goal == r.goal


def test_start_connector_cost_cannot_exceed_length_limit():
    r = request(start=Pose(x=1.25,z=-5),goal=Pose(x=2,z=-5),
                goal_tolerance=.5,max_path_length=.1)
    assert AStarPlanner().plan(r,world()).status is PlanningStatus.NO_PATH
