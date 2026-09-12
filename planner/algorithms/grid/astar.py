"""Deterministic A* to a goal region with optional exact-goal connectors."""
from dataclasses import dataclass
from heapq import heappop, heappush
from itertools import count
from math import hypot
from numbers import Integral
from time import perf_counter
from core import Path, Pose
from planner.base import GlobalPlanner
from planner._checks import scalar
from planner.request import PlanningRequest
from planner.result import PlanningResult, PlanningStatus
from planner.space import PlanningSpace, BoundedPlanningSpace
from planner.validation import validate_request, validate_path
from .occupancy import OccupancyGrid


@dataclass(frozen=True)
class AStarConfig:
    """Algorithm settings: metres, 4/8 connectivity and bounded resource use."""
    resolution: float = 1.0
    connectivity: int = 8
    max_expanded_nodes: int = 100000
    max_grid_nodes: int = 250000

    def __post_init__(self) -> None:
        object.__setattr__(self, "resolution", scalar(self.resolution, "resolution"))
        if self.resolution == 0:
            raise ValueError("resolution must be positive")
        for name in ("connectivity", "max_expanded_nodes", "max_grid_nodes"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Integral):
                raise TypeError(f"{name} must be integer")
            if value < 1:
                raise ValueError(f"{name} must be positive")
        if self.connectivity not in (4, 8):
            raise ValueError("connectivity must be 4 or 8")


class AStarPlanner(GlobalPlanner):
    """Euclidean A*, optimal on the configured connector/lattice graph."""
    def __init__(self, config: AStarConfig = AStarConfig()) -> None:
        if not isinstance(config, AStarConfig):
            raise TypeError("config must be AStarConfig")
        self.config = config

    @property
    def name(self) -> str:
        """Stable baseline identifier."""
        return "astar_xz"

    def plan(self, request: PlanningRequest, space: PlanningSpace) -> PlanningResult:
        """Return a collision-validated geometric path or explicit failure."""
        begun = perf_counter()
        expanded = 0

        def result(status, path=None, message=None):
            return PlanningResult(status, self.name, path=path,
                planning_time=perf_counter()-begun,
                path_cost=None if path is None else path.length(),
                expanded_nodes=expanded, message=message)

        status = validate_request(request, space)
        if not isinstance(space, BoundedPlanningSpace):
            raise TypeError("AStarPlanner requires BoundedPlanningSpace")
        if status is not None:
            return result(status)
        start, goal = request.start.pose, request.goal
        radius = request.constraints.required_clearance
        tolerance = request.constraints.goal_tolerance

        def distance(a: Pose, b: Pose) -> float:
            return hypot(a.x-b.x, a.z-b.z)

        def heuristic(pose: Pose) -> float:
            return max(0.0, distance(pose, goal)-tolerance)

        if distance(start, goal) <= tolerance:
            path = Path.from_list([start])
            check = validate_path(path, space, request.constraints, request=request)
            if not check.valid:
                return result(PlanningStatus.INTERNAL_ERROR, message=check.message)
            return result(PlanningStatus.SUCCESS, path)
        try:
            grid = OccupancyGrid(space, self.config.resolution, radius, self.config.max_grid_nodes)
        except OverflowError as exc:
            return result(PlanningStatus.LIMIT_REACHED, message=str(exc))

        starts = [n for n in grid.endpoint_candidates(start) if grid.is_free(n)
                  and space.is_segment_valid(start, grid.index_to_pose(n), radius)]
        goals = {n for n in grid.endpoint_candidates(goal) if grid.is_free(n)
                 and space.is_segment_valid(grid.index_to_pose(n), goal, radius)}
        if not starts:
            return result(PlanningStatus.NO_PATH, message="No valid start connector at this resolution")
        # None is the virtual exact goal. Each heap entry stores its g-score
        # so stale entries can be discarded after a better relaxation.
        queue, closed, parents, costs = [], set(), {}, {}
        serial = count()
        limit = request.constraints.max_path_length
        for node in starts:
            pose = grid.index_to_pose(node)
            g = distance(start, pose)
            if limit is not None and g+heuristic(pose) > limit:
                continue
            costs[node], parents[node] = g, None
            heappush(queue, (g+heuristic(pose), next(serial), g, node))
        while queue:
            _, _, g, node = heappop(queue)
            if node in closed or g != costs[node]:
                continue
            if node is None or distance(grid.index_to_pose(node), goal) <= tolerance:
                chain = []
                cursor = parents[None] if node is None else node
                while cursor is not None:
                    chain.append(grid.index_to_pose(cursor))
                    cursor = parents[cursor]
                poses = [start]
                for pose in reversed(chain):
                    if distance(poses[-1], pose) > 0:
                        poses.append(pose)
                if node is None:
                    if distance(poses[-1], goal) == 0:
                        poses[-1] = goal
                    else:
                        poses.append(goal)
                elif distance(poses[-1], goal) == 0:
                    poses[-1] = goal
                path = Path.from_list(poses)
                check = validate_path(path, space, request.constraints, request=request)
                if not check.valid:
                    return result(PlanningStatus.INTERNAL_ERROR, message=check.message)
                return result(PlanningStatus.SUCCESS, path)
            if expanded >= self.config.max_expanded_nodes:
                return result(PlanningStatus.LIMIT_REACHED, message="Expansion budget exhausted")
            closed.add(node)
            expanded += 1
            pose = grid.index_to_pose(node)
            edges = [(n, grid.index_to_pose(n)) for n in grid.neighbors(node, self.config.connectivity)]
            if node in goals:
                edges.append((None, goal))
            for neighbor, target in edges:
                if neighbor in closed or not space.is_segment_valid(pose, target, radius):
                    continue
                candidate = g+distance(pose,target)
                estimate = candidate+heuristic(target)
                if limit is not None and estimate > limit:
                    continue
                if candidate < costs.get(neighbor, float("inf")):
                    costs[neighbor], parents[neighbor] = candidate, node
                    heappush(queue, (estimate, next(serial), candidate, neighbor))
        return result(PlanningStatus.NO_PATH,
                      message="No path in this finite grid graph under the requested constraints")
