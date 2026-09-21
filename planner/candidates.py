"""Bounded geometric candidate search; empirical ranking is not feasibility proof."""
from dataclasses import dataclass, replace
from math import atan2, ceil, hypot, pi
from time import perf_counter
from typing import Callable

from core import Path, Pose
from .algorithms.grid import AStarConfig, AStarPlanner
from .base import GlobalPlanner
from .request import PlanningRequest
from .result import PlanningResult, PlanningStatus
from .space import PlanningSpace
from .validation import validate_path, validate_request
from ._checks import scalar


def simplify_path(path: Path) -> Path:
    """Remove duplicate positions and same-direction collinear points, never U-turns.

    Caller must revalidate the result with the original request and space.
    Orientation fields are irrelevant to this x-z geometric simplification.
    """
    points = []
    for p in path:
        if points and p.x == points[-1].x and p.z == points[-1].z:
            continue
        while len(points) >= 2:
            a, b = points[-2:]
            ux, uz, vx, vz = b.x-a.x, b.z-a.z, p.x-b.x, p.z-b.z
            if ux*vx+uz*vz <= 0 or abs(ux*vz-uz*vx) > 1e-12*hypot(ux, uz)*hypot(vx, vz):
                break
            points.pop()
        points.append(p)
    return Path(tuple(points))


@dataclass(frozen=True)
class CandidateConfig:
    """Metre/radian empirical weights; all terms nonnegative, MINIMUM cost wins.

    Buffers change only candidate generation, not execution safety thresholds.
    Turn/spacing costs rank completed paths, never enter A*'s search state.
    No optimality is claimed beyond minimum score in this finite candidate set.
    """
    buffers: tuple[float, ...] = (0., .15, .3, .5, .75)
    turn_weight: float = 1.
    clearance_weight: float = 2.
    spacing_weight: float = 2.
    unknown_weight: float = .25
    preferred_spacing: float = 4.
    preferred_clearance: float = .75
    sample_spacing: float = .1

    def __post_init__(self) -> None:
        object.__setattr__(self, 'buffers', tuple(scalar(v, 'buffer') for v in self.buffers))
        if not self.buffers or len(self.buffers) > 16 or self.buffers[0] != 0:
            raise ValueError('one to sixteen buffers, starting with zero, required')
        for name in ('turn_weight', 'clearance_weight', 'spacing_weight', 'unknown_weight',
                     'preferred_spacing', 'preferred_clearance', 'sample_spacing'):
            object.__setattr__(self, name, scalar(getattr(self, name), name))
        if self.sample_spacing == 0:
            raise ValueError('sample_spacing must be positive')


def geometry_cost(path: Path, space: PlanningSpace, required: float,
                  boundary_margin: Callable[[Pose, float], float], config: CandidateConfig) -> dict:
    """Sampled soft net-clearance cost; continuous validation remains mandatory."""
    lengths = [a.distance_to(b) for a, b in zip(path, path.poses[1:])]
    headings = [atan2(b.z-a.z, b.x-a.x) for a, b in zip(path, path.poses[1:])]
    turns = [abs((b-a+pi) % (2*pi)-pi) for a, b in zip(headings, headings[1:])]
    obstacle, boundary, penalty = float('inf'), float('inf'), 0.
    for a, b, length in zip(path, path.poses[1:], lengths):
        n = max(1, ceil(length/config.sample_spacing))
        for i in range(n+1):
            p = Pose(x=a.x+(b.x-a.x)*i/n, z=a.z+(b.z-a.z)*i/n)
            o, m = space.clearance(p)-required, boundary_margin(p, required)
            obstacle, boundary = min(obstacle, o), min(boundary, m)
            penalty += max(0., config.preferred_clearance-min(o, m))*length/(n+1)
    spacing = sum(max(0., config.preferred_spacing-s) for s in lengths[1:-1])
    total = (sum(lengths)+config.turn_weight*sum(turns)+config.clearance_weight*penalty
             +config.spacing_weight*spacing+config.unknown_weight*len(turns))
    return dict(total_cost=total, length=sum(lengths), turn_radians=turns,
                corner_spacing=lengths[1:-1], clearance_deficit_integral=penalty,
                spacing_deficit=spacing, sampled_obstacle_net=obstacle,
                sampled_boundary_net=boundary, unknown_turns=len(turns),
                evidence_status='UNKNOWN', evidence_reason='Actual entry, memory and corridor not matched')


class CandidatePlanner(GlobalPlanner):
    """Generate buffered geometric A* alternatives and choose minimum final cost."""
    def __init__(self, astar: AStarConfig, boundary_margin: Callable[[Pose, float], float],
                 config: CandidateConfig = CandidateConfig()) -> None:
        if not isinstance(astar, AStarConfig) or not isinstance(config, CandidateConfig):
            raise TypeError('invalid planner configuration')
        if not callable(boundary_margin):
            raise TypeError('explicit boundary margin capability required')
        self.astar, self.boundary_margin, self.config = astar, boundary_margin, config
        self.diagnostics: list[dict] = []

    @property
    def name(self) -> str:
        return 'finite_geometric_candidates_v1'

    def plan(self, request: PlanningRequest, space: PlanningSpace) -> PlanningResult:
        """No dynamics queries, no physical-feasibility or global cost-optimality claim."""
        begun = perf_counter()
        self.diagnostics = []
        status = validate_request(request, space)
        if status is not None:
            return PlanningResult(status, self.name)
        choices, seen, expanded = [], set(), 0
        for index, buffer in enumerate(self.config.buffers):
            constraints = replace(request.constraints, safety_margin=request.constraints.safety_margin+buffer)
            result = AStarPlanner(self.astar).plan(replace(request, constraints=constraints), space)
            expanded += result.expanded_nodes or 0
            row = dict(index=index, buffer=buffer, status=result.status.name,
                       planning_time=result.planning_time, expanded_nodes=result.expanded_nodes)
            self.diagnostics.append(row)
            if not result.success:
                continue
            path = simplify_path(result.path)
            validation = validate_path(path, space, request.constraints, request=request)
            if not validation.valid:
                row.update(status='POSTPROCESS_REJECTED', message=validation.message)
                continue
            key = tuple((p.x, p.z) for p in path)
            row['duplicate'] = key in seen
            row['path'] = key
            row['geometry'] = geometry_cost(path, space, request.constraints.required_clearance,
                                            self.boundary_margin, self.config)
            if key not in seen:
                choices.append((row['geometry']['total_cost'], index, path))
                seen.add(key)
        if not choices:
            return PlanningResult(PlanningStatus.NO_PATH, self.name, planning_time=perf_counter()-begun,
                expanded_nodes=expanded, message='No candidate within finite generation budget; not physical infeasibility')
        cost, index, path = min(choices, key=lambda item: item[:2])
        self.diagnostics[index]['selected'] = True
        return PlanningResult(PlanningStatus.SUCCESS, self.name, path=path, path_cost=cost,
                              expanded_nodes=expanded, planning_time=perf_counter()-begun)
