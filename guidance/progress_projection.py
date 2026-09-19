"""Pure ENU x-z geometry with explicit, path-bound progress state.

No clock, dynamics, obstacle model or controller participates in projection.
All distances, including the forward window and advance slack, are metres.
"""

from bisect import bisect_right
from dataclasses import dataclass, field
from math import hypot, isfinite
from numbers import Real

from core.path import Path
from core.pose import Pose


def _number(value: float, name: str, *, minimum: float | None = None,
            positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a real number")
    value = float(value)
    if not isfinite(value):
        raise ValueError(f"{name} must be finite")
    if positive and value <= 0 or minimum is not None and value < minimum:
        raise ValueError(f"{name} is outside its admissible range")
    return value


def _pose(pose: Pose) -> None:
    if not isinstance(pose, Pose):
        raise TypeError("position must be a Pose")
    for name in ("x", "y", "z", "roll", "pitch", "yaw"):
        _number(getattr(pose, name), name)
    if any(getattr(pose, name) != 0 for name in ("y", "roll", "yaw")):
        raise ValueError("guidance supports only the ENU x-z plane")


def _distance(a: Pose, b: Pose) -> float:
    return _number(hypot(a.x - b.x, a.z - b.z), "distance")


@dataclass(frozen=True)
class PathGeometry:
    """Validated polyline, retaining the first index of each duplicate run.

    Waypoint orientation is not used for geometry. Non-consecutive duplicates
    are retained: they can represent a loop rather than redundant waypoints.
    """

    path: Path
    geometry_epsilon: float = 1e-9
    points: tuple[Pose, ...] = field(init=False)
    original_indices: tuple[int, ...] = field(init=False)
    lengths: tuple[float, ...] = field(init=False)
    cumulative_s: tuple[float, ...] = field(init=False)

    def __post_init__(self) -> None:
        if not isinstance(self.path, Path):
            raise TypeError("path must be a Path")
        _number(self.geometry_epsilon, "geometry_epsilon", positive=True)
        points, indices = [], []
        for index, pose in enumerate(self.path):
            _pose(pose)
            if not points or (pose.x, pose.z) != (points[-1].x, points[-1].z):
                points.append(Pose(x=pose.x, z=pose.z))
                indices.append(index)
        lengths, cumulative = [], [0.0]
        for a, b in zip(points, points[1:]):
            length = _distance(a, b)
            if length < self.geometry_epsilon:
                raise ValueError("nonzero segment is below geometry_epsilon")
            lengths.append(length)
            end_s = _number(cumulative[-1] + length, "cumulative arc")
            if end_s <= cumulative[-1]:
                raise ValueError("segment length is lost at cumulative arc precision")
            cumulative.append(end_s)
        object.__setattr__(self, "points", tuple(points))
        object.__setattr__(self, "original_indices", tuple(indices))
        object.__setattr__(self, "lengths", tuple(lengths))
        object.__setattr__(self, "cumulative_s", tuple(cumulative))

    @property
    def total_length(self) -> float:
        """Total polyline arc length in metres (zero for a singleton)."""
        return self.cumulative_s[-1]

    def point_at_s(self, s: float) -> Pose:
        """Interpolate position at an arc in [0, total_length], without smoothing."""
        s = _number(s, "s", minimum=0)
        if s > self.total_length:
            raise ValueError("s exceeds path length")
        if not self.lengths or s == self.total_length:
            return self.points[-1]
        i = min(bisect_right(self.cumulative_s, s) - 1, len(self.lengths) - 1)
        mu = (s - self.cumulative_s[i]) / self.lengths[i]
        a, b = self.points[i:i + 2]
        return Pose(x=a.x + mu * (b.x - a.x), z=a.z + mu * (b.z - a.z))


@dataclass(frozen=True)
class SegmentProjection:
    """Finite projection; residual errors use tangent and left/up normal."""

    mu: float
    lambda_raw: float
    projection_point: Pose
    along_track_error: float
    cross_track_error: float


def project_segment(start: Pose, end: Pose, position: Pose,
                    geometry_epsilon: float = 1e-9) -> SegmentProjection:
    """Project onto a nondegenerate segment, keeping the unclipped parameter."""
    for point in (start, end, position):
        _pose(point)
    _number(geometry_epsilon, "geometry_epsilon", positive=True)
    length = _distance(start, end)
    if length < geometry_epsilon:
        raise ValueError("cannot project onto a degenerate segment")
    tx, tz = (end.x - start.x) / length, (end.z - start.z) / length
    dx, dz = position.x - start.x, position.z - start.z
    lam = _number((dx * tx + dz * tz) / length, "lambda_raw")
    mu = min(1.0, max(0.0, lam))
    point = Pose(x=start.x + mu * (end.x - start.x),
                 z=start.z + mu * (end.z - start.z))
    ex, ez = position.x - point.x, position.z - point.z
    along = _number(ex * tx + ez * tz, "along_track_error")
    cross = _number(-ex * tz + ez * tx, "cross_track_error")
    return SegmentProjection(mu, lam, point, along, cross)


@dataclass(frozen=True)
class ProjectionConfig:
    """Explicit geometric settings; no tuned experiment defaults are implied."""

    switch_radius: float
    forward_window: float
    progress_slack: float
    max_switches_per_step: int

    def __post_init__(self) -> None:
        _number(self.switch_radius, "switch_radius", positive=True)
        _number(self.forward_window, "forward_window", positive=True)
        _number(self.progress_slack, "progress_slack", minimum=0)
        if isinstance(self.max_switches_per_step, bool) or not isinstance(
                self.max_switches_per_step, int):
            raise TypeError("max_switches_per_step must be an integer")
        if self.max_switches_per_step < 1:
            raise ValueError("max_switches_per_step must be positive")


@dataclass(frozen=True)
class ProgressState:
    """Explicit committed state; initialize with the actual initial position.

    segment_index denotes the active segment, not necessarily the diagnostic
    nearest projection. total_switches is cumulative; a singleton uses index 0.
    """

    geometry: PathGeometry
    previous_position: Pose
    segment_index: int = 0
    committed_s: float = 0.0
    total_switches: int = 0

    def __post_init__(self) -> None:
        if not isinstance(self.geometry, PathGeometry):
            raise TypeError("geometry must be PathGeometry")
        _pose(self.previous_position)
        for name in ("segment_index", "total_switches"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{name} must be an integer")
            if value < 0:
                raise ValueError(f"{name} cannot be negative")
        _number(self.committed_s, "committed_s", minimum=0)
        g, i = self.geometry, self.segment_index
        if i >= max(1, len(g.lengths)):
            raise ValueError("segment_index outside geometry")
        lower, upper = (g.cumulative_s[i], g.cumulative_s[i + 1]) if g.lengths else (0, 0)
        if not lower <= self.committed_s <= upper:
            raise ValueError("committed_s and active segment are inconsistent")


@dataclass(frozen=True)
class ProjectionResult:
    """Projection diagnostics and an atomic next state.

    mu/errors/projected_s belong to projected_segment_index. segment_index is
    committed and never retreats. switch_count counts only this call's switches.
    A singleton has no tangent: its residual errors and parameters are zero.
    """

    segment_index: int
    projected_segment_index: int
    mu: float
    lambda_raw: float
    projection_point: Pose
    projected_s: float
    committed_s: float
    along_track_error: float
    cross_track_error: float
    switch_count: int
    progress_clamped: bool
    switch_limit_reached: bool
    next_state: ProgressState


def project_progress(position: Pose, state: ProgressState,
                     config: ProjectionConfig) -> ProjectionResult:
    """Search only sequentially gated local segments and commit bounded progress.

    One preceding segment is allowed for diagnostics only. Equal distances use
    the smaller projected arc, then the smaller index. Forward-window clipping
    and displacement-plus-slack clipping both set progress_clamped.
    """
    _pose(position)
    if not isinstance(state, ProgressState):
        raise TypeError("state must be ProgressState")
    if not isinstance(config, ProjectionConfig):
        raise TypeError("config must be ProjectionConfig")
    g, active = state.geometry, state.segment_index
    if not g.lengths:
        next_state = ProgressState(g, position)
        return ProjectionResult(0, 0, 0.0, 0.0, g.points[0], 0.0, 0.0,
                                0.0, 0.0, 0, False, False, next_state)

    def projection(i: int) -> SegmentProjection:
        return project_segment(g.points[i], g.points[i + 1], position, g.geometry_epsilon)

    window_end = min(g.total_length, state.committed_s + config.forward_window)

    def gate(i: int) -> bool:
        if i >= len(g.lengths) - 1 or g.cumulative_s[i + 1] > window_end:
            return False
        p = projection(i)
        radius = min(config.switch_radius, .45 * g.lengths[i], .45 * g.lengths[i + 1])
        return (_distance(position, g.points[i + 1]) <= radius or
                p.lambda_raw >= 1.0 and abs(p.cross_track_error) <= radius)

    admitted = active
    while admitted - active < config.max_switches_per_step and gate(admitted):
        admitted += 1
    limit_reached = admitted - active == config.max_switches_per_step and gate(admitted)
    candidates = []
    for i in range(max(0, active - 1), admitted + 1):
        p = projection(i)
        arc = g.cumulative_s[i] + p.mu * g.lengths[i]
        candidates.append((_distance(position, p.projection_point), arc, i, p))
    _, projected_s, projected_index, p = min(candidates, key=lambda item: item[:3])
    desired = max(state.committed_s, projected_s)
    advance = _number(_distance(position, state.previous_position) + config.progress_slack,
                      "maximum progress advance")
    committed = min(desired, window_end, state.committed_s + advance)
    new_active = active
    while new_active < admitted and g.cumulative_s[new_active + 1] <= committed:
        new_active += 1
    switches = new_active - active
    next_state = ProgressState(g, position, new_active, committed,
                               state.total_switches + switches)
    return ProjectionResult(new_active, projected_index, p.mu, p.lambda_raw,
                            p.projection_point, projected_s, committed,
                            p.along_track_error, p.cross_track_error, switches,
                            committed < desired, limit_reached, next_state)
