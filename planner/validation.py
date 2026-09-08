"""Shared request and full-path geometric validation."""
from dataclasses import dataclass
from math import hypot
from core import Path
from ._checks import check_pose
from .constraints import PlanningConstraints
from .request import PlanningRequest
from .result import PlanningStatus
from .space import PlanningSpace


@dataclass(frozen=True)
class PathValidationResult:
    """First failure; segment index i denotes waypoint i to i+1."""
    valid: bool
    invalid_waypoint_index: int | None = None
    invalid_segment_index: int | None = None
    message: str | None = None


def validate_request(request: PlanningRequest, space: PlanningSpace) -> PlanningStatus | None:
    """Return INVALID_START/GOAL for environmental infeasibility, else None."""
    if not isinstance(request, PlanningRequest):
        raise TypeError("request must be PlanningRequest")
    radius = request.constraints.required_clearance
    if not space.is_pose_valid(request.start.pose, radius):
        return PlanningStatus.INVALID_START
    if not space.is_pose_valid(request.goal, radius):
        return PlanningStatus.INVALID_GOAL
    return None


def validate_path(path: Path, space: PlanningSpace, constraints: PlanningConstraints,
                  *, request: PlanningRequest | None = None) -> PathValidationResult:
    """Check geometry and length; optional request enables endpoint checks.

    Start position tolerance is 1e-9 m; goal uses goal_tolerance. Orientation
    feasibility and velocity constraints are outside this geometric contract.
    """
    if not isinstance(path, Path) or not isinstance(constraints, PlanningConstraints):
        raise TypeError("expected Core Path and PlanningConstraints")
    if request is not None:
        if not isinstance(request, PlanningRequest):
            raise TypeError("request must be PlanningRequest")
        if request.constraints != constraints:
            raise ValueError("request and validation constraints differ")
    if len(path) == 0:
        return PathValidationResult(False, message="empty path")
    for i, pose in enumerate(path):
        try:
            check_pose(pose)
        except (TypeError, ValueError) as exc:
            return PathValidationResult(False, invalid_waypoint_index=i, message=str(exc))
        if not space.is_pose_valid(pose, constraints.required_clearance):
            return PathValidationResult(False, invalid_waypoint_index=i, message="invalid waypoint")
    for i in range(len(path) - 1):
        if not space.is_segment_valid(path[i], path[i+1], constraints.required_clearance):
            return PathValidationResult(False, invalid_segment_index=i, message="invalid segment")
    if request is not None:
        for index, expected, tolerance, label in (
            (0, request.start.pose, 1e-9, "start mismatch"),
            (len(path)-1, request.goal, constraints.goal_tolerance, "goal mismatch"),
        ):
            if hypot(path[index].x-expected.x, path[index].z-expected.z) > tolerance:
                return PathValidationResult(False, invalid_waypoint_index=index, message=label)
    if constraints.max_path_length is not None and path.length() > constraints.max_path_length:
        return PathValidationResult(False, message="maximum path length exceeded")
    return PathValidationResult(True)
