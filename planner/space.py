"""Backend-independent planning queries and the WorldModel adapter."""
from typing import Protocol, runtime_checkable
from core import Pose
from environment import WorldModel
from ._checks import check_pose, scalar


@runtime_checkable
class PlanningSpace(Protocol):
    """Static x-z space; obstacle contact invalid, boundary contact valid."""
    def contains(self, pose: Pose) -> bool:
        """Whether the centre lies within the operating boundary."""
        ...

    def clearance(self, pose: Pose) -> float:
        """Obstacle surface distance in metres, infinity for no obstacles."""
        ...

    def is_pose_valid(self, pose: Pose, required_clearance: float = 0.0) -> bool:
        """Check circular footprint boundary and obstacle constraints."""
        ...

    def is_segment_valid(self, start: Pose, end: Pose,
                         required_clearance: float = 0.0) -> bool:
        """Check the entire swept footprint, not just sampled points."""
        ...


@runtime_checkable
class BoundedPlanningSpace(PlanningSpace, Protocol):
    """Optional finite search bounds for grid and future sampling planners."""
    @property
    def bounds(self) -> tuple[float, float, float, float]:
        """Return (x_min, z_min, x_max, z_max) in world metres."""
        ...


class EnvironmentPlanningSpace:
    """Adapt public WorldModel queries without exposing backend geometry."""
    def __init__(self, environment: WorldModel) -> None:
        if not isinstance(environment, WorldModel):
            raise TypeError("environment must be WorldModel")
        self._environment = environment

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        """Public operating-boundary bounding box; not a free-space guarantee."""
        return self._environment.boundary.bounds

    def contains(self, pose: Pose) -> bool:
        """Check centre containment in the planning boundary."""
        check_pose(pose)
        return self._environment.contains(pose)

    def clearance(self, pose: Pose) -> float:
        """Return obstacle clearance, excluding distance to boundary."""
        check_pose(pose)
        return self._environment.clearance(pose)

    def is_pose_valid(self, pose: Pose, required_clearance: float = 0.0) -> bool:
        """Check the same footprint semantics as a zero-length segment."""
        return self.is_segment_valid(pose, pose, required_clearance)

    def is_segment_valid(self, start: Pose, end: Pose,
                         required_clearance: float = 0.0) -> bool:
        """Delegate continuous collision checks to Environment."""
        check_pose(start)
        check_pose(end)
        radius = scalar(required_clearance, "required_clearance")
        return self._environment.is_segment_valid(start, end, radius)
