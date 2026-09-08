"""Immutable planning task, independent of environment ownership."""
from dataclasses import dataclass
from core import Pose, Twist, VehicleState
from .constraints import PlanningConstraints
from ._checks import check_pose, scalar


@dataclass(frozen=True)
class PlanningRequest:
    """Start state and position goal in the ENU x-z plane."""
    start: VehicleState
    goal: Pose
    constraints: PlanningConstraints

    def __post_init__(self) -> None:
        if not isinstance(self.start, VehicleState):
            raise TypeError("start must be VehicleState")
        if not isinstance(self.constraints, PlanningConstraints):
            raise TypeError("constraints must be PlanningConstraints")
        check_pose(self.start.pose)
        check_pose(self.goal)
        if not isinstance(self.start.twist, Twist):
            raise TypeError("start.twist must be Twist")
        scalar(self.start.timestamp, "timestamp", False)
        for name in ("u", "v", "w", "p", "q", "r"):
            scalar(getattr(self.start.twist, name), name, False)
        if any(abs(getattr(self.start.twist, name)) > 1e-12 for name in ("v", "p", "r")):
            raise ValueError("start has inactive velocity components")
