"""Uniform outcome and diagnostic metadata for global planners."""
from dataclasses import dataclass
from enum import Enum, auto
from numbers import Integral
from core import Path
from ._checks import scalar


class PlanningStatus(Enum):
    """Expected planning outcomes; malformed API inputs still raise errors."""
    SUCCESS = auto()
    INVALID_START = auto()
    INVALID_GOAL = auto()
    NO_PATH = auto()
    TIMEOUT = auto()
    LIMIT_REACHED = auto()
    INTERNAL_ERROR = auto()


@dataclass(frozen=True)
class PlanningResult:
    """A success owns a nonempty Core Path; failures never carry a path."""
    status: PlanningStatus
    planner_name: str
    path: Path | None = None
    planning_time: float | None = None
    path_cost: float | None = None
    expanded_nodes: int | None = None
    message: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.status, PlanningStatus):
            raise TypeError("status must be PlanningStatus")
        if not isinstance(self.planner_name, str) or not self.planner_name.strip():
            raise ValueError("planner_name must be nonempty")
        if self.success:
            if not isinstance(self.path, Path) or len(self.path) == 0:
                raise ValueError("SUCCESS requires a nonempty Path")
        elif self.path is not None:
            raise ValueError("failure must not carry a path")
        for name in ("planning_time", "path_cost"):
            if getattr(self, name) is not None:
                object.__setattr__(self, name, scalar(getattr(self, name), name))
        if self.expanded_nodes is not None:
            if isinstance(self.expanded_nodes, bool) or not isinstance(self.expanded_nodes, Integral):
                raise TypeError("expanded_nodes must be integer")
            if self.expanded_nodes < 0:
                raise ValueError("expanded_nodes must be nonnegative")
        if self.message is not None and not isinstance(self.message, str):
            raise TypeError("message must be string or None")

    @property
    def success(self) -> bool:
        """Whether this outcome represents a successful plan."""
        return self.status is PlanningStatus.SUCCESS
