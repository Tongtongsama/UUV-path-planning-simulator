"""Minimal global planning abstraction; algorithms own execution flow."""
from abc import ABC, abstractmethod
from .request import PlanningRequest
from .result import PlanningResult
from .space import PlanningSpace


class GlobalPlanner(ABC):
    """Map a task and planning space to a geometric planning outcome."""
    @property
    @abstractmethod
    def name(self) -> str:
        """Stable diagnostic identifier for this planner."""
        ...

    @abstractmethod
    def plan(self, request: PlanningRequest, space: PlanningSpace) -> PlanningResult:
        """Return a path or normal failure status without generating time data."""
        ...


Planner = GlobalPlanner
