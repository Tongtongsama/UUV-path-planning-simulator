"""Algorithm-independent geometric planning constraints."""
from dataclasses import dataclass
from ._checks import scalar


@dataclass(frozen=True)
class PlanningConstraints:
    """Distances in metres; clearance applies to obstacles and boundary."""
    vehicle_radius: float
    safety_margin: float = 0.0
    goal_tolerance: float = 0.1
    max_path_length: float | None = None

    def __post_init__(self) -> None:
        for name in ("vehicle_radius", "safety_margin", "goal_tolerance", "max_path_length"):
            value = getattr(self, name)
            if name != "max_path_length" or value is not None:
                object.__setattr__(self, name, scalar(value, name))
        scalar(self.required_clearance, "required_clearance")

    @property
    def required_clearance(self) -> float:
        """Total circular footprint radius including safety margin."""
        return self.vehicle_radius + self.safety_margin
