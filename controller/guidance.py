"""Explicit inner-loop targets: no geometric position reference or hidden bypass."""
from dataclasses import dataclass
from ._validation import finite_scalar


@dataclass(frozen=True)
class BodyGuidance3DOF:
    """Desired body surge u (m/s), nose-up pitch (rad) and q (rad/s)."""
    u: float
    pitch: float
    q: float = 0.0

    def __post_init__(self):
        for name in ('u','pitch','q'):
            object.__setattr__(self,name,finite_scalar(getattr(self,name),name=name))
