"""Planning-plane ocean-current interfaces for Environment v0.3."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from environment.coordinates import PlanningPosition, as_xz
from environment.obstacle import _finite_number


def _validate_query(position: PlanningPosition, time: float) -> None:
    """Validate common current-query inputs without retaining them."""
    as_xz(position)
    _finite_number(time, "time")


class CurrentField(ABC):
    """Backend-independent interface for current velocity in the x-z plane."""

    @abstractmethod
    def velocity_at(
        self, position: PlanningPosition, time: float
    ) -> np.ndarray:
        """Return finite planning-plane velocity ``[v_x, v_z]``."""


@dataclass(frozen=True, slots=True)
class NoCurrent(CurrentField):
    """Current field that returns zero velocity everywhere and at every time."""

    def velocity_at(
        self, position: PlanningPosition, time: float
    ) -> np.ndarray:
        _validate_query(position, time)
        return np.zeros(2, dtype=np.float64)


@dataclass(frozen=True, slots=True, init=False)
class ConstantCurrent(CurrentField):
    """Spatially and temporally constant current in planning coordinates."""

    _velocity: tuple[float, float]

    def __init__(self, velocity: Sequence[float] | np.ndarray) -> None:
        vx, vz = as_xz(velocity)
        object.__setattr__(self, "_velocity", (float(vx), float(vz)))

    @property
    def velocity(self) -> np.ndarray:
        """Return an independent ``[v_x, v_z]`` copy of the configured value."""
        return np.asarray(self._velocity, dtype=np.float64)

    def velocity_at(
        self, position: PlanningPosition, time: float
    ) -> np.ndarray:
        _validate_query(position, time)
        return self.velocity

