"""Unified planning-world queries composed from Environment domain objects."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass, field

import numpy as np

from environment.boundary import Boundary
from environment.coordinates import PlanningPosition, as_xz
from environment.current import CurrentField, NoCurrent
from environment.obstacle import Obstacle, _non_negative


@dataclass(frozen=True, slots=True, eq=False)
class WorldModel:
    """Lightweight, immutable planning view of one static environment."""

    boundary: Boundary
    obstacles: tuple[Obstacle, ...] = ()
    current: CurrentField = field(default_factory=NoCurrent)

    def __init__(
        self,
        boundary: Boundary,
        obstacles: Iterable[Obstacle] = (),
        current: CurrentField | None = None,
    ) -> None:
        try:
            obstacle_tuple = tuple(obstacles)
        except TypeError as exc:
            raise TypeError("obstacles must be an iterable of Obstacle objects") from exc

        object.__setattr__(self, "boundary", boundary)
        object.__setattr__(self, "obstacles", obstacle_tuple)
        object.__setattr__(self, "current", NoCurrent() if current is None else current)
        self._validate()

    def _validate(self) -> None:
        if not isinstance(self.boundary, Boundary):
            raise TypeError("boundary must be a Boundary")
        if not all(isinstance(obstacle, Obstacle) for obstacle in self.obstacles):
            raise TypeError("All obstacles must be Obstacle objects")
        if not isinstance(self.current, CurrentField):
            raise TypeError("current must implement CurrentField")

        ids = [obstacle.obstacle_id for obstacle in self.obstacles]
        if len(ids) != len(set(ids)):
            raise ValueError("Obstacle IDs must be unique within a WorldModel")

    def contains(
        self, position: PlanningPosition, clearance: float = 0.0
    ) -> bool:
        """Return whether a point or footprint is covered by the boundary."""
        return self.boundary.contains(position, clearance=clearance)

    def collides(self, position: PlanningPosition, radius: float = 0.0) -> bool:
        """Return whether a point or circular footprint touches any obstacle."""
        radius_value = _non_negative(radius, "radius")
        normalized = as_xz(position)
        return any(
            obstacle.collides(normalized, radius=radius_value)
            for obstacle in self.obstacles
        )

    def clearance(self, position: PlanningPosition) -> float:
        """Return distance to the nearest obstacle, or infinity if none exist."""
        normalized = as_xz(position)
        if not self.obstacles:
            return float("inf")
        return min(obstacle.distance_to(normalized) for obstacle in self.obstacles)

    def nearby_obstacles(
        self, position: PlanningPosition, radius: float
    ) -> tuple[Obstacle, ...]:
        """Return obstacles within surface distance ``radius`` in scenario order."""
        radius_value = _non_negative(radius, "radius")
        normalized = as_xz(position)
        return tuple(
            obstacle
            for obstacle in self.obstacles
            if obstacle.distance_to(normalized) <= radius_value
        )

    def current_at(self, position: PlanningPosition, time: float) -> np.ndarray:
        """Return planning-plane current velocity ``[v_x, v_z]``."""
        return self.current.velocity_at(position, time)

