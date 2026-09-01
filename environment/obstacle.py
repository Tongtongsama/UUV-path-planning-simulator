"""Static obstacle domain wrapper backed internally by Shapely."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any

import numpy as np
from shapely.geometry import Point, Polygon, box
from shapely.geometry.base import BaseGeometry

from environment.coordinates import PlanningPosition, as_xz


def _finite_number(value: object, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be a number") from exc
    if not np.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def _non_negative(value: object, name: str) -> float:
    number = _finite_number(value, name)
    if number < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return number


def _point(position: PlanningPosition) -> Point:
    x, z = as_xz(position)
    return Point(float(x), float(z))


def _validated_polygon(vertices: Sequence[Sequence[float]]) -> Polygon:
    try:
        points = [tuple(as_xz(vertex)) for vertex in vertices]
    except TypeError as exc:
        raise TypeError("Polygon vertices must be a sequence of [x, z] values") from exc
    if len(points) < 3:
        raise ValueError("Polygon requires at least three vertices")

    geometry = Polygon(points)
    if geometry.is_empty or geometry.area <= 0.0:
        raise ValueError("Polygon must have non-zero area")
    if not geometry.is_valid:
        raise ValueError("Polygon geometry is invalid")
    return geometry


@dataclass(frozen=True, slots=True, eq=False)
class Obstacle:
    """A uniquely identified static obstacle in the planning x-z plane."""

    obstacle_id: str
    _geometry: BaseGeometry = field(repr=False)
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not isinstance(self.obstacle_id, str) or not self.obstacle_id.strip():
            raise ValueError("obstacle_id must be a non-empty string")
        if not isinstance(self._geometry, BaseGeometry):
            raise TypeError("Obstacle geometry must be a Shapely geometry")
        if self._geometry.is_empty or self._geometry.area <= 0.0:
            raise ValueError("Obstacle geometry must have non-zero area")
        if not self._geometry.is_valid:
            raise ValueError("Obstacle geometry is invalid")
        if not isinstance(self.metadata, Mapping):
            raise TypeError("metadata must be a mapping")

        object.__setattr__(self, "obstacle_id", self.obstacle_id.strip())
        object.__setattr__(self, "metadata", MappingProxyType(dict(self.metadata)))

    @classmethod
    def circle(
        cls,
        obstacle_id: str,
        center: PlanningPosition,
        radius: float,
        *,
        metadata: Mapping[str, Any] | None = None,
    ) -> "Obstacle":
        """Create a circular obstacle using Shapely's buffered-point backend."""
        radius_value = _finite_number(radius, "radius")
        if radius_value <= 0.0:
            raise ValueError("radius must be positive")
        geometry = _point(center).buffer(radius_value)
        return cls(obstacle_id, geometry, metadata or {})

    @classmethod
    def rectangle(
        cls,
        obstacle_id: str,
        *,
        x_min: float,
        x_max: float,
        z_min: float,
        z_max: float,
        metadata: Mapping[str, Any] | None = None,
    ) -> "Obstacle":
        """Create an axis-aligned rectangular obstacle."""
        xmin = _finite_number(x_min, "x_min")
        xmax = _finite_number(x_max, "x_max")
        zmin = _finite_number(z_min, "z_min")
        zmax = _finite_number(z_max, "z_max")
        if xmin >= xmax:
            raise ValueError("x_min must be less than x_max")
        if zmin >= zmax:
            raise ValueError("z_min must be less than z_max")
        return cls(obstacle_id, box(xmin, zmin, xmax, zmax), metadata or {})

    @classmethod
    def polygon(
        cls,
        obstacle_id: str,
        vertices: Sequence[Sequence[float]],
        *,
        metadata: Mapping[str, Any] | None = None,
    ) -> "Obstacle":
        """Create a polygon obstacle after validating backend geometry."""
        return cls(obstacle_id, _validated_polygon(vertices), metadata or {})

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        """Return backend-independent ``(x_min, z_min, x_max, z_max)`` bounds."""
        return tuple(float(value) for value in self._geometry.bounds)  # type: ignore[return-value]

    def collides(self, position: PlanningPosition, radius: float = 0.0) -> bool:
        """Return whether a circular footprint touches or overlaps the obstacle."""
        radius_value = _non_negative(radius, "radius")
        point = _point(position)
        footprint = point if radius_value == 0.0 else point.buffer(radius_value)
        return bool(self._geometry.intersects(footprint))

    def distance_to(self, position: PlanningPosition) -> float:
        """Return non-negative distance from a point to the obstacle surface."""
        return float(self._geometry.distance(_point(position)))

