"""Allowed operating-region domain wrapper backed internally by Shapely."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field

from shapely.geometry import Point, Polygon, box
from shapely.geometry.base import BaseGeometry

from environment.coordinates import PlanningPosition, as_xz
from environment.obstacle import _finite_number, _non_negative, _validated_polygon


def _point(position: PlanningPosition) -> Point:
    x, z = as_xz(position)
    return Point(float(x), float(z))


@dataclass(frozen=True, slots=True, eq=False)
class Boundary:
    """Polygonal region in which the UUV footprint must remain."""

    _geometry: BaseGeometry = field(repr=False)

    def __post_init__(self) -> None:
        if not isinstance(self._geometry, BaseGeometry):
            raise TypeError("Boundary geometry must be a Shapely geometry")
        if self._geometry.is_empty or self._geometry.area <= 0.0:
            raise ValueError("Boundary geometry must have non-zero area")
        if not self._geometry.is_valid:
            raise ValueError("Boundary geometry is invalid")

    @classmethod
    def rectangle(
        cls,
        *,
        x_min: float,
        x_max: float,
        z_min: float,
        z_max: float,
    ) -> "Boundary":
        """Create an axis-aligned rectangular operating region."""
        xmin = _finite_number(x_min, "x_min")
        xmax = _finite_number(x_max, "x_max")
        zmin = _finite_number(z_min, "z_min")
        zmax = _finite_number(z_max, "z_max")
        if xmin >= xmax:
            raise ValueError("x_min must be less than x_max")
        if zmin >= zmax:
            raise ValueError("z_min must be less than z_max")
        return cls(box(xmin, zmin, xmax, zmax))

    @classmethod
    def polygon(cls, vertices: Sequence[Sequence[float]]) -> "Boundary":
        """Create a polygonal operating region."""
        return cls(_validated_polygon(vertices))

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        """Return backend-independent ``(x_min, z_min, x_max, z_max)`` bounds."""
        return tuple(float(value) for value in self._geometry.bounds)  # type: ignore[return-value]

    def contains(self, position: PlanningPosition, clearance: float = 0.0) -> bool:
        """Return whether a point or circular footprint is inside the boundary.

        Boundary contact is valid. At positive clearance the entire buffered
        footprint must be covered by the allowed polygon.
        """
        clearance_value = _non_negative(clearance, "clearance")
        point = _point(position)
        footprint = point if clearance_value == 0.0 else point.buffer(clearance_value)
        return bool(self._geometry.covers(footprint))

    def distance_to_boundary(self, position: PlanningPosition) -> float:
        """Return minimum unsigned distance from a point to the boundary edge."""
        return float(self._geometry.boundary.distance(_point(position)))

