"""WGS84 local tangent-plane conversion for chart-derived planning planes.

The project plans in ENU ``x-z``; this module keeps the vertical axis implicit
and exposes only the horizontal local tangent plane.  The approximation is
intended for chart-sized areas (a few kilometres), not global navigation.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

WGS84_A = 6378137.0
WGS84_E2 = 6.6943799901413165e-3


def _finite(value: float, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be a number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


@dataclass(frozen=True, slots=True)
class Origin:
    """WGS84 local tangent-plane origin."""
    latitude: float
    longitude: float
    altitude_m: float = 0.0

    def __post_init__(self) -> None:
        lat = _finite(self.latitude, "latitude")
        lon = _finite(self.longitude, "longitude")
        alt = _finite(self.altitude_m, "altitude_m")
        if not -90.0 <= lat <= 90.0:
            raise ValueError("latitude must be in [-90, 90]")
        if not -180.0 <= lon <= 180.0:
            raise ValueError("longitude must be in [-180, 180]")
        object.__setattr__(self, "latitude", lat)
        object.__setattr__(self, "longitude", lon)
        object.__setattr__(self, "altitude_m", alt)

    def as_json(self) -> dict[str, float]:
        return {"latitude": self.latitude, "longitude": self.longitude,
                "altitude_m": self.altitude_m}


class LocalTangentPlane:
    """Small-area WGS84 -> local ENU ``(x, z)`` conversion.

    ``x`` is east and ``z`` is north, matching the project's planning plane.
    The meridian/prime-vertical radii are evaluated at the origin, which keeps
    the round trip linear and deterministic for chart-scale extents.
    """
    def __init__(self, origin: Origin) -> None:
        self.origin = origin
        phi = math.radians(origin.latitude)
        sin_phi = math.sin(phi)
        denom = 1.0 - WGS84_E2 * sin_phi * sin_phi
        self._n = WGS84_A / math.sqrt(denom)
        self._m = WGS84_A * (1.0 - WGS84_E2) / (denom ** 1.5)
        self._cos_phi = math.cos(phi)

    def to_enu(self, latitude: float, longitude: float) -> tuple[float, float]:
        lat = _finite(latitude, "latitude")
        lon = _finite(longitude, "longitude")
        dlat = math.radians(lat - self.origin.latitude)
        dlon = math.radians(_wrap_longitude(lon - self.origin.longitude))
        x = dlon * self._n * self._cos_phi
        z = dlat * self._m
        return float(x), float(z)

    def to_geodetic(self, x: float, z: float) -> tuple[float, float]:
        x = _finite(x, "x")
        z = _finite(z, "z")
        lat = self.origin.latitude + math.degrees(z / self._m)
        lon = self.origin.longitude + math.degrees(x / (self._n * self._cos_phi))
        return float(lat), float(_wrap_longitude(lon))


def _wrap_longitude(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0
