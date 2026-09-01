"""Coordinate conversion for the Environment v0.3 vertical planning plane.

Core uses an ENU world frame. Environment v0.3 uses the vertical ``x-z``
planning plane, represented as ``[x, z]``. When passed to Shapely, the second
Shapely coordinate stores project ``z`` and must not be interpreted as
``Pose.y``.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from core import Pose


PlanningPosition = Pose | Sequence[float] | np.ndarray


def _finite_xz(values: object) -> np.ndarray:
    """Return a validated, independent float64 planning-plane vector."""
    try:
        coordinates = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise TypeError("Planning position must contain two numeric values") from exc

    if coordinates.ndim != 1 or coordinates.shape != (2,):
        raise ValueError(
            f"Expected planning position shape (2,), got {coordinates.shape}"
        )
    if not np.all(np.isfinite(coordinates)):
        raise ValueError("Planning position coordinates must be finite")
    return coordinates.copy()


def pose_to_xz(pose: Pose) -> np.ndarray:
    """Map Core ENU ``Pose(x, y, z)`` to planning coordinates ``[x, z]``."""
    if not isinstance(pose, Pose):
        raise TypeError(f"Expected Pose, got {type(pose).__name__}")
    return _finite_xz((pose.x, pose.z))


def as_xz(position: PlanningPosition) -> np.ndarray:
    """Normalize a Core Pose or explicit ``[x, z]`` value for world queries."""
    if isinstance(position, Pose):
        return pose_to_xz(position)
    return _finite_xz(position)


def xz_to_pose(
    position: Sequence[float] | np.ndarray,
    *,
    world_y: float = 0.0,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
) -> Pose:
    """Create a Core ENU Pose from ``[x, z]`` and explicit ignored-axis data."""
    x, z = _finite_xz(position)
    extras = np.asarray((world_y, roll, pitch, yaw), dtype=np.float64)
    if not np.all(np.isfinite(extras)):
        raise ValueError("Pose components must be finite")
    return Pose(
        x=float(x),
        y=float(world_y),
        z=float(z),
        roll=float(roll),
        pitch=float(pitch),
        yaw=float(yaw),
    )

