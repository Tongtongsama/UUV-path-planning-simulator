"""Internal validation helpers for Control v0.5."""

from __future__ import annotations

from numbers import Real

import numpy as np

from core import VehicleState


INACTIVE_TOLERANCE = 1e-12


def finite_scalar(value: Real, *, name: str) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a real scalar")
    result = float(value)
    if not np.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def validate_state(state: VehicleState, *, name: str) -> None:
    if not isinstance(state, VehicleState):
        raise TypeError(f"{name} must be VehicleState")
    try:
        values = np.asarray(state.to_numpy())
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must contain real numeric values") from exc
    if values.dtype.kind not in "iuf" or values.shape != (12,):
        raise ValueError(f"{name} must contain twelve real numeric values")
    numeric = np.asarray(values, dtype=np.float64)
    if not np.all(np.isfinite(numeric)):
        raise ValueError(f"{name} must contain only finite values")
    finite_scalar(state.timestamp, name=f"{name}.timestamp")
    if np.any(np.abs(numeric[[1, 3, 5, 7, 9, 11]]) > INACTIVE_TOLERANCE):
        raise ValueError(f"{name} contains non-zero inactive 3-DOF components")
