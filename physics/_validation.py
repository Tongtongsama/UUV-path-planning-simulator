"""Internal numerical validation helpers for Physics v0.4."""

from __future__ import annotations

from collections.abc import Sequence
from numbers import Real

import numpy as np


def finite_real_array(
    values: Sequence[Real] | np.ndarray,
    *,
    shape: tuple[int, ...],
    name: str,
) -> np.ndarray:
    """Return an independent float64 array after strict public-input checks."""
    try:
        candidate = np.asarray(values)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must contain real numeric values") from exc

    if candidate.dtype.kind not in "iuf":
        raise ValueError(f"{name} must contain non-boolean real numeric values")
    if candidate.shape != shape:
        raise ValueError(f"Expected {name} shape {shape}, got {candidate.shape}")

    result = np.array(candidate, dtype=np.float64, copy=True)
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result


def finite_real_scalar(value: Real, *, name: str) -> float:
    """Return a finite built-in float while rejecting bool and non-real input."""
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a real scalar")
    result = float(value)
    if not np.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def readonly(array: np.ndarray) -> np.ndarray:
    """Mark an owned NumPy array read-only and return it."""
    array.setflags(write=False)
    return array
