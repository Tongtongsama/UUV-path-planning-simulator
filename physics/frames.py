"""ENU/SNAME transformations for the Physics v0.4 vertical plane."""

from __future__ import annotations

from numbers import Real

import numpy as np

from physics._validation import finite_real_array, finite_real_scalar, readonly


def body_to_world_velocity_matrix(pitch: Real) -> np.ndarray:
    """Return the 3-DOF map from body ``[u,w,q]`` to world rates."""
    theta = finite_real_scalar(pitch, name="pitch")
    cosine = np.cos(theta)
    sine = np.sin(theta)
    return readonly(
        np.array(
            [[cosine, sine, 0.0], [sine, -cosine, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
    )


def world_current_to_body(
    current_velocity: np.ndarray, pitch: Real
) -> np.ndarray:
    """Map world ENU current ``[v_cx,v_cz]`` to body ``[u_c,w_c]``."""
    current = finite_real_array(
        current_velocity, shape=(2,), name="current_velocity"
    )
    transform = body_to_world_velocity_matrix(pitch)[:2, :2]
    return readonly(np.asarray(transform @ current, dtype=np.float64))
