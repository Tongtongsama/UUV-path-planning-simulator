"""Validated derivative value type for continuous 3-DOF dynamics."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from physics._validation import finite_real_array, readonly


@dataclass(frozen=True)
class StateDerivative3DOF:
    """Configuration and body-velocity derivatives in normative order."""

    eta_dot: np.ndarray
    nu_dot: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "eta_dot",
            readonly(finite_real_array(self.eta_dot, shape=(3,), name="eta_dot")),
        )
        object.__setattr__(
            self,
            "nu_dot",
            readonly(finite_real_array(self.nu_dot, shape=(3,), name="nu_dot")),
        )

    def to_numpy(self) -> np.ndarray:
        """Return an independent ``[eta_dot,nu_dot]`` float64 vector."""
        return np.concatenate((self.eta_dot, self.nu_dot)).astype(
            np.float64, copy=False
        )
