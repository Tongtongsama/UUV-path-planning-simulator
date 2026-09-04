"""Canonical validated parameter model for Physics v0.4."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from physics._validation import finite_real_array, finite_real_scalar, readonly


@dataclass(frozen=True)
class UUV3DOFParameters:
    mass_rb: np.ndarray
    mass_added: np.ndarray
    damping_linear: np.ndarray
    damping_quadratic: np.ndarray
    input_matrix: np.ndarray
    restoring_pitch_coefficient: float

    def __post_init__(self) -> None:
        names = (
            "mass_rb",
            "mass_added",
            "damping_linear",
            "damping_quadratic",
            "input_matrix",
        )
        matrices = {
            name: finite_real_array(getattr(self, name), shape=(3, 3), name=name)
            for name in names
        }

        for name in names:
            matrix = matrices[name]
            if not np.array_equal(matrix, np.diag(np.diag(matrix))):
                raise ValueError(f"{name} must be diagonal in Physics v0.4")

        if np.any(np.diag(matrices["mass_rb"]) <= 0.0):
            raise ValueError("mass_rb must be positive definite")
        if matrices["mass_rb"][0, 0] != matrices["mass_rb"][1, 1]:
            raise ValueError("mass_rb surge and heave entries must equal vehicle mass")
        if np.any(np.diag(matrices["mass_added"]) < 0.0):
            raise ValueError("mass_added must be positive semidefinite")
        if np.any(np.diag(matrices["mass_rb"] + matrices["mass_added"]) <= 0.0):
            raise ValueError("combined mass matrix must be positive definite")
        for name in ("damping_linear", "damping_quadratic"):
            if np.any(np.diag(matrices[name]) < 0.0):
                raise ValueError(f"{name} coefficients must be non-negative")
        if not np.all(np.isin(np.diag(matrices["input_matrix"]), (0.0, 1.0))):
            raise ValueError("input_matrix diagonal entries must be 0.0 or 1.0")

        for name, matrix in matrices.items():
            object.__setattr__(self, name, readonly(matrix))
        object.__setattr__(
            self,
            "restoring_pitch_coefficient",
            finite_real_scalar(
                self.restoring_pitch_coefficient,
                name="restoring_pitch_coefficient",
            ),
        )
        if self.restoring_pitch_coefficient < 0.0:
            raise ValueError("restoring_pitch_coefficient must be non-negative")

    @property
    def mass(self) -> np.ndarray:
        """Return an independent combined rigid-body and added-mass matrix."""
        return self.mass_rb + self.mass_added

    def is_control_admissible(
        self, reduced_control: np.ndarray, tolerance: float = 1e-12
    ) -> bool:
        """Return whether control lies in the selector-defined input subspace."""
        control = finite_real_array(
            reduced_control, shape=(3,), name="reduced_control"
        )
        tol = finite_real_scalar(tolerance, name="tolerance")
        if tol < 0.0:
            raise ValueError("tolerance must be non-negative")
        disabled = np.diag(self.input_matrix) == 0.0
        return bool(np.all(np.abs(control[disabled]) <= tol))
