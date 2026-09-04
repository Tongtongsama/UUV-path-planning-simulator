"""Deterministic fixed-step integrators for Physics v0.4."""

from __future__ import annotations

from dataclasses import dataclass
from numbers import Real
from typing import Protocol

import numpy as np

from physics._validation import finite_real_array, finite_real_scalar
from physics.derivative import StateDerivative3DOF


class DerivativeFunction3DOF(Protocol):
    def __call__(self, reduced_state: np.ndarray) -> StateDerivative3DOF: ...


class Integrator(Protocol):
    def step(
        self,
        derivative: DerivativeFunction3DOF,
        state: np.ndarray,
        dt: Real,
    ) -> np.ndarray: ...


def _validated_state(state: np.ndarray) -> np.ndarray:
    return finite_real_array(state, shape=(6,), name="state")


def _validated_derivative(
    function: DerivativeFunction3DOF, state: np.ndarray
) -> np.ndarray:
    if not callable(function):
        raise TypeError("derivative must be callable")
    result = function(state.copy())
    if not isinstance(result, StateDerivative3DOF):
        raise TypeError("derivative must return StateDerivative3DOF")
    return finite_real_array(
        result.to_numpy(), shape=(6,), name="derivative result"
    )


def _validated_dt(dt: Real) -> float:
    step = finite_real_scalar(dt, name="dt")
    if step <= 0.0:
        raise ValueError("dt must be strictly positive")
    return step


@dataclass(frozen=True)
class EulerIntegrator:
    def step(
        self, derivative: DerivativeFunction3DOF, state: np.ndarray, dt: Real
    ) -> np.ndarray:
        y = _validated_state(state)
        h = _validated_dt(dt)
        result = y + h * _validated_derivative(derivative, y)
        if not np.all(np.isfinite(result)):
            raise ValueError("integration result must be finite")
        return result


@dataclass(frozen=True)
class RK4Integrator:
    def step(
        self, derivative: DerivativeFunction3DOF, state: np.ndarray, dt: Real
    ) -> np.ndarray:
        y = _validated_state(state)
        h = _validated_dt(dt)
        k1 = _validated_derivative(derivative, y)
        k2 = _validated_derivative(derivative, y + 0.5 * h * k1)
        k3 = _validated_derivative(derivative, y + 0.5 * h * k2)
        k4 = _validated_derivative(derivative, y + h * k3)
        result = y + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        if not np.all(np.isfinite(result)):
            raise ValueError("integration result must be finite")
        return result
