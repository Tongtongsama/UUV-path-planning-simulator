"""Approved vertical-plane 3-DOF dynamics and Python reference backend."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from core import ControlInput, VehicleState
from physics._validation import finite_real_array, finite_real_scalar, readonly
from physics.backend import (
    extract_reduced_control,
    extract_reduced_state,
    rebuild_vehicle_state,
)
from physics.derivative import StateDerivative3DOF
from physics.frames import body_to_world_velocity_matrix, world_current_to_body
from physics.integrators import Integrator, RK4Integrator
from physics.parameters import UUV3DOFParameters


def rigid_body_coriolis(parameters: UUV3DOFParameters, nu: np.ndarray) -> np.ndarray:
    """Return the approved reduced rigid-body Coriolis matrix."""
    velocity = finite_real_array(nu, shape=(3,), name="nu")
    mass = parameters.mass_rb[0, 0]
    q = velocity[2]
    return readonly(
        np.array(
            [[0.0, mass * q, 0.0], [-mass * q, 0.0, 0.0], [0.0, 0.0, 0.0]],
            dtype=np.float64,
        )
    )


def added_mass_coriolis(
    parameters: UUV3DOFParameters, nu_relative: np.ndarray
) -> np.ndarray:
    """Return the approved reduced added-mass Coriolis matrix."""
    velocity = finite_real_array(
        nu_relative, shape=(3,), name="nu_relative"
    )
    added_u, added_w, _ = np.diag(parameters.mass_added)
    u_relative, w_relative, _ = velocity
    return readonly(
        np.array(
            [
                [0.0, 0.0, added_w * w_relative],
                [0.0, 0.0, -added_u * u_relative],
                [-added_w * w_relative, added_u * u_relative, 0.0],
            ],
            dtype=np.float64,
        )
    )


@dataclass(frozen=True)
class Python3DOFDynamics:
    """Continuous dynamics implementing the approved Physics v0.4 equation."""

    parameters: UUV3DOFParameters

    def __post_init__(self) -> None:
        if not isinstance(self.parameters, UUV3DOFParameters):
            raise TypeError("parameters must be UUV3DOFParameters")

    def derivatives(
        self,
        reduced_state: np.ndarray,
        reduced_control: np.ndarray,
        current_velocity: np.ndarray,
    ) -> StateDerivative3DOF:
        state = finite_real_array(
            reduced_state, shape=(6,), name="reduced_state"
        )
        control = finite_real_array(
            reduced_control, shape=(3,), name="reduced_control"
        )
        current = finite_real_array(
            current_velocity, shape=(2,), name="current_velocity"
        )
        if not self.parameters.is_control_admissible(control):
            raise ValueError("control is outside the input_matrix selector subspace")

        pitch = state[2]
        nu = state[3:6]
        current_body = world_current_to_body(current, pitch)
        nu_relative = nu - np.array(
            [current_body[0], current_body[1], 0.0], dtype=np.float64
        )

        eta_dot = body_to_world_velocity_matrix(pitch) @ nu
        q = nu[2]
        current_body_dot = np.array(
            [-q * current_body[1], q * current_body[0], 0.0],
            dtype=np.float64,
        )
        damping = (
            self.parameters.damping_linear @ nu_relative
            + self.parameters.damping_quadratic
            @ (np.abs(nu_relative) * nu_relative)
        )
        restoring = np.array(
            [0.0, 0.0, self.parameters.restoring_pitch_coefficient * np.sin(pitch)],
            dtype=np.float64,
        )
        rhs = (
            self.parameters.input_matrix @ control
            - rigid_body_coriolis(self.parameters, nu) @ nu
            - added_mass_coriolis(self.parameters, nu_relative) @ nu_relative
            - damping
            - restoring
            + self.parameters.mass_added @ current_body_dot
        )
        nu_dot = np.linalg.solve(self.parameters.mass, rhs)
        return StateDerivative3DOF(eta_dot=eta_dot, nu_dot=nu_dot)


@dataclass(frozen=True)
class Python3DOFBackend:
    """Core-facing fixed-step Python backend; RK4 is the default integrator."""

    parameters: UUV3DOFParameters
    integrator: Integrator = field(default_factory=RK4Integrator)
    _dynamics: Python3DOFDynamics = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if not isinstance(self.parameters, UUV3DOFParameters):
            raise TypeError("parameters must be UUV3DOFParameters")
        if not callable(getattr(self.integrator, "step", None)):
            raise TypeError("integrator must satisfy the Integrator protocol")
        object.__setattr__(self, "_dynamics", Python3DOFDynamics(self.parameters))

    def step(
        self,
        state: VehicleState,
        control: ControlInput,
        current_velocity: np.ndarray,
        dt: float,
    ) -> VehicleState:
        reduced_state = extract_reduced_state(state)
        reduced_control = extract_reduced_control(control)
        current = finite_real_array(
            current_velocity, shape=(2,), name="current_velocity"
        )
        step_size = finite_real_scalar(dt, name="dt")
        if step_size <= 0.0:
            raise ValueError("dt must be strictly positive")
        if not self.parameters.is_control_admissible(reduced_control):
            raise ValueError("control is outside the input_matrix selector subspace")

        def derivative(stage_state: np.ndarray) -> StateDerivative3DOF:
            return self._dynamics.derivatives(stage_state, reduced_control, current)

        result = self.integrator.step(derivative, reduced_state, step_size)
        timestamp = state.timestamp + step_size
        if not np.isfinite(timestamp):
            raise ValueError("result timestamp must be finite")
        return rebuild_vehicle_state(result, timestamp)
