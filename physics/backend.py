"""Physics backend protocol and Core/reduced-state boundary mapping."""

from __future__ import annotations

from numbers import Real
from typing import Protocol

import numpy as np

from core import ControlInput, Pose, Twist, VehicleState
from physics._validation import finite_real_array, finite_real_scalar


INACTIVE_TOLERANCE = 1e-12


class PhysicsBackend(Protocol):
    def step(
        self,
        state: VehicleState,
        control: ControlInput,
        current_velocity: np.ndarray,
        dt: float,
    ) -> VehicleState: ...


def extract_reduced_state(state: VehicleState) -> np.ndarray:
    if not isinstance(state, VehicleState):
        raise TypeError(f"Expected VehicleState, got {type(state).__name__}")
    full = finite_real_array(state.to_numpy(), shape=(12,), name="VehicleState")
    if np.any(np.abs(full[[1, 3, 5, 7, 9, 11]]) > INACTIVE_TOLERANCE):
        raise ValueError("VehicleState contains non-zero inactive 3-DOF components")
    finite_real_scalar(state.timestamp, name="timestamp")
    return full[[0, 2, 4, 6, 8, 10]].copy()


def rebuild_vehicle_state(reduced_state: np.ndarray, timestamp: Real) -> VehicleState:
    state = finite_real_array(reduced_state, shape=(6,), name="reduced_state")
    time = finite_real_scalar(timestamp, name="timestamp")
    return VehicleState(
        pose=Pose(x=state[0], z=state[1], pitch=state[2]),
        twist=Twist(u=state[3], w=state[4], q=state[5]),
        timestamp=time,
    )


def extract_reduced_control(control: ControlInput) -> np.ndarray:
    if not isinstance(control, ControlInput):
        raise TypeError(f"Expected ControlInput, got {type(control).__name__}")
    full = finite_real_array(control.to_numpy(), shape=(6,), name="ControlInput")
    if np.any(np.abs(full[[1, 3, 5]]) > INACTIVE_TOLERANCE):
        raise ValueError("ControlInput contains non-zero inactive 3-DOF components")
    return full[[0, 2, 4]].copy()
