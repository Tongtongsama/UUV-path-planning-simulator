"""Deterministic closed-loop Control v0.5 and Physics v0.4 demonstration."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from controller import CascadedPID3DOFController
from core import ControlInput, Pose, Trajectory, Twist, VehicleState
from physics import Python3DOFBackend
from simulation.controller_config import load_baseline_controller_parameters
from simulation.vehicle_config import load_synthetic_parameters


@dataclass(frozen=True)
class ClosedLoopRun:
    trajectory: Trajectory
    controls: tuple[ControlInput, ...]


@dataclass(frozen=True)
class ControlTrackingResult:
    reference: VehicleState
    no_current: ClosedLoopRun
    constant_current: ClosedLoopRun


def _run(
    initial: VehicleState,
    reference: VehicleState,
    current_velocity: np.ndarray,
    *,
    dt: float,
    steps: int,
) -> ClosedLoopRun:
    if isinstance(steps, bool) or not isinstance(steps, int):
        raise TypeError("steps must be an integer")
    if steps < 1:
        raise ValueError("steps must be positive")
    controller = CascadedPID3DOFController(load_baseline_controller_parameters())
    physics = Python3DOFBackend(load_synthetic_parameters())
    states = [initial]
    controls = []
    state = initial
    for _ in range(steps):
        control = controller.step(state, reference, dt)
        state = physics.step(state, control, current_velocity, dt)
        controls.append(control)
        states.append(state)
    return ClosedLoopRun(Trajectory.from_list(states), tuple(controls))


def run_control_tracking_demo(
    dt: float = 0.05, steps: int = 1200
) -> ControlTrackingResult:
    initial = VehicleState(
        pose=Pose(x=5.0, z=-15.0),
        twist=Twist.zero(),
        timestamp=0.0,
    )
    reference = VehicleState(
        pose=Pose(x=15.0, z=-5.0),
        twist=Twist.zero(),
        timestamp=0.0,
    )
    return ControlTrackingResult(
        reference=reference,
        no_current=_run(initial, reference, np.zeros(2), dt=dt, steps=steps),
        constant_current=_run(
            initial,
            reference,
            np.array([0.2, 0.0]),
            dt=dt,
            steps=steps,
        ),
    )


def _position_error(state: VehicleState, reference: VehicleState) -> float:
    return float(
        np.hypot(reference.pose.x - state.pose.x, reference.pose.z - state.pose.z)
    )


def main() -> None:
    result = run_control_tracking_demo()
    initial = result.no_current.trajectory[0]
    print("Control v0.5 tracking demo completed")
    print(f"Initial position error: {_position_error(initial, result.reference):.3f} m")
    for label, run in (
        ("No current", result.no_current),
        ("Constant current", result.constant_current),
    ):
        final = run.trajectory[-1]
        maximum_force = max(abs(control.tau_x) for control in run.controls)
        maximum_moment = max(abs(control.tau_m) for control in run.controls)
        print(
            f"{label}: final error={_position_error(final, result.reference):.3f} m, "
            f"pitch={final.pose.pitch:.3f} rad, "
            f"max|tau_x|={maximum_force:.3f} N, "
            f"max|tau_m|={maximum_moment:.3f} N m"
        )


if __name__ == "__main__":
    main()
