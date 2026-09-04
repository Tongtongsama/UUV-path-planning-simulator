"""Small deterministic Physics v0.4 free-response and thrust scenarios."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from core import ControlInput, Pose, Trajectory, Twist, VehicleState
from environment import NoCurrent, Scenario
from physics import Python3DOFBackend
from simulation.vehicle_config import load_synthetic_parameters


@dataclass(frozen=True)
class PhysicsSmokeResult:
    free_response: Trajectory
    constant_thrust: Trajectory
    pitch_response: Trajectory
    constant_current: Trajectory
    zero_current_equivalent: bool
    all_states_inside_boundary: bool


DEFAULT_WORLD_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "scenarios"
    / "simple_static_world.yaml"
)


def _simulate(
    backend: Python3DOFBackend,
    initial: VehicleState,
    control: ControlInput,
    *,
    dt: float,
    steps: int,
) -> Trajectory:
    if isinstance(steps, bool) or not isinstance(steps, int):
        raise TypeError("steps must be an integer")
    if steps < 1:
        raise ValueError("steps must be positive")
    states = [initial]
    state = initial
    for _ in range(steps):
        state = backend.step(state, control, np.zeros(2), dt)
        states.append(state)
    return Trajectory.from_list(states)


def run_physics_smoke_demo(dt: float = 0.05, steps: int = 200) -> PhysicsSmokeResult:
    """Exercise the stable Physics step boundary and Environment queries."""
    backend = Python3DOFBackend(load_synthetic_parameters())
    free_initial = VehicleState(
        pose=Pose(x=5.0, z=-2.0), twist=Twist(u=1.0), timestamp=0.0
    )
    common_initial = VehicleState(Pose(x=5.0, z=-2.0), Twist.zero(), 0.0)
    free_response = _simulate(
        backend, free_initial, ControlInput.zero(), dt=dt, steps=steps
    )
    constant_thrust = _simulate(
        backend,
        common_initial,
        ControlInput(tau_x=10.0),
        dt=dt,
        steps=steps,
    )
    pitch_response = _simulate(
        backend,
        common_initial,
        ControlInput(tau_m=1.0),
        dt=dt,
        steps=steps,
    )

    scenario = Scenario.from_yaml(DEFAULT_WORLD_CONFIG)
    current_states = [common_initial]
    current_state = common_initial
    for _ in range(steps):
        current = scenario.world.current_at(current_state.pose, current_state.timestamp)
        current_state = backend.step(
            current_state, ControlInput(tau_x=10.0), current, dt
        )
        current_states.append(current_state)
    constant_current = Trajectory.from_list(current_states)

    no_current = NoCurrent().velocity_at(common_initial.pose, common_initial.timestamp)
    zero_from_field = backend.step(
        common_initial, ControlInput(tau_x=1.0), no_current, dt
    )
    zero_explicit = backend.step(
        common_initial, ControlInput(tau_x=1.0), np.zeros(2), dt
    )
    trajectories = (free_response, constant_thrust, pitch_response, constant_current)
    all_inside = all(
        scenario.world.contains(state.pose) for trajectory in trajectories for state in trajectory
    )

    return PhysicsSmokeResult(
        free_response=free_response,
        constant_thrust=constant_thrust,
        pitch_response=pitch_response,
        constant_current=constant_current,
        zero_current_equivalent=zero_from_field == zero_explicit,
        all_states_inside_boundary=all_inside,
    )


def main() -> None:
    result = run_physics_smoke_demo()
    free_final = result.free_response[-1]
    thrust_final = result.constant_thrust[-1]
    print("Physics v0.4 smoke demo completed")
    print(f"Free response: u 1.000 -> {free_final.twist.u:.3f} m/s")
    print(
        "Constant thrust: "
        f"x={thrust_final.pose.x:.3f} m, u={thrust_final.twist.u:.3f} m/s"
    )
    print(f"Pitch response peak pitch: {max(abs(s.pose.pitch) for s in result.pitch_response):.3f} rad")
    print(f"NoCurrent equals explicit zero: {result.zero_current_equivalent}")
    print(f"All trajectories inside boundary: {result.all_states_inside_boundary}")


if __name__ == "__main__":
    main()
