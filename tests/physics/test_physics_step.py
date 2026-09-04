import numpy as np
import pytest

from core import ControlInput, Twist, VehicleState
from physics import EulerIntegrator, Python3DOFBackend, UUV3DOFParameters


def parameters():
    return UUV3DOFParameters(
        mass_rb=np.diag([10.0, 10.0, 4.0]),
        mass_added=np.diag([2.0, 3.0, 1.0]),
        damping_linear=np.zeros((3, 3)),
        damping_quadratic=np.zeros((3, 3)),
        input_matrix=np.diag([1.0, 0.0, 1.0]),
        restoring_pitch_coefficient=0.0,
    )


def test_backend_connects_core_dynamics_and_integrator():
    backend = Python3DOFBackend(parameters(), EulerIntegrator())
    initial = VehicleState.zero(timestamp=2.0)
    result = backend.step(initial, ControlInput(tau_x=12.0), [0.0, 0.0], 0.5)
    assert result is not initial
    assert result.timestamp == initial.timestamp + 0.5
    assert result.twist.u == 0.5
    assert result.pose.x == 0.0


def test_underactuated_backend_rejects_nonzero_heave_force():
    backend = Python3DOFBackend(parameters())
    with pytest.raises(ValueError, match="selector subspace"):
        backend.step(VehicleState.zero(), ControlInput(tau_z=1.0), [0, 0], 0.1)


def test_backend_is_deterministic_and_does_not_mutate_inputs():
    backend = Python3DOFBackend(parameters())
    initial = VehicleState.zero()
    control = ControlInput(tau_x=1.0)
    current = np.array([0.2, -0.1])
    first = backend.step(initial, control, current, 0.1)
    second = backend.step(initial, control, current, 0.1)
    assert first == second
    assert initial == VehicleState.zero()
    assert np.array_equal(current, [0.2, -0.1])


def test_linear_free_surge_matches_analytical_solution():
    value = parameters()
    value = UUV3DOFParameters(
        mass_rb=value.mass_rb,
        mass_added=value.mass_added,
        damping_linear=np.diag([3.0, 0.0, 0.0]),
        damping_quadratic=np.zeros((3, 3)),
        input_matrix=value.input_matrix,
        restoring_pitch_coefficient=0.0,
    )
    backend = Python3DOFBackend(value)
    state = VehicleState(VehicleState.zero().pose, Twist(u=2.0), 0.0)
    dt = 0.01
    for _ in range(100):
        state = backend.step(state, ControlInput.zero(), [0, 0], dt)

    total_surge_mass = 12.0
    expected = 2.0 * np.exp(-(3.0 / total_surge_mass) * 1.0)
    assert state.twist.u == pytest.approx(expected, rel=1e-9)
