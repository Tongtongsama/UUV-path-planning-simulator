import numpy as np

from physics import (
    Python3DOFDynamics,
    UUV3DOFParameters,
    added_mass_coriolis,
    rigid_body_coriolis,
)


def parameters(*, actuated_heave=True, damping=True):
    return UUV3DOFParameters(
        mass_rb=np.diag([10.0, 10.0, 4.0]),
        mass_added=np.diag([2.0, 3.0, 1.0]),
        damping_linear=np.diag([1.0, 2.0, 3.0]) if damping else np.zeros((3, 3)),
        damping_quadratic=np.diag([0.1, 0.2, 0.3]) if damping else np.zeros((3, 3)),
        input_matrix=np.diag([1.0, float(actuated_heave), 1.0]),
        restoring_pitch_coefficient=5.0,
    )


def test_coriolis_matrices_match_expanded_reduction():
    value = parameters()
    nu = np.array([2.0, 3.0, 0.5])
    assert np.array_equal(
        rigid_body_coriolis(value, nu),
        [[0.0, 5.0, 0.0], [-5.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    )
    assert np.array_equal(
        added_mass_coriolis(value, nu),
        [[0.0, 0.0, 9.0], [0.0, 0.0, -4.0], [-9.0, 4.0, 0.0]],
    )


def test_coriolis_is_skew_symmetric_and_power_neutral():
    value = parameters()
    nu = np.array([2.0, -3.0, 0.5])
    for matrix in (rigid_body_coriolis(value, nu), added_mass_coriolis(value, nu)):
        assert np.array_equal(matrix.T, -matrix)
        assert abs(nu @ matrix @ nu) < 1e-12


def test_zero_state_is_equilibrium_without_input_or_current():
    derivative = Python3DOFDynamics(parameters()).derivatives(
        np.zeros(6), np.zeros(3), np.zeros(2)
    )
    assert np.array_equal(derivative.to_numpy(), np.zeros(6))


def test_axis_aligned_acceleration_matches_scalar_equations():
    dynamics = Python3DOFDynamics(parameters(damping=False))
    derivative = dynamics.derivatives(
        np.zeros(6), np.array([12.0, 13.0, 5.0]), np.zeros(2)
    )
    assert np.allclose(derivative.nu_dot, [1.0, 1.0, 1.0])


def test_linear_and_quadratic_damping_match_scalar_surge_equation():
    value = parameters()
    derivative = Python3DOFDynamics(value).derivatives(
        np.array([0, 0, 0, 2.0, 0, 0]), np.zeros(3), np.zeros(2)
    )
    expected_u_dot = -(1.0 * 2.0 + 0.1 * abs(2.0) * 2.0) / 12.0
    assert np.isclose(derivative.nu_dot[0], expected_u_dot)
    assert np.allclose(derivative.nu_dot[1:], 0.0)


def test_world_current_body_rotation_contributes_added_mass_correction():
    value = parameters(damping=False)
    state = np.array([0, 0, 0, 0, 0, 2.0], dtype=float)
    derivative = Python3DOFDynamics(value).derivatives(
        state, np.zeros(3), np.array([3.0, 0.0])
    )
    # At theta=0: [uc,wc]=[3,0], so nu_c_dot=[0,6,0].
    # Expanded Coriolis terms are included independently by the model.
    nu = state[3:]
    nu_r = np.array([-3.0, 0.0, 2.0])
    expected_rhs = (
        -rigid_body_coriolis(value, nu) @ nu
        -added_mass_coriolis(value, nu_r) @ nu_r
        +value.mass_added @ np.array([0.0, 6.0, 0.0])
    )
    assert np.allclose(derivative.nu_dot, np.linalg.solve(value.mass, expected_rhs))


def test_restoring_moment_points_toward_zero_pitch():
    derivative = Python3DOFDynamics(parameters(damping=False)).derivatives(
        np.array([0, 0, 0.2, 0, 0, 0]), np.zeros(3), np.zeros(2)
    )
    assert derivative.nu_dot[2] < 0.0


def test_damping_is_energy_dissipative():
    value = parameters()
    velocity = np.array([1.2, -0.7, 0.4])
    damping_force = (
        value.damping_linear @ velocity
        + value.damping_quadratic @ (np.abs(velocity) * velocity)
    )
    assert velocity @ damping_force > 0.0
