import numpy as np
import pytest

from physics import EulerIntegrator, RK4Integrator, StateDerivative3DOF


def derivative(values):
    values = np.asarray(values, dtype=np.float64)
    return StateDerivative3DOF(values[:3], values[3:])


def test_euler_one_step_matches_hand_calculation():
    state = np.zeros(6)
    result = EulerIntegrator().step(lambda _: derivative(np.ones(6)), state, 0.25)
    assert np.array_equal(result, np.full(6, 0.25))


def test_rk4_integrates_constant_derivative_exactly():
    state = np.arange(6.0)
    result = RK4Integrator().step(lambda _: derivative(np.full(6, 2.0)), state, 0.25)
    assert np.array_equal(result, state + 0.5)


def test_rk4_is_more_accurate_for_exponential_case():
    initial = np.ones(6)
    dt = 0.2
    exact = np.full(6, np.exp(dt))
    euler_error = np.linalg.norm(EulerIntegrator().step(lambda y: derivative(y), initial, dt) - exact)
    rk4_error = np.linalg.norm(RK4Integrator().step(lambda y: derivative(y), initial, dt) - exact)
    assert rk4_error < euler_error


@pytest.mark.parametrize("dt", [0.0, -1.0, np.inf, np.nan, True])
def test_integrators_reject_invalid_dt(dt):
    with pytest.raises((TypeError, ValueError)):
        RK4Integrator().step(lambda y: derivative(y), np.zeros(6), dt)


def test_integrator_rejects_invalid_derivative_result():
    with pytest.raises(TypeError):
        EulerIntegrator().step(lambda _: np.zeros(5), np.zeros(6), 0.1)
