import numpy as np
import pytest

from physics import StateDerivative3DOF


def test_derivative_copies_and_freezes_components():
    source = np.array([1.0, 2.0, 3.0])
    derivative = StateDerivative3DOF(source, [4, 5, 6])
    source[0] = 99.0

    assert derivative.eta_dot.dtype == np.float64
    assert derivative.eta_dot[0] == 1.0
    assert not derivative.eta_dot.flags.writeable
    assert not derivative.nu_dot.flags.writeable


def test_to_numpy_is_independent_and_normatively_ordered():
    derivative = StateDerivative3DOF([1, 2, 3], [4, 5, 6])
    result = derivative.to_numpy()
    result[0] = 99.0
    assert np.array_equal(derivative.to_numpy(), [1, 2, 3, 4, 5, 6])


@pytest.mark.parametrize("bad", [[1, 2], [1, 2, np.inf], [True, False, True]])
def test_derivative_rejects_invalid_components(bad):
    with pytest.raises(ValueError):
        StateDerivative3DOF(bad, [0, 0, 0])
