"""Contract tests for planning-plane current fields."""

from pathlib import Path as PathLib
import sys

import numpy as np
import pytest

PROJECT_ROOT = PathLib(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from core import Pose
from environment import ConstantCurrent, CurrentField, NoCurrent


def test_no_current_returns_float64_zero_vector() -> None:
    current = NoCurrent()

    velocity = current.velocity_at(Pose(x=1.0, y=99.0, z=-2.0), time=10.0)

    assert isinstance(current, CurrentField)
    assert velocity.shape == (2,)
    assert velocity.dtype == np.float64
    np.testing.assert_array_equal(velocity, [0.0, 0.0])


def test_constant_current_is_independent_of_position_and_time() -> None:
    current = ConstantCurrent([0.2, -0.1])

    first = current.velocity_at([0.0, 0.0], time=0.0)
    second = current.velocity_at(Pose(x=100.0, y=50.0, z=-30.0), time=99.0)

    np.testing.assert_array_equal(first, [0.2, -0.1])
    np.testing.assert_array_equal(second, [0.2, -0.1])


def test_constant_current_does_not_retain_or_expose_mutable_array() -> None:
    source = np.array([0.3, 0.4])
    current = ConstantCurrent(source)
    source[:] = 9.0

    returned = current.velocity
    returned[:] = -1.0

    np.testing.assert_array_equal(current.velocity, [0.3, 0.4])


@pytest.mark.parametrize(
    "velocity",
    [[1.0], [1.0, 2.0, 3.0], [np.nan, 0.0], [0.0, np.inf]],
)
def test_constant_current_rejects_invalid_velocity(velocity: list[float]) -> None:
    with pytest.raises(ValueError):
        ConstantCurrent(velocity)


@pytest.mark.parametrize("time", [np.nan, np.inf, -np.inf])
def test_current_query_rejects_non_finite_time(time: float) -> None:
    with pytest.raises(ValueError, match="time must be finite"):
        NoCurrent().velocity_at([0.0, 0.0], time=time)


def test_current_query_rejects_invalid_position() -> None:
    with pytest.raises(ValueError, match="shape"):
        ConstantCurrent([0.2, 0.0]).velocity_at([1.0], time=0.0)

