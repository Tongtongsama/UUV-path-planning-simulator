import numpy as np
import pytest

from physics import UUV3DOFParameters


def parameters(**overrides):
    values = {
        "mass_rb": np.diag([10.0, 10.0, 4.0]),
        "mass_added": np.diag([2.0, 3.0, 1.0]),
        "damping_linear": np.diag([1.0, 2.0, 3.0]),
        "damping_quadratic": np.diag([0.1, 0.2, 0.3]),
        "input_matrix": np.diag([1.0, 0.0, 1.0]),
        "restoring_pitch_coefficient": 5.0,
    }
    values.update(overrides)
    return UUV3DOFParameters(**values)


def test_parameters_copy_and_freeze_arrays():
    source = np.diag([10.0, 10.0, 4.0])
    value = parameters(mass_rb=source)
    source[0, 0] = 99.0
    assert value.mass_rb[0, 0] == 10.0
    assert all(not getattr(value, name).flags.writeable for name in (
        "mass_rb", "mass_added", "damping_linear", "damping_quadratic", "input_matrix"
    ))
    assert np.array_equal(value.mass, np.diag([12.0, 13.0, 5.0]))


def test_underactuated_control_admissibility():
    value = parameters()
    assert value.is_control_admissible([1.0, 0.0, 2.0])
    assert value.is_control_admissible([1.0, 1e-12, 2.0])
    assert not value.is_control_admissible([1.0, 1e-6, 2.0])


@pytest.mark.parametrize(
    ("field", "bad"),
    [
        ("mass_rb", np.diag([0.0, 1.0, 1.0])),
        ("mass_rb", np.diag([1.0, 2.0, 1.0])),
        ("mass_added", np.diag([-1.0, 1.0, 1.0])),
        ("damping_linear", np.diag([-1.0, 1.0, 1.0])),
        ("damping_quadratic", np.ones((3, 3))),
        ("input_matrix", np.diag([1.0, 0.5, 1.0])),
    ],
)
def test_parameters_reject_invalid_matrices(field, bad):
    with pytest.raises(ValueError):
        parameters(**{field: bad})


def test_parameters_reject_negative_restoring_coefficient():
    with pytest.raises(ValueError):
        parameters(restoring_pitch_coefficient=-1.0)
