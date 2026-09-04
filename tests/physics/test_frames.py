import numpy as np
import pytest

from physics import body_to_world_velocity_matrix, world_current_to_body


@pytest.mark.parametrize(
    ("pitch", "velocity", "expected"),
    [
        (0.0, [2.0, 0.0, 0.0], [2.0, 0.0, 0.0]),
        (0.0, [0.0, 2.0, 0.0], [0.0, -2.0, 0.0]),
        (np.pi / 2.0, [2.0, 0.0, 0.0], [0.0, 2.0, 0.0]),
        (-np.pi / 2.0, [2.0, 0.0, 0.0], [0.0, -2.0, 0.0]),
    ],
)
def test_frame_audit_cases(pitch, velocity, expected):
    actual = body_to_world_velocity_matrix(pitch) @ velocity
    assert np.allclose(actual, expected, atol=1e-12)


def test_upward_world_current_is_negative_body_heave_at_zero_pitch():
    assert np.array_equal(world_current_to_body([0, 3], 0), [0.0, -3.0])


def test_current_transform_is_self_inverse():
    pitch = 0.37
    current = np.array([1.2, -0.4])
    transform = body_to_world_velocity_matrix(pitch)[:2, :2]
    body = world_current_to_body(current, pitch)
    assert np.allclose(transform @ body, current)


@pytest.mark.parametrize("bad", [[1.0], [1.0, 2.0, 3.0], [1.0, np.nan], [True, False]])
def test_current_rejects_invalid_input(bad):
    with pytest.raises(ValueError):
        world_current_to_body(bad, 0.0)
