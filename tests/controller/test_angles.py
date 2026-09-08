import math

import pytest

from controller import wrap_to_pi


@pytest.mark.parametrize(
    ("angle", "expected"),
    [
        (0.0, 0.0),
        (math.pi, -math.pi),
        (-math.pi, -math.pi),
        (3.0 * math.pi, -math.pi),
        (-3.0 * math.pi, -math.pi),
        (math.pi + 0.2, -math.pi + 0.2),
    ],
)
def test_wrap_to_pi_half_open_contract(angle, expected):
    assert wrap_to_pi(angle) == pytest.approx(expected)


@pytest.mark.parametrize("bad", [True, float("nan"), float("inf"), "0"])
def test_wrap_to_pi_rejects_invalid_values(bad):
    with pytest.raises((TypeError, ValueError)):
        wrap_to_pi(bad)
