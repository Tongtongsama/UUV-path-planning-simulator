import pytest

from controller import CascadedPID3DOFParameters


def values():
    return dict(
        position_to_surge_gain=0.3,
        depth_to_pitch_gain=0.2,
        maximum_surge_reference=1.5,
        maximum_pitch_correction=0.5,
        surge_kp=10.0,
        surge_ki=1.0,
        pitch_kp=8.0,
        pitch_ki=0.5,
        pitch_rate_kd=3.0,
        maximum_tau_x=20.0,
        maximum_tau_m=8.0,
        surge_integral_limit=4.0,
        pitch_integral_limit=1.0,
    )


def test_parameters_accept_zero_gains_and_convert_real_values():
    data = values()
    data["surge_ki"] = 0
    result = CascadedPID3DOFParameters(**data)
    assert result.surge_ki == 0.0
    assert isinstance(result.surge_ki, float)


@pytest.mark.parametrize("bad", [-1.0, float("nan"), float("inf"), True, "1"])
def test_parameters_reject_invalid_gain(bad):
    data = values()
    data["surge_kp"] = bad
    with pytest.raises((TypeError, ValueError)):
        CascadedPID3DOFParameters(**data)


@pytest.mark.parametrize(
    "name",
    [
        "maximum_surge_reference",
        "maximum_pitch_correction",
        "maximum_tau_x",
        "maximum_tau_m",
        "surge_integral_limit",
        "pitch_integral_limit",
    ],
)
def test_limits_must_be_strictly_positive(name):
    data = values()
    data[name] = 0.0
    with pytest.raises(ValueError):
        CascadedPID3DOFParameters(**data)
