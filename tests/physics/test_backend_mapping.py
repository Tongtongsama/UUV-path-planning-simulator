import numpy as np
import pytest

from core import ControlInput, Pose, Twist, VehicleState
from physics import extract_reduced_control, extract_reduced_state, rebuild_vehicle_state


def test_state_round_trip_uses_normative_order_and_zeroes_inactive_dofs():
    state = VehicleState(Pose(x=1.0, z=2.0, pitch=3.0), Twist(u=4.0, w=5.0, q=6.0), 7.0)
    reduced = extract_reduced_state(state)
    assert np.array_equal(reduced, [1, 2, 3, 4, 5, 6])
    rebuilt = rebuild_vehicle_state(reduced, 8.0)
    assert np.array_equal(rebuilt.to_numpy(), [1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0])
    assert rebuilt.timestamp == 8.0


@pytest.mark.parametrize(
    "state",
    [
        VehicleState(Pose(y=1e-6), Twist.zero()),
        VehicleState(Pose.zero(), Twist(v=1e-6)),
        VehicleState(Pose(x=np.nan), Twist.zero()),
    ],
)
def test_state_mapping_rejects_inactive_or_nonfinite_values(state):
    with pytest.raises(ValueError):
        extract_reduced_state(state)


def test_control_mapping_uses_normative_order():
    assert np.array_equal(
        extract_reduced_control(ControlInput(tau_x=1, tau_z=2, tau_m=3)),
        [1, 2, 3],
    )


def test_control_mapping_rejects_inactive_force():
    with pytest.raises(ValueError):
        extract_reduced_control(ControlInput(tau_y=1e-6))
