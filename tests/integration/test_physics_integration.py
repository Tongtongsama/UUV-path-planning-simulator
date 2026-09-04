import pytest

from core import Trajectory
from simulation.physics_smoke_demo import run_physics_smoke_demo
from simulation.vehicle_config import load_synthetic_parameters


def test_named_synthetic_configuration_loads_with_underactuated_selector():
    parameters = load_synthetic_parameters()
    assert parameters.mass_rb[0, 0] == 20.0
    assert parameters.mass_added[1, 1] == 8.0
    assert not parameters.is_control_admissible([0.0, 1.0, 0.0])


def test_physics_smoke_free_response_and_constant_thrust():
    result = run_physics_smoke_demo(dt=0.05, steps=100)

    assert isinstance(result.free_response, Trajectory)
    assert isinstance(result.constant_thrust, Trajectory)
    assert len(result.free_response) == 101
    assert len(result.constant_thrust) == 101

    free_speeds = [state.twist.u for state in result.free_response]
    assert all(later < earlier for earlier, later in zip(free_speeds, free_speeds[1:]))
    assert 0.0 < free_speeds[-1] < free_speeds[0]

    thrust_final = result.constant_thrust[-1]
    assert thrust_final.pose.x > 0.0
    assert thrust_final.twist.u > 0.0
    assert thrust_final.timestamp == pytest.approx(5.0)
    assert any(abs(state.pose.pitch) > 0.0 for state in result.pitch_response[1:])
    assert result.constant_current[-1] != result.constant_thrust[-1]
    assert result.zero_current_equivalent
    assert result.all_states_inside_boundary


def test_physics_smoke_is_deterministic():
    first = run_physics_smoke_demo(dt=0.1, steps=10)
    second = run_physics_smoke_demo(dt=0.1, steps=10)
    assert first == second
