import copy

import numpy as np
import pytest

from controller import CascadedPID3DOFController, CascadedPID3DOFParameters
from core import Pose, Twist, VehicleState


def parameters(**overrides):
    values = dict(
        position_to_surge_gain=1.0,
        depth_to_pitch_gain=0.5,
        maximum_surge_reference=2.0,
        maximum_pitch_correction=0.5,
        surge_kp=10.0,
        surge_ki=2.0,
        pitch_kp=8.0,
        pitch_ki=1.0,
        pitch_rate_kd=3.0,
        maximum_tau_x=20.0,
        maximum_tau_m=8.0,
        surge_integral_limit=4.0,
        pitch_integral_limit=1.0,
    )
    values.update(overrides)
    return CascadedPID3DOFParameters(**values)


def state(*, x=0.0, z=0.0, pitch=0.0, u=0.0, w=0.0, q=0.0):
    return VehicleState(Pose(x=x, z=z, pitch=pitch), Twist(u=u, w=w, q=q), 0.0)


def test_zero_error_from_reset_produces_zero_complete_control():
    controller = CascadedPID3DOFController(parameters())
    output = controller.step(state(), state(), 0.1)
    assert np.array_equal(output.to_numpy(), np.zeros(6))


def test_forward_and_above_reference_have_required_signs():
    controller = CascadedPID3DOFController(parameters())
    output = controller.step(state(), state(x=1.0, z=1.0), 0.1)
    assert output.tau_x > 0.0
    assert output.tau_m > 0.0
    assert output.tau_y == output.tau_z == output.tau_k == output.tau_n == 0.0


def test_pitch_rate_feedback_opposes_positive_rate():
    controller = CascadedPID3DOFController(parameters())
    output = controller.step(state(q=1.0), state(), 0.1)
    assert output.tau_m < 0.0


def test_outputs_are_limited_and_windup_is_blocked():
    controller = CascadedPID3DOFController(parameters(maximum_tau_x=1.0, maximum_tau_m=1.0))
    output = controller.step(state(), state(x=100.0, z=100.0), 0.5)
    assert output.tau_x == 1.0
    assert output.tau_m == 1.0
    assert controller.memory.surge_error_integral == 0.0
    assert controller.memory.pitch_error_integral == 0.0


def test_integrals_are_clipped_and_reset_to_exact_zero():
    controller = CascadedPID3DOFController(
        parameters(surge_kp=0.0, pitch_kp=0.0, pitch_rate_kd=0.0)
    )
    for _ in range(100):
        controller.step(state(), state(u=1.0, pitch=0.2), 0.1)
    assert controller.memory.surge_error_integral == 4.0
    assert controller.memory.pitch_error_integral == 1.0
    controller.reset()
    assert controller.memory.surge_error_integral == 0.0
    assert controller.memory.pitch_error_integral == 0.0


def test_invalid_call_leaves_memory_unchanged():
    controller = CascadedPID3DOFController(parameters())
    controller.step(state(), state(u=0.5), 0.1)
    before = controller.memory
    with pytest.raises(ValueError):
        controller.step(state(), state(), 0.0)
    assert controller.memory == before


def test_inactive_state_is_rejected_without_memory_change():
    controller = CascadedPID3DOFController(parameters())
    invalid = VehicleState(Pose(y=1.0), Twist.zero(), 0.0)
    with pytest.raises(ValueError, match="inactive"):
        controller.step(invalid, state(), 0.1)
    assert controller.memory.surge_error_integral == 0.0


def test_inputs_are_not_mutated_and_sequences_are_deterministic():
    current = state(x=0.2, z=-1.0, pitch=0.1, u=0.3, q=-0.1)
    reference = state(x=2.0, z=0.5, pitch=0.0, u=0.5)
    snapshots = copy.deepcopy((current, reference))
    first = CascadedPID3DOFController(parameters())
    second = CascadedPID3DOFController(parameters())
    first_outputs = [first.step(current, reference, 0.1) for _ in range(5)]
    second_outputs = [second.step(current, reference, 0.1) for _ in range(5)]
    assert first_outputs == second_outputs
    assert (current, reference) == snapshots
