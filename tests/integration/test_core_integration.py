"""Integration smoke test for the deterministic Core validation pipeline."""

from dataclasses import FrozenInstanceError
from pathlib import Path as PathLib
import sys

import numpy as np
import pytest

PROJECT_ROOT = PathLib(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from core import ControlInput, Path, Pose, Trajectory, Twist, VehicleState
from simulation.core_integration_demo import (
    kinematic_physics_stub,
    run_core_integration_demo,
)


def test_core_integration_pipeline() -> None:
    result = run_core_integration_demo(time_step=1.0)

    # Path is a geometric, immutable, non-empty Pose sequence.
    assert isinstance(result.path, Path)
    assert len(result.path) == 3
    assert result.path.start == result.path[0]
    assert result.path.goal == result.path[-1]
    assert list(iter(result.path)) == list(result.path.poses)
    assert result.path.length() == pytest.approx(2.0 * np.sqrt(5.25))
    assert all(isinstance(pose, Pose) for pose in result.path)

    # Trajectory adds time through VehicleState.timestamp only.
    assert isinstance(result.trajectory, Trajectory)
    assert len(result.trajectory) == len(result.path)
    timestamps = [state.timestamp for state in result.trajectory]
    assert timestamps == [0.0, 1.0, 2.0]
    assert all(np.isfinite(timestamps))
    assert all(t2 > t1 for t1, t2 in zip(timestamps, timestamps[1:]))
    assert result.trajectory[0].pose is result.path[0]
    assert all(isinstance(state.twist, Twist) for state in result.trajectory)

    # Controller output remains a complete six-dimensional body-frame value.
    assert len(result.controls) == 2
    assert all(isinstance(control, ControlInput) for control in result.controls)
    assert all(control.to_numpy().shape == (6,) for control in result.controls)

    # The stub creates new immutable snapshots; earlier states stay unchanged.
    assert len(result.history) == 3
    assert all(isinstance(state, VehicleState) for state in result.history)
    assert all(
        newer is not older
        for older, newer in zip(result.history, result.history[1:])
    )
    assert result.history[0] == VehicleState.zero(timestamp=0.0)
    with pytest.raises(FrozenInstanceError):
        result.history[0].timestamp = 99.0  # type: ignore[misc]

    # Timestamp is metadata: numerical conversion is strictly 12-D.
    for state in (*result.trajectory.states, *result.history):
        vector = state.to_numpy()
        assert vector.shape == (12,)
        np.testing.assert_array_equal(
            vector,
            np.concatenate([state.pose.to_numpy(), state.twist.to_numpy()]),
        )
        restored = VehicleState.from_numpy(vector, timestamp=state.timestamp)
        assert restored.is_close(state)

    # ENU z-up and SNAME w-down are visibly different conventions.
    descending = kinematic_physics_stub(
        VehicleState.zero(timestamp=0.0),
        ControlInput(tau_z=1.0),
        time_step=1.0,
    )
    assert descending.twist.w == pytest.approx(1.0)
    assert descending.pose.z == pytest.approx(-1.0)
