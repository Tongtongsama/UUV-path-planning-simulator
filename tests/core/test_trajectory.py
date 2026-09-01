"""Unit tests for core.trajectory.Trajectory."""

import pytest
import numpy as np
import sys
from pathlib import Path as PathLib

PROJECT_ROOT = PathLib(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from core.pose import Pose
from core.twist import Twist
from core.vehicle_state import VehicleState
from core.trajectory import Trajectory


# =============================================================================
# Fixtures
# =============================================================================

@pytest.fixture
def state_t0():
    return VehicleState(pose=Pose.zero(), twist=Twist.zero(), timestamp=0.0)


@pytest.fixture
def state_t1():
    return VehicleState(
        pose=Pose(1.0, 0.0, 0.0),
        twist=Twist(u=1.0),
        timestamp=1.0,
    )


@pytest.fixture
def state_t2():
    return VehicleState(
        pose=Pose(2.0, 0.0, 0.0),
        twist=Twist(u=1.0),
        timestamp=2.0,
    )


@pytest.fixture
def traj_two(state_t0, state_t1):
    return Trajectory(states=(state_t0, state_t1))


@pytest.fixture
def traj_three(state_t0, state_t1, state_t2):
    return Trajectory(states=(state_t0, state_t1, state_t2))


# =============================================================================
# Construction Tests
# =============================================================================

class TestConstruction:

    def test_construct_from_tuple(self, state_t0, state_t1):
        traj = Trajectory(states=(state_t0, state_t1))
        assert len(traj) == 2

    def test_empty_raises(self):
        with pytest.raises(ValueError, match="at least one VehicleState"):
            Trajectory(states=())

    def test_non_vehiclestate_raises(self, state_t0):
        with pytest.raises(TypeError, match="VehicleState instances"):
            Trajectory(states=(state_t0, "not_a_state"))  # type: ignore[arg-type]

    def test_from_list(self, state_t0, state_t1):
        traj = Trajectory.from_list([state_t0, state_t1])
        assert len(traj) == 2

    def test_from_list_empty_raises(self):
        with pytest.raises(ValueError, match="at least one VehicleState"):
            Trajectory.from_list([])

    def test_single_state_allowed(self, state_t0):
        traj = Trajectory(states=(state_t0,))
        assert len(traj) == 1
        assert traj.duration == 0.0

    # ---- Timestamp validation ----

    def test_non_increasing_timestamps_raises(self, state_t0):
        state_bad = VehicleState(
            pose=Pose.zero(), twist=Twist.zero(), timestamp=0.0  # same as t0
        )
        with pytest.raises(ValueError, match="strictly increasing"):
            Trajectory(states=(state_t0, state_bad))

    def test_decreasing_timestamps_raises(self, state_t0, state_t1):
        # swap timestamps: t0=0.0, t1=1.0, but in wrong order
        s0 = VehicleState(pose=Pose.zero(), twist=Twist.zero(), timestamp=1.0)
        s1 = VehicleState(pose=Pose.zero(), twist=Twist.zero(), timestamp=0.0)
        with pytest.raises(ValueError, match="strictly increasing"):
            Trajectory(states=(s0, s1))

    def test_nan_timestamp_raises(self):
        s0 = VehicleState(pose=Pose.zero(), twist=Twist.zero(), timestamp=0.0)
        s1 = VehicleState(pose=Pose.zero(), twist=Twist.zero(), timestamp=np.nan)
        with pytest.raises(ValueError, match="finite"):
            Trajectory(states=(s0, s1))

    def test_inf_timestamp_raises(self):
        s0 = VehicleState(pose=Pose.zero(), twist=Twist.zero(), timestamp=0.0)
        s1 = VehicleState(pose=Pose.zero(), twist=Twist.zero(), timestamp=np.inf)
        with pytest.raises(ValueError, match="finite"):
            Trajectory(states=(s0, s1))

    def test_neg_inf_timestamp_raises(self):
        s0 = VehicleState(pose=Pose.zero(), twist=Twist.zero(), timestamp=-np.inf)
        with pytest.raises(ValueError, match="finite"):
            Trajectory(states=(s0,))


# =============================================================================
# Property Tests
# =============================================================================

class TestProperties:

    def test_start_time(self, traj_three):
        assert traj_three.start_time == 0.0

    def test_end_time(self, traj_three):
        assert traj_three.end_time == 2.0

    def test_duration(self, traj_three):
        assert traj_three.duration == 2.0

    def test_duration_single_state(self, state_t0):
        traj = Trajectory(states=(state_t0,))
        assert traj.duration == 0.0

    def test_start_time_equals_end_time_single_state(self, state_t0):
        traj = Trajectory(states=(state_t0,))
        assert traj.start_time == traj.end_time


# =============================================================================
# Comparison Tests
# =============================================================================

class TestComparison:

    def test_is_close_identical(self, traj_three):
        assert traj_three.is_close(traj_three)

    def test_is_close_different_length(self, traj_two, traj_three):
        assert not traj_two.is_close(traj_three)

    def test_is_close_same_values(self, state_t0, state_t1):
        t1 = Trajectory(states=(state_t0, state_t1))
        s0 = VehicleState(pose=Pose.zero(), twist=Twist.zero(), timestamp=0.0)
        s1 = VehicleState(
            pose=Pose(1.0, 0.0, 0.0), twist=Twist(u=1.0), timestamp=1.0
        )
        t2 = Trajectory(states=(s0, s1))
        assert t1.is_close(t2)

    def test_is_close_different_values(self, state_t0, state_t1, state_t2):
        t1 = Trajectory(states=(state_t0, state_t1))
        t2 = Trajectory(states=(state_t0, state_t2))
        assert not t1.is_close(t2)

    def test_is_close_tolerance(self):
        s0 = VehicleState(pose=Pose.zero(), twist=Twist.zero(), timestamp=0.0)
        s1 = VehicleState(
            pose=Pose(1e-5, 0.0, 0.0), twist=Twist.zero(), timestamp=1.0
        )
        t1 = Trajectory(states=(s0, s1))
        t2 = Trajectory(states=(
            VehicleState(pose=Pose.zero(), twist=Twist.zero(), timestamp=0.0),
            VehicleState(pose=Pose.zero(), twist=Twist.zero(), timestamp=1.0),
        ))
        assert not t1.is_close(t2)
        assert t1.is_close(t2, pos_tol=1e-4)

    def test_is_close_rejects_non_trajectory(self, traj_two):
        with pytest.raises(TypeError, match="Expected Trajectory"):
            traj_two.is_close("not a trajectory")  # type: ignore[arg-type]


# =============================================================================
# Sequence Protocol Tests
# =============================================================================

class TestSequenceProtocol:

    def test_len(self, traj_three):
        assert len(traj_three) == 3

    def test_getitem(self, traj_three, state_t0, state_t1, state_t2):
        assert traj_three[0].is_close(state_t0)
        assert traj_three[1].is_close(state_t1)
        assert traj_three[2].is_close(state_t2)

    def test_iter(self, traj_three):
        states = list(traj_three)
        assert len(states) == 3
        assert all(isinstance(s, VehicleState) for s in states)


# =============================================================================
# Immutability Tests
# =============================================================================

class TestImmutability:

    def test_cannot_replace_states(self, traj_two):
        with pytest.raises(Exception):
            traj_two.states = ()  # type: ignore[misc]

    def test_hashable(self, traj_two, traj_three):
        d = {traj_two: "two", traj_three: "three"}
        assert d[traj_two] == "two"
