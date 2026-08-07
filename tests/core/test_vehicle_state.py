"""
Unit tests for core.vehicle_state.VehicleState.

Core contracts under test:
    1. VehicleState composes Pose and Twist — no duplicated fields.
    2. to_numpy() returns the 12-D physical state vector [η; ν].
       Timestamp is metadata, NOT included in the state vector.
    3. from_numpy() accepts (12,) array + separate timestamp.
    4. is_close() delegates to Pose.is_close and Twist.is_close.
    5. No silent initialization — pose and twist are required.
"""

import pytest
import numpy as np
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from core.pose import Pose
from core.twist import Twist
from core.vehicle_state import VehicleState


# =============================================================================
# Fixtures
# =============================================================================

@pytest.fixture
def sample_pose():
    """A non-zero Pose."""
    return Pose(x=10.0, y=20.0, z=-5.0, roll=0.0, pitch=0.0, yaw=1.57)


@pytest.fixture
def sample_twist():
    """A non-zero Twist."""
    return Twist(u=1.0, v=0.5, w=0.0, p=0.0, q=0.0, r=0.3)


@pytest.fixture
def sample_state(sample_pose, sample_twist):
    """A typical VehicleState."""
    return VehicleState(pose=sample_pose, twist=sample_twist, timestamp=10.0)


@pytest.fixture
def zero_state():
    """A zero VehicleState."""
    return VehicleState(pose=Pose.zero(), twist=Twist.zero(), timestamp=0.0)


# =============================================================================
# Construction Tests
# =============================================================================

class TestConstruction:
    """Tests for VehicleState construction."""

    def test_explicit_pose_and_twist(self, sample_pose, sample_twist):
        """VehicleState should accept explicit Pose and Twist."""
        state = VehicleState(pose=sample_pose, twist=sample_twist)
        assert state.pose is sample_pose
        assert state.twist is sample_twist
        assert state.timestamp == 0.0  # default

    def test_default_timestamp(self, sample_pose, sample_twist):
        """Timestamp should default to 0.0 when not provided."""
        state = VehicleState(pose=sample_pose, twist=sample_twist)
        assert state.timestamp == 0.0

    def test_explicit_timestamp(self, sample_pose, sample_twist):
        """Timestamp should be settable at construction."""
        state = VehicleState(pose=sample_pose, twist=sample_twist, timestamp=42.5)
        assert state.timestamp == 42.5

    def test_zero_factory_default_timestamp(self):
        """zero() should return origin pose, zero twist, timestamp=0.0."""
        state = VehicleState.zero()
        assert state.pose.is_close(Pose.zero())
        assert state.twist.is_close(Twist.zero())
        assert state.timestamp == 0.0

    def test_zero_factory_custom_timestamp(self):
        """zero(timestamp) should set the timestamp."""
        state = VehicleState.zero(timestamp=5.0)
        assert state.pose.is_close(Pose.zero())
        assert state.twist.is_close(Twist.zero())
        assert state.timestamp == 5.0

    def test_construction_requires_pose_and_twist(self):
        """VehicleState() with no arguments should raise TypeError."""
        with pytest.raises(TypeError):
            VehicleState()  # type: ignore[call-arg]

    def test_construction_requires_pose(self, sample_twist):
        """VehicleState(twist=...) without pose should raise TypeError."""
        with pytest.raises(TypeError):
            VehicleState(twist=sample_twist)  # type: ignore[call-arg]

    def test_construction_requires_twist(self, sample_pose):
        """VehicleState(pose=...) without twist should raise TypeError."""
        with pytest.raises(TypeError):
            VehicleState(pose=sample_pose)  # type: ignore[call-arg]

    def test_pose_and_twist_are_stored_by_reference(self, sample_pose, sample_twist):
        """The exact Pose and Twist instances passed in should be stored."""
        state = VehicleState(pose=sample_pose, twist=sample_twist)
        assert state.pose is sample_pose
        assert state.twist is sample_twist


# =============================================================================
# Composition Tests
# =============================================================================

class TestComposition:
    """Tests that VehicleState composes Pose and Twist without duplication."""

    def test_pose_is_pose_instance(self, sample_state):
        """state.pose should be a Pose instance."""
        assert isinstance(sample_state.pose, Pose)

    def test_twist_is_twist_instance(self, sample_state):
        """state.twist should be a Twist instance."""
        assert isinstance(sample_state.twist, Twist)

    def test_no_duplicated_x_field(self, sample_state):
        """VehicleState should NOT have an 'x' attribute."""
        assert not hasattr(sample_state, "x")

    def test_no_duplicated_y_field(self, sample_state):
        """VehicleState should NOT have a 'y' attribute."""
        assert not hasattr(sample_state, "y")

    def test_no_duplicated_z_field(self, sample_state):
        """VehicleState should NOT have a 'z' attribute."""
        assert not hasattr(sample_state, "z")

    def test_no_duplicated_roll_field(self, sample_state):
        """VehicleState should NOT have a 'roll' attribute."""
        assert not hasattr(sample_state, "roll")

    def test_no_duplicated_pitch_field(self, sample_state):
        """VehicleState should NOT have a 'pitch' attribute."""
        assert not hasattr(sample_state, "pitch")

    def test_no_duplicated_yaw_field(self, sample_state):
        """VehicleState should NOT have a 'yaw' attribute."""
        assert not hasattr(sample_state, "yaw")

    def test_no_duplicated_u_field(self, sample_state):
        """VehicleState should NOT have a 'u' attribute."""
        assert not hasattr(sample_state, "u")

    def test_no_duplicated_v_field(self, sample_state):
        """VehicleState should NOT have a 'v' attribute."""
        assert not hasattr(sample_state, "v")

    def test_no_duplicated_w_field(self, sample_state):
        """VehicleState should NOT have a 'w' attribute."""
        assert not hasattr(sample_state, "w")

    def test_no_duplicated_p_field(self, sample_state):
        """VehicleState should NOT have a 'p' attribute."""
        assert not hasattr(sample_state, "p")

    def test_no_duplicated_q_field(self, sample_state):
        """VehicleState should NOT have a 'q' attribute."""
        assert not hasattr(sample_state, "q")

    def test_no_duplicated_r_field(self, sample_state):
        """VehicleState should NOT have a 'r' attribute."""
        assert not hasattr(sample_state, "r")

    def test_pose_methods_accessible(self, sample_state):
        """Pose methods should be accessible via state.pose."""
        assert sample_state.pose.distance_to(Pose.zero()) > 0.0
        assert sample_state.pose.heading == sample_state.pose.yaw

    def test_twist_methods_accessible(self, sample_state):
        """Twist methods should be accessible via state.twist."""
        assert sample_state.twist.norm_linear() > 0.0
        assert sample_state.twist.speed_2d() > 0.0


# =============================================================================
# 12-D Contract Tests
# =============================================================================

class Test12DContract:
    """Tests that to_numpy/from_numpy respect the 12-D state vector contract."""

    def test_to_numpy_shape(self, sample_state):
        """to_numpy should return shape (12,)."""
        arr = sample_state.to_numpy()
        assert arr.shape == (12,)

    def test_to_numpy_zero_shape(self, zero_state):
        """to_numpy of zero state should also be (12,)."""
        assert zero_state.to_numpy().shape == (12,)

    def test_to_numpy_correct_ordering(self, sample_pose, sample_twist):
        """to_numpy should be [pose(6); twist(6)]."""
        state = VehicleState(pose=sample_pose, twist=sample_twist)
        arr = state.to_numpy()
        expected = np.concatenate([sample_pose.to_numpy(), sample_twist.to_numpy()])
        np.testing.assert_array_equal(arr, expected)

    def test_to_numpy_timestamp_not_included(self, sample_state):
        """Timestamp must NOT appear in the to_numpy output."""
        arr = sample_state.to_numpy()
        # The output must be exactly [pose; twist], length 12
        expected = np.concatenate([
            sample_state.pose.to_numpy(),
            sample_state.twist.to_numpy(),
        ])
        np.testing.assert_array_equal(arr, expected)
        assert len(arr) == 12
    
    def test_from_numpy_requires_exactly_12(self):
        """from_numpy should reject arrays with shape != (12,)."""
        with pytest.raises(ValueError, match="Expected shape"):
            VehicleState.from_numpy(np.zeros(13))
        with pytest.raises(ValueError, match="Expected shape"):
            VehicleState.from_numpy(np.zeros(11))
        with pytest.raises(ValueError, match="1-D"):
            VehicleState.from_numpy(np.zeros((2, 6)))
        with pytest.raises(ValueError, match="1-D"):
            VehicleState.from_numpy(np.zeros((12, 1)))
    


    def test_from_numpy_accepts_12(self):
        """from_numpy should accept (12,) arrays."""
        arr = np.array([1, 2, 3, 0, 0, 0.5, 0.1, 0.2, 0, 0, 0, 0.05])
        state = VehicleState.from_numpy(arr)
        assert state.pose.x == 1.0
        assert state.twist.u == 0.1

    def test_roundtrip_preserves_physical_state(self, sample_state):
        """to_numpy -> from_numpy should preserve physical state exactly."""
        arr = sample_state.to_numpy()
        reconstructed = VehicleState.from_numpy(arr)
        # Physical state should be identical
        assert reconstructed.pose.is_close(sample_state.pose)
        assert reconstructed.twist.is_close(sample_state.twist)

    def test_roundtrip_timestamp_must_be_passed_separately(self, sample_state):
        """After roundtrip, timestamp is NOT recovered from the array."""
        arr = sample_state.to_numpy()
        reconstructed = VehicleState.from_numpy(arr)  # no timestamp arg
        # Timestamp should be the default (0.0), not the original (10.0)
        assert reconstructed.timestamp == 0.0
        assert reconstructed.timestamp != sample_state.timestamp

    def test_roundtrip_with_explicit_timestamp(self, sample_state):
        """Roundtrip with explicit timestamp should fully preserve the state."""
        arr = sample_state.to_numpy()
        reconstructed = VehicleState.from_numpy(arr, timestamp=sample_state.timestamp)
        assert reconstructed.is_close(sample_state)

    def test_from_numpy_default_timestamp(self):
        """from_numpy without timestamp should default to 0.0."""
        arr = np.zeros(12)
        state = VehicleState.from_numpy(arr)
        assert state.timestamp == 0.0

    def test_from_numpy_explicit_timestamp(self):
        """from_numpy with timestamp should use the provided value."""
        arr = np.zeros(12)
        state = VehicleState.from_numpy(arr, timestamp=99.9)
        assert state.timestamp == 99.9


# =============================================================================
# Comparison Tests
# =============================================================================

class TestComparison:
    """Tests for is_close with delegated tolerance."""

    def test_is_close_identical(self, sample_state):
        """A state should be close to itself."""
        assert sample_state.is_close(sample_state)

    def test_is_close_different_pose(self, sample_state):
        """States with different poses should not be close."""
        other_pose = Pose(x=999.0, y=0.0, z=0.0)
        other = VehicleState(pose=other_pose, twist=sample_state.twist,
                             timestamp=sample_state.timestamp)
        assert not sample_state.is_close(other)

    def test_is_close_different_twist(self, sample_state):
        """States with different twists should not be close."""
        other_twist = Twist(u=999.0)
        other = VehicleState(pose=sample_state.pose, twist=other_twist,
                             timestamp=sample_state.timestamp)
        assert not sample_state.is_close(other)

    def test_is_close_different_timestamp(self, sample_state):
        """States with different timestamps should not be close."""
        other = VehicleState(pose=sample_state.pose, twist=sample_state.twist,
                             timestamp=999.0)
        assert not sample_state.is_close(other)

    def test_is_close_pose_tolerance(self, sample_state):
        """Pose tolerance should be forwarded to Pose.is_close."""
        slightly_off_pose = Pose(x=sample_state.pose.x + 1e-5,
                                 y=sample_state.pose.y,
                                 z=sample_state.pose.z,
                                 yaw=sample_state.pose.yaw)
        other = VehicleState(pose=slightly_off_pose, twist=sample_state.twist,
                             timestamp=sample_state.timestamp)
        # Default pos_tol=1e-6 should reject
        assert not sample_state.is_close(other)
        # Custom pos_tol=1e-4 should accept
        assert sample_state.is_close(other, pos_tol=1e-4)

    def test_is_close_twist_tolerance(self, sample_state):
        """Twist tolerance should be forwarded to Twist.is_close."""
        slightly_off_twist = Twist(
            u=sample_state.twist.u + 1e-5,
            v=sample_state.twist.v,      # preserve original
            w=sample_state.twist.w,
            p=sample_state.twist.p,
            q=sample_state.twist.q,
            r=sample_state.twist.r,      # preserve original
        )
        other = VehicleState(pose=sample_state.pose, twist=slightly_off_twist,
                             timestamp=sample_state.timestamp)
        assert not sample_state.is_close(other)
        assert sample_state.is_close(other, lin_tol=1e-4)    

    def test_is_close_timestamp_tolerance(self, sample_state):
        """Timestamp tolerance should be respected."""
        other = VehicleState(pose=sample_state.pose, twist=sample_state.twist,
                             timestamp=sample_state.timestamp + 1e-6)
        # Default time_tol=1e-9 should reject 1e-6 difference
        assert not sample_state.is_close(other)
        # Custom time_tol=1e-5 should accept
        assert sample_state.is_close(other, time_tol=1e-5)

    def test_is_close_rejects_non_vehicle_state(self, sample_state):
        """is_close should raise TypeError for non-VehicleState argument."""
        with pytest.raises(TypeError, match="Expected VehicleState"):
            sample_state.is_close("not a state")  # type: ignore[arg-type]


# =============================================================================
# Immutability Tests
# =============================================================================

class TestImmutability:
    """Tests that VehicleState is truly immutable."""

    def test_cannot_replace_pose(self, sample_state):
        """Setting pose should raise FrozenInstanceError."""
        with pytest.raises(Exception):
            sample_state.pose = Pose.zero()  # type: ignore[misc]

    def test_cannot_replace_twist(self, sample_state):
        """Setting twist should raise FrozenInstanceError."""
        with pytest.raises(Exception):
            sample_state.twist = Twist.zero()  # type: ignore[misc]

    def test_cannot_change_timestamp(self, sample_state):
        """Setting timestamp should raise FrozenInstanceError."""
        with pytest.raises(Exception):
            sample_state.timestamp = 999.0  # type: ignore[misc]

    def test_hashable(self, sample_state, zero_state):
        """Frozen dataclass should be hashable."""
        d = {sample_state: "sample", zero_state: "zero"}
        assert d[sample_state] == "sample"
        assert d[zero_state] == "zero"