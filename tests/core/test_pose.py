"""
Unit tests for core.pose.Pose.

Tests cover:
    - Construction (default, factory methods)
    - Properties (xy, heading)
    - Conversion methods (to_list, to_numpy, position, orientation_euler)
    - Geometry methods (distance_to, norm_position, norm_xy, is_close)
    - Immutability (frozen dataclass)
"""

import pytest
import numpy as np
import sys
from pathlib import Path

# Add project root to path so we can import uuv_simulator
# Adjust this if your project structure differs
PROJECT_ROOT = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from core.pose import Pose


# =============================================================================
# Fixtures
# =============================================================================

@pytest.fixture
def origin():
    """Pose at the origin with zero orientation."""
    return Pose()


@pytest.fixture
def pose_a():
    """A typical 3-DOF pose: (3, 4, 0, 0, 0, pi/2)."""
    return Pose(x=3.0, y=4.0, z=0.0, roll=0.0, pitch=0.0, yaw=np.pi / 2)


@pytest.fixture
def pose_b():
    """Another pose: (0, 0, 0, 0, 0, pi)."""
    return Pose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=np.pi)


@pytest.fixture
def pose_3d():
    """A 6-DOF pose for future compatibility tests."""
    return Pose(x=1.0, y=2.0, z=3.0, roll=0.1, pitch=0.2, yaw=0.3)


# =============================================================================
# Construction Tests
# =============================================================================

class TestConstruction:
    """Tests for Pose construction: default values and factory methods."""

    def test_default_pose_is_origin(self):
        """A default-constructed Pose should be at the origin with zero orientation."""
        p = Pose()
        assert p.x == 0.0
        assert p.y == 0.0
        assert p.z == 0.0
        assert p.roll == 0.0
        assert p.pitch == 0.0
        assert p.yaw == 0.0

    def test_default_pose_is_all_zeros(self, origin):
        """Fixture-based: origin should have all zeros."""
        assert origin.x == 0.0
        assert origin.y == 0.0
        assert origin.z == 0.0
        assert origin.roll == 0.0
        assert origin.pitch == 0.0
        assert origin.yaw == 0.0

    def test_explicit_construction(self):
        """Pose should accept all 6 arguments positionally."""
        p = Pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
        assert p.x == 1.0
        assert p.y == 2.0
        assert p.z == 3.0
        assert p.roll == 0.1
        assert p.pitch == 0.2
        assert p.yaw == 0.3

    def test_keyword_construction(self):
        """Pose should accept keyword arguments."""
        p = Pose(x=5.0, y=6.0, yaw=1.0)
        assert p.x == 5.0
        assert p.y == 6.0
        assert p.z == 0.0  # default
        assert p.yaw == 1.0

    def test_from_list(self):
        """from_list should construct from a 6-element list."""
        data = [1.0, 2.0, 3.0, 0.0, 0.0, 1.57]
        p = Pose.from_list(data)
        assert p.x == 1.0
        assert p.y == 2.0
        assert p.z == 3.0
        assert p.yaw == 1.57

    def test_from_list_rejects_wrong_length(self):
        """from_list should raise ValueError if list length != 6."""
        with pytest.raises(ValueError, match="Expected 6 elements"):
            Pose.from_list([1.0, 2.0, 3.0])

    def test_from_numpy(self):
        """from_numpy should construct from a (6,) numpy array."""
        arr = np.array([1.0, 2.0, 3.0, 0.0, 0.0, 1.57])
        p = Pose.from_numpy(arr)
        assert p.x == 1.0
        assert p.y == 2.0
        assert p.z == 3.0
        assert p.yaw == 1.57

    def test_from_numpy_rejects_wrong_shape(self):
        """from_numpy should raise ValueError if shape != (6,)."""
        arr = np.array([1.0, 2.0, 3.0])
        with pytest.raises(ValueError, match="Expected shape"):
            Pose.from_numpy(arr)

    def test_from_position_and_yaw(self):
        """from_position_and_yaw should set roll and pitch to 0."""
        p = Pose.from_position_and_yaw(x=1.0, y=2.0, z=-5.0, yaw=0.5)
        assert p.x == 1.0
        assert p.y == 2.0
        assert p.z == -5.0
        assert p.roll == 0.0
        assert p.pitch == 0.0
        assert p.yaw == 0.5


# =============================================================================
# Property Tests
# =============================================================================

class TestProperties:
    """Tests for Pose properties: xy and heading."""

    def test_xy_returns_2d_array(self, pose_a):
        """xy should return [x, y] as a numpy array."""
        xy = pose_a.xy
        assert isinstance(xy, np.ndarray)
        assert xy.shape == (2,)
        assert xy[0] == 3.0
        assert xy[1] == 4.0

    def test_xy_at_origin(self, origin):
        """xy at origin should be [0, 0]."""
        xy = origin.xy
        np.testing.assert_array_equal(xy, np.array([0.0, 0.0]))

    def test_heading_equals_yaw(self, pose_a):
        """heading should return the yaw value."""
        assert pose_a.heading == pose_a.yaw
        assert pose_a.heading == np.pi / 2

    def test_heading_zero(self, origin):
        """heading at origin should be 0.0."""
        assert origin.heading == 0.0


# =============================================================================
# Conversion Tests
# =============================================================================

class TestConversion:
    """Tests for conversion methods: to_list, to_numpy, position, orientation."""

    def test_to_list(self, pose_a):
        """to_list should return a list of 6 floats."""
        lst = pose_a.to_list()
        assert isinstance(lst, list)
        assert len(lst) == 6
        assert lst == [3.0, 4.0, 0.0, 0.0, 0.0, np.pi / 2]

    def test_to_numpy(self, pose_a):
        """to_numpy should return a (6,) numpy array."""
        arr = pose_a.to_numpy()
        assert isinstance(arr, np.ndarray)
        assert arr.shape == (6,)
        expected = np.array([3.0, 4.0, 0.0, 0.0, 0.0, np.pi / 2])
        np.testing.assert_array_equal(arr, expected)

    def test_position_3d(self, pose_3d):
        """position() should return [x, y, z]."""
        pos = pose_3d.position()
        assert isinstance(pos, np.ndarray)
        assert pos.shape == (3,)
        np.testing.assert_array_equal(pos, np.array([1.0, 2.0, 3.0]))

    def test_orientation_euler(self, pose_3d):
        """orientation_euler() should return [roll, pitch, yaw]."""
        ori = pose_3d.orientation_euler()
        assert isinstance(ori, np.ndarray)
        assert ori.shape == (3,)
        np.testing.assert_array_equal(ori, np.array([0.1, 0.2, 0.3]))

    def test_to_list_roundtrip(self, pose_3d):
        """Pose -> to_list -> from_list should produce an equivalent Pose."""
        lst = pose_3d.to_list()
        reconstructed = Pose.from_list(lst)
        assert reconstructed.is_close(pose_3d)


# =============================================================================
# Geometry Tests
# =============================================================================

class TestGeometry:
    """Tests for geometry methods: distance_to, norm_position, norm_xy, is_close."""

    # ---- distance_to ----

    def test_distance_to_self_is_zero(self, pose_a):
        """Distance from a Pose to itself should be 0."""
        assert pose_a.distance_to(pose_a) == 0.0

    def test_distance_to_origin(self, pose_a, origin):
        """Distance from (3,4,0) to origin should be 5.0 (3-4-5 triangle)."""
        assert pose_a.distance_to(origin) == 5.0

    def test_distance_to_symmetric(self, pose_a, origin):
        """distance_to should be symmetric."""
        assert pose_a.distance_to(origin) == origin.distance_to(pose_a)

    def test_distance_to_3d(self, pose_3d, origin):
        """Distance from (1,2,3) to origin should be sqrt(1+4+9) = sqrt(14)."""
        expected = np.sqrt(14.0)
        assert pose_3d.distance_to(origin) == pytest.approx(expected)

    def test_distance_to_rejects_non_pose(self, pose_a):
        """distance_to should raise TypeError for non-Pose arguments."""
        with pytest.raises(TypeError, match="Expected Pose"):
            pose_a.distance_to((3.0, 4.0, 0.0))

    # ---- norm_position ----

    def test_norm_position_origin(self, origin):
        """Norm of origin should be 0."""
        assert origin.norm_position() == 0.0

    def test_norm_position(self, pose_a):
        """Norm of (3,4,0) should be 5.0."""
        assert pose_a.norm_position() == 5.0

    def test_norm_position_3d(self, pose_3d):
        """Norm of (1,2,3) should be sqrt(14)."""
        assert pose_3d.norm_position() == pytest.approx(np.sqrt(14.0))

    # ---- norm_xy ----

    def test_norm_xy_origin(self, origin):
        """2D norm of origin should be 0."""
        assert origin.norm_xy() == 0.0

    def test_norm_xy(self, pose_a):
        """2D norm of (3,4) should be 5.0."""
        assert pose_a.norm_xy() == 5.0

    def test_norm_xy_ignores_z(self):
        """norm_xy should ignore z coordinate."""
        p = Pose(x=3.0, y=4.0, z=100.0)
        assert p.norm_xy() == 5.0
        assert p.norm_position() > 5.0

    # ---- is_close ----

    def test_is_close_identical(self, pose_a):
        """Identical Poses should be close."""
        assert pose_a.is_close(pose_a)

    def test_is_close_with_default_tolerance(self):
        """Poses differing by 1e-7 should be close with default tol=1e-6."""
        p1 = Pose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
        p2 = Pose(1.0 + 1e-7, 2.0, 3.0, 0.0, 0.0, 0.0)
        assert p1.is_close(p2)

    def test_is_close_rejects_large_difference(self):
        """Poses differing by 0.1 should not be close with default tol."""
        p1 = Pose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
        p2 = Pose(1.1, 2.0, 3.0, 0.0, 0.0, 0.0)
        assert not p1.is_close(p2)

    def test_is_close_custom_tolerance(self):
        """Custom tolerance should be respected."""
        p1 = Pose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
        p2 = Pose(1.1, 2.0, 3.0, 0.0, 0.0, 0.0)
        # With pos_tol=0.2, 0.1 difference should pass
        assert p1.is_close(p2, pos_tol=0.2)

    def test_is_close_separate_pos_ang_tolerance(self):
        """pos_tol and ang_tol should be independent."""
        p1 = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        p2 = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        # Large angle difference should fail with small ang_tol
        assert not p1.is_close(p2, ang_tol=0.01)
        # But pass with large ang_tol
        assert p1.is_close(p2, ang_tol=2.0)

    def test_is_close_rejects_non_pose(self, pose_a):
        """is_close should raise TypeError for non-Pose arguments."""
        with pytest.raises(TypeError, match="Expected Pose"):
            pose_a.is_close((3.0, 4.0, 0.0, 0.0, 0.0, 0.0))


# =============================================================================
# Immutability Tests
# =============================================================================

class TestImmutability:
    """Tests that Pose is truly immutable (frozen dataclass)."""

    def test_cannot_set_x(self, pose_a):
        """Setting x should raise FrozenInstanceError."""
        with pytest.raises(Exception):  # dataclasses.FrozenInstanceError
            pose_a.x = 10.0  # type: ignore[misc]

    def test_cannot_set_yaw(self, pose_a):
        """Setting yaw should raise FrozenInstanceError."""
        with pytest.raises(Exception):
            pose_a.yaw = 0.0  # type: ignore[misc]

    def test_cannot_set_xy_property(self, pose_a):
        """Setting xy property should raise AttributeError."""
        with pytest.raises(AttributeError):
            pose_a.xy = np.array([1.0, 2.0])  # type: ignore[misc]

    def test_cannot_set_heading_property(self, pose_a):
        """Setting heading property should raise AttributeError."""
        with pytest.raises(AttributeError):
            pose_a.heading = 0.0  # type: ignore[misc]

    def test_hashable(self, pose_a, pose_b):
        """Frozen dataclasses should be hashable (can be dict keys)."""
        d = {pose_a: "a", pose_b: "b"}
        assert d[pose_a] == "a"
        assert d[pose_b] == "b"


# =============================================================================
# Edge Case Tests
# =============================================================================

class TestEdgeCases:
    """Tests for edge cases: negative values, large values, zero, etc."""

    def test_negative_coordinates(self):
        """Pose should handle negative coordinates."""
        p = Pose(x=-5.0, y=-10.0, z=-20.0)
        assert p.x == -5.0
        assert p.norm_position() == pytest.approx(np.sqrt(25 + 100 + 400))

    def test_large_values(self):
        """Pose should handle large coordinate values without overflow."""
        p = Pose(x=1e6, y=1e6, z=1e6)
        assert p.norm_position() == pytest.approx(np.sqrt(3) * 1e6)

    def test_angle_wrap_not_handled(self):
        """
        Note: Pose does NOT normalize angles.
        yaw = 3*pi is stored as-is. 
        This is intentional: normalization is the responsibility of 
        the Physics Engine or Controller, not the data model.
        """
        p = Pose(yaw=3 * np.pi)
        assert p.yaw == 3 * np.pi
        # is_close should still work for identical values
        assert p.is_close(Pose(yaw=3 * np.pi))
        # But NOT for angle-wrapped equivalents (expected behavior)
        assert not p.is_close(Pose(yaw=np.pi))

    def test_numpy_type_preservation(self):
        """to_numpy should return float64 regardless of input type."""
        p = Pose(x=1, y=2)  # ints
        arr = p.to_numpy()
        assert arr.dtype == np.float64