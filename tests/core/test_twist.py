"""
Unit tests for core.twist.Twist.

Tests cover:
    - Construction (default, factory methods)
    - Properties (linear, angular, velocity_2d)
    - Conversion methods (to_list, to_numpy)
    - Geometry/Physics methods (norm_linear, norm_angular, speed_2d, is_close)
    - Immutability (frozen dataclass)
"""

import pytest
import numpy as np
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from core.twist import Twist


# =============================================================================
# Fixtures
# =============================================================================

@pytest.fixture
def zero_twist():
    """Twist with all zero velocities."""
    return Twist()


@pytest.fixture
def surge_only():
    """Pure forward motion: u=2.0, all others zero."""
    return Twist(u=2.0)


@pytest.fixture
def full_3dof():
    """Typical 3-DOF twist: surge, sway, and yaw rate."""
    return Twist(u=1.0, v=0.5, r=0.3)


@pytest.fixture
def full_6dof():
    """A 6-DOF twist for future compatibility tests."""
    return Twist(u=1.0, v=2.0, w=3.0, p=0.1, q=0.2, r=0.3)


# =============================================================================
# Construction Tests
# =============================================================================

class TestConstruction:
    """Tests for Twist construction: default values and factory methods."""

    def test_default_twist_is_zero(self):
        """A default-constructed Twist should have all zeros."""
        t = Twist()
        assert t.u == 0.0
        assert t.v == 0.0
        assert t.w == 0.0
        assert t.p == 0.0
        assert t.q == 0.0
        assert t.r == 0.0

    def test_default_twist_fixture(self, zero_twist):
        """Fixture-based: zero_twist should have all zeros."""
        assert zero_twist.u == 0.0
        assert zero_twist.v == 0.0
        assert zero_twist.w == 0.0
        assert zero_twist.p == 0.0
        assert zero_twist.q == 0.0
        assert zero_twist.r == 0.0

    def test_explicit_construction(self):
        """Twist should accept all 6 arguments positionally."""
        t = Twist(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
        assert t.u == 1.0
        assert t.v == 2.0
        assert t.w == 3.0
        assert t.p == 0.1
        assert t.q == 0.2
        assert t.r == 0.3

    def test_keyword_construction(self):
        """Twist should accept keyword arguments."""
        t = Twist(u=1.5, r=0.5)
        assert t.u == 1.5
        assert t.v == 0.0  # default
        assert t.w == 0.0  # default
        assert t.p == 0.0  # default
        assert t.q == 0.0  # default
        assert t.r == 0.5

    def test_from_list(self):
        """from_list should construct from a 6-element list."""
        data = [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]
        t = Twist.from_list(data)
        assert t.u == 1.0
        assert t.v == 2.0
        assert t.w == 3.0
        assert t.p == 0.1
        assert t.q == 0.2
        assert t.r == 0.3

    def test_from_list_rejects_wrong_length(self):
        """from_list should raise ValueError if list length != 6."""
        with pytest.raises(ValueError, match="Expected 6 elements"):
            Twist.from_list([1.0, 2.0, 3.0])

    def test_from_numpy(self):
        """from_numpy should construct from a (6,) numpy array."""
        arr = np.array([1.0, 2.0, 3.0, 0.1, 0.2, 0.3])
        t = Twist.from_numpy(arr)
        assert t.u == 1.0
        assert t.v == 2.0
        assert t.w == 3.0
        assert t.p == 0.1
        assert t.q == 0.2
        assert t.r == 0.3

    def test_from_numpy_rejects_wrong_shape(self):
        """from_numpy should raise ValueError if shape != (6,)."""
        arr = np.array([1.0, 2.0, 3.0])
        with pytest.raises(ValueError, match="Expected shape"):
            Twist.from_numpy(arr)

    def test_from_3dof(self):
        """from_3dof should set w, p, q to 0.0."""
        t = Twist.from_3dof(u=1.0, v=0.5, r=0.3)
        assert t.u == 1.0
        assert t.v == 0.5
        assert t.w == 0.0
        assert t.p == 0.0
        assert t.q == 0.0
        assert t.r == 0.3


# =============================================================================
# Property Tests
# =============================================================================

class TestProperties:
    """Tests for Twist properties: linear, angular, velocity_2d."""

    def test_linear_returns_3d_array(self, full_6dof):
        """linear should return [u, v, w] as a numpy array."""
        lin = full_6dof.linear
        assert isinstance(lin, np.ndarray)
        assert lin.shape == (3,)
        np.testing.assert_array_equal(lin, np.array([1.0, 2.0, 3.0]))

    def test_linear_at_zero(self, zero_twist):
        """linear at zero twist should be [0, 0, 0]."""
        lin = zero_twist.linear
        np.testing.assert_array_equal(lin, np.array([0.0, 0.0, 0.0]))

    def test_angular_returns_3d_array(self, full_6dof):
        """angular should return [p, q, r] as a numpy array."""
        ang = full_6dof.angular
        assert isinstance(ang, np.ndarray)
        assert ang.shape == (3,)
        np.testing.assert_array_equal(ang, np.array([0.1, 0.2, 0.3]))

    def test_angular_at_zero(self, zero_twist):
        """angular at zero twist should be [0, 0, 0]."""
        ang = zero_twist.angular
        np.testing.assert_array_equal(ang, np.array([0.0, 0.0, 0.0]))

    def test_velocity_2d_returns_2d_array(self, full_3dof):
        """velocity_2d should return [u, v] as a numpy array."""
        vel2d = full_3dof.velocity_2d
        assert isinstance(vel2d, np.ndarray)
        assert vel2d.shape == (2,)
        np.testing.assert_array_equal(vel2d, np.array([1.0, 0.5]))

    def test_velocity_2d_at_zero(self, zero_twist):
        """velocity_2d at zero twist should be [0, 0]."""
        vel2d = zero_twist.velocity_2d
        np.testing.assert_array_equal(vel2d, np.array([0.0, 0.0]))


# =============================================================================
# Conversion Tests
# =============================================================================

class TestConversion:
    """Tests for conversion methods: to_list, to_numpy."""

    def test_to_list(self, full_6dof):
        """to_list should return a list of 6 floats."""
        lst = full_6dof.to_list()
        assert isinstance(lst, list)
        assert len(lst) == 6
        assert lst == [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]

    def test_to_numpy(self, full_6dof):
        """to_numpy should return a (6,) numpy array."""
        arr = full_6dof.to_numpy()
        assert isinstance(arr, np.ndarray)
        assert arr.shape == (6,)
        expected = np.array([1.0, 2.0, 3.0, 0.1, 0.2, 0.3])
        np.testing.assert_array_equal(arr, expected)

    def test_to_list_roundtrip(self, full_6dof):
        """Twist -> to_list -> from_list should produce an equivalent Twist."""
        lst = full_6dof.to_list()
        reconstructed = Twist.from_list(lst)
        assert reconstructed.is_close(full_6dof)


# =============================================================================
# Geometry / Physics Tests
# =============================================================================

class TestGeometry:
    """Tests for physics methods: norm_linear, norm_angular, speed_2d, is_close."""

    # ---- norm_linear ----

    def test_norm_linear_zero(self, zero_twist):
        """Norm of zero twist should be 0."""
        assert zero_twist.norm_linear() == 0.0

    def test_norm_linear_surge_only(self, surge_only):
        """Norm of pure surge (2,0,0) should be 2.0."""
        assert surge_only.norm_linear() == 2.0

    def test_norm_linear_3dof(self, full_3dof):
        """Norm of (1.0, 0.5, 0.0) should be sqrt(1 + 0.25) = sqrt(1.25)."""
        expected = np.sqrt(1.0 + 0.25)
        assert full_3dof.norm_linear() == pytest.approx(expected)

    def test_norm_linear_6dof(self, full_6dof):
        """Norm of (1,2,3) should be sqrt(1+4+9) = sqrt(14)."""
        expected = np.sqrt(14.0)
        assert full_6dof.norm_linear() == pytest.approx(expected)

    def test_norm_linear_negative(self):
        """Norm should be positive even with negative components."""
        t = Twist(u=-3.0, v=-4.0)
        assert t.norm_linear() == 5.0

    # ---- norm_angular ----

    def test_norm_angular_zero(self, zero_twist):
        """Angular norm of zero twist should be 0."""
        assert zero_twist.norm_angular() == 0.0

    def test_norm_angular_yaw_only(self, full_3dof):
        """Angular norm of (0,0,0, 0,0,0.3) should be 0.3."""
        assert full_3dof.norm_angular() == 0.3

    def test_norm_angular_6dof(self, full_6dof):
        """Angular norm of (0.1, 0.2, 0.3) should be sqrt(0.01+0.04+0.09) = sqrt(0.14)."""
        expected = np.sqrt(0.14)
        assert full_6dof.norm_angular() == pytest.approx(expected)

    # ---- speed_2d ----

    def test_speed_2d_zero(self, zero_twist):
        """2D speed of zero twist should be 0."""
        assert zero_twist.speed_2d() == 0.0

    def test_speed_2d_surge_only(self, surge_only):
        """2D speed of pure surge should equal |u|."""
        assert surge_only.speed_2d() == 2.0

    def test_speed_2d_3dof(self, full_3dof):
        """2D speed of (1.0, 0.5) should be sqrt(1.25)."""
        expected = np.sqrt(1.25)
        assert full_3dof.speed_2d() == pytest.approx(expected)

    def test_speed_2d_ignores_w(self):
        """speed_2d should ignore w (heave) component."""
        t = Twist(u=3.0, v=4.0, w=100.0)
        assert t.speed_2d() == 5.0
        assert t.norm_linear() > 5.0

    # ---- is_close ----

    def test_is_close_identical(self, full_6dof):
        """Identical Twists should be close."""
        assert full_6dof.is_close(full_6dof)

    def test_is_close_with_default_tolerance(self):
        """Twists differing by 1e-7 should be close with default tol=1e-6."""
        t1 = Twist(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
        t2 = Twist(1.0 + 1e-7, 2.0, 3.0, 0.1, 0.2, 0.3)
        assert t1.is_close(t2)

    def test_is_close_rejects_large_difference(self):
        """Twists differing by 0.1 should not be close with default tol."""
        t1 = Twist(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
        t2 = Twist(1.1, 2.0, 3.0, 0.1, 0.2, 0.3)
        assert not t1.is_close(t2)

    def test_is_close_custom_tolerance(self):
        """Custom tolerance should be respected."""
        t1 = Twist(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
        t2 = Twist(1.1, 2.0, 3.0, 0.1, 0.2, 0.3)
        # With lin_tol=0.2, 0.1 difference should pass
        assert t1.is_close(t2, lin_tol=0.2)

    def test_is_close_separate_lin_ang_tolerance(self):
        """lin_tol and ang_tol should be independent."""
        t1 = Twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        t2 = Twist(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        # Large angular difference should fail with small ang_tol
        assert not t1.is_close(t2, ang_tol=0.01)
        # But pass with large ang_tol
        assert t1.is_close(t2, ang_tol=2.0)

    def test_is_close_rejects_non_twist(self, zero_twist):
        """is_close should raise TypeError for non-Twist arguments."""
        with pytest.raises(TypeError, match="Expected Twist"):
            zero_twist.is_close((1.0, 2.0, 3.0, 0.1, 0.2, 0.3))


# =============================================================================
# Immutability Tests
# =============================================================================

class TestImmutability:
    """Tests that Twist is truly immutable (frozen dataclass)."""

    def test_cannot_set_u(self, full_3dof):
        """Setting u should raise FrozenInstanceError."""
        with pytest.raises(Exception):
            full_3dof.u = 10.0  # type: ignore[misc]

    def test_cannot_set_r(self, full_3dof):
        """Setting r should raise FrozenInstanceError."""
        with pytest.raises(Exception):
            full_3dof.r = 0.0  # type: ignore[misc]

    def test_cannot_set_linear_property(self, full_3dof):
        """Setting linear property should raise AttributeError."""
        with pytest.raises(AttributeError):
            full_3dof.linear = np.array([1.0, 2.0, 3.0])  # type: ignore[misc]

    def test_cannot_set_angular_property(self, full_3dof):
        """Setting angular property should raise AttributeError."""
        with pytest.raises(AttributeError):
            full_3dof.angular = np.array([0.1, 0.2, 0.3])  # type: ignore[misc]

    def test_cannot_set_velocity_2d_property(self, full_3dof):
        """Setting velocity_2d property should raise AttributeError."""
        with pytest.raises(AttributeError):
            full_3dof.velocity_2d = np.array([1.0, 0.5])  # type: ignore[misc]

    def test_hashable(self, full_3dof, full_6dof):
        """Frozen dataclasses should be hashable (can be dict keys)."""
        d = {full_3dof: "3dof", full_6dof: "6dof"}
        assert d[full_3dof] == "3dof"
        assert d[full_6dof] == "6dof"


# =============================================================================
# Edge Case Tests
# =============================================================================

class TestEdgeCases:
    """Tests for edge cases: negative values, large values, zero, etc."""

    def test_negative_velocities(self):
        """Twist should handle negative velocities (reverse motion)."""
        t = Twist(u=-1.0, v=-2.0, w=-3.0, p=-0.1, q=-0.2, r=-0.3)
        assert t.u == -1.0
        assert t.v == -2.0
        assert t.r == -0.3
        # Norms should still be positive
        assert t.norm_linear() == pytest.approx(np.sqrt(14.0))
        assert t.norm_angular() == pytest.approx(np.sqrt(0.14))

    def test_large_values(self):
        """Twist should handle large velocity values without overflow."""
        t = Twist(u=1e6, v=1e6, r=1e6)
        assert t.speed_2d() == pytest.approx(np.sqrt(2) * 1e6)
        assert t.norm_angular() == 1e6

    def test_numpy_type_preservation(self):
        """to_numpy should return float64 regardless of input type."""
        t = Twist(u=1, v=2, r=3)  # ints
        arr = t.to_numpy()
        assert arr.dtype == np.float64