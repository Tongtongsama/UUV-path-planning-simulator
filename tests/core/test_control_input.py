"""
Unit tests for core.control_input.ControlInput.

Core contract under test:
    ControlInput is ALWAYS a complete 6-DOF generalized-force container.
    It does NOT know which DOFs are active / directly actuated.
    That decision belongs to the Physics Engine and vehicle configuration.

Tests cover:
    - Construction (default, factory methods)
    - The 6-DOF completeness contract
    - Properties (force_vector, moment_vector, generalized_force)
    - Conversion methods (to_list, to_numpy)
    - Physics methods (norm_force, norm_moment, is_zero, is_close)
    - Immutability (frozen dataclass)
"""

import pytest
import numpy as np
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from core.control_input import ControlInput


# =============================================================================
# Fixtures
# =============================================================================

@pytest.fixture
def zero_input():
    """Zero control input (all components zero)."""
    return ControlInput()


@pytest.fixture
def surge_only():
    """Pure surge force: tau_x=10.0, all others zero."""
    return ControlInput(tau_x=10.0)


@pytest.fixture
def horizontal_fully_actuated():
    """Fully actuated horizontal plane: [X, Y, N] non-zero, others zero."""
    return ControlInput(tau_x=10.0, tau_y=5.0, tau_n=2.0)


@pytest.fixture
def vertical_underactuated():
    """Underactuated vertical plane: [X, Z, M] non-zero, others zero."""
    return ControlInput(tau_x=10.0, tau_z=8.0, tau_m=1.5)


@pytest.fixture
def full_6dof():
    """All 6 components non-zero."""
    return ControlInput(tau_x=1.0, tau_y=2.0, tau_z=3.0,
                        tau_k=0.1, tau_m=0.2, tau_n=0.3)


# =============================================================================
# Core Contract: 6-DOF Completeness
# =============================================================================

class Test6DOFContract:
    """
    Tests that verify ControlInput is ALWAYS a complete 6-DOF container,
    regardless of which components are non-zero.
    """

    def test_default_has_6_fields(self):
        """A default ControlInput must expose all 6 tau_* attributes."""
        ci = ControlInput()
        assert hasattr(ci, "tau_x")
        assert hasattr(ci, "tau_y")
        assert hasattr(ci, "tau_z")
        assert hasattr(ci, "tau_k")
        assert hasattr(ci, "tau_m")
        assert hasattr(ci, "tau_n")

    def test_fully_actuated_still_has_all_fields(self, horizontal_fully_actuated):
        """Even when only 3 components are non-zero, all 6 fields exist."""
        ci = horizontal_fully_actuated
        # Active
        assert ci.tau_x == 10.0
        assert ci.tau_y == 5.0
        assert ci.tau_n == 2.0
        # Inactive but present
        assert ci.tau_z == 0.0
        assert ci.tau_k == 0.0
        assert ci.tau_m == 0.0

    def test_underactuated_still_has_all_fields(self, vertical_underactuated):
        """Underactuated vertical plane: inactive DOFs are still present as 0.0."""
        ci = vertical_underactuated
        assert ci.tau_x == 10.0
        assert ci.tau_z == 8.0
        assert ci.tau_m == 1.5
        # Inactive but present
        assert ci.tau_y == 0.0
        assert ci.tau_k == 0.0
        assert ci.tau_n == 0.0

    def test_generalized_force_always_length_6(self, horizontal_fully_actuated,
                                                vertical_underactuated, full_6dof):
        """generalized_force must always return a (6,) array, regardless of
        how many components are non-zero."""
        for ci in [horizontal_fully_actuated, vertical_underactuated, full_6dof,
                   ControlInput.zero()]:
            gf = ci.generalized_force
            assert isinstance(gf, np.ndarray)
            assert gf.shape == (6,), f"Expected (6,), got {gf.shape}"

    def test_to_list_always_length_6(self, horizontal_fully_actuated,
                                      vertical_underactuated, full_6dof):
        """to_list must always return a list of length 6."""
        for ci in [horizontal_fully_actuated, vertical_underactuated, full_6dof,
                   ControlInput.zero()]:
            lst = ci.to_list()
            assert isinstance(lst, list)
            assert len(lst) == 6, f"Expected 6, got {len(lst)}"

    def test_to_numpy_always_shape_6(self, horizontal_fully_actuated,
                                      vertical_underactuated, full_6dof):
        """to_numpy must always return a (6,) array."""
        for ci in [horizontal_fully_actuated, vertical_underactuated, full_6dof,
                   ControlInput.zero()]:
            arr = ci.to_numpy()
            assert isinstance(arr, np.ndarray)
            assert arr.shape == (6,), f"Expected (6,), got {arr.shape}"

    def test_no_dof_config_methods(self):
        """
        ControlInput must NOT expose methods that imply knowledge of
        which DOFs are active (no 'is_horizontal', 'is_vertical', etc.).
        """
        ci = ControlInput()
        forbidden = [
            "is_horizontal", "is_vertical", "is_fully_actuated",
            "is_underactuated", "active_dofs", "dof_mask",
            "horizontal_components", "vertical_components",
        ]
        for attr in forbidden:
            assert not hasattr(ci, attr), (
                f"ControlInput must not expose '{attr}' — "
                f"DOF configuration belongs to Physics Engine"
            )


# =============================================================================
# Construction Tests
# =============================================================================

class TestConstruction:
    """Tests for ControlInput construction."""

    def test_default_is_zero(self):
        """A default ControlInput should have all zeros."""
        ci = ControlInput()
        assert ci.tau_x == 0.0
        assert ci.tau_y == 0.0
        assert ci.tau_z == 0.0
        assert ci.tau_k == 0.0
        assert ci.tau_m == 0.0
        assert ci.tau_n == 0.0

    def test_zero_factory(self):
        """zero() should return a ControlInput with all zeros."""
        ci = ControlInput.zero()
        assert ci.tau_x == 0.0
        assert ci.tau_y == 0.0
        assert ci.tau_z == 0.0
        assert ci.tau_k == 0.0
        assert ci.tau_m == 0.0
        assert ci.tau_n == 0.0

    def test_explicit_construction(self):
        """ControlInput should accept all 6 arguments positionally."""
        ci = ControlInput(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
        assert ci.tau_x == 1.0
        assert ci.tau_y == 2.0
        assert ci.tau_z == 3.0
        assert ci.tau_k == 0.1
        assert ci.tau_m == 0.2
        assert ci.tau_n == 0.3

    def test_keyword_construction(self):
        """ControlInput should accept keyword arguments with defaults."""
        ci = ControlInput(tau_x=5.0, tau_n=0.5)
        assert ci.tau_x == 5.0
        assert ci.tau_y == 0.0  # default
        assert ci.tau_z == 0.0  # default
        assert ci.tau_k == 0.0  # default
        assert ci.tau_m == 0.0  # default
        assert ci.tau_n == 0.5

    def test_from_list(self):
        """from_list should construct from a 6-element list."""
        data = [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]
        ci = ControlInput.from_list(data)
        assert ci.tau_x == 1.0
        assert ci.tau_y == 2.0
        assert ci.tau_z == 3.0
        assert ci.tau_k == 0.1
        assert ci.tau_m == 0.2
        assert ci.tau_n == 0.3

    def test_from_list_rejects_wrong_length(self):
        """from_list should raise ValueError if length != 6."""
        with pytest.raises(ValueError, match="Expected 6 elements"):
            ControlInput.from_list([1.0, 2.0, 3.0])

    def test_from_numpy(self):
        """from_numpy should construct from a (6,) numpy array."""
        arr = np.array([1.0, 2.0, 3.0, 0.1, 0.2, 0.3])
        ci = ControlInput.from_numpy(arr)
        assert ci.tau_x == 1.0
        assert ci.tau_y == 2.0
        assert ci.tau_z == 3.0
        assert ci.tau_k == 0.1
        assert ci.tau_m == 0.2
        assert ci.tau_n == 0.3

    def test_from_numpy_rejects_wrong_shape(self):
        """from_numpy should raise ValueError if shape != (6,)."""
        arr = np.array([1.0, 2.0, 3.0])
        with pytest.raises(ValueError, match="Expected shape"):
            ControlInput.from_numpy(arr)


# =============================================================================
# Property Tests
# =============================================================================

class TestProperties:
    """Tests for ControlInput properties."""

    def test_force_vector(self, full_6dof):
        """force_vector should return [tau_x, tau_y, tau_z]."""
        fv = full_6dof.force_vector
        assert isinstance(fv, np.ndarray)
        assert fv.shape == (3,)
        np.testing.assert_array_equal(fv, np.array([1.0, 2.0, 3.0]))

    def test_force_vector_zero(self, zero_input):
        """force_vector of zero input should be [0, 0, 0]."""
        np.testing.assert_array_equal(zero_input.force_vector, np.zeros(3))

    def test_moment_vector(self, full_6dof):
        """moment_vector should return [tau_k, tau_m, tau_n]."""
        mv = full_6dof.moment_vector
        assert isinstance(mv, np.ndarray)
        assert mv.shape == (3,)
        np.testing.assert_array_equal(mv, np.array([0.1, 0.2, 0.3]))

    def test_moment_vector_zero(self, zero_input):
        """moment_vector of zero input should be [0, 0, 0]."""
        np.testing.assert_array_equal(zero_input.moment_vector, np.zeros(3))

    def test_generalized_force(self, full_6dof):
        """generalized_force should return full (6,) vector."""
        gf = full_6dof.generalized_force
        assert isinstance(gf, np.ndarray)
        assert gf.shape == (6,)
        np.testing.assert_array_equal(
            gf, np.array([1.0, 2.0, 3.0, 0.1, 0.2, 0.3])
        )

    def test_generalized_force_zero(self, zero_input):
        """generalized_force of zero input should be all zeros."""
        np.testing.assert_array_equal(zero_input.generalized_force, np.zeros(6))


# =============================================================================
# Conversion Tests
# =============================================================================

class TestConversion:
    """Tests for conversion methods."""

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
        np.testing.assert_array_equal(
            arr, np.array([1.0, 2.0, 3.0, 0.1, 0.2, 0.3])
        )

    def test_roundtrip_list(self, full_6dof):
        """ControlInput -> to_list -> from_list should be equivalent."""
        lst = full_6dof.to_list()
        reconstructed = ControlInput.from_list(lst)
        assert reconstructed.is_close(full_6dof)

    def test_roundtrip_numpy(self, full_6dof):
        """ControlInput -> to_numpy -> from_numpy should be equivalent."""
        arr = full_6dof.to_numpy()
        reconstructed = ControlInput.from_numpy(arr)
        assert reconstructed.is_close(full_6dof)

    def test_to_numpy_equals_generalized_force(self, full_6dof):
        """to_numpy() should be identical to generalized_force property."""
        np.testing.assert_array_equal(full_6dof.to_numpy(),
                                       full_6dof.generalized_force)


# =============================================================================
# Physics Tests
# =============================================================================

class TestPhysics:
    """Tests for physics methods: norm_force, norm_moment, is_zero, is_close."""

    # ---- norm_force ----

    def test_norm_force_zero(self, zero_input):
        assert zero_input.norm_force() == 0.0

    def test_norm_force_surge_only(self, surge_only):
        assert surge_only.norm_force() == 10.0

    def test_norm_force_full(self, full_6dof):
        expected = np.sqrt(1.0 + 4.0 + 9.0)  # sqrt(14)
        assert full_6dof.norm_force() == pytest.approx(expected)

    def test_norm_force_ignores_moments(self):
        """norm_force should NOT include moment components."""
        ci = ControlInput(tau_x=3.0, tau_y=4.0, tau_n=1000.0)
        assert ci.norm_force() == 5.0

    # ---- norm_moment ----

    def test_norm_moment_zero(self, zero_input):
        assert zero_input.norm_moment() == 0.0

    def test_norm_moment_full(self, full_6dof):
        expected = np.sqrt(0.01 + 0.04 + 0.09)  # sqrt(0.14)
        assert full_6dof.norm_moment() == pytest.approx(expected)

    def test_norm_moment_ignores_forces(self):
        """norm_moment should NOT include force components."""
        ci = ControlInput(tau_m=0.3, tau_n=0.4, tau_x=1000.0)
        assert ci.norm_moment() == 0.5

    # ---- is_zero ----

    def test_is_zero_default(self, zero_input):
        assert zero_input.is_zero() is True

    def test_is_zero_factory(self):
        assert ControlInput.zero().is_zero() is True

    def test_is_zero_surge_nonzero(self, surge_only):
        assert surge_only.is_zero() is False

    def test_is_zero_small_nonzero(self):
        """A value below default tol (1e-9) should still be zero."""
        ci = ControlInput(tau_x=1e-10)
        assert ci.is_zero() is True

    def test_is_zero_custom_tolerance(self):
        """Custom tolerance should be respected."""
        ci = ControlInput(tau_x=0.01)
        assert ci.is_zero(tol=1e-9) is False
        assert ci.is_zero(tol=0.1) is True

    def test_is_zero_requires_all_components(self):
        """is_zero should return False if ANY component is non-zero."""
        ci = ControlInput(tau_n=0.001)
        assert ci.is_zero() is False

    # ---- is_close ----

    def test_is_close_identical(self, full_6dof):
        assert full_6dof.is_close(full_6dof)

    def test_is_close_small_difference(self):
        ci1 = ControlInput(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
        ci2 = ControlInput(1.0 + 1e-7, 2.0, 3.0, 0.1, 0.2, 0.3)
        assert ci1.is_close(ci2)

    def test_is_close_large_difference(self):
        ci1 = ControlInput(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
        ci2 = ControlInput(1.1, 2.0, 3.0, 0.1, 0.2, 0.3)
        assert not ci1.is_close(ci2)

    def test_is_close_custom_tolerance(self):
        ci1 = ControlInput(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
        ci2 = ControlInput(1.1, 2.0, 3.0, 0.1, 0.2, 0.3)
        assert ci1.is_close(ci2, force_tol=0.2)

    def test_is_close_separate_force_moment_tolerance(self):
        """force_tol and moment_tol should be independent."""
        ci1 = ControlInput.zero()
        ci2 = ControlInput(tau_n=1.0)  # moment differs by 1.0
        assert not ci1.is_close(ci2, moment_tol=0.01)
        assert ci1.is_close(ci2, moment_tol=2.0)

    def test_is_close_rejects_non_control_input(self):
        with pytest.raises(TypeError, match="Expected ControlInput"):
            ControlInput.zero().is_close((1.0, 2.0, 3.0, 0.1, 0.2, 0.3))


# =============================================================================
# Immutability Tests
# =============================================================================

class TestImmutability:
    """Tests that ControlInput is truly immutable."""

    def test_cannot_set_tau_x(self, full_6dof):
        with pytest.raises(Exception):
            full_6dof.tau_x = 100.0  # type: ignore[misc]

    def test_cannot_set_tau_n(self, full_6dof):
        with pytest.raises(Exception):
            full_6dof.tau_n = 0.0  # type: ignore[misc]

    def test_cannot_set_force_vector(self, full_6dof):
        with pytest.raises(AttributeError):
            full_6dof.force_vector = np.array([1.0, 2.0, 3.0])  # type: ignore[misc]

    def test_cannot_set_generalized_force(self, full_6dof):
        with pytest.raises(AttributeError):
            full_6dof.generalized_force = np.zeros(6)  # type: ignore[misc]

    def test_hashable(self, full_6dof, zero_input):
        d = {full_6dof: "full", zero_input: "zero"}
        assert d[full_6dof] == "full"
        assert d[zero_input] == "zero"


# =============================================================================
# Edge Case Tests
# =============================================================================

class TestEdgeCases:
    """Edge case tests."""

    def test_negative_forces(self):
        """Negative forces represent reverse thrust."""
        ci = ControlInput(tau_x=-10.0, tau_y=-5.0, tau_n=-2.0)
        assert ci.tau_x == -10.0
        assert ci.tau_y == -5.0
        assert ci.tau_n == -2.0
        # Norms should still be positive
        assert ci.norm_force() == pytest.approx(np.sqrt(100 + 25))
        assert ci.norm_moment() == 2.0

    def test_large_values(self):
        """Should handle large values without overflow."""
        ci = ControlInput(tau_x=1e6, tau_y=1e6, tau_n=1e6)
        assert ci.norm_force() == pytest.approx(np.sqrt(2) * 1e6)

    def test_mixed_signs(self):
        """Components can have different signs."""
        ci = ControlInput(tau_x=10.0, tau_y=-5.0, tau_z=0.0,
                          tau_k=0.0, tau_m=0.0, tau_n=-2.0)
        assert ci.norm_force() == pytest.approx(np.sqrt(100 + 25))
        assert ci.norm_moment() == 2.0

    def test_numpy_type_preservation(self):
        """to_numpy should return float64."""
        ci = ControlInput(tau_x=1, tau_y=2, tau_n=3)  # ints
        arr = ci.to_numpy()
        assert arr.dtype == np.float64