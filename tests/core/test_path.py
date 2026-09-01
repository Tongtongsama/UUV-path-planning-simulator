"""Unit tests for core.path.Path."""

import pytest
import numpy as np
import sys
from pathlib import Path as PathLib

PROJECT_ROOT = PathLib(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from core.pose import Pose
from core.path import Path


# =============================================================================
# Fixtures
# =============================================================================

@pytest.fixture
def p0():
    return Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@pytest.fixture
def p1():
    return Pose(3.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@pytest.fixture
def p2():
    return Pose(3.0, 4.0, 0.0, 0.0, 0.0, np.pi / 2)


@pytest.fixture
def straight_path(p0, p1):
    return Path(poses=(p0, p1))


@pytest.fixture
def three_point_path(p0, p1, p2):
    return Path(poses=(p0, p1, p2))


# =============================================================================
# Construction Tests
# =============================================================================

class TestConstruction:

    def test_construct_from_tuple(self, p0, p1):
        path = Path(poses=(p0, p1))
        assert len(path) == 2

    def test_empty_raises(self):
        with pytest.raises(ValueError, match="at least one Pose"):
            Path(poses=())

    def test_non_pose_raises(self, p0):
        with pytest.raises(TypeError, match="Pose instances"):
            Path(poses=(p0, "not_a_pose"))  # type: ignore[arg-type]

    def test_from_list(self, p0, p1):
        path = Path.from_list([p0, p1])
        assert len(path) == 2
        assert path.start.is_close(p0)

    def test_from_list_empty_raises(self):
        with pytest.raises(ValueError, match="at least one Pose"):
            Path.from_list([])

    def test_single_pose_allowed(self, p0):
        path = Path(poses=(p0,))
        assert len(path) == 1
        assert path.start.is_close(p0)
        assert path.goal.is_close(p0)

    def test_pose_instances_stored_directly(self, p0, p1):
        path = Path(poses=(p0, p1))
        assert path.poses[0] is p0
        assert path.poses[1] is p1


# =============================================================================
# Property Tests
# =============================================================================

class TestProperties:

    def test_start(self, three_point_path, p0):
        assert three_point_path.start.is_close(p0)

    def test_goal(self, three_point_path, p2):
        assert three_point_path.goal.is_close(p2)

    def test_start_equals_goal_single_point(self, p0):
        path = Path(poses=(p0,))
        assert path.start.is_close(p0)
        assert path.goal.is_close(p0)
        assert path.start.is_close(path.goal)


# =============================================================================
# Geometry Tests
# =============================================================================

class TestGeometry:

    def test_length_straight(self, straight_path):
        assert straight_path.length() == 3.0

    def test_length_three_point(self, three_point_path):
        # p0->p1: 3.0, p1->p2: sqrt(0 + 16 + 0) = 4.0
        assert three_point_path.length() == pytest.approx(7.0)

    def test_length_single_point(self, p0):
        path = Path(poses=(p0,))
        assert path.length() == 0.0

    def test_length_orientation_ignored(self):
        """Orientation change without position change contributes 0 to length."""
        p1 = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        p2 = Pose(0.0, 0.0, 0.0, 0.0, 0.0, np.pi)
        path = Path(poses=(p1, p2))
        assert path.length() == 0.0

    def test_length_with_z(self):
        """Length includes z-component."""
        p1 = Pose(0.0, 0.0, 0.0)
        p2 = Pose(0.0, 0.0, 5.0)
        path = Path(poses=(p1, p2))
        assert path.length() == 5.0


# =============================================================================
# Comparison Tests
# =============================================================================

class TestComparison:

    def test_is_close_identical(self, three_point_path):
        assert three_point_path.is_close(three_point_path)

    def test_is_close_different_length(self, straight_path, three_point_path):
        assert not straight_path.is_close(three_point_path)

    def test_is_close_same_structure(self, p0, p1):
        path1 = Path(poses=(p0, p1))
        path2 = Path(poses=(Pose(0.0, 0.0, 0.0), Pose(3.0, 0.0, 0.0)))
        assert path1.is_close(path2)

    def test_is_close_different_values(self, p0, p1, p2):
        path1 = Path(poses=(p0, p1))
        path2 = Path(poses=(p0, p2))
        assert not path1.is_close(path2)

    def test_is_close_tolerance(self):
        p1 = Pose(0.0, 0.0, 0.0)
        p2 = Pose(0.0, 0.0, 0.0)
        p3 = Pose(1e-5, 0.0, 0.0)
        path1 = Path(poses=(p1, p2))
        path2 = Path(poses=(p1, p3))
        assert not path1.is_close(path2)
        assert path1.is_close(path2, pos_tol=1e-4)

    def test_is_close_rejects_non_path(self, straight_path):
        with pytest.raises(TypeError, match="Expected Path"):
            straight_path.is_close("not a path")  # type: ignore[arg-type]


# =============================================================================
# Sequence Protocol Tests
# =============================================================================

class TestSequenceProtocol:

    def test_len(self, three_point_path):
        assert len(three_point_path) == 3

    def test_getitem(self, three_point_path, p0, p1, p2):
        assert three_point_path[0].is_close(p0)
        assert three_point_path[1].is_close(p1)
        assert three_point_path[2].is_close(p2)

    def test_iter(self, three_point_path):
        poses = list(three_point_path)
        assert len(poses) == 3
        assert all(isinstance(p, Pose) for p in poses)


# =============================================================================
# Immutability Tests
# =============================================================================

class TestImmutability:

    def test_cannot_replace_poses(self, straight_path):
        with pytest.raises(Exception):
            straight_path.poses = ()  # type: ignore[misc]

    def test_hashable(self, straight_path, three_point_path):
        d = {straight_path: "straight", three_point_path: "three"}
        assert d[straight_path] == "straight"
