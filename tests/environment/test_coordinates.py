"""Tests for the centralized Core ENU <-> planning x-z mapping."""

from pathlib import Path as PathLib
import sys

import numpy as np
import pytest

PROJECT_ROOT = PathLib(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from core import Pose
from environment.coordinates import as_xz, pose_to_xz, xz_to_pose


def test_pose_to_xz_ignores_core_y() -> None:
    pose = Pose(x=12.5, y=999.0, z=-8.0, yaw=0.4)

    np.testing.assert_array_equal(pose_to_xz(pose), [12.5, -8.0])
    np.testing.assert_array_equal(as_xz(pose), [12.5, -8.0])


def test_as_xz_accepts_an_explicit_planning_position() -> None:
    np.testing.assert_array_equal(as_xz((3.0, -2.0)), [3.0, -2.0])
    assert as_xz(np.array([3, -2])).dtype == np.float64


@pytest.mark.parametrize(
    "position",
    [(), (1.0,), (1.0, 2.0, 3.0), [[1.0, 2.0]], np.zeros((2, 1))],
)
def test_as_xz_rejects_wrong_shape(position: object) -> None:
    with pytest.raises(ValueError, match="shape"):
        as_xz(position)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    "position",
    [(np.nan, 0.0), (0.0, np.inf), (-np.inf, 0.0)],
)
def test_as_xz_rejects_non_finite_coordinates(position: tuple[float, float]) -> None:
    with pytest.raises(ValueError, match="finite"):
        as_xz(position)


def test_pose_to_xz_requires_pose() -> None:
    with pytest.raises(TypeError, match="Expected Pose"):
        pose_to_xz((1.0, 2.0))  # type: ignore[arg-type]


def test_xz_to_pose_preserves_explicit_enu_components() -> None:
    pose = xz_to_pose(
        [4.0, -6.0],
        world_y=2.0,
        roll=0.1,
        pitch=-0.2,
        yaw=0.3,
    )

    assert pose == Pose(4.0, 2.0, -6.0, 0.1, -0.2, 0.3)
    np.testing.assert_array_equal(pose_to_xz(pose), [4.0, -6.0])


def test_xz_to_pose_rejects_non_finite_extra_components() -> None:
    with pytest.raises(ValueError, match="finite"):
        xz_to_pose([0.0, 0.0], world_y=np.nan)

