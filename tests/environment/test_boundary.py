"""Contract tests for allowed operating-region boundaries."""

from pathlib import Path as PathLib
import sys

import numpy as np
import pytest

PROJECT_ROOT = PathLib(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from core import Pose
from environment import Boundary


@pytest.fixture
def rectangle() -> Boundary:
    return Boundary.rectangle(x_min=0.0, x_max=10.0, z_min=-10.0, z_max=0.0)


def test_inside_outside_and_boundary_contact_semantics(rectangle: Boundary) -> None:
    assert rectangle.contains([5.0, -5.0])
    assert rectangle.contains([0.0, -5.0])  # boundary contact is valid
    assert rectangle.contains(Pose(x=10.0, y=999.0, z=-10.0))
    assert not rectangle.contains([-0.01, -5.0])


def test_positive_clearance_requires_complete_footprint(rectangle: Boundary) -> None:
    assert rectangle.contains([1.0, -5.0], clearance=1.0)
    assert not rectangle.contains([0.5, -5.0], clearance=1.0)
    assert not rectangle.contains([0.0, -5.0], clearance=0.01)


def test_distance_to_boundary_is_unsigned(rectangle: Boundary) -> None:
    assert rectangle.distance_to_boundary([5.0, -5.0]) == pytest.approx(5.0)
    assert rectangle.distance_to_boundary([0.0, -5.0]) == 0.0
    assert rectangle.distance_to_boundary([-2.0, -5.0]) == pytest.approx(2.0)


def test_polygon_boundary() -> None:
    boundary = Boundary.polygon([[0.0, 0.0], [4.0, 0.0], [2.0, -3.0]])

    assert boundary.contains([2.0, -1.0])
    assert not boundary.contains([0.0, -3.0])


def test_bounds_do_not_expose_shapely_geometry(rectangle: Boundary) -> None:
    assert rectangle.bounds == (0.0, -10.0, 10.0, 0.0)


@pytest.mark.parametrize("clearance", [-1.0, np.inf, np.nan])
def test_contains_rejects_invalid_clearance(
    rectangle: Boundary, clearance: float
) -> None:
    with pytest.raises(ValueError):
        rectangle.contains([5.0, -5.0], clearance=clearance)


@pytest.mark.parametrize(
    "vertices",
    [
        [[0.0, 0.0], [1.0, 1.0]],
        [[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]],
        [[0.0, 0.0], [2.0, 2.0], [0.0, 2.0], [2.0, 0.0]],
    ],
)
def test_polygon_rejects_invalid_geometry(vertices: list[list[float]]) -> None:
    with pytest.raises(ValueError):
        Boundary.polygon(vertices)


def test_rectangle_rejects_reversed_bounds() -> None:
    with pytest.raises(ValueError, match="x_min"):
        Boundary.rectangle(x_min=1.0, x_max=0.0, z_min=-1.0, z_max=0.0)

