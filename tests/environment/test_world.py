"""Contract tests for the unified planning WorldModel queries."""

from dataclasses import FrozenInstanceError
from pathlib import Path as PathLib
import sys

import numpy as np
import pytest

PROJECT_ROOT = PathLib(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from core import Pose
from environment import Boundary, ConstantCurrent, NoCurrent, Obstacle, WorldModel


@pytest.fixture
def circle() -> Obstacle:
    return Obstacle.circle("rock", [3.0, -3.0], 1.0)


@pytest.fixture
def wall() -> Obstacle:
    return Obstacle.rectangle(
        "wall", x_min=7.0, x_max=8.0, z_min=-8.0, z_max=-2.0
    )


@pytest.fixture
def world(circle: Obstacle, wall: Obstacle) -> WorldModel:
    return WorldModel(
        boundary=Boundary.rectangle(
            x_min=0.0, x_max=10.0, z_min=-10.0, z_max=0.0
        ),
        obstacles=[circle, wall],
        current=ConstantCurrent([0.2, 0.0]),
    )


def test_contains_delegates_boundary_semantics(world: WorldModel) -> None:
    assert world.contains(Pose(x=0.0, y=500.0, z=-5.0))
    assert not world.contains([0.0, -5.0], clearance=0.1)
    assert not world.contains([-0.1, -5.0])


def test_collides_checks_all_obstacles_and_vehicle_radius(world: WorldModel) -> None:
    assert world.collides([3.0, -3.0])
    assert world.collides([7.0, -5.0])
    assert not world.collides([5.0, -5.0])
    assert world.collides([5.0, -3.0], radius=1.0)


def test_clearance_returns_nearest_surface_distance(world: WorldModel) -> None:
    assert world.clearance([3.0, -3.0]) == 0.0
    assert world.clearance([5.0, -3.0]) == pytest.approx(1.0)


def test_clearance_without_obstacles_is_positive_infinity() -> None:
    empty_world = WorldModel(
        Boundary.rectangle(x_min=0.0, x_max=1.0, z_min=-1.0, z_max=0.0)
    )

    assert empty_world.clearance([0.5, -0.5]) == float("inf")
    assert isinstance(empty_world.current, NoCurrent)


def test_nearby_obstacles_preserves_scenario_order(
    world: WorldModel, circle: Obstacle, wall: Obstacle
) -> None:
    assert world.nearby_obstacles([5.0, -3.0], radius=1.0) == (circle,)
    assert world.nearby_obstacles([5.0, -3.0], radius=2.0) == (circle, wall)
    assert world.nearby_obstacles([5.0, -3.0], radius=0.5) == ()


def test_current_at_delegates_current_field(world: WorldModel) -> None:
    np.testing.assert_array_equal(world.current_at([5.0, -5.0], 3.0), [0.2, 0.0])


def test_world_copies_obstacle_iterable_and_is_immutable(
    circle: Obstacle, wall: Obstacle
) -> None:
    source = [circle]
    world = WorldModel(
        Boundary.rectangle(x_min=0.0, x_max=10.0, z_min=-10.0, z_max=0.0),
        source,
    )
    source.append(wall)

    assert world.obstacles == (circle,)
    with pytest.raises(FrozenInstanceError):
        world.obstacles = ()  # type: ignore[misc]


def test_duplicate_obstacle_ids_are_rejected(circle: Obstacle) -> None:
    duplicate = Obstacle.circle("rock", [8.0, -8.0], 0.5)
    boundary = Boundary.rectangle(x_min=0.0, x_max=10.0, z_min=-10.0, z_max=0.0)

    with pytest.raises(ValueError, match="unique"):
        WorldModel(boundary, [circle, duplicate])


def test_world_rejects_wrong_component_types(circle: Obstacle) -> None:
    boundary = Boundary.rectangle(x_min=0.0, x_max=10.0, z_min=-10.0, z_max=0.0)

    with pytest.raises(TypeError, match="boundary"):
        WorldModel("not a boundary")  # type: ignore[arg-type]
    with pytest.raises(TypeError, match="obstacles"):
        WorldModel(boundary, [circle, "not an obstacle"])  # type: ignore[list-item]
    with pytest.raises(TypeError, match="current"):
        WorldModel(boundary, [circle], current="not a current")  # type: ignore[arg-type]


@pytest.mark.parametrize("radius", [-1.0, np.nan, np.inf])
def test_world_radius_queries_reject_invalid_values(
    world: WorldModel, radius: float
) -> None:
    with pytest.raises(ValueError):
        world.collides([5.0, -5.0], radius=radius)
    with pytest.raises(ValueError):
        world.nearby_obstacles([5.0, -5.0], radius=radius)

