"""Contract tests for static obstacle wrappers."""

from pathlib import Path as PathLib
import sys

import numpy as np
import pytest

PROJECT_ROOT = PathLib(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from core import Pose
from environment import Obstacle


def test_circle_collision_contact_and_distance() -> None:
    obstacle = Obstacle.circle("rock", [10.0, -5.0], 2.0)

    assert obstacle.collides([10.0, -5.0])
    assert obstacle.collides([12.0, -5.0])  # touching counts
    assert not obstacle.collides([12.01, -5.0])
    assert obstacle.distance_to([10.0, -5.0]) == 0.0
    assert obstacle.distance_to([13.0, -5.0]) == pytest.approx(1.0)


def test_vehicle_radius_can_turn_clear_point_into_collision() -> None:
    obstacle = Obstacle.circle("rock", [0.0, 0.0], 1.0)

    assert not obstacle.collides([2.0, 0.0])
    assert obstacle.collides([2.0, 0.0], radius=1.0)


def test_rectangle_accepts_core_pose_using_x_and_z() -> None:
    obstacle = Obstacle.rectangle(
        "wall", x_min=2.0, x_max=4.0, z_min=-8.0, z_max=-3.0
    )

    assert obstacle.collides(Pose(x=3.0, y=999.0, z=-4.0))
    assert obstacle.collides([2.0, -4.0])  # edge contact
    assert not obstacle.collides([1.9, -4.0])
    assert obstacle.bounds == (2.0, -8.0, 4.0, -3.0)


def test_polygon_and_metadata_are_immutable_at_mapping_level() -> None:
    source_metadata = {"kind": "reef"}
    obstacle = Obstacle.polygon(
        "reef", [[0.0, 0.0], [3.0, 0.0], [0.0, 3.0]], metadata=source_metadata
    )
    source_metadata["kind"] = "changed"

    assert obstacle.collides([0.5, 0.5])
    assert obstacle.metadata["kind"] == "reef"
    with pytest.raises(TypeError):
        obstacle.metadata["kind"] = "changed"  # type: ignore[index]


@pytest.mark.parametrize("obstacle_id", ["", "   ", None])
def test_obstacle_rejects_invalid_id(obstacle_id: object) -> None:
    with pytest.raises(ValueError, match="obstacle_id"):
        Obstacle.circle(obstacle_id, [0.0, 0.0], 1.0)  # type: ignore[arg-type]


@pytest.mark.parametrize("radius", [0.0, -1.0, np.inf, np.nan])
def test_circle_rejects_invalid_radius(radius: float) -> None:
    with pytest.raises(ValueError):
        Obstacle.circle("bad", [0.0, 0.0], radius)


@pytest.mark.parametrize(
    "bounds",
    [
        (1.0, 1.0, -2.0, 0.0),
        (2.0, 1.0, -2.0, 0.0),
        (0.0, 1.0, -1.0, -1.0),
        (0.0, np.inf, -1.0, 0.0),
    ],
)
def test_rectangle_rejects_invalid_bounds(bounds: tuple[float, ...]) -> None:
    with pytest.raises(ValueError):
        Obstacle.rectangle(
            "bad", x_min=bounds[0], x_max=bounds[1],
            z_min=bounds[2], z_max=bounds[3]
        )


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
        Obstacle.polygon("bad", vertices)


@pytest.mark.parametrize("radius", [-1.0, np.inf, np.nan])
def test_collision_rejects_invalid_vehicle_radius(radius: float) -> None:
    obstacle = Obstacle.circle("rock", [0.0, 0.0], 1.0)
    with pytest.raises(ValueError):
        obstacle.collides([2.0, 0.0], radius=radius)

