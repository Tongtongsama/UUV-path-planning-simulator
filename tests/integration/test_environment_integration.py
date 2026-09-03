"""Integration test for YAML -> WorldModel -> Core Path environment queries."""

from pathlib import Path as PathLib
import sys

import numpy as np

PROJECT_ROOT = PathLib(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from core import Path, Pose
from environment import Scenario, WorldModel
from simulation.environment_smoke_demo import (
    DEFAULT_SCENARIO,
    run_environment_smoke_demo,
)


def test_environment_smoke_pipeline() -> None:
    result = run_environment_smoke_demo(DEFAULT_SCENARIO, vehicle_radius=0.5)
    world = result.scenario.world

    assert isinstance(world, WorldModel)
    assert [obstacle.obstacle_id for obstacle in world.obstacles] == [
        "rock_01", "wall_01"
    ]
    assert isinstance(result.safe_path, Path)
    assert all(isinstance(pose, Pose) for pose in result.safe_path)
    assert world.contains(result.safe_path.start, clearance=0.5)
    assert world.contains(result.safe_path.goal, clearance=0.5)
    assert len(result.safe_collisions) > len(result.safe_path)
    assert len(result.colliding_collisions) > len(result.colliding_path)
    assert not any(result.safe_collisions)
    assert any(result.colliding_collisions)
    assert world.clearance([34.0, -12.0]) == 1.0
    np.testing.assert_array_equal(
        world.current_at(result.safe_path.start, time=100.0),
        [0.2, 0.0],
    )


def test_repository_yaml_rebuilds_equivalent_environment() -> None:
    first = Scenario.from_yaml(DEFAULT_SCENARIO)
    second = Scenario.from_yaml(DEFAULT_SCENARIO)

    for position in ([5.0, -2.0], [30.0, -12.0], [57.0, -10.0]):
        assert first.world.contains(position) == second.world.contains(position)
        assert first.world.collides(position) == second.world.collides(position)
        assert first.world.clearance(position) == second.world.clearance(position)
