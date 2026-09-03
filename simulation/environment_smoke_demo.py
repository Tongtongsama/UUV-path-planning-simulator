"""Executable Environment v0.3 interface-validation scenario.

This is not a planner. It loads one YAML scenario and submits known safe and
known colliding Core Paths to the public WorldModel query interface.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil
from pathlib import Path as FilePath

from core import Path, Pose
from environment import Scenario


DEFAULT_SCENARIO = (
    FilePath(__file__).resolve().parents[1]
    / "config"
    / "scenarios"
    / "simple_static_world.yaml"
)


@dataclass(frozen=True, slots=True)
class EnvironmentSmokeResult:
    """Immutable result of the Environment interface smoke scenario."""

    scenario: Scenario
    safe_path: Path
    colliding_path: Path
    safe_collisions: tuple[bool, ...]
    colliding_collisions: tuple[bool, ...]


def _pose(x: float, z: float) -> Pose:
    return Pose.from_position_and_yaw(x=x, y=0.0, z=z, yaw=0.0)


def _sample_path(path: Path, maximum_spacing: float = 0.25) -> tuple[Pose, ...]:
    """Densely sample straight test segments; this is not a planner."""
    if maximum_spacing <= 0.0:
        raise ValueError("maximum_spacing must be positive")

    samples: list[Pose] = [path.start]
    for start, end in zip(path.poses, path.poses[1:]):
        steps = max(1, ceil(start.distance_to(end) / maximum_spacing))
        for step in range(1, steps + 1):
            fraction = step / steps
            samples.append(
                _pose(
                    start.x + fraction * (end.x - start.x),
                    start.z + fraction * (end.z - start.z),
                )
            )
    return tuple(samples)


def run_environment_smoke_demo(
    scenario_path: str | FilePath = DEFAULT_SCENARIO,
    vehicle_radius: float = 0.5,
) -> EnvironmentSmokeResult:
    """Load the example scenario and exercise it with two deterministic paths."""
    scenario = Scenario.from_yaml(scenario_path)
    world = scenario.world

    # This path passes above both obstacles in the vertical x-z plane.
    safe_path = Path.from_list(
        [_pose(5.0, -2.0), _pose(25.0, -2.0), _pose(50.0, -2.0), _pose(80.0, -2.0)]
    )
    # This path deliberately places one waypoint at the centre of rock_01.
    colliding_path = Path.from_list(
        [_pose(5.0, -12.0), _pose(30.0, -12.0), _pose(80.0, -12.0)]
    )

    safe_samples = _sample_path(safe_path)
    colliding_samples = _sample_path(colliding_path)

    for pose in (*safe_samples, *colliding_samples):
        if not world.contains(pose, clearance=vehicle_radius):
            raise ValueError("Demo path contains a sampled point outside the world boundary")

    safe_collisions = tuple(
        world.collides(pose, radius=vehicle_radius) for pose in safe_samples
    )
    colliding_collisions = tuple(
        world.collides(pose, radius=vehicle_radius) for pose in colliding_samples
    )

    # Exercise the remaining stable queries without reaching into Shapely.
    for pose in safe_samples:
        world.clearance(pose)
        world.nearby_obstacles(pose, radius=10.0)
        world.current_at(pose, time=0.0)

    return EnvironmentSmokeResult(
        scenario=scenario,
        safe_path=safe_path,
        colliding_path=colliding_path,
        safe_collisions=safe_collisions,
        colliding_collisions=colliding_collisions,
    )


def main() -> None:
    result = run_environment_smoke_demo()
    print(f"Loaded scenario: {result.scenario.name}")
    print(f"Obstacles: {len(result.scenario.world.obstacles)}")
    print(f"Safe path collision free: {not any(result.safe_collisions)}")
    print(f"Invalid path collision detected: {any(result.colliding_collisions)}")
    print(
        "Current [v_x, v_z]: "
        f"{result.scenario.world.current_at(result.safe_path.start, 0.0)}"
    )


if __name__ == "__main__":
    main()
