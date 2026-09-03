"""Validated Python/YAML scenario loading for Environment v0.3."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from environment.boundary import Boundary
from environment.current import ConstantCurrent, CurrentField, NoCurrent
from environment.obstacle import Obstacle
from environment.world import WorldModel


def _mapping(value: object, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise TypeError(f"{label} must be a mapping")
    return value


def _sequence(value: object, label: str) -> Sequence[Any]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise TypeError(f"{label} must be a sequence")
    return value


def _required(config: Mapping[str, Any], key: str, label: str) -> Any:
    if key not in config:
        raise ValueError(f"{label} is missing required field '{key}'")
    return config[key]


def _parse_boundary(raw: object) -> Boundary:
    config = _mapping(raw, "boundary")
    boundary_type = _required(config, "type", "boundary")
    if boundary_type == "rectangle":
        return Boundary.rectangle(
            x_min=_required(config, "x_min", "boundary"),
            x_max=_required(config, "x_max", "boundary"),
            z_min=_required(config, "z_min", "boundary"),
            z_max=_required(config, "z_max", "boundary"),
        )
    if boundary_type == "polygon":
        return Boundary.polygon(_required(config, "vertices", "boundary"))
    raise ValueError(f"Unknown boundary type: {boundary_type!r}")


def _parse_obstacle(raw: object, index: int) -> Obstacle:
    label = f"obstacles[{index}]"
    config = _mapping(raw, label)
    obstacle_id = _required(config, "id", label)
    obstacle_type = _required(config, "type", label)
    metadata = config.get("metadata")
    if metadata is not None:
        metadata = _mapping(metadata, f"{label}.metadata")

    if obstacle_type == "circle":
        return Obstacle.circle(
            obstacle_id,
            _required(config, "center", label),
            _required(config, "radius", label),
            metadata=metadata,
        )
    if obstacle_type == "rectangle":
        return Obstacle.rectangle(
            obstacle_id,
            x_min=_required(config, "x_min", label),
            x_max=_required(config, "x_max", label),
            z_min=_required(config, "z_min", label),
            z_max=_required(config, "z_max", label),
            metadata=metadata,
        )
    if obstacle_type == "polygon":
        return Obstacle.polygon(
            obstacle_id,
            _required(config, "vertices", label),
            metadata=metadata,
        )
    raise ValueError(f"Unknown obstacle type in {label}: {obstacle_type!r}")


def _parse_current(raw: object | None) -> CurrentField:
    if raw is None:
        return NoCurrent()
    config = _mapping(raw, "current")
    current_type = _required(config, "type", "current")
    if current_type in ("none", "no_current"):
        return NoCurrent()
    if current_type == "constant":
        return ConstantCurrent(_required(config, "velocity", "current"))
    raise ValueError(f"Unknown current type: {current_type!r}")


@dataclass(frozen=True, slots=True, eq=False)
class Scenario:
    """Named, validated scenario and its deterministic planning WorldModel."""

    name: str
    world: WorldModel
    planning_plane: str = "xz"
    frame: str = "world_enu"

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "Scenario":
        """Validate a Python mapping and construct its planning world."""
        config = _mapping(data, "scenario")
        name = _required(config, "name", "scenario")
        if not isinstance(name, str) or not name.strip():
            raise ValueError("scenario name must be a non-empty string")

        planning_plane = _required(config, "planning_plane", "scenario")
        if planning_plane != "xz":
            raise ValueError("Environment v0.3 requires planning_plane: xz")

        frame = _required(config, "frame", "scenario")
        if frame != "world_enu":
            raise ValueError("Environment v0.3 requires frame: world_enu")

        raw_obstacles = _sequence(config.get("obstacles", ()), "obstacles")
        obstacles = tuple(
            _parse_obstacle(raw, index)
            for index, raw in enumerate(raw_obstacles)
        )
        world = WorldModel(
            boundary=_parse_boundary(_required(config, "boundary", "scenario")),
            obstacles=obstacles,
            current=_parse_current(config.get("current")),
        )
        return cls(
            name=name.strip(),
            world=world,
            planning_plane=planning_plane,
            frame=frame,
        )

    @classmethod
    def from_yaml(cls, path: str | Path) -> "Scenario":
        """Load a UTF-8 YAML file and construct a validated scenario."""
        source = Path(path)
        try:
            with source.open("r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            raise ValueError(f"Malformed scenario YAML: {source}") from exc
        return cls.from_dict(data)

