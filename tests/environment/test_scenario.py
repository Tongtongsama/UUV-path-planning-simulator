"""Tests for deterministic Python/YAML Environment scenario loading."""

from pathlib import Path as FilePath
from pathlib import Path as PathLib
import sys

import numpy as np
import pytest

PROJECT_ROOT = PathLib(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from environment import ConstantCurrent, NoCurrent, Scenario


def _base_config() -> dict:
    return {
        "name": "test_world",
        "planning_plane": "xz",
        "frame": "world_enu",
        "boundary": {
            "type": "rectangle",
            "x_min": 0.0,
            "x_max": 20.0,
            "z_min": -10.0,
            "z_max": 0.0,
        },
        "obstacles": [
            {"id": "circle", "type": "circle", "center": [5.0, -5.0], "radius": 1.0},
            {"id": "wall", "type": "rectangle", "x_min": 10.0, "x_max": 11.0, "z_min": -8.0, "z_max": -2.0},
            {"id": "reef", "type": "polygon", "vertices": [[14.0, -2.0], [17.0, -2.0], [15.0, -5.0]]},
        ],
        "current": {"type": "constant", "velocity": [0.2, -0.1]},
    }


def test_from_dict_constructs_all_supported_shapes_and_current() -> None:
    scenario = Scenario.from_dict(_base_config())

    assert scenario.name == "test_world"
    assert scenario.planning_plane == "xz"
    assert scenario.frame == "world_enu"
    assert [obstacle.obstacle_id for obstacle in scenario.world.obstacles] == [
        "circle", "wall", "reef"
    ]
    assert scenario.world.collides([5.0, -5.0])
    assert scenario.world.collides([10.5, -5.0])
    assert scenario.world.collides([15.0, -3.0])
    assert isinstance(scenario.world.current, ConstantCurrent)
    np.testing.assert_array_equal(scenario.world.current_at([1.0, -1.0], 2.0), [0.2, -0.1])


def test_missing_current_defaults_to_no_current() -> None:
    config = _base_config()
    del config["current"]

    scenario = Scenario.from_dict(config)

    assert isinstance(scenario.world.current, NoCurrent)


def test_same_config_produces_observably_equivalent_worlds() -> None:
    first = Scenario.from_dict(_base_config())
    second = Scenario.from_dict(_base_config())
    positions = ([1.0, -1.0], [5.0, -5.0], [12.0, -6.0], [15.0, -3.0])

    for position in positions:
        assert first.world.contains(position) == second.world.contains(position)
        assert first.world.collides(position, radius=0.5) == second.world.collides(position, radius=0.5)
        assert first.world.clearance(position) == pytest.approx(second.world.clearance(position))
        assert [o.obstacle_id for o in first.world.nearby_obstacles(position, 3.0)] == [
            o.obstacle_id for o in second.world.nearby_obstacles(position, 3.0)
        ]
        np.testing.assert_array_equal(
            first.world.current_at(position, 4.0),
            second.world.current_at(position, 4.0),
        )


def test_from_yaml_loads_utf8_scenario(tmp_path: FilePath) -> None:
    path = tmp_path / "scenario.yaml"
    path.write_text(
        """name: 海洋测试
planning_plane: xz
frame: world_enu
boundary:
  type: polygon
  vertices: [[0, 0], [10, 0], [10, -5], [0, -5]]
obstacles: []
current:
  type: none
""",
        encoding="utf-8",
    )

    scenario = Scenario.from_yaml(path)

    assert scenario.name == "海洋测试"
    assert scenario.world.contains([5.0, -2.0])


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("planning_plane", "xy", "planning_plane"),
        ("frame", "ned", "frame"),
        ("name", "", "name"),
    ],
)
def test_rejects_unsupported_header_values(field: str, value: str, message: str) -> None:
    config = _base_config()
    config[field] = value
    with pytest.raises(ValueError, match=message):
        Scenario.from_dict(config)


def test_rejects_duplicate_obstacle_ids() -> None:
    config = _base_config()
    config["obstacles"].append(
        {"id": "circle", "type": "circle", "center": [18.0, -8.0], "radius": 0.5}
    )

    with pytest.raises(ValueError, match="unique"):
        Scenario.from_dict(config)


@pytest.mark.parametrize(
    ("section", "unknown_type"),
    [("boundary", "ellipse"), ("obstacles", "mesh"), ("current", "vortex")],
)
def test_rejects_unknown_component_types(section: str, unknown_type: str) -> None:
    config = _base_config()
    if section == "obstacles":
        config[section][0]["type"] = unknown_type
    else:
        config[section]["type"] = unknown_type

    with pytest.raises(ValueError, match="Unknown"):
        Scenario.from_dict(config)


def test_rejects_missing_required_field() -> None:
    config = _base_config()
    del config["boundary"]["x_max"]

    with pytest.raises(ValueError, match="x_max"):
        Scenario.from_dict(config)


def test_rejects_non_mapping_scenario() -> None:
    with pytest.raises(TypeError, match="scenario must be a mapping"):
        Scenario.from_dict([])  # type: ignore[arg-type]


def test_rejects_malformed_yaml(tmp_path: FilePath) -> None:
    path = tmp_path / "bad.yaml"
    path.write_text("boundary: [unterminated", encoding="utf-8")

    with pytest.raises(ValueError, match="Malformed scenario YAML"):
        Scenario.from_yaml(path)

