from pathlib import Path
import json
import xml.etree.ElementTree as ET

import pytest

from environment import WorldModel
from ros2.twin_axial_chart_v1 import Chart, Origin, LocalTangentPlane

FIXTURE = Path(__file__).parent / "fixtures" / "chart_synthetic_harbor_v1.json"


def test_geo_roundtrip_and_origin_zero():
    origin = Origin(1.25, 103.75)
    plane = LocalTangentPlane(origin)
    assert plane.to_enu(1.25, 103.75) == pytest.approx((0.0, 0.0), abs=1e-12)
    lat, lon = 1.2523, 103.7531
    x, z = plane.to_enu(lat, lon)
    lat2, lon2 = plane.to_geodetic(x, z)
    assert lat2 == pytest.approx(lat, abs=1e-10)
    assert lon2 == pytest.approx(lon, abs=1e-10)


def test_synthetic_chart_loads_to_world():
    chart = Chart.from_path(FIXTURE)
    assert chart.name == "synthetic_harbor_v1"
    assert len(chart.obstacles) == 2
    assert {o.obstacle_id for o in chart.obstacles} == {"west_land", "east_no_go"}
    assert isinstance(chart.world, WorldModel)
    bounds = chart.boundary.bounds
    assert bounds[0] == pytest.approx(-667.759, abs=0.1)
    assert bounds[1] == pytest.approx(-663.449, abs=0.1)
    assert bounds[2] == pytest.approx(667.759, abs=0.1)
    assert bounds[3] == pytest.approx(663.449, abs=0.1)
    assert chart.source_sha256


def test_obstacles_are_exact_polygons_and_collide():
    chart = Chart.from_path(FIXTURE)
    land = next(o for o in chart.obstacles if o.obstacle_id == "west_land")
    no_go = next(o for o in chart.obstacles if o.obstacle_id == "east_no_go")
    assert land.metadata["kind"] == "land"
    assert no_go.metadata["kind"] == "no_go"
    land_center = ((land._geometry.bounds[0] + land._geometry.bounds[2]) / 2,
                   (land._geometry.bounds[1] + land._geometry.bounds[3]) / 2)
    no_go_center = ((no_go._geometry.bounds[0] + no_go._geometry.bounds[2]) / 2,
                    (no_go._geometry.bounds[1] + no_go._geometry.bounds[3]) / 2)
    assert chart.world.collides(land_center)
    assert chart.world.collides(no_go_center)
    assert not chart.world.collides((0.0, 0.0))
    assert chart.world.is_segment_valid((-100.0, 100.0), (100.0, 100.0))


def test_depth_grid_has_expected_samples_and_interpolation():
    chart = Chart.from_path(FIXTURE)
    assert chart.depth.shape[0] >= 2 and chart.depth.shape[1] >= 2
    assert chart.depth.minimum() <= 20.0 + 1e-3
    assert chart.depth.maximum() >= 20.0 - 1e-3
    x, z = chart.plane.to_enu(1.25, 103.75)
    assert chart.depth.at(x, z) == pytest.approx(20.0, abs=1e-2)
    assert chart.depth.at(0.0, 0.0) == pytest.approx(20.0, abs=1e-2)
    with pytest.raises(ValueError):
        chart.depth.at(10_000.0, 0.0)


def test_writes_chart_world_depth_and_manifest(tmp_path):
    chart = Chart.from_path(FIXTURE)
    out = chart.write(tmp_path / "chart")
    assert (out / "chart.json").is_file()
    assert (out / "depth.png").is_file()
    assert (out / "minimal.world").is_file()
    manifest = json.loads((out / "chart.json").read_text())
    assert manifest["schema"] == "chart-v1"
    assert manifest["origin"]["latitude"] == 1.25
    assert manifest["depth"]["minimum_m"] <= manifest["depth"]["maximum_m"]
    root = ET.parse(out / "minimal.world").getroot()
    world = root.find("world")
    spherical = world.find("spherical_coordinates")
    assert float(spherical.find("latitude_deg").text) == pytest.approx(1.25)
    assert float(spherical.find("longitude_deg").text) == pytest.approx(103.75)
    names = {m.get("name") for m in world.findall("model")}
    assert {"chart_west_land", "chart_east_no_go", "chart_seabed"} <= names
    assert world.find("./model[@name='chart_seabed']/link/visual/geometry/heightmap/uri").text == "depth.png"
    image = __import__('PIL.Image', fromlist=['Image']).open(out / 'depth.png')
    assert image.size == (chart.depth.shape[0], chart.depth.shape[1])
    for obstacle in chart.obstacles:
        model = world.find(f"./model[@name='chart_{obstacle.obstacle_id}']")
        assert model is not None
        x0, z0, x1, z1 = obstacle._geometry.bounds
        size = [float(v) for v in model.find('./link/collision/geometry/box/size').text.split()]
        assert size[0] == pytest.approx(x1 - x0, abs=1e-6)
        assert size[2] == pytest.approx(z1 - z0, abs=1e-6)


def test_manifest_is_json_serializable_and_deterministic():
    chart = Chart.from_path(FIXTURE)
    first = chart.manifest()
    second = Chart.from_path(FIXTURE).manifest()
    assert first == second
    json.dumps(first, allow_nan=False)


def test_rejects_invalid_schema_origin_and_duplicate_features():
    base = json.loads(FIXTURE.read_text())
    with pytest.raises(ValueError, match="schema"):
        Chart.from_dict({**base, "schema": "nope"})
    with pytest.raises(ValueError, match="origin"):
        Chart.from_dict({k: v for k, v in base.items() if k != "origin"})
    duplicate = json.loads(json.dumps(base))
    duplicate["features"][1]["id"] = duplicate["features"][0]["id"]
    with pytest.raises(ValueError, match="duplicate"):
        Chart.from_dict(duplicate)
    bad_kind = json.loads(json.dumps(base))
    bad_kind["features"][0]["kind"] = "mystery"
    with pytest.raises(ValueError, match="kind"):
        Chart.from_dict(bad_kind)


def test_rejects_feature_and_sounding_outside_bounds():
    base = json.loads(FIXTURE.read_text())
    outside = json.loads(json.dumps(base))
    outside["features"][0]["coordinates"] = [[100.0, 1.0], [100.1, 1.0], [100.1, 1.1], [100.0, 1.1], [100.0, 1.0]]
    with pytest.raises(ValueError, match="outside"):
        Chart.from_dict(outside)
    bad_depth = json.loads(json.dumps(base))
    bad_depth["depth"]["soundings"][0][0] = 100.0
    with pytest.raises(ValueError, match="outside"):
        Chart.from_dict(bad_depth)
