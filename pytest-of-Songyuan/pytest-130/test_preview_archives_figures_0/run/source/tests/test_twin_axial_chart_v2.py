import json
from pathlib import Path

import pytest

from ros2.twin_axial_chart_v2 import ChartV2, validate_metadata
from ros2.twin_axial_chart_v2.mesh import obj_bounds, wall_segments


def _write_fixture(tmp_path: Path) -> Path:
    geo = {
        "type": "FeatureCollection",
        "features": [
            {"type": "Feature", "id": "land_west", "properties": {"kind": "land"},
             "geometry": {"type": "Polygon", "coordinates": [[
                 [103.744, 1.244], [103.747, 1.244], [103.747, 1.248],
                 [103.744, 1.248], [103.744, 1.244]]]}},
            {"type": "Feature", "id": "no_go_east", "properties": {"kind": "no_go"},
             "geometry": {"type": "Polygon", "coordinates": [[
                 [103.753, 1.252], [103.754, 1.252], [103.754, 1.253],
                 [103.753, 1.253], [103.753, 1.252]]]}},
        ],
    }
    (tmp_path / "coastline.geojson").write_text(json.dumps(geo), encoding="utf-8")
    (tmp_path / "bathymetry.csv").write_text(
        "longitude,latitude,depth_m\n"
        "103.750,1.250,20\n103.746,1.246,12\n103.752,1.252,8\n103.748,1.253,15\n",
        encoding="utf-8")
    meta = {
        "schema": "chart-v2", "name": "synthetic_v2", "crs": "EPSG:4326",
        "origin": {"latitude": 1.25, "longitude": 103.75, "altitude_m": 0},
        "bounds": {"west": 103.744, "east": 103.754, "south": 1.244, "north": 1.253},
        "license": {"name": "synthetic public-domain test", "url": "local://test",
                     "public_domain": True, "redistributable": True,
                     "attribution": "test"},
        "provenance": {"dataset": "synthetic", "edition": "1", "issue_date": "2026-11-16",
                       "status": "test", "source_url": "local://test",
                       "source_sha256": "a" * 64, "downloaded_utc": "2026-11-16T00:00:00Z",
                       "notes": "test"},
        "datum": {"chart_datum": "MLLW", "vertical_unit": "m",
                  "depth_sign": "positive_down", "tide": "none"},
        "coastline": "coastline.geojson",
        "bathymetry": {"path": "bathymetry.csv", "resolution_m": 100.0},
    }
    path = tmp_path / "metadata.json"
    path.write_text(json.dumps(meta), encoding="utf-8")
    return path


def test_synthetic_v2_roundtrip_and_world(tmp_path):
    chart = ChartV2.from_path(_write_fixture(tmp_path))
    assert chart.metadata["license"]["public_domain"] is True
    assert chart.metadata["datum"]["chart_datum"] == "MLLW"
    assert len(chart.obstacles) == 2
    assert chart.boundary.bounds[2] > chart.boundary.bounds[0]
    o = chart.obstacles[0]._geometry.bounds
    assert chart.world.collides(((o[0] + o[2]) / 2, (o[1] + o[3]) / 2))
    assert 5.0 <= chart.depth.minimum() < chart.depth.maximum() <= 20.0
    out = chart.write(tmp_path / "out")
    assert (out / "chart.json").is_file()
    assert (out / "minimal.world").is_file()
    assert (out / "depth.png").is_file()
    assert (out / "source" / "coastline.geojson").is_file()
    world = (out / "minimal.world").read_text(encoding="utf-8")
    assert "<mesh>" in world and "shore_0" in world
    for obstacle in chart.obstacles:
        assert len(wall_segments(obstacle._geometry)) >= 4
        obj = out / f"chart_{obstacle.obstacle_id}.obj"
        assert obj.is_file()
        x0, _, z0, x1, _, z1 = obj_bounds(obj)
        bx0, bz0, bx1, bz1 = obstacle._geometry.bounds
        assert (x0, z0, x1, z1) == pytest.approx((bx0, bz0, bx1, bz1), abs=1e-6)
    manifest = json.loads((out / "chart.json").read_text())
    expected_walls = sum(len(wall_segments(o._geometry)) for o in chart.obstacles)
    assert sum(1 for m in world.split('<collision ') if 'name="shore_' in m) == expected_walls
    assert all(o['collision'] == 'exact_edge_walls' for o in manifest['obstacles'])
    assert manifest["license"]["redistributable"] is True
    assert manifest["datum"]["depth_sign"] == "positive_down"
    assert manifest["provenance"]["source_files"]["coastline"]


def test_missing_licence_or_datum_is_rejected(tmp_path):
    path = _write_fixture(tmp_path)
    data = json.loads(path.read_text())
    for key in ("license", "datum", "provenance"):
        broken = dict(data); broken.pop(key)
        with pytest.raises((ValueError, TypeError)):
            validate_metadata(broken)


def test_bad_depth_sign_and_units_are_rejected(tmp_path):
    path = _write_fixture(tmp_path); data = json.loads(path.read_text())
    bad = json.loads(json.dumps(data)); bad["datum"]["depth_sign"] = "positive_up"
    with pytest.raises(ValueError):
        validate_metadata(bad)
    bad = json.loads(json.dumps(data)); bad["datum"]["vertical_unit"] = "ft"
    with pytest.raises(ValueError):
        validate_metadata(bad)


def test_geojson_line_is_rejected(tmp_path):
    path = _write_fixture(tmp_path)
    geo_path = tmp_path / "coastline.geojson"; geo = json.loads(geo_path.read_text())
    geo["features"][0]["geometry"] = {"type": "LineString", "coordinates": [[103.744, 1.244], [103.747, 1.248]]}
    geo_path.write_text(json.dumps(geo), encoding="utf-8")
    with pytest.raises(ValueError, match="Polygon"):
        ChartV2.from_path(path)


def test_bad_csv_columns_are_rejected(tmp_path):
    path = _write_fixture(tmp_path)
    (tmp_path / "bathymetry.csv").write_text("lon,lat,depth\n103.75,1.25,1\n", encoding="utf-8")
    with pytest.raises(ValueError, match="columns"):
        ChartV2.from_path(path)


PILOT = Path(__file__).resolve().parents[1] / "artifacts/chart_v2_20261116_01/pilot_portsmouth_2km/metadata.json"


@pytest.mark.skipif(not PILOT.is_file(), reason="NOAA pilot not built")
def test_noaa_pilot_metadata_and_world():
    chart = ChartV2.from_path(PILOT)
    assert chart.metadata["name"] == "noaa_portsmouth_2km_v1"
    assert chart.metadata["license"]["public_domain"] is True
    assert chart.metadata["provenance"]["dataset"].startswith("NOAA ENC")
    assert chart.metadata["datum"]["chart_datum"] == "MLLW"
    assert len(chart.obstacles) >= 1
    assert chart.boundary.bounds[0] < 0 < chart.boundary.bounds[2]
    assert chart.depth.minimum() >= 0.0
