"""Synthetic chart -> local ENU -> WorldModel / SDF skeleton.

This is deliberately a *chart adapter* skeleton, not an ENC/S-57 reader.  The
first supported input is a small, explicit ``chart-v1`` JSON document.  Real
chart formats are added behind the same validated ``Chart`` output.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable
from xml.etree import ElementTree as ET

import numpy as np

from environment import Boundary, Obstacle, WorldModel
from .geo import LocalTangentPlane, Origin

SCHEMA = "chart-v1"
SUPPORTED_KINDS = {"land", "shore", "no_go", "obstacle"}


def _number(value: Any, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{label} must be a number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def _point(pair: Any, label: str) -> tuple[float, float]:
    if not isinstance(pair, (list, tuple)) or len(pair) != 2:
        raise ValueError(f"{label} must be [longitude, latitude]")
    lon = _number(pair[0], label + " longitude")
    lat = _number(pair[1], label + " latitude")
    if not -180.0 <= lon <= 180.0 or not -90.0 <= lat <= 90.0:
        raise ValueError(f"{label} is outside WGS84 bounds")
    return lon, lat


def _closed_ring(raw: Any, label: str) -> list[tuple[float, float]]:
    if not isinstance(raw, list) or len(raw) < 3:
        raise ValueError(f"{label} requires at least three vertices")
    ring = [_point(pair, f"{label}[{i}]") for i, pair in enumerate(raw)]
    if ring[0] != ring[-1]:
        ring.append(ring[0])
    # Drop a repeated terminal vertex for Shapely, but keep the closure in JSON.
    return ring


@dataclass(frozen=True)
class DepthGrid:
    x_min: float
    x_max: float
    z_min: float
    z_max: float
    resolution_m: float
    values: np.ndarray

    def __post_init__(self) -> None:
        if self.values.ndim != 2:
            raise ValueError("depth values must be a 2-D array")
        if self.values.shape[0] < 2 or self.values.shape[1] < 2:
            raise ValueError("depth grid must have at least 2x2 cells")
        if not np.all(np.isfinite(self.values)):
            raise ValueError("depth grid contains non-finite values")
        if not self.x_min < self.x_max or not self.z_min < self.z_max:
            raise ValueError("invalid depth grid bounds")
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")

    @property
    def shape(self) -> tuple[int, int]:
        return self.values.shape

    def at(self, x: float, z: float) -> float:
        x = _number(x, "x"); z = _number(z, "z")
        if not (self.x_min - 1e-12 <= x <= self.x_max + 1e-12 and
                self.z_min - 1e-12 <= z <= self.z_max + 1e-12):
            raise ValueError("depth query outside chart grid")
        fx = (x - self.x_min) / self.resolution_m
        fz = (z - self.z_min) / self.resolution_m
        i = min(self.values.shape[0] - 2, max(0, int(math.floor(fx))))
        j = min(self.values.shape[1] - 2, max(0, int(math.floor(fz))))
        tx = fx - i; tz = fz - j
        a = self.values[i, j] * (1 - tx) + self.values[i + 1, j] * tx
        b = self.values[i, j + 1] * (1 - tx) + self.values[i + 1, j + 1] * tx
        return float(a * (1 - tz) + b * tz)

    def minimum(self) -> float:
        return float(np.min(self.values))

    def maximum(self) -> float:
        return float(np.max(self.values))


@dataclass(frozen=True)
class Chart:
    name: str
    origin: Origin
    boundary: Boundary
    obstacles: tuple[Obstacle, ...]
    depth: DepthGrid
    source_sha256: str
    source: dict[str, Any]

    @property
    def world(self) -> WorldModel:
        return WorldModel(self.boundary, self.obstacles)

    @property
    def plane(self) -> LocalTangentPlane:
        return LocalTangentPlane(self.origin)

    def manifest(self) -> dict[str, Any]:
        return {
            "schema": SCHEMA,
            "name": self.name,
            "origin": self.origin.as_json(),
            "source_sha256": self.source_sha256,
            "boundary": {"kind": "rectangle", "bounds": list(self.boundary.bounds)},
            "obstacles": [
                {"id": o.obstacle_id, "kind": o.metadata.get("kind", "obstacle"),
                 "bounds": list(o._geometry.bounds)}
                for o in self.obstacles
            ],
            "depth": {
                "x_min": self.depth.x_min, "x_max": self.depth.x_max,
                "z_min": self.depth.z_min, "z_max": self.depth.z_max,
                "resolution_m": self.depth.resolution_m,
                "shape": list(self.depth.shape),
                "minimum_m": self.depth.minimum(),
                "maximum_m": self.depth.maximum(),
            },
        }

    @classmethod
    def from_path(cls, path: str | Path) -> "Chart":
        path = Path(path)
        raw = path.read_bytes()
        try:
            data = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError(f"chart is not valid UTF-8 JSON: {path}") from exc
        return cls.from_dict(data, source_sha256=hashlib.sha256(raw).hexdigest())

    @classmethod
    def from_dict(cls, data: dict[str, Any], *, source_sha256: str = "") -> "Chart":
        if not isinstance(data, dict):
            raise TypeError("chart root must be an object")
        if data.get("schema") != SCHEMA:
            raise ValueError(f"chart schema must be {SCHEMA!r}")
        name = data.get("name")
        if not isinstance(name, str) or not name.strip():
            raise ValueError("chart name must be a non-empty string")
        crs = data.get("crs", "EPSG:4326")
        if crs not in ("EPSG:4326", "WGS84", "wgs84"):
            raise ValueError("only EPSG:4326 charts are supported in chart-v1")
        raw_origin = data.get("origin")
        if not isinstance(raw_origin, dict):
            raise ValueError("chart origin is required")
        origin = Origin(
            latitude=_number(raw_origin.get("latitude"), "origin.latitude"),
            longitude=_number(raw_origin.get("longitude"), "origin.longitude"),
            altitude_m=_number(raw_origin.get("altitude_m", 0.0), "origin.altitude_m"),
        )
        plane = LocalTangentPlane(origin)

        features = data.get("features")
        if not isinstance(features, list) or not features:
            raise ValueError("chart features must be a non-empty list")
        seen: set[str] = set()
        obstacles: list[Obstacle] = []
        feature_records: list[dict[str, Any]] = []
        all_points: list[tuple[float, float]] = []
        for index, feature in enumerate(features):
            if not isinstance(feature, dict):
                raise TypeError(f"features[{index}] must be an object")
            feature_id = feature.get("id")
            if not isinstance(feature_id, str) or not feature_id.strip():
                raise ValueError(f"features[{index}].id must be non-empty")
            if feature_id in seen:
                raise ValueError(f"duplicate chart feature id: {feature_id}")
            seen.add(feature_id)
            kind = feature.get("kind")
            if kind not in SUPPORTED_KINDS:
                raise ValueError(f"unsupported chart feature kind: {kind!r}")
            ring = _closed_ring(feature.get("coordinates"), f"features[{index}]")
            enu = [plane.to_enu(lat, lon) for lon, lat in ring]
            all_points.extend(enu)
            try:
                obstacle = Obstacle.polygon(feature_id, enu[:-1], metadata={"kind": kind})
            except (TypeError, ValueError) as exc:
                raise ValueError(f"invalid chart polygon {feature_id}: {exc}") from exc
            obstacles.append(obstacle)
            feature_records.append({"id": feature_id, "kind": kind,
                                    "vertices": [[round(x, 9), round(z, 9)] for x, z in enu]})

        if not all_points:
            raise ValueError("chart has no polygon vertices")
        xs = [p[0] for p in all_points]; zs = [p[1] for p in all_points]
        x_min, x_max = min(xs), max(xs); z_min, z_max = min(zs), max(zs)
        raw_bounds = data.get("bounds")
        if raw_bounds is not None:
            if not isinstance(raw_bounds, dict):
                raise TypeError("chart bounds must be an object")
            west = _number(raw_bounds.get("west"), "bounds.west")
            east = _number(raw_bounds.get("east"), "bounds.east")
            south = _number(raw_bounds.get("south"), "bounds.south")
            north = _number(raw_bounds.get("north"), "bounds.north")
            if not (-180 <= west < east <= 180 and -90 <= south < north <= 90):
                raise ValueError("chart bounds must be ordered WGS84 west/east/south/north")
            corners = [plane.to_enu(south, west), plane.to_enu(south, east),
                       plane.to_enu(north, west), plane.to_enu(north, east)]
            x_min = min(p[0] for p in corners); x_max = max(p[0] for p in corners)
            z_min = min(p[1] for p in corners); z_max = max(p[1] for p in corners)
            for x, z in all_points:
                if not (x_min - 1e-9 <= x <= x_max + 1e-9 and z_min - 1e-9 <= z <= z_max + 1e-9):
                    raise ValueError("chart feature lies outside the declared bounds")
        if x_max - x_min <= 0 or z_max - z_min <= 0:
            raise ValueError("chart boundary has no area")
        boundary = Boundary.rectangle(x_min=x_min, x_max=x_max, z_min=z_min, z_max=z_max)

        raw_depth = data.get("depth")
        if not isinstance(raw_depth, dict):
            raise ValueError("chart depth block is required")
        resolution = _number(raw_depth.get("resolution_m"), "depth.resolution_m")
        if resolution <= 0.0:
            raise ValueError("depth.resolution_m must be positive")
        soundings = raw_depth.get("soundings")
        if not isinstance(soundings, list) or not soundings:
            raise ValueError("depth.soundings must be a non-empty list")
        samples: list[tuple[float, float, float]] = []
        for index, sounding in enumerate(soundings):
            if not isinstance(sounding, (list, tuple)) or len(sounding) != 3:
                raise ValueError(f"depth.soundings[{index}] must be [lon, lat, depth_m]")
            lon, lat = _point(sounding[:2], f"depth.soundings[{index}]")
            dep = _number(sounding[2], f"depth.soundings[{index}].depth_m")
            if dep < 0.0:
                raise ValueError("depth values must be non-negative")
            x, z = plane.to_enu(lat, lon)
            if not (x_min - 1e-12 <= x <= x_max + 1e-12 and z_min - 1e-12 <= z <= z_max + 1e-12):
                raise ValueError(f"depth sounding {index} lies outside the chart polygons")
            samples.append((x, z, dep))
        nx = int(math.floor((x_max - x_min) / resolution + 0.5)) + 1
        nz = int(math.floor((z_max - z_min) / resolution + 0.5)) + 1
        nx = max(2, nx); nz = max(2, nz)
        xs_grid = x_min + np.arange(nx) * resolution
        zs_grid = z_min + np.arange(nz) * resolution
        values = np.empty((nx, nz), dtype=float)
        for i, x in enumerate(xs_grid):
            for j, z in enumerate(zs_grid):
                best = min(samples, key=lambda s: (s[0] - x) ** 2 + (s[1] - z) ** 2)
                distance = math.hypot(best[0] - x, best[1] - z)
                if distance <= 1e-9:
                    values[i, j] = best[2]
                else:
                    weights = [1.0 / max(1e-12, (s[0] - x) ** 2 + (s[1] - z) ** 2) for s in samples]
                    values[i, j] = sum(w * s[2] for w, s in zip(weights, samples)) / sum(weights)
        grid = DepthGrid(x_min, x_max, z_min, z_max, resolution, values)
        return cls(name.strip(), origin, boundary, tuple(obstacles), grid,
                   source_sha256, {"features": feature_records, "source": data})

    def write(self, directory: str | Path) -> Path:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "chart.json").write_text(json.dumps(self.manifest(), indent=2, sort_keys=True) + "\n",
                                               encoding="utf-8")
        (directory / "minimal.world").write_text(self.to_world_xml(), encoding="utf-8")
        _write_depth_png(directory / "depth.png", self.depth)
        return directory

    def to_world_xml(self) -> str:
        root = ET.Element("sdf", {"version": "1.7"})
        world = ET.SubElement(root, "world", {"name": self.name})
        ET.SubElement(world, "scene")
        physics = ET.SubElement(world, "physics", {"type": "ode"})
        ET.SubElement(physics, "max_step_size").text = "0.05"
        spherical = ET.SubElement(world, "spherical_coordinates")
        ET.SubElement(spherical, "surface_model").text = "EARTH_WGS84"
        ET.SubElement(spherical, "world_frame_orientation").text = "ENU"
        ET.SubElement(spherical, "latitude_deg").text = f"{self.origin.latitude:.12f}"
        ET.SubElement(spherical, "longitude_deg").text = f"{self.origin.longitude:.12f}"
        ET.SubElement(spherical, "elevation").text = f"{self.origin.altitude_m:.6f}"
        for obstacle in self.obstacles:
            x0, z0, x1, z1 = obstacle._geometry.bounds
            model = ET.SubElement(world, "model", {"name": f"chart_{obstacle.obstacle_id}"})
            ET.SubElement(model, "static").text = "true"
            ET.SubElement(model, "pose").text = f"{(x0+x1)/2:.9f} 0 {(z0+z1)/2:.9f} 0 0 0"
            link = ET.SubElement(model, "link", {"name": "link"})
            collision = ET.SubElement(link, "collision", {"name": "collision"})
            geometry = ET.SubElement(collision, "geometry")
            box = ET.SubElement(geometry, "box")
            ET.SubElement(box, "size").text = f"{x1-x0:.9f} 4 {z1-z0:.9f}"
        seabed = ET.SubElement(world, "model", {"name": "chart_seabed"})
        ET.SubElement(seabed, "static").text = "true"
        ET.SubElement(seabed, "pose").text = f"{(self.depth.x_min+self.depth.x_max)/2:.9f} 0 {(self.depth.z_min+self.depth.z_max)/2:.9f} 0 0 0"
        link = ET.SubElement(seabed, "link", {"name": "link"})
        visual = ET.SubElement(link, "visual", {"name": "visual"})
        geometry = ET.SubElement(visual, "geometry")
        heightmap = ET.SubElement(geometry, "heightmap")
        ET.SubElement(heightmap, "uri").text = "depth.png"
        ET.SubElement(heightmap, "size").text = f"{self.depth.x_max-self.depth.x_min:.9f} {self.depth.z_max-self.depth.z_min:.9f} 1"
        return ET.tostring(root, encoding="unicode")


def _write_depth_png(path: Path, depth: DepthGrid) -> None:
    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError("Pillow is required to write chart depth.png") from exc
    values = depth.values
    lo, hi = float(values.min()), float(values.max())
    if hi - lo <= 1e-12:
        scaled = np.zeros(values.shape, dtype=np.uint8)
    else:
        scaled = ((values - lo) / (hi - lo) * 255.0).round().astype(np.uint8)
    Image.fromarray(scaled.T, mode="L").save(path)
