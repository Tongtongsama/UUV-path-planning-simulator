"""chart-v2 adapter: provenance/licence/datum + GeoJSON coastline + CSV bathy."""
from __future__ import annotations
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import shutil
from typing import Any

from environment import Boundary, Obstacle, WorldModel
from ros2.twin_axial_chart_v1 import Origin, LocalTangentPlane
from ros2.twin_axial_chart_v1.chart import DepthGrid, _write_depth_png
from .schema import validate_metadata
from .geojson import load_coastline
from .bathymetry import load_bathymetry
from .mesh import write_obj, _safe_name, wall_segments
from .world import world_xml


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


@dataclass(frozen=True)
class ChartV2:
    metadata: dict[str, Any]
    source_metadata_path: Path
    source_paths: dict[str, Path]
    origin: Origin
    boundary: Boundary
    obstacles: tuple[Obstacle, ...]
    depth: DepthGrid
    source_hashes: dict[str, str]

    @property
    def plane(self) -> LocalTangentPlane:
        return LocalTangentPlane(self.origin)

    @property
    def world(self) -> WorldModel:
        return WorldModel(self.boundary, self.obstacles)

    def manifest(self) -> dict[str, Any]:
        m = self.metadata
        return {
            "schema": "chart-v2",
            "name": m["name"],
            "origin": {"latitude": self.origin.latitude, "longitude": self.origin.longitude,
                       "altitude_m": self.origin.altitude_m},
            "license": m["license"],
            "provenance": {**m["provenance"], "source_files": self.source_hashes},
            "datum": m["datum"],
            "bounds": {"kind": "rectangle", "bounds": list(self.boundary.bounds)},
            "obstacles": [
                {"id": o.obstacle_id, "kind": o.metadata.get("kind", "obstacle"),
                 "bounds": list(o._geometry.bounds),
                 "mesh": f"chart_{_safe_name(o.obstacle_id)}.obj",
                 "collision": "exact_edge_walls", "walls": len(wall_segments(o._geometry))}
                for o in self.obstacles
            ],
            "depth": {"x_min": self.depth.x_min, "x_max": self.depth.x_max,
                      "z_min": self.depth.z_min, "z_max": self.depth.z_max,
                      "resolution_m": self.depth.resolution_m,
                      "shape": list(self.depth.shape),
                      "minimum_m": self.depth.minimum(), "maximum_m": self.depth.maximum()},
        }

    @classmethod
    def from_path(cls, metadata_path: str | Path) -> "ChartV2":
        path = Path(metadata_path)
        data = validate_metadata(json.loads(path.read_text(encoding="utf-8-sig")))
        base = path.parent
        origin = Origin(float(data["origin"]["latitude"]), float(data["origin"]["longitude"]),
                        float(data["origin"].get("altitude_m", 0.0)))
        plane = LocalTangentPlane(origin)
        b = data["bounds"]
        corners = [plane.to_enu(float(b["south"]), float(b["west"])),
                   plane.to_enu(float(b["south"]), float(b["east"])),
                   plane.to_enu(float(b["north"]), float(b["west"])),
                   plane.to_enu(float(b["north"]), float(b["east"]))]
        x_min = min(p[0] for p in corners); x_max = max(p[0] for p in corners)
        z_min = min(p[1] for p in corners); z_max = max(p[1] for p in corners)
        boundary = Boundary.rectangle(x_min=x_min, x_max=x_max, z_min=z_min, z_max=z_max)
        coastline_path = (base / data["coastline"]).resolve()
        bathy = data["bathymetry"]
        bathy_path = (base / bathy["path"]).resolve()
        obstacles = load_coastline(coastline_path, plane,
                                   default_kind=data.get("default_feature_kind", "land"))
        for obstacle in obstacles:
            ox0, oz0, ox1, oz1 = obstacle._geometry.bounds
            if not (x_min - 1e-8 <= ox0 and ox1 <= x_max + 1e-8 and
                    z_min - 1e-8 <= oz0 and oz1 <= z_max + 1e-8):
                raise ValueError(f"coastline obstacle {obstacle.obstacle_id} is outside bounds")
        depth = load_bathymetry(
            bathy_path, plane, x_min=x_min, x_max=x_max, z_min=z_min, z_max=z_max,
            resolution_m=float(bathy["resolution_m"]),
            longitude_column=bathy.get("longitude_column", "longitude"),
            latitude_column=bathy.get("latitude_column", "latitude"),
            depth_column=bathy.get("depth_column", "depth_m"))
        hashes = {"metadata": _sha(path), "coastline": _sha(coastline_path),
                  "bathymetry": _sha(bathy_path)}
        return cls(data, path, {"coastline": coastline_path, "bathymetry": bathy_path},
                   origin, boundary, obstacles, depth, hashes)

    def write(self, directory: str | Path) -> Path:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "chart.json").write_text(json.dumps(self.manifest(), indent=2, sort_keys=True) + "\n",
                                               encoding="utf-8")
        mesh_names = {}
        for obstacle in self.obstacles:
            name = f"chart_{_safe_name(obstacle.obstacle_id)}.obj"
            write_obj(obstacle._geometry, directory / name)
            mesh_names[obstacle.obstacle_id] = name
        (directory / "minimal.world").write_text(world_xml(self, mesh_names), encoding="utf-8")
        _write_depth_png(directory / "depth.png", self.depth)
        source_dir = directory / "source"
        source_dir.mkdir(exist_ok=True)
        for label, path in self.source_paths.items():
            shutil.copy2(path, source_dir / path.name)
        shutil.copy2(self.source_metadata_path, source_dir / self.source_metadata_path.name)
        return directory
