"""GeoJSON coastline parser for chart-v2.

Only Polygon/MultiPolygon features are accepted here.  Line-only coastlines are
rejected rather than silently buffered: a real chart adapter must decide how to
close a shoreline, which is a separate, explicit operation.
"""
from __future__ import annotations
import json
from pathlib import Path
from typing import Any

from shapely.geometry import Polygon
from environment import Obstacle
from ros2.twin_axial_chart_v1 import Origin, LocalTangentPlane

ALLOWED_KINDS = {"land", "shore", "no_go", "obstacle"}


def _number(value: Any, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{label} must be numeric") from exc
    return number


def _ring(raw: Any, plane: LocalTangentPlane, label: str) -> list[tuple[float, float]]:
    if not isinstance(raw, list) or len(raw) < 3:
        raise ValueError(f"{label} requires at least three positions")
    ring = []
    for i, pair in enumerate(raw):
        if not isinstance(pair, (list, tuple)) or len(pair) < 2:
            raise ValueError(f"{label}[{i}] must be [longitude, latitude]")
        lon = _number(pair[0], f"{label}[{i}] longitude")
        lat = _number(pair[1], f"{label}[{i}] latitude")
        ring.append(plane.to_enu(lat, lon))
    if ring[0] != ring[-1]:
        ring.append(ring[0])
    return ring


def load_coastline(path: str | Path, plane: LocalTangentPlane, *, default_kind: str = "land") -> tuple[Obstacle, ...]:
    path = Path(path)
    try:
        data = json.loads(path.read_text(encoding="utf-8-sig"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid coastline GeoJSON: {path}") from exc
    if data.get("type") != "FeatureCollection":
        raise ValueError("coastline must be a GeoJSON FeatureCollection")
    features = data.get("features")
    if not isinstance(features, list) or not features:
        raise ValueError("coastline FeatureCollection is empty")
    obstacles: list[Obstacle] = []
    seen: set[str] = set()
    for index, feature in enumerate(features):
        if not isinstance(feature, dict):
            raise TypeError(f"coastline features[{index}] must be an object")
        props = feature.get("properties") or {}
        if not isinstance(props, dict):
            raise TypeError(f"coastline features[{index}].properties must be an object")
        kind = props.get("kind", default_kind)
        if kind not in ALLOWED_KINDS:
            raise ValueError(f"unsupported coastline kind: {kind!r}")
        fid = str(feature.get("id") or props.get("id") or f"feature_{index}")
        geometry = feature.get("geometry")
        if not isinstance(geometry, dict):
            raise ValueError(f"coastline {fid} has no geometry")
        gtype = geometry.get("type")
        coords = geometry.get("coordinates")
        if gtype == "Polygon":
            parts = [coords]
        elif gtype == "MultiPolygon":
            parts = coords
        else:
            raise ValueError(f"coastline {fid} must be Polygon/MultiPolygon, got {gtype!r}")
        for part_index, rings in enumerate(parts):
            if not isinstance(rings, list) or not rings:
                raise ValueError(f"coastline {fid} has an empty polygon")
            shell = _ring(rings[0], plane, f"{fid}.shell")
            holes = [_ring(ring, plane, f"{fid}.hole") for ring in rings[1:]]
            geom = Polygon(shell, holes)
            if not geom.is_valid or geom.is_empty or geom.area <= 0.0:
                raise ValueError(f"coastline {fid} has an invalid polygon")
            oid = fid if gtype == "Polygon" else f"{fid}_{part_index}"
            if oid in seen:
                raise ValueError(f"duplicate coastline id: {oid}")
            seen.add(oid)
            obstacles.append(Obstacle(oid, geom, {"kind": kind, "source_feature": fid}))
    return tuple(obstacles)
