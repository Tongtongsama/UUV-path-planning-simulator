"""CSV bathymetry parser for chart-v2.

The CSV must be explicit: longitude, latitude, depth_m.  Depth is positive
down and referenced to the metadata datum.  This module deliberately does not
guess the meaning of arbitrary chart columns.
"""
from __future__ import annotations
import csv
import math
from pathlib import Path
from typing import Any

import numpy as np
from ros2.twin_axial_chart_v1 import LocalTangentPlane
from ros2.twin_axial_chart_v1.chart import DepthGrid


def _number(value: Any, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{label} must be numeric") from exc
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def load_bathymetry(
    path: str | Path,
    plane: LocalTangentPlane,
    *,
    x_min: float,
    x_max: float,
    z_min: float,
    z_max: float,
    resolution_m: float,
    longitude_column: str = "longitude",
    latitude_column: str = "latitude",
    depth_column: str = "depth_m",
) -> DepthGrid:
    path = Path(path)
    if resolution_m <= 0.0:
        raise ValueError("bathymetry resolution_m must be positive")
    samples: list[tuple[float, float, float]] = []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        required = {longitude_column, latitude_column, depth_column}
        if reader.fieldnames is None or not required <= set(reader.fieldnames):
            raise ValueError(f"bathymetry CSV must contain columns {sorted(required)}")
        for row_index, row in enumerate(reader, start=2):
            if not any((row.get(k) or "").strip() for k in required):
                continue
            lon = _number(row[longitude_column], f"{path}:{row_index} longitude")
            lat = _number(row[latitude_column], f"{path}:{row_index} latitude")
            dep = _number(row[depth_column], f"{path}:{row_index} depth")
            if dep < 0.0:
                raise ValueError(f"{path}:{row_index}: depth must be non-negative")
            x, z = plane.to_enu(lat, lon)
            if not (x_min - 1e-9 <= x <= x_max + 1e-9 and z_min - 1e-9 <= z <= z_max + 1e-9):
                raise ValueError(f"{path}:{row_index}: sounding outside chart bounds")
            samples.append((x, z, dep))
    if not samples:
        raise ValueError("bathymetry CSV contains no usable samples")
    nx = max(2, int(math.floor((x_max - x_min) / resolution_m + 0.5)) + 1)
    nz = max(2, int(math.floor((z_max - z_min) / resolution_m + 0.5)) + 1)
    xs = x_min + np.arange(nx) * resolution_m
    zs = z_min + np.arange(nz) * resolution_m
    values = np.empty((nx, nz), dtype=float)
    for i, x in enumerate(xs):
        for j, z in enumerate(zs):
            weights = [1.0 / max(1e-12, (s[0] - x) ** 2 + (s[1] - z) ** 2) for s in samples]
            values[i, j] = sum(w * s[2] for w, s in zip(weights, samples)) / sum(weights)
    return DepthGrid(x_min, x_max, z_min, z_max, resolution_m, values)
