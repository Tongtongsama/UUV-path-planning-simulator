"""Validation for the chart-v2 provenance/licence/datum envelope."""
from __future__ import annotations
import math
import re
from typing import Any

REQUIRED_TOP = ("schema","name","crs","origin","bounds","license","provenance","datum","coastline","bathymetry")


def _number(value: Any, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{label} must be numeric") from exc
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def validate_metadata(data: Any) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise TypeError("chart-v2 metadata must be an object")
    if data.get("schema") != "chart-v2":
        raise ValueError("chart-v2 metadata schema must be 'chart-v2'")
    for key in REQUIRED_TOP:
        if key not in data:
            raise ValueError(f"chart-v2 metadata is missing {key!r}")
    if data["crs"] not in ("EPSG:4326", "WGS84", "wgs84"):
        raise ValueError("chart-v2 currently accepts only EPSG:4326")
    origin = data["origin"]
    if not isinstance(origin, dict):
        raise TypeError("origin must be an object")
    lat = _number(origin.get("latitude"), "origin.latitude")
    lon = _number(origin.get("longitude"), "origin.longitude")
    alt = _number(origin.get("altitude_m", 0.0), "origin.altitude_m")
    if not -90 <= lat <= 90 or not -180 <= lon <= 180:
        raise ValueError("origin must be valid WGS84")
    bounds = data["bounds"]
    if not isinstance(bounds, dict):
        raise TypeError("bounds must be an object")
    west = _number(bounds.get("west"), "bounds.west")
    east = _number(bounds.get("east"), "bounds.east")
    south = _number(bounds.get("south"), "bounds.south")
    north = _number(bounds.get("north"), "bounds.north")
    if not (-180 <= west < east <= 180 and -90 <= south < north <= 90):
        raise ValueError("bounds must be ordered WGS84 west/east/south/north")
    license_ = data["license"]
    if not isinstance(license_, dict):
        raise TypeError("license must be an object")
    for key in ("name","url"):
        if not isinstance(license_.get(key), str) or not license_[key].strip():
            raise ValueError(f"license.{key} must be non-empty")
    for key in ("public_domain","redistributable"):
        if not isinstance(license_.get(key), bool):
            raise ValueError(f"license.{key} must be boolean")
    provenance = data["provenance"]
    if not isinstance(provenance, dict):
        raise TypeError("provenance must be an object")
    for key in ("dataset","source_url","source_sha256","downloaded_utc","status"):
        if not isinstance(provenance.get(key), str) or not provenance[key].strip():
            raise ValueError(f"provenance.{key} must be non-empty")
    digest = provenance["source_sha256"]
    if not re.fullmatch(r"[0-9a-fA-F]{64}", digest):
        raise ValueError("provenance.source_sha256 must be 64 hex characters")
    datum = data["datum"]
    if not isinstance(datum, dict):
        raise TypeError("datum must be an object")
    for key in ("chart_datum","vertical_unit","depth_sign","tide"):
        if not isinstance(datum.get(key), str) or not datum[key].strip():
            raise ValueError(f"datum.{key} must be non-empty")
    if datum["vertical_unit"] != "m":
        raise ValueError("chart-v2 pilot requires vertical_unit 'm'")
    if datum["depth_sign"] != "positive_down":
        raise ValueError("chart-v2 pilot requires depth_sign 'positive_down'")
    if not isinstance(data["coastline"], str) or not data["coastline"].strip():
        raise ValueError("coastline must be a relative path")
    bathy = data["bathymetry"]
    if not isinstance(bathy, dict) or not isinstance(bathy.get("path"), str):
        raise ValueError("bathymetry.path is required")
    resolution = _number(bathy.get("resolution_m"), "bathymetry.resolution_m")
    if resolution <= 0:
        raise ValueError("bathymetry.resolution_m must be positive")
    return data
