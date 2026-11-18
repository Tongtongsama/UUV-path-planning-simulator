"""Clearly labelled current-data contract: synthetic, measured, estimated.

This module does not change the frozen G12/v4 model. It defines how a future
trial must declare which component is truth, which is estimated, and whether
the data is synthetic or measured.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import csv, json, math
from typing import Protocol

SCHEMA = "current-data-v1"
SOURCE_TYPES = ("synthetic_schedule", "measured", "estimated")


@dataclass(frozen=True)
class CurrentVector:
    time_s: float
    east_mps: float
    north_mps: float
    up_mps: float
    uncertainty_mps: float
    source_type: str
    source_id: str
    quality: str = "unknown"

    @property
    def is_measured(self) -> bool:
        return self.source_type == "measured"

    @property
    def is_synthetic(self) -> bool:
        return self.source_type == "synthetic_schedule"

    @property
    def is_estimated(self) -> bool:
        return self.source_type == "estimated"

    def as_enu(self):
        return (self.east_mps, self.north_mps, self.up_mps)


class CurrentProvider(Protocol):
    def at(self, time_s: float, position_enu_m=None) -> CurrentVector:
        ...


@dataclass(frozen=True)
class CurrentMetadata:
    source_type: str
    source_id: str
    license: str
    provenance: str
    datum: str
    time_reference: str = "UTC"
    vertical_reference: str = "unknown"

    def validate(self):
        if self.source_type not in SOURCE_TYPES:
            raise ValueError("Unknown current source_type")
        for name in ("source_id", "license", "provenance", "datum", "time_reference"):
            if not str(getattr(self, name)).strip():
                raise ValueError(f"Missing {name} for {self.source_type} current")
        if self.source_type == "synthetic_schedule" and "synthetic" not in self.provenance.lower():
            raise ValueError("Synthetic current must be labelled synthetic in provenance")
        if self.source_type == "measured" and "synthetic" in self.provenance.lower():
            raise ValueError("Measured current cannot carry synthetic provenance")
        return self


class ConstantProvider:
    def __init__(self, east, north, up, metadata: CurrentMetadata, uncertainty=0.0):
        self.east, self.north, self.up = east, north, up
        self.metadata = metadata.validate()
        self.uncertainty = uncertainty

    def at(self, time_s, position_enu_m=None):
        return CurrentVector(float(time_s), self.east, self.north, self.up,
                             float(self.uncertainty), self.metadata.source_type, self.metadata.source_id)


class SyntheticScheduleProvider:
    """Adapter around the frozen v4 F2 schedule; explicitly synthetic."""
    def __init__(self, metadata: CurrentMetadata):
        if metadata.source_type != "synthetic_schedule":
            raise ValueError("SyntheticScheduleProvider requires synthetic_schedule metadata")
        self.metadata = metadata.validate()
        from ros2.twin_axial_g12_current_v4.schedule import current
        self._current = current

    def at(self, time_s, position_enu_m=None):
        east, up = self._current(float(time_s))
        return CurrentVector(float(time_s), east, 0.0, up, 0.0,
                             self.metadata.source_type, self.metadata.source_id)


class MeasuredCSVProvider:
    """Uniform measured current profile with explicit time and provenance."""
    def __init__(self, csv_path, metadata: CurrentMetadata):
        if metadata.source_type != "measured":
            raise ValueError("MeasuredCSVProvider requires measured metadata")
        self.metadata = metadata.validate()
        rows = []
        with Path(csv_path).open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            required = {"time_s", "east_mps", "north_mps", "up_mps", "uncertainty_mps"}
            if not required.issubset(reader.fieldnames or []):
                raise ValueError(f"Measured CSV missing columns: {sorted(required)}")
            for row in reader:
                rows.append(tuple(float(row[k]) for k in ("time_s", "east_mps", "north_mps", "up_mps", "uncertainty_mps")))
        if len(rows) < 2:
            raise ValueError("Measured current needs at least two samples")
        if any(b[0] <= a[0] for a, b in zip(rows, rows[1:])):
            raise ValueError("Measured current times must be strictly increasing")
        if any(not all(math.isfinite(x) for x in row) for row in rows):
            raise ValueError("Measured current contains non-finite values")
        self.rows = rows

    def at(self, time_s, position_enu_m=None):
        t = float(time_s)
        if t < self.rows[0][0] or t > self.rows[-1][0]:
            raise ValueError("Measured current does not extrapolate outside its time span")
        for a, b in zip(self.rows, self.rows[1:]):
            if a[0] <= t <= b[0]:
                u = (t - a[0]) / (b[0] - a[0])
                vals = [a[i] + u * (b[i] - a[i]) for i in range(1, 4)]
                unc = max(a[4], b[4])
                return CurrentVector(t, vals[0], vals[1], vals[2], unc,
                                     self.metadata.source_type, self.metadata.source_id)
        raise ValueError("Measured current interpolation failed")


class EstimatedProvider:
    """Label an estimator output; it must not masquerade as measured truth."""
    def __init__(self, source_id, estimate_fn, uncertainty_mps=0.0, derived_from=""):
        if not source_id.strip() or not derived_from.strip():
            raise ValueError("Estimated current needs source_id and derived_from")
        self.source_id = source_id
        self.estimate_fn = estimate_fn
        self.uncertainty_mps = float(uncertainty_mps)
        self.derived_from = derived_from

    def at(self, time_s, position_enu_m=None):
        value = self.estimate_fn(float(time_s))
        if not all(math.isfinite(float(x)) for x in value):
            raise ValueError("Estimated current is non-finite")
        return CurrentVector(float(time_s), float(value[0]), float(value[1]), float(value[2]),
                             self.uncertainty_mps, "estimated", self.source_id)


class PairedCurrent:
    """Keep truth and estimate explicitly separate."""
    def __init__(self, truth: CurrentProvider, estimate: CurrentProvider):
        self.truth = truth
        self.estimate = estimate

    def true_at(self, time_s, position_enu_m=None):
        return self.truth.at(time_s, position_enu_m)

    def estimated_at(self, time_s, position_enu_m=None):
        return self.estimate.at(time_s, position_enu_m)
