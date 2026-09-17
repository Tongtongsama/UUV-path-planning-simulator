"""Validated immutable parameters for the v0.5 cascaded PID baseline."""

from __future__ import annotations

from dataclasses import dataclass, fields

from controller._validation import finite_scalar


@dataclass(frozen=True)
class CascadedPID3DOFParameters:
    position_to_surge_gain: float
    depth_to_pitch_gain: float
    maximum_surge_reference: float
    maximum_pitch_correction: float
    surge_kp: float
    surge_ki: float
    pitch_kp: float
    pitch_ki: float
    pitch_rate_kd: float
    maximum_tau_x: float
    maximum_tau_m: float
    surge_integral_limit: float
    pitch_integral_limit: float
    restoring_pitch_coefficient: float = 0.0

    def __post_init__(self) -> None:
        for item in fields(self):
            value = finite_scalar(getattr(self, item.name), name=item.name)
            if value < 0.0:
                raise ValueError(f"{item.name} must be non-negative")
            object.__setattr__(self, item.name, value)

        for name in (
            "maximum_surge_reference",
            "maximum_pitch_correction",
            "maximum_tau_x",
            "maximum_tau_m",
            "surge_integral_limit",
            "pitch_integral_limit",
        ):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be strictly positive")
