"""Vehicle-configuration loading at the Simulation/Physics boundary."""

from __future__ import annotations

from pathlib import Path

import yaml

from physics import UUV3DOFParameters


DEFAULT_VEHICLE_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "vehicles"
    / "uuv_3dof_synthetic_v1.yaml"
)


def load_synthetic_parameters(
    config_path: str | Path = DEFAULT_VEHICLE_CONFIG,
) -> UUV3DOFParameters:
    """Load the named test-only model without adding YAML to Physics runtime."""
    with Path(config_path).open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError("vehicle configuration must be a mapping")
    if document.get("name") != "uuv_3dof_synthetic_v1":
        raise ValueError("unexpected vehicle configuration name")
    return UUV3DOFParameters(
        mass_rb=document["mass_rb"],
        mass_added=document["mass_added"],
        damping_linear=document["damping_linear"],
        damping_quadratic=document["damping_quadratic"],
        input_matrix=document["input_matrix"],
        restoring_pitch_coefficient=document["restoring_pitch_coefficient"],
    )
