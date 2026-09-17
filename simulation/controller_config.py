"""Controller configuration loading at the Simulation boundary."""

from __future__ import annotations

from pathlib import Path
from dataclasses import MISSING

import yaml

from controller import CascadedPID3DOFParameters


DEFAULT_CONTROLLER_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "controllers"
    / "cascaded_pid_3dof_baseline.yaml"
)


def load_baseline_controller_parameters(
    config_path: str | Path = DEFAULT_CONTROLLER_CONFIG,
) -> CascadedPID3DOFParameters:
    with Path(config_path).open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError("controller configuration must be a mapping")
    if document.get("name") not in ("cascaded_pid_3dof_baseline","cascaded_pid_3dof_restoring_candidate"):
        raise ValueError("unexpected controller configuration name")
    parameter_names = CascadedPID3DOFParameters.__dataclass_fields__
    return CascadedPID3DOFParameters(
        **{name: document[name] if name in document else field.default
           for name,field in parameter_names.items()
           if name in document or field.default is not MISSING}
    )
