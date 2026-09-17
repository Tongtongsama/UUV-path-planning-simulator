"""Control v0.5 underactuated 3-DOF baseline."""

from controller.angles import wrap_to_pi
from controller.backend import Controller3DOF, LimitAwareController3DOF
from controller.limits import ControlLimits3DOF
from controller.guidance import BodyGuidance3DOF
from controller.backend import GuidanceController3DOF
from controller.cascaded_pid_3dof import (
    CascadedPID3DOFController,
    ControllerMemory3DOF,
    ControllerDiagnostics3DOF,
)
from controller.parameters import CascadedPID3DOFParameters

__all__ = [
    "CascadedPID3DOFController",
    "CascadedPID3DOFParameters",
    "Controller3DOF",
    "ControllerMemory3DOF",
    "ControllerDiagnostics3DOF",
    "ControlLimits3DOF",
    "BodyGuidance3DOF",
    "GuidanceController3DOF",
    "LimitAwareController3DOF",
    "wrap_to_pi",
]
