"""Public protocol for Control v0.5 implementations."""

from typing import Protocol, runtime_checkable
from controller.limits import ControlLimits3DOF
from controller.guidance import BodyGuidance3DOF

from core import ControlInput, VehicleState


class Controller3DOF(Protocol):
    def step(
        self,
        current: VehicleState,
        reference: VehicleState,
        dt: float,
    ) -> ControlInput: ...

    def reset(self) -> None: ...


@runtime_checkable
class LimitAwareController3DOF(Protocol):
    """Optional capability: integrate against this step's final available limits."""
    def step_with_limits(self, current: VehicleState, reference: VehicleState,
                         dt: float, limits: ControlLimits3DOF) -> ControlInput: ...


@runtime_checkable
class GuidanceController3DOF(Protocol):
    """Explicitly bypass position/depth outer loops, retaining inner PID and FF."""
    def step_guidance(self,current: VehicleState,target: BodyGuidance3DOF,
                      dt: float,limits: ControlLimits3DOF) -> ControlInput: ...
