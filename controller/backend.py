"""Public protocol for Control v0.5 implementations."""

from typing import Protocol

from core import ControlInput, VehicleState


class Controller3DOF(Protocol):
    def step(
        self,
        current: VehicleState,
        reference: VehicleState,
        dt: float,
    ) -> ControlInput: ...

    def reset(self) -> None: ...
