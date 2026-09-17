"""Symmetric generalized-force limits supplied before integral updates."""
from dataclasses import dataclass
from ._validation import finite_scalar


@dataclass(frozen=True)
class ControlLimits3DOF:
    """Available surge force (N) and pitch moment (N m), not actuator allocation."""
    maximum_tau_x: float
    maximum_tau_m: float

    def __post_init__(self) -> None:
        for name in ('maximum_tau_x','maximum_tau_m'):
            value=finite_scalar(getattr(self,name),name=name)
            if value<=0:
                raise ValueError(f'{name} must be positive')
            object.__setattr__(self,name,value)
