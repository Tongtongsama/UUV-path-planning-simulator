"""Pure-Python vertical-plane Physics v0.4 foundations."""

from physics.backend import (
    PhysicsBackend,
    extract_reduced_control,
    extract_reduced_state,
    rebuild_vehicle_state,
)
from physics.derivative import StateDerivative3DOF
from physics.frames import body_to_world_velocity_matrix, world_current_to_body
from physics.integrators import (
    DerivativeFunction3DOF,
    EulerIntegrator,
    Integrator,
    RK4Integrator,
)
from physics.parameters import UUV3DOFParameters
from physics.uuv_3dof import (
    Python3DOFBackend,
    Python3DOFDynamics,
    added_mass_coriolis,
    rigid_body_coriolis,
)

__all__ = [
    "DerivativeFunction3DOF",
    "EulerIntegrator",
    "Integrator",
    "PhysicsBackend",
    "Python3DOFBackend",
    "Python3DOFDynamics",
    "RK4Integrator",
    "StateDerivative3DOF",
    "UUV3DOFParameters",
    "added_mass_coriolis",
    "body_to_world_velocity_matrix",
    "extract_reduced_control",
    "extract_reduced_state",
    "rebuild_vehicle_state",
    "rigid_body_coriolis",
    "world_current_to_body",
]
