"""
Core data model for the UUV Simulator.

This package contains the fundamental data structures that serve as the
common "language" shared by all modules in the simulator.

All classes are:
    - Immutable (frozen dataclasses)
    - Pure data containers (no algorithms or side effects)
    - Independent of all other packages

Exports:
    Pose            - Position and orientation in ENU frame
    Twist           - Linear and angular velocities in body frame
    ControlInput    - Generalized forces and moments in body frame
    VehicleState    - Time-stamped composition of Pose + Twist
    Path            - Geometric sequence of Poses
    Trajectory      - Time-parameterized sequence of VehicleStates
"""

from core.pose import Pose
from core.twist import Twist
from core.control_input import ControlInput
from core.vehicle_state import VehicleState
from core.path import Path
from core.trajectory import Trajectory

__all__ = [
    "Pose",
    "Twist",
    "ControlInput",
    "VehicleState",
    "Path",
    "Trajectory",
]