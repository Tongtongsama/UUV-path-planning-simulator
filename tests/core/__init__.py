"""
Core data model for the UUV Simulator.

This package contains the fundamental data structures that serve as the
common "language" shared by all modules in the simulator.

All classes defined here are:
    - Immutable (frozen dataclasses)
    - Pure data containers (no algorithms, no side effects)
    - Independent of all other packages

Exports:
    Pose            - Position and orientation in ENU frame
    Twist           - Linear and angular velocities in body frame
    ControlInput    - Actuator commands
    VehicleState    - Combined state vector for the UUV
    Trajectory      - Generic sequence of waypoints/states
"""

from core.pose import Pose

# Future imports (uncomment as implemented):
# from core.twist import Twist
# from core.control_input import ControlInput
# from core.vehicle_state import VehicleState
# from core.trajectory import Trajectory

__all__ = [
    "Pose",
    # "Twist",
    # "ControlInput",
    # "VehicleState",
    # "Trajectory",
]