"""
Vehicle state representation for the UUV Simulator.

VehicleState is a time-stamped composition of Pose and Twist.
It is the complete state object produced by the Physics Engine at each
time step and consumed by Controller, Planner, and Logger.

Frame Convention:
    Pose   → ENU inertial frame (x, y, z, roll, pitch, yaw)
    Twist  → SNAME body-fixed frame (u, v, w, p, q, r)

Design Principles:
    1. VehicleState composes Pose and Twist — it does not duplicate fields.
    2. to_numpy() returns the 12-D physical state vector [η; ν].
       Timestamp is metadata, NOT a state variable.
    3. Pose and Twist remain first-class objects, accessible directly.
    4. No silent initialization — all fields must be explicitly provided.
"""

from dataclasses import dataclass
import numpy as np

from core.pose import Pose
from core.twist import Twist


@dataclass(frozen=True)
class VehicleState:
    """
    Complete time-stamped state of the UUV.

    Composed of:
        pose:      Position and orientation in ENU inertial frame (η)
        twist:     Linear and angular velocities in body frame (ν)
        timestamp: Simulation time in seconds (metadata, not part of state vector)

    Attributes:
        pose:      Pose instance
        twist:     Twist instance
        timestamp: Simulation time (s)
    """

    pose: Pose
    twist: Twist
    timestamp: float = 0.0

    # =========================================================================
    # Factory Methods
    # =========================================================================

    @classmethod
    def zero(cls, timestamp: float = 0.0) -> "VehicleState":
        """
        Construct a VehicleState at origin with zero velocity.

        Args:
            timestamp: Simulation time (s). Default: 0.0.

        Returns:
            VehicleState with Pose.zero() and Twist.zero().
        """
        return cls(pose=Pose.zero(), twist=Twist.zero(), timestamp=timestamp)

    # =========================================================================
    # Conversion Methods
    # =========================================================================
    

    def to_numpy(self) -> np.ndarray:
        """
        Convert to the 12-D physical state vector [η; ν].

        Layout: [x, y, z, roll, pitch, yaw, u, v, w, p, q, r]

        Timestamp is explicitly excluded — it is temporal metadata,
        not a dynamical state variable.

        Returns:
            numpy array of shape (12,).
        """
        return np.concatenate([
            self.pose.to_numpy(),   # (6,)
            self.twist.to_numpy(),  # (6,)
        ])

    @classmethod
    def from_numpy(cls, array: np.ndarray, timestamp: float = 0.0) -> "VehicleState":
        """
        Construct from a 12-D physical state vector plus timestamp.

        Layout: [x, y, z, roll, pitch, yaw, u, v, w, p, q, r]

        Timestamp is a separate parameter because it is metadata,
        not part of the dynamical state.

        Args:
            array:     numpy array of shape (12,).
            timestamp: Simulation time (s). Default: 0.0.

        Returns:
            VehicleState instance.
        """
        array = np.asarray(array, dtype=np.float64)
        if array.ndim != 1:
            raise ValueError(f"Expected 1-D array, got {array.ndim}-D")
        if array.shape != (12,):
            raise ValueError(f"Expected shape (12,), got {array.shape}")
        pose = Pose.from_numpy(array[0:6])
        twist = Twist.from_numpy(array[6:12])
        return cls(pose=pose, twist=twist, timestamp=timestamp)
    

    # =========================================================================
    # Comparison Methods
    # =========================================================================

    def is_close(self, other: "VehicleState",
                 pos_tol: float = 1e-6, ang_tol: float = 1e-6,
                 lin_tol: float = 1e-6, ang_vel_tol: float = 1e-6,
                 time_tol: float = 1e-9) -> bool:
        """
        Check if two VehicleStates are approximately equal.

        Delegates to Pose.is_close and Twist.is_close, then compares
        timestamps separately.

        Args:
            other:       Another VehicleState instance.
            pos_tol:     Tolerance for position (m).
            ang_tol:     Tolerance for orientation (rad).
            lin_tol:     Tolerance for linear velocity (m/s).
            ang_vel_tol: Tolerance for angular velocity (rad/s).
            time_tol:    Tolerance for timestamp (s).

        Returns:
            True if all components are within tolerance.
        """
        if not isinstance(other, VehicleState):
            raise TypeError(f"Expected VehicleState, got {type(other).__name__}")
        return (
            self.pose.is_close(other.pose, pos_tol=pos_tol, ang_tol=ang_tol)
            and self.twist.is_close(other.twist, lin_tol=lin_tol, ang_tol=ang_vel_tol)
            and abs(self.timestamp - other.timestamp) < time_tol
        )

    # =========================================================================
    # Dunder Methods
    # =========================================================================

    def __str__(self) -> str:
        return (
            f"VehicleState(t={self.timestamp:.3f}s, "
            f"pose={self.pose}, twist={self.twist})"
        )

    def __repr__(self) -> str:
        return self.__str__()