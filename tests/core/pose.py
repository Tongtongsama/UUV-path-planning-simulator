"""
Pose representation for the UUV Simulator.

Coordinate Frame: ENU (East-North-Up)
    x: East  (m)
    y: North (m)
    z: Up    (m)
    roll:  rotation about x-axis (rad)
    pitch: rotation about y-axis (rad)
    yaw:   rotation about z-axis (rad, positive: right-hand rule about z-up)
"""

from dataclasses import dataclass
from typing import Tuple
import numpy as np


@dataclass(frozen=True)
class Pose:
    """
    Represents the position and orientation of the UUV in ENU frame.

    For 3-DOF operation:
        - z, roll, pitch are maintained but set to 0.0
        - Only x, y, yaw are actively used in dynamics
        - Data structure remains compatible with future 6-DOF extension

    Attributes:
        x:     East position (m)
        y:     North position (m)
        z:     Up position (m), 0.0 in 3-DOF mode
        roll:  Euler angle about x-axis (rad), 0.0 in 3-DOF mode
        pitch: Euler angle about y-axis (rad), 0.0 in 3-DOF mode
        yaw:   Euler angle about z-axis (rad)
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    # =========================================================================
    # Properties
    # =========================================================================

    @property
    def xy(self) -> np.ndarray:
        """
        2D position in the horizontal plane.

        Returns:
            numpy array [x, y].

        Usage:
            - Planner: 2D heuristic distance computations
            - Obstacle: 2D collision checks
        """
        return np.array([self.x, self.y])

    @property
    def heading(self) -> float:
        """
        Semantic alias for yaw in the context of 3-DOF horizontal motion.

        In ENU frame:
            heading = 0      → pointing East
            heading = pi/2   → pointing North
            heading = pi     → pointing West
            heading = -pi/2  → pointing South

        Returns:
            yaw angle in radians.

        Usage:
            - Planner: waypoint heading
            - Controller: heading error computation
        """
        return self.yaw

    # =========================================================================
    # Factory Methods
    # =========================================================================

    @classmethod
    def from_list(cls, data: list) -> "Pose":
        """Construct Pose from a list [x, y, z, roll, pitch, yaw]."""
        if len(data) != 6:
            raise ValueError(f"Expected 6 elements, got {len(data)}")
        return cls(*data)

    @classmethod
    def from_numpy(cls, array: np.ndarray) -> "Pose":
        """Construct Pose from a (6,) numpy array."""
        array = np.asarray(array).flatten()
        if array.shape != (6,):
            raise ValueError(f"Expected shape (6,), got {array.shape}")
        return cls(
            float(array[0]), float(array[1]), float(array[2]),
            float(array[3]), float(array[4]), float(array[5]),
        )

    @classmethod
    def from_position_and_yaw(cls, x: float, y: float, z: float, yaw: float) -> "Pose":
        """Construct a 3-DOF compatible Pose with only position and yaw.

        This is the primary factory for 3-DOF operation.
        roll and pitch are set to 0.0.
        """
        return cls(x=x, y=y, z=z, roll=0.0, pitch=0.0, yaw=yaw)

    # =========================================================================
    # Conversion Methods
    # =========================================================================

    def to_list(self) -> list:
        """Convert to list [x, y, z, roll, pitch, yaw]."""
        return [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]

    def to_numpy(self) -> np.ndarray:
        """Convert to numpy array (6,)."""
        return np.array([self.x, self.y, self.z, self.roll, self.pitch, self.yaw])

    def position(self) -> np.ndarray:
        """Return 3D position vector [x, y, z]."""
        return np.array([self.x, self.y, self.z])

    def orientation_euler(self) -> np.ndarray:
        """Return orientation as Euler angles [roll, pitch, yaw]."""
        return np.array([self.roll, self.pitch, self.yaw])

    # =========================================================================
    # Geometry Methods
    # =========================================================================

    def distance_to(self, other: "Pose") -> float:
        """
        Compute the Euclidean distance to another Pose.

        Uses the full 3D position (x, y, z).
        In 3-DOF mode with z=0, this reduces to the 2D distance sqrt((Δx)² + (Δy)²).

        Args:
            other: Another Pose instance.

        Returns:
            Euclidean distance in meters.

        Usage:
            - Planner: heuristic cost between waypoints
            - Obstacle: distance from UUV to obstacle center
        """
        if not isinstance(other, Pose):
            raise TypeError(f"Expected Pose, got {type(other).__name__}")
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return float(np.sqrt(dx * dx + dy * dy + dz * dz))

    def norm_position(self) -> float:
        """
        Compute the Euclidean norm of the position vector.

        Returns:
            ||[x, y, z]|| = sqrt(x² + y² + z²)

        Usage:
            - Physics: computing distance from origin
            - Controller: computing position error magnitude
        """
        return float(np.sqrt(self.x * self.x + self.y * self.y + self.z * self.z))

    def norm_xy(self) -> float:
        """
        Compute the 2D Euclidean norm of the horizontal position.

        Returns:
            ||[x, y]|| = sqrt(x² + y²)

        Usage:
            - Planner: 2D path length computations in 3-DOF mode
        """
        return float(np.sqrt(self.x * self.x + self.y * self.y))

    def is_close(self, other: "Pose", pos_tol: float = 1e-6, ang_tol: float = 1e-6) -> bool:
        """
        Check if two Poses are approximately equal.

        Compares both position and orientation element-wise.
        Position components are compared with pos_tol,
        orientation components are compared with ang_tol.

        Args:
            other:   Another Pose instance.
            pos_tol: Absolute tolerance for position (m). Default: 1e-6.
            ang_tol: Absolute tolerance for orientation (rad). Default: 1e-6.

        Returns:
            True if all components are within tolerance.

        Usage:
            - Tests: assert pose1.is_close(pose2)
            - Physics: checking if UUV has reached a waypoint
        """
        if not isinstance(other, Pose):
            raise TypeError(f"Expected Pose, got {type(other).__name__}")
        return (
            abs(self.x - other.x) < pos_tol
            and abs(self.y - other.y) < pos_tol
            and abs(self.z - other.z) < pos_tol
            and abs(self.roll - other.roll) < ang_tol
            and abs(self.pitch - other.pitch) < ang_tol
            and abs(self.yaw - other.yaw) < ang_tol
        )

    # =========================================================================
    # Dunder Methods
    # =========================================================================

    def __str__(self) -> str:
        return (
            f"Pose(x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f}, "
            f"roll={self.roll:.3f}, pitch={self.pitch:.3f}, yaw={self.yaw:.3f})"
        )

    def __repr__(self) -> str:
        return self.__str__()