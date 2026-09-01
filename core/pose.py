"""
Pose representation for the UUV Simulator.

Coordinate Frame: ENU (East-North-Up)
    x: East  (m)
    y: North (m)
    z: Up    (m)
    roll:  rotation about x-axis (rad)
    pitch: rotation about y-axis (rad)
    yaw:   rotation about z-axis (rad, positive: right-hand rule about z-up)

Design Principle:
    Pose always contains all 6 components.
    Which subset is active (e.g., horizontal plane [x, y, yaw],
    vertical plane [x, z, pitch]) is determined by the Physics Engine
    configuration, not by the data model.
"""

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class Pose:
    """
    Represents the position and orientation of the UUV in ENU frame.

    All 6 components are always present. In any given simulation,
    the Physics Engine uses only the relevant subset based on its
    degrees-of-freedom configuration.

    Attributes:
        x:     East position (m)
        y:     North position (m)
        z:     Up position (m)
        roll:  Euler angle about x-axis (rad)
        pitch: Euler angle about y-axis (rad)
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
        """2D horizontal position [x, y] (m)."""
        return np.array([self.x, self.y])

    @property
    def heading(self) -> float:
        """Semantic alias for yaw in horizontal motion context (rad)."""
        return self.yaw

    # =========================================================================
    # Factory Methods
    # =========================================================================

        return cls(x=x, y=y, z=z, roll=0.0, pitch=0.0, yaw=yaw)
    @classmethod
    def zero(cls) -> "Pose":
        """Construct a Pose at the origin with zero orientation."""
        return cls()

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
    def from_position_and_yaw(cls, x: float = 0.0, y: float = 0.0, 
                               z: float = 0.0, yaw: float = 0.0) -> "Pose":
        """
        Construct a Pose from position and yaw only.
        Roll and pitch are set to 0.
        
        This is a convenience method for horizontal plane operations
        where only position and heading are relevant.
        
        Args:
            x: East position (m)
            y: North position (m)  
            z: Up position (m)
            yaw: Heading angle about z-axis (rad)
            
        Returns:
            Pose with specified position and yaw, roll=0, pitch=0
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
        Euclidean distance to another Pose (3D).

        In horizontal-only operation with z=0, this reduces to 2D distance.
        """
        if not isinstance(other, Pose):
            raise TypeError(f"Expected Pose, got {type(other).__name__}")
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return float(np.sqrt(dx * dx + dy * dy + dz * dz))

    def norm_position(self) -> float:
        """Euclidean norm of the 3D position vector sqrt(x² + y² + z²)."""
        return float(np.sqrt(self.x * self.x + self.y * self.y + self.z * self.z))

    def norm_xy(self) -> float:
        """Euclidean norm of horizontal position sqrt(x² + y²)."""
        return float(np.sqrt(self.x * self.x + self.y * self.y))

    def is_close(self, other: "Pose", pos_tol: float = 1e-6, ang_tol: float = 1e-6) -> bool:
        """
        Check if two Poses are approximately equal.

        Position components compared with pos_tol (m).
        Orientation components compared with ang_tol (rad).
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