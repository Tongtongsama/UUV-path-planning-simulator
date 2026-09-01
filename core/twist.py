"""
Twist (velocity) representation for the UUV Simulator.

Coordinate Frame: Body Frame
    u: Surge velocity  — linear velocity along body x-axis (m/s, forward positive)
    v: Sway velocity   — linear velocity along body y-axis (m/s, right positive)
    w: Heave velocity  — linear velocity along body z-axis (m/s, down positive)
    p: Roll rate       — angular velocity about body x-axis (rad/s)
    q: Pitch rate      — angular velocity about body y-axis (rad/s)
    r: Yaw rate        — angular velocity about body z-axis (rad/s)

Design Principle:
    Twist always contains all 6 components.
    Which subset is active is determined by the Physics Engine configuration.

Reference: Fossen, "Handbook of Marine Craft Hydrodynamics and Motion Control"
           SNAME (1950) notation convention
"""

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class Twist:
    """
    Represents linear and angular velocities of the UUV in the body frame.

    Uses standard SNAME notation:
        Linear:  u (surge), v (sway), w (heave)
        Angular: p (roll),  q (pitch), r (yaw)

    All 6 components are always present. Active subset determined by
    Physics Engine configuration.

    Attributes:
        u: Surge velocity (m/s), forward positive
        v: Sway velocity (m/s), right positive
        w: Heave velocity (m/s), down positive
        p: Roll rate (rad/s)
        q: Pitch rate (rad/s)
        r: Yaw rate (rad/s)
    """

    u: float = 0.0
    v: float = 0.0
    w: float = 0.0
    p: float = 0.0
    q: float = 0.0
    r: float = 0.0

    # =========================================================================
    # Properties
    # =========================================================================

    @property
    def linear(self) -> np.ndarray:
        """Linear velocity vector [u, v, w] in body frame (m/s)."""
        return np.array([self.u, self.v, self.w])

    @property
    def angular(self) -> np.ndarray:
        """Angular velocity vector [p, q, r] in body frame (rad/s)."""
        return np.array([self.p, self.q, self.r])

    @property
    def velocity_2d(self) -> np.ndarray:
        """2D horizontal velocity [u, v] (m/s)."""
        return np.array([self.u, self.v])

    # =========================================================================
    # Factory Methods
    # =========================================================================

    @classmethod
    def zero(cls) -> "Twist":
        """Construct a Twist with all velocities zero."""
        return cls()

    @classmethod
    def from_list(cls, data: list) -> "Twist":
        """Construct Twist from a list [u, v, w, p, q, r]."""
        if len(data) != 6:
            raise ValueError(f"Expected 6 elements, got {len(data)}")
        return cls(*data)

    @classmethod
    def from_numpy(cls, array: np.ndarray) -> "Twist":
        """Construct Twist from a (6,) numpy array."""
        array = np.asarray(array).flatten()
        if array.shape != (6,):
            raise ValueError(f"Expected shape (6,), got {array.shape}")
        return cls(
            float(array[0]), float(array[1]), float(array[2]),
            float(array[3]), float(array[4]), float(array[5]),
        )

    @classmethod
    def from_3dof(cls, u: float = 0.0, v: float = 0.0, r: float = 0.0) -> "Twist":
        """
        Construct a Twist from 3-DOF horizontal velocities only.
        w, p, q are set to 0.
        
        This is a convenience method for horizontal plane operations
        where only surge, sway, and yaw rate are relevant.
        
        Args:
            u: Surge velocity (m/s), forward positive
            v: Sway velocity (m/s), right positive
            r: Yaw rate (rad/s)
            
        Returns:
            Twist with specified u, v, r, and w=p=q=0
        """
        return cls(u=u, v=v, w=0.0, p=0.0, q=0.0, r=r)
    # =========================================================================
    # Conversion Methods
    # =========================================================================

    def to_list(self) -> list:
        """Convert to list [u, v, w, p, q, r]."""
        return [self.u, self.v, self.w, self.p, self.q, self.r]

    def to_numpy(self) -> np.ndarray:
        """Convert to numpy array (6,)."""
        return np.array([self.u, self.v, self.w, self.p, self.q, self.r])

    # =========================================================================
    # Geometry / Physics Methods
    # =========================================================================

    def norm_linear(self) -> float:
        """Magnitude of linear velocity sqrt(u² + v² + w²) in m/s."""
        return float(np.sqrt(self.u * self.u + self.v * self.v + self.w * self.w))

    def norm_angular(self) -> float:
        """Magnitude of angular velocity sqrt(p² + q² + r²) in rad/s."""
        return float(np.sqrt(self.p * self.p + self.q * self.q + self.r * self.r))

    def speed_2d(self) -> float:
        """Horizontal speed sqrt(u² + v²) in m/s."""
        return float(np.sqrt(self.u * self.u + self.v * self.v))

    def is_close(self, other: "Twist", lin_tol: float = 1e-6, ang_tol: float = 1e-6) -> bool:
        """
        Check if two Twists are approximately equal.

        Linear components compared with lin_tol (m/s).
        Angular components compared with ang_tol (rad/s).
        """
        if not isinstance(other, Twist):
            raise TypeError(f"Expected Twist, got {type(other).__name__}")
        return (
            abs(self.u - other.u) < lin_tol
            and abs(self.v - other.v) < lin_tol
            and abs(self.w - other.w) < lin_tol
            and abs(self.p - other.p) < ang_tol
            and abs(self.q - other.q) < ang_tol
            and abs(self.r - other.r) < ang_tol
        )

    # =========================================================================
    # Dunder Methods
    # =========================================================================

    def __str__(self) -> str:
        return (
            f"Twist(u={self.u:.3f}, v={self.v:.3f}, w={self.w:.3f}, "
            f"p={self.p:.3f}, q={self.q:.3f}, r={self.r:.3f})"
        )

    def __repr__(self) -> str:
        return self.__str__()