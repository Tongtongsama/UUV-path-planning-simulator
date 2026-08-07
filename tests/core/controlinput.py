"""
Control input representation for the UUV Simulator.

Represents the generalized forces and moments applied to the UUV by actuators
(thrusters, rudders, fins, etc.) expressed in the body frame.

Frame Convention: SNAME body-fixed frame
    tau_x: Force along body x-axis (N, surge direction)
    tau_y: Force along body y-axis (N, sway direction)
    tau_z: Force along body z-axis (N, heave direction)
    tau_k: Moment about body x-axis (N·m, roll direction)
    tau_m: Moment about body y-axis (N·m, pitch direction)
    tau_n: Moment about body z-axis (N·m, yaw direction)

Design Principle:
    ControlInput always contains all 6 components — it is a complete
    generalized-force container. Which components are active / directly
    actuated is determined by the Physics Engine and vehicle configuration,
    not by the data model itself.

Reference: Fossen, "Handbook of Marine Craft Hydrodynamics and Motion Control"
           τ = [X, Y, Z, K, M, N]^T in SNAME notation
"""

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class ControlInput:
    """
    Generalized forces and moments acting on the UUV in the body frame.

    This is the output of the Controller module and the input to the
    Physics Engine (dynamics).

    All 6 components are always present. The vehicle's actuator configuration
    (e.g., fully actuated horizontal plane, underactuated vertical plane, etc.)
    determines which components receive non-zero values — this decision lives
    in the Physics Engine and vehicle configuration, not in ControlInput.

    Attributes:
        tau_x: Force along body x-axis (N), surge
        tau_y: Force along body y-axis (N), sway
        tau_z: Force along body z-axis (N), heave
        tau_k: Moment about body x-axis (N·m), roll
        tau_m: Moment about body y-axis (N·m), pitch
        tau_n: Moment about body z-axis (N·m), yaw
    """

    tau_x: float = 0.0
    tau_y: float = 0.0
    tau_z: float = 0.0
    tau_k: float = 0.0
    tau_m: float = 0.0
    tau_n: float = 0.0

    # =========================================================================
    # Properties
    # =========================================================================

    @property
    def force_vector(self) -> np.ndarray:
        """Force vector [tau_x, tau_y, tau_z] in body frame (N)."""
        return np.array([self.tau_x, self.tau_y, self.tau_z])

    @property
    def moment_vector(self) -> np.ndarray:
        """Moment vector [tau_k, tau_m, tau_n] in body frame (N·m)."""
        return np.array([self.tau_k, self.tau_m, self.tau_n])

    @property
    def generalized_force(self) -> np.ndarray:
        """Full generalized force vector τ (6,) = [X, Y, Z, K, M, N]^T."""
        return np.array([self.tau_x, self.tau_y, self.tau_z,
                         self.tau_k, self.tau_m, self.tau_n])

    # =========================================================================
    # Factory Methods
    # =========================================================================

    @classmethod
    def zero(cls) -> "ControlInput":
        """Construct a zero control input (no actuation)."""
        return cls()

    @classmethod
    def from_list(cls, data: list) -> "ControlInput":
        """Construct from a list [tau_x, tau_y, tau_z, tau_k, tau_m, tau_n]."""
        if len(data) != 6:
            raise ValueError(f"Expected 6 elements, got {len(data)}")
        return cls(*data)

    @classmethod
    def from_numpy(cls, array: np.ndarray) -> "ControlInput":
        """Construct from a (6,) numpy array."""
        array = np.asarray(array).flatten()
        if array.shape != (6,):
            raise ValueError(f"Expected shape (6,), got {array.shape}")
        return cls(
            float(array[0]), float(array[1]), float(array[2]),
            float(array[3]), float(array[4]), float(array[5]),
        )

    # =========================================================================
    # Conversion Methods
    # =========================================================================

    def to_list(self) -> list:
        """Convert to list [tau_x, tau_y, tau_z, tau_k, tau_m, tau_n]."""
        return [self.tau_x, self.tau_y, self.tau_z,
                self.tau_k, self.tau_m, self.tau_n]

    def to_numpy(self) -> np.ndarray:
        """Convert to numpy array (6,)."""
        return self.generalized_force

    # =========================================================================
    # Physics Methods
    # =========================================================================

    def norm_force(self) -> float:
        """Magnitude of the force vector sqrt(tau_x² + tau_y² + tau_z²) in N."""
        return float(np.sqrt(self.tau_x**2 + self.tau_y**2 + self.tau_z**2))

    def norm_moment(self) -> float:
        """Magnitude of the moment vector sqrt(tau_k² + tau_m² + tau_n²) in N·m."""
        return float(np.sqrt(self.tau_k**2 + self.tau_m**2 + self.tau_n**2))

    def is_zero(self, tol: float = 1e-9) -> bool:
        """Check if this is effectively a zero control input."""
        return (
            abs(self.tau_x) < tol and abs(self.tau_y) < tol and
            abs(self.tau_z) < tol and abs(self.tau_k) < tol and
            abs(self.tau_m) < tol and abs(self.tau_n) < tol
        )

    def is_close(self, other: "ControlInput", force_tol: float = 1e-6,
                 moment_tol: float = 1e-6) -> bool:
        """
        Check if two ControlInputs are approximately equal.

        Forces compared with force_tol (N).
        Moments compared with moment_tol (N·m).
        """
        if not isinstance(other, ControlInput):
            raise TypeError(f"Expected ControlInput, got {type(other).__name__}")
        return (
            abs(self.tau_x - other.tau_x) < force_tol and
            abs(self.tau_y - other.tau_y) < force_tol and
            abs(self.tau_z - other.tau_z) < force_tol and
            abs(self.tau_k - other.tau_k) < moment_tol and
            abs(self.tau_m - other.tau_m) < moment_tol and
            abs(self.tau_n - other.tau_n) < moment_tol
        )

    # =========================================================================
    # Dunder Methods
    # =========================================================================

    def __str__(self) -> str:
        return (
            f"ControlInput(τ_x={self.tau_x:.3f}, τ_y={self.tau_y:.3f}, "
            f"τ_z={self.tau_z:.3f}, τ_k={self.tau_k:.3f}, "
            f"τ_m={self.tau_m:.3f}, τ_n={self.tau_n:.3f})"
        )

    def __repr__(self) -> str:
        return self.__str__()