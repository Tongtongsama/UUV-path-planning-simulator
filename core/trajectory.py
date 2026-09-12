"""
Trajectory representation for the UUV Simulator.

A Trajectory is a time-parameterized sequence of VehicleStates representing
a reference motion that the Controller should track.

Trajectory is the output of trajectory generation (which takes a Path
and adds timing/velocity profiles) and serves as the reference input
to the Controller module.

Design Principles:
    1. Trajectory is time-parameterized — each VehicleState carries its own
       timestamp.
    2. Timestamps are NOT duplicated in a separate list (single source of truth).
    3. Trajectory is immutable (frozen dataclass with tuple).
    4. Empty Trajectory is not allowed.
    5. All elements must be VehicleState instances (validated at construction).
    6. Timestamps must be finite and strictly monotonically increasing.
    7. Minimal interface — no sampling, interpolation, or reversal in Core.
"""

from dataclasses import dataclass
import numpy as np

from core.vehicle_state import VehicleState


@dataclass(frozen=True)
class Trajectory:
    """
    An ordered time-parameterized sequence of VehicleStates.

    Each VehicleState contains its own timestamp. The sequence must be
    strictly increasing in time with all timestamps finite.

    Attributes:
        states: Tuple of VehicleState instances. Must be non-empty,
                all VehicleState instances, with strictly increasing
                finite timestamps. Input iterables are snapshotted to a tuple.
    """

    states: tuple[VehicleState, ...]

    def __post_init__(self) -> None:
        """Snapshot input, then validate types and strictly increasing times."""
        object.__setattr__(self, "states", tuple(self.states))
        if len(self.states) == 0:
            raise ValueError("Trajectory must contain at least one VehicleState")

        if not all(isinstance(state, VehicleState) for state in self.states):
            raise TypeError(
                "All Trajectory elements must be VehicleState instances"
            )

        timestamps = [state.timestamp for state in self.states]
        if not all(np.isfinite(t) for t in timestamps):
            raise ValueError("Trajectory timestamps must be finite")

        for i in range(len(timestamps) - 1):
            if timestamps[i + 1] <= timestamps[i]:
                raise ValueError(
                    f"Timestamps must be strictly increasing. "
                    f"Found t[{i}]={timestamps[i]}, t[{i+1}]={timestamps[i+1]}"
                )

    # =========================================================================
    # Factory Methods
    # =========================================================================

    @classmethod
    def from_list(cls, states: list[VehicleState]) -> "Trajectory":
        """Construct a Trajectory from a list of VehicleStates."""
        return cls(states=tuple(states))

    # =========================================================================
    # Properties
    # =========================================================================

    @property
    def start_time(self) -> float:
        """Timestamp of the first state (s)."""
        return self.states[0].timestamp

    @property
    def end_time(self) -> float:
        """Timestamp of the last state (s)."""
        return self.states[-1].timestamp

    @property
    def duration(self) -> float:
        """
        Total time span of the trajectory.

        Returns:
            end_time - start_time in seconds.
        """
        return self.end_time - self.start_time

    # =========================================================================
    # Comparison Methods
    # =========================================================================

    def is_close(self, other: "Trajectory",
                 pos_tol: float = 1e-6, ang_tol: float = 1e-6,
                 lin_tol: float = 1e-6, ang_vel_tol: float = 1e-6,
                 time_tol: float = 1e-9) -> bool:
        """
        Check if two Trajectories are approximately equal.

        Args:
            other:       Another Trajectory instance.
            pos_tol:     Tolerance for position (m).
            ang_tol:     Tolerance for orientation (rad).
            lin_tol:     Tolerance for linear velocity (m/s).
            ang_vel_tol: Tolerance for angular velocity (rad/s).
            time_tol:    Tolerance for timestamp (s).

        Returns:
            True if both have the same number of states and all
            corresponding VehicleStates are close.
        """
        if not isinstance(other, Trajectory):
            raise TypeError(f"Expected Trajectory, got {type(other).__name__}")
        if len(self.states) != len(other.states):
            return False
        return all(
            s1.is_close(s2, pos_tol=pos_tol, ang_tol=ang_tol,
                        lin_tol=lin_tol, ang_vel_tol=ang_vel_tol,
                        time_tol=time_tol)
            for s1, s2 in zip(self.states, other.states)
        )

    # =========================================================================
    # Sequence Protocol
    # =========================================================================

    def __len__(self) -> int:
        """Number of states in the trajectory."""
        return len(self.states)

    def __getitem__(self, index: int) -> VehicleState:
        """Access state by index."""
        return self.states[index]

    def __iter__(self):
        """Iterate over states."""
        return iter(self.states)

    # =========================================================================
    # Dunder Methods
    # =========================================================================

    def __str__(self) -> str:
        return (
            f"Trajectory({len(self.states)} states, "
            f"t=[{self.start_time:.2f}, {self.end_time:.2f}]s, "
            f"duration={self.duration:.2f}s)"
        )

    def __repr__(self) -> str:
        return self.__str__()
