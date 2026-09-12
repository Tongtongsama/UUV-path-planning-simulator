"""
Geometric path representation for the UUV Simulator.

A Path is a purely geometric sequence of Poses representing a route
through configuration space. It has no time information.

Path is the output of the Planner module (A*, RRT*, etc.) and serves
as input to trajectory generation before being tracked by the Controller.

Design Principles:
    1. Path is purely geometric — no timestamps.
    2. Path is immutable (frozen dataclass with tuple).
    3. Empty Path is not allowed.
    4. All elements must be Pose instances (validated at construction).
    5. Minimal interface — no smoothing, resampling, or interpolation.
"""

from dataclasses import dataclass

from core.pose import Pose


@dataclass(frozen=True)
class Path:
    """
    An ordered sequence of Poses defining a geometric route.

    Attributes:
        poses: Tuple of Pose instances defining the path.
               Must be non-empty. Input iterables are snapshotted to a tuple.
    """

    poses: tuple[Pose, ...]

    def __post_init__(self) -> None:
        """Snapshot the input before validating non-empty Pose membership."""
        object.__setattr__(self, "poses", tuple(self.poses))
        if len(self.poses) == 0:
            raise ValueError("Path must contain at least one Pose")
        if not all(isinstance(pose, Pose) for pose in self.poses):
            raise TypeError("All Path elements must be Pose instances")

    # =========================================================================
    # Factory Methods
    # =========================================================================

    @classmethod
    def from_list(cls, poses: list[Pose]) -> "Path":
        """Construct a Path from a list of Poses."""
        return cls(poses=tuple(poses))

    # =========================================================================
    # Properties
    # =========================================================================

    @property
    def start(self) -> Pose:
        """The first Pose in the path."""
        return self.poses[0]

    @property
    def goal(self) -> Pose:
        """The last Pose in the path."""
        return self.poses[-1]

    # =========================================================================
    # Geometry Methods
    # =========================================================================

    def length(self) -> float:
        """
        Total translational length of the path.

        Computed as the sum of Euclidean position distances between
        consecutive Poses. Orientation differences do not contribute
        to path length.

        Returns:
            Path length in meters.
        """
        total = 0.0
        for i in range(len(self.poses) - 1):
            total += self.poses[i].distance_to(self.poses[i + 1])
        return total

    # =========================================================================
    # Comparison Methods
    # =========================================================================

    def is_close(self, other: "Path",
                 pos_tol: float = 1e-6, ang_tol: float = 1e-6) -> bool:
        """
        Check if two Paths are approximately equal.

        Args:
            other:   Another Path instance.
            pos_tol: Tolerance for position (m).
            ang_tol: Tolerance for orientation (rad).

        Returns:
            True if both paths have the same number of waypoints
            and all corresponding Poses are close.
        """
        if not isinstance(other, Path):
            raise TypeError(f"Expected Path, got {type(other).__name__}")
        if len(self.poses) != len(other.poses):
            return False
        return all(
            p1.is_close(p2, pos_tol=pos_tol, ang_tol=ang_tol)
            for p1, p2 in zip(self.poses, other.poses)
        )

    # =========================================================================
    # Sequence Protocol
    # =========================================================================

    def __len__(self) -> int:
        """Number of waypoints in the path."""
        return len(self.poses)

    def __getitem__(self, index: int) -> Pose:
        """Access waypoint by index."""
        return self.poses[index]

    def __iter__(self):
        """Iterate over waypoints."""
        return iter(self.poses)

    # =========================================================================
    # Dunder Methods
    # =========================================================================

    def __str__(self) -> str:
        return f"Path({len(self.poses)} waypoints, start={self.start}, goal={self.goal})"

    def __repr__(self) -> str:
        return self.__str__()
