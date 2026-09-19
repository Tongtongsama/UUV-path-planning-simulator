"""Geometric guidance output; no mission-success or controller ownership."""
from dataclasses import dataclass

from core.pose import Pose
from .progress_projection import ProjectionResult, _number, _pose


@dataclass(frozen=True)
class GuidanceCommand:
    """Raw LOS reference and projection evidence, not a timed trajectory.

    pitch_rate_ref=0 is a baseline reference, NOT the derivative of raw LOS.
    terminal_ready only requests handoff eligibility, never mission success.
    The caller explicitly carries projection.next_state and pitch_ref forward.
    """

    pitch_ref: float
    pitch_rate_ref: float
    surge_speed_ref: float
    lookahead_point: Pose
    distance_to_goal: float
    terminal_ready: bool
    projection: ProjectionResult

    def __post_init__(self) -> None:
        _number(self.pitch_ref, "pitch_ref")
        _number(self.pitch_rate_ref, "pitch_rate_ref")
        _number(self.surge_speed_ref, "surge_speed_ref", minimum=0)
        _number(self.distance_to_goal, "distance_to_goal", minimum=0)
        _pose(self.lookahead_point)
        if not isinstance(self.terminal_ready, bool):
            raise TypeError("terminal_ready must be bool")
        if not isinstance(self.projection, ProjectionResult):
            raise TypeError("projection must be ProjectionResult")

    @property
    def progress_s(self) -> float:
        """Monotonic committed arc, in metres."""
        return self.projection.committed_s

    @property
    def projected_s(self) -> float:
        """Uncommitted diagnostic projection arc, in metres."""
        return self.projection.projected_s

    @property
    def segment_index(self) -> int:
        """Committed active segment (diagnostic segment is stored separately)."""
        return self.projection.segment_index

    @property
    def projection_point(self) -> Pose:
        """Finite diagnostic projection on the admitted local path."""
        return self.projection.projection_point

    @property
    def cross_track_error(self) -> float:
        """Signed residual along the projected segment's left/up normal."""
        return self.projection.cross_track_error

    @property
    def along_track_error(self) -> float:
        """Residual along the projected finite segment's tangent."""
        return self.projection.along_track_error

    @property
    def switch_count(self) -> int:
        """Number of committed transitions in this update."""
        return self.projection.switch_count
