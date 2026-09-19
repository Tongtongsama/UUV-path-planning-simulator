"""Pure finite-segment guidance geometry, independent of closed-loop modules."""

from .progress_projection import (
    PathGeometry, ProgressState, ProjectionConfig, ProjectionResult,
    SegmentProjection, project_progress, project_segment,
)
from .guidance_command import GuidanceCommand
from .los_guidance import LOSConfig, compute_los

__all__ = [
    "PathGeometry", "ProgressState", "ProjectionConfig", "ProjectionResult",
    "SegmentProjection", "project_progress", "project_segment",
    "GuidanceCommand", "LOSConfig", "compute_los",
]
