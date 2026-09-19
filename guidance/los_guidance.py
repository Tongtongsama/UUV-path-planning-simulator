"""Fixed-lookahead, constant-speed LOS in the ENU vertical plane.

This Phase A function neither calls closed-loop modules nor owns their phases.
It intentionally has no dt input: geometric progress is displacement-bounded.
"""
from dataclasses import dataclass
from math import atan2

from core.pose import Pose
from .guidance_command import GuidanceCommand
from .progress_projection import (
    ProgressState, ProjectionConfig, _distance, _number, project_progress,
)


@dataclass(frozen=True)
class LOSConfig:
    """Explicit fixed geometry/speed settings, not fitted experiment defaults.

    Phase B must check terminal_entry_distance against mission goal tolerance.
    Phase A has no mission/safety configuration to modify or infer that value.
    """

    lookahead: float
    nominal_speed: float
    terminal_entry_distance: float

    def __post_init__(self) -> None:
        for name in ("lookahead", "nominal_speed", "terminal_entry_distance"):
            _number(getattr(self, name), name, positive=True)


def compute_los(position: Pose, state: ProgressState,
                projection_config: ProjectionConfig, config: LOSConfig,
                previous_pitch_ref: float | None = None) -> GuidanceCommand:
    """Advance explicit projection state and point toward a finite lookahead.

    Zero target displacement holds the caller's previous finite pitch, else the
    final tangent. A singleton has no tangent, so uses actual pitch as fallback.
    Singleton paths request terminal handling directly; this is not success.
    No speed reduction is performed, even when terminal_ready becomes true:
    an eventual phase owner must take over before applying a terminal command.
    """
    if not isinstance(config, LOSConfig):
        raise TypeError("config must be LOSConfig")
    if previous_pitch_ref is not None:
        _number(previous_pitch_ref, "previous_pitch_ref")
    projection = project_progress(position, state, projection_config)
    g = state.geometry
    target = g.point_at_s(min(projection.committed_s + config.lookahead, g.total_length))
    dx, dz = target.x - position.x, target.z - position.z
    if dx != 0 or dz != 0:
        _number(dx, "target dx")
        _number(dz, "target dz")
        pitch = atan2(dz, dx)
    elif previous_pitch_ref is not None:
        pitch = previous_pitch_ref
    elif g.lengths:
        a, b = g.points[-2:]
        pitch = atan2(b.z - a.z, b.x - a.x)
    else:
        pitch = position.pitch
    distance = _distance(position, g.points[-1])
    ready = not g.lengths or (
        projection.segment_index == len(g.lengths) - 1
        and g.total_length - projection.committed_s <= config.terminal_entry_distance
        and distance <= config.terminal_entry_distance
    )
    return GuidanceCommand(pitch, 0.0, config.nominal_speed, target,
                           distance, ready, projection)
