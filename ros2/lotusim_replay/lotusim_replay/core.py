"""Transport-neutral mapping of frozen vertical-plane histories to LOTUSim.

This module deliberately has no ROS, controller, guidance, planner, navigation,
or physics dependency.  Its input is the already verified replay payload.
"""

from dataclasses import dataclass
from math import cos, isfinite, sin
from pathlib import Path
from typing import Iterable, Sequence


SUPPORTED_ARMS = ("P1", "APPROACH")


@dataclass(frozen=True)
class ReplayPose:
    """A LOTUSim world pose (ENU position, FLU orientation)."""

    source_time: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def lotusim_pose(sample: Sequence[float], lateral_offset: float = 0.0) -> ReplayPose:
    """Map ``[time,x,z,pitch,...]`` to LOTUSim's ENU/FLU pose contract.

    The frozen model uses body forward/right/down and positive nose-up pitch.
    LOTUSim's message documents body front/left/up, so the equivalent world
    orientation is R_y(-pitch).  Position is already expressed in ENU x/z.
    """
    if len(sample) < 4:
        raise ValueError("Replay sample must contain time, x, z and pitch")
    time_s, x, z, pitch = (float(sample[i]) for i in range(4))
    values = (time_s, x, z, pitch, float(lateral_offset))
    if not all(isfinite(value) for value in values):
        raise ValueError("Replay pose contains a non-finite value")
    half = -0.5 * pitch
    return ReplayPose(time_s, x, float(lateral_offset), z, 0.0, sin(half), 0.0, cos(half))


def interpolate(samples: Sequence[Sequence[float]], source_time: float) -> Sequence[float]:
    """Linearly interpolate display pose without creating evidence samples."""
    if not samples:
        raise ValueError("Replay has no samples")
    if source_time <= samples[0][0]:
        return samples[0]
    if source_time >= samples[-1][0]:
        return samples[-1]

    lo, hi = 0, len(samples) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if samples[mid][0] <= source_time:
            lo = mid
        else:
            hi = mid
    before, after = samples[lo], samples[hi]
    alpha = (source_time - before[0]) / (after[0] - before[0])
    # Only pose fields are consumed by LOTUSim, but retaining the full row
    # makes this helper useful to transport-neutral tests and future adapters.
    return [a + alpha * (b - a) for a, b in zip(before, after)]


def select_scene(payload: dict, scene_name: str) -> dict:
    matches = [scene for scene in payload.get("scenes", ()) if scene.get("name") == scene_name]
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one replay scene named {scene_name!r}")
    return matches[0]


def select_arms(scene: dict, requested: Iterable[str]) -> dict:
    names = tuple(name.upper() for name in requested)
    if not names:
        raise ValueError("At least one replay arm is required")
    if len(set(names)) != len(names):
        raise ValueError("Replay arms must be unique")
    unknown = set(names).difference(SUPPORTED_ARMS)
    if unknown:
        raise ValueError(f"Unsupported replay arm(s): {', '.join(sorted(unknown))}")
    arms = scene.get("arms", {})
    return {name: arms[name] for name in names}


def load_verified_payload(repository_root: Path) -> dict:
    """Use the existing frozen-evidence verifier without rerunning simulation."""
    import sys

    root = Path(repository_root).resolve()
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    try:
        from visualization.replay_export import build_payload
    except ImportError as error:
        raise ValueError(f"Not a UUV simulator repository: {root}") from error
    return build_payload(root)

