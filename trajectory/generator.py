"""Deterministic geometric post-processing and constant-speed timing."""
from dataclasses import dataclass
from math import atan2, ceil, hypot, isfinite
from numbers import Integral, Real

from core import Path, Pose, Trajectory, Twist, VehicleState
from planner import PlanningConstraints, PlanningRequest, PlanningSpace, validate_path


def finite(value: float, name: str) -> float:
    """Require a finite non-boolean real scalar."""
    if isinstance(value,bool) or not isinstance(value,Real):
        raise TypeError(f"{name} must be real")
    value=float(value)
    if not isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


@dataclass(frozen=True)
class TrajectoryConfig:
    """Speed in m/s, maximum sample spacing in m, bounded output size."""
    nominal_speed: float = 0.5
    sample_spacing: float = 0.25
    shortcut: bool = True
    max_samples: int = 100000

    def __post_init__(self) -> None:
        for name in ("nominal_speed","sample_spacing"):
            value=finite(getattr(self,name),name)
            if value <= 0:
                raise ValueError(f"{name} must be positive")
            object.__setattr__(self,name,value)
        if not isinstance(self.shortcut,bool):
            raise TypeError("shortcut must be bool")
        if isinstance(self.max_samples,bool) or not isinstance(self.max_samples,Integral):
            raise TypeError("max_samples must be integer")
        if self.max_samples < 1:
            raise ValueError("max_samples must be positive")


@dataclass(frozen=True)
class TrajectoryGenerationResult:
    """Immutable intermediate paths retained for audit and comparison figures."""
    raw_path: Path
    deduplicated_path: Path
    shortcut_path: Path
    sampled_path: Path
    trajectory: Trajectory


def generate_trajectory(path: Path, space: PlanningSpace, constraints: PlanningConstraints,
                        config: TrajectoryConfig = TrajectoryConfig(), *,
                        start_time: float = 0.0,
                        request: PlanningRequest | None = None) -> TrajectoryGenerationResult:
    """Validate each stage and return a time-stamped x-z polyline reference.

    Removes exact consecutive position duplicates, greedily shortcuts to the
    farthest visible subsequent point, and subdivides each segment without
    skipping original corners. Invalid geometry raises ValueError. This is
    not a dynamically feasible smoothing or actuator-constrained solution.
    """
    if not isinstance(config,TrajectoryConfig):
        raise TypeError("config must be TrajectoryConfig")
    start_time=finite(start_time,"start_time")

    def checked(candidate: Path) -> Path:
        check=validate_path(candidate,space,constraints,request=request)
        if not check.valid:
            raise ValueError(f"invalid trajectory geometry: {check.message}")
        return candidate

    checked(path)
    poses=[path[0]]
    for pose in path.poses[1:]:
        if (pose.x,pose.z) != (poses[-1].x,poses[-1].z):
            poses.append(pose)
    deduplicated=checked(Path.from_list(poses))
    reduced=[poses[0]]
    i=0
    while i < len(poses)-1:
        j=len(poses)-1 if config.shortcut else i+1
        while j > i+1 and not space.is_segment_valid(poses[i],poses[j],constraints.required_clearance):
            j-=1
        reduced.append(poses[j])
        i=j
    shortcut=checked(Path.from_list(reduced))
    counts=[]
    for a,b in zip(reduced,reduced[1:]):
        length=hypot(b.x-a.x,b.z-a.z)
        ratio=length/config.sample_spacing
        if not isfinite(ratio) or ratio > config.max_samples:
            raise ValueError("trajectory exceeds max_samples")
        counts.append(max(1,ceil(ratio)))
    if 1+sum(counts) > config.max_samples:
        raise ValueError("trajectory exceeds max_samples")
    samples=[reduced[0]]
    for a,b,count in zip(reduced,reduced[1:],counts):
        for k in range(1,count+1):
            # Preserve each original corner and endpoint exactly.
            samples.append(b if k==count else Pose(x=a.x+(b.x-a.x)*k/count,z=a.z+(b.z-a.z)*k/count))
    sampled=checked(Path.from_list(samples))
    states=[]
    timestamp=start_time
    last_pitch=samples[0].pitch
    for i,pose in enumerate(samples):
        if i < len(samples)-1:
            nxt=samples[i+1]
            length=hypot(nxt.x-pose.x,nxt.z-pose.z)
            if length == 0:
                raise ValueError("sample spacing cannot be represented at these coordinates")
            pitch=atan2(nxt.z-pose.z,nxt.x-pose.x)
            twist=Twist(u=config.nominal_speed)
        else:
            pitch=last_pitch
            twist=Twist.zero()
        states.append(VehicleState(Pose(x=pose.x,z=pose.z,pitch=pitch),twist,timestamp))
        last_pitch=pitch
        if i < len(samples)-1:
            next_time=timestamp+length/config.nominal_speed
            if not isfinite(next_time) or next_time <= timestamp:
                raise ValueError("trajectory timestamps cannot represent positive segment duration")
            timestamp=next_time
    trajectory=Trajectory.from_list(states)
    checked(Path.from_list([s.pose for s in trajectory]))
    return TrajectoryGenerationResult(path,deduplicated,shortcut,sampled,trajectory)
