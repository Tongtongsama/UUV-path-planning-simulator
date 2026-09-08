"""Scalar and vertical-plane validation shared by Planner contracts."""
from math import isfinite
from numbers import Real
from core import Pose


def scalar(value: float, name: str, nonnegative: bool = True) -> float:
    """Require a finite real scalar, optionally nonnegative."""
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a real scalar")
    value = float(value)
    if not isfinite(value) or (nonnegative and value < 0):
        raise ValueError(f"{name} must be finite and in range")
    return value


def check_pose(pose: Pose) -> None:
    """Reject nonfinite values and inactive y/roll/yaw components."""
    if not isinstance(pose, Pose):
        raise TypeError("pose must be Pose")
    for name in ("x", "y", "z", "roll", "pitch", "yaw"):
        scalar(getattr(pose, name), name, False)
    if any(abs(getattr(pose, name)) > 1e-12 for name in ("y", "roll", "yaw")):
        raise ValueError("pose must lie in the x-z vertical plane")
