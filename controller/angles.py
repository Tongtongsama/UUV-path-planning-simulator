"""Local angle-error utilities for Control v0.5."""

from __future__ import annotations

import math
from numbers import Real

from controller._validation import finite_scalar


def wrap_to_pi(angle: Real) -> float:
    """Wrap a finite angle to the half-open interval ``[-pi, pi)``."""
    value = finite_scalar(angle, name="angle")
    return (value + math.pi) % (2.0 * math.pi) - math.pi
