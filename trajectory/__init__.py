"""Collision-validated piecewise-linear reference trajectory generation."""
from .generator import TrajectoryConfig, TrajectoryGenerationResult, generate_trajectory
from .sampling import sample_reference

__all__ = ["TrajectoryConfig", "TrajectoryGenerationResult", "generate_trajectory", "sample_reference"]
