"""Analytic finite-segment ENU sign and clipping checks."""
import math

import pytest

from core import Pose
from guidance import project_segment


@pytest.mark.parametrize("end", [Pose(x=2), Pose(x=2, z=2), Pose(x=2, z=-2)])
@pytest.mark.parametrize("lam", [-.5, .25, 1.5])
def test_finite_projection_and_unclipped_parameter(end, lam):
    length = math.hypot(end.x, end.z)
    tx, tz = end.x / length, end.z / length
    point = Pose(x=lam * end.x - .3 * tz, z=lam * end.z + .3 * tx)
    result = project_segment(Pose(), end, point)
    mu = min(1, max(0, lam))
    assert result.lambda_raw == pytest.approx(lam)
    assert result.mu == pytest.approx(mu)
    assert result.projection_point.x == pytest.approx(mu * end.x)
    assert result.projection_point.z == pytest.approx(mu * end.z)
    assert result.cross_track_error == pytest.approx(.3)
    assert result.along_track_error == pytest.approx((lam - mu) * length)


@pytest.mark.parametrize("z", [-2, 0, 2])
def test_horizontal_cross_track_sign(z):
    assert project_segment(Pose(), Pose(x=3), Pose(x=1, z=z)).cross_track_error == z


def test_zero_length_rejected():
    with pytest.raises(ValueError, match="degenerate"):
        project_segment(Pose(), Pose(), Pose())


@pytest.mark.parametrize("point", [Pose(x=float("nan")), Pose(z=float("inf")), Pose(y=1)])
def test_invalid_segment_input(point):
    with pytest.raises(ValueError):
        project_segment(Pose(), Pose(x=1), point)
