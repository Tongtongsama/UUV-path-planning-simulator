"""Path normalization, immutable state and invalid API inputs."""
from dataclasses import FrozenInstanceError

import pytest

from core import Path, Pose
from guidance import PathGeometry, ProgressState, ProjectionConfig, project_progress


def test_duplicates_preserve_first_original_index_and_nonconsecutive_return():
    geometry = PathGeometry(Path((Pose(), Pose(pitch=1), Pose(x=2), Pose(x=2), Pose())))
    assert geometry.original_indices == (0, 2, 4)
    assert geometry.lengths == (2, 2)
    assert geometry.cumulative_s == (0, 2, 4)
    assert geometry.point_at_s(1) == Pose(x=1)
    assert geometry.point_at_s(3) == Pose(x=1)
    assert geometry.point_at_s(4) == Pose()
    with pytest.raises(FrozenInstanceError):
        geometry.points = ()


def test_singleton_and_all_duplicate_path():
    geometry = PathGeometry(Path((Pose(x=3), Pose(x=3))))
    state = ProgressState(geometry, Pose())
    result = project_progress(Pose(x=1), state, ProjectionConfig(.1, 1, .1, 2))
    assert geometry.total_length == 0
    assert geometry.point_at_s(0) == Pose(x=3)
    assert result.projection_point == Pose(x=3)
    assert result.committed_s == result.projected_s == result.cross_track_error == 0
    assert result.switch_count == 0


@pytest.mark.parametrize("point", [Pose(x=float("nan")), Pose(z=float("inf")),
                                   Pose(y=.01), Pose(roll=.01), Pose(yaw=.01)])
def test_reject_invalid_path_pose(point):
    with pytest.raises(ValueError):
        PathGeometry(Path((point,)))


def test_tiny_nonzero_segment_rejected_not_silently_removed():
    with pytest.raises(ValueError, match="epsilon"):
        PathGeometry(Path((Pose(), Pose(x=1e-12))))


def test_empty_path_rejected_by_core():
    with pytest.raises(ValueError):
        PathGeometry(Path(()))


@pytest.mark.parametrize("s", [-1, 3, float("inf"), float("nan")])
def test_invalid_arc(s):
    with pytest.raises(ValueError):
        PathGeometry(Path((Pose(), Pose(x=2)))).point_at_s(s)


@pytest.mark.parametrize("kwargs", [dict(switch_radius=0), dict(forward_window=-1),
    dict(progress_slack=-1), dict(max_switches_per_step=0), dict(progress_slack=float("inf"))])
def test_config_value_validation(kwargs):
    values = dict(switch_radius=.1, forward_window=1, progress_slack=0, max_switches_per_step=2)
    values.update(kwargs)
    with pytest.raises(ValueError):
        ProjectionConfig(**values)


@pytest.mark.parametrize("kwargs", [dict(switch_radius=True), dict(forward_window="1"),
                                   dict(max_switches_per_step=True), dict(max_switches_per_step=1.5)])
def test_config_type_validation(kwargs):
    values = dict(switch_radius=.1, forward_window=1, progress_slack=0, max_switches_per_step=2)
    values.update(kwargs)
    with pytest.raises(TypeError):
        ProjectionConfig(**values)


def test_api_types_and_inconsistent_state():
    g = PathGeometry(Path((Pose(), Pose(x=1), Pose(x=2))))
    config = ProjectionConfig(.1, 1, 0, 2)
    with pytest.raises(TypeError):
        PathGeometry([Pose()])
    with pytest.raises(TypeError):
        project_progress(Pose(), object(), config)
    with pytest.raises(TypeError):
        project_progress(Pose(), ProgressState(g, Pose()), object())
    with pytest.raises(TypeError):
        ProgressState(g, Pose(), segment_index=True)
    with pytest.raises(ValueError):
        ProgressState(g, Pose(), segment_index=1, committed_s=.5)
    with pytest.raises(ValueError):
        ProgressState(g, Pose(), committed_s=1.5)


def test_overflow_rejected():
    with pytest.raises(ValueError):
        PathGeometry(Path((Pose(x=-1e308), Pose(x=1e308))))
