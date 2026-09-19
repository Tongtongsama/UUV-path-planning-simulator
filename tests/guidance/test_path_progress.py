"""Sequential gate, topology, bounded advance and backward-motion regression."""
import math

import pytest

from core import Path, Pose
from guidance import PathGeometry, ProgressState, ProjectionConfig, project_progress


def geometry(*points):
    return PathGeometry(Path(tuple(Pose(x=x, z=z) for x, z in points)))


CONFIG = ProjectionConfig(.2, 10, .1, 3)


@pytest.mark.parametrize("end", [(2, 1), (1, 2), (0, 1)])
def test_corner_sequential_gate_and_atomic_transition(end):
    g = geometry((0, 0), (1, 0), end)
    result = project_progress(Pose(x=1), ProgressState(g, Pose()), CONFIG)
    assert result.segment_index == 1
    assert result.committed_s == 1
    assert result.switch_count == result.next_state.total_switches == 1


def test_backward_motion_changes_diagnostics_but_not_committed_state():
    g = geometry((0, 0), (1, 0), (2, 0))
    state = ProgressState(g, Pose(x=1.5), 1, 1.5, 1)
    result = project_progress(Pose(x=.5), state, CONFIG)
    assert result.projected_segment_index == 0
    assert result.projected_s == .5
    assert result.committed_s == 1.5
    assert result.segment_index == 1
    assert result.switch_count == 0


def test_endpoint_infinite_plane_does_not_admit_far_lateral_position():
    g = geometry((0, 0), (1, 0), (1, 5))
    r = project_progress(Pose(x=1.1, z=4), ProgressState(g, Pose()), CONFIG)
    assert r.lambda_raw > 1
    assert r.segment_index == r.projected_segment_index == 0
    assert r.committed_s == 1


@pytest.mark.parametrize("points,position", [
    ([(0, 0), (3, 0), (3, .1), (0, .1)], Pose(x=.5, z=.1)),
    ([(0, 0), (3, 0), (3, 1), (0, 1), (0, .1), (3, .1)], Pose(x=1, z=.1)),
    ([(0, 0), (2, 2), (0, 2), (2, 0)], Pose(x=1, z=.8)),
])
def test_nearby_nonadjacent_branch_cannot_steal_projection(points, position):
    g = geometry(*points)
    r = project_progress(position, ProgressState(g, Pose()), CONFIG)
    assert r.segment_index == r.projected_segment_index == 0


def test_multi_short_segments_switch_cap_and_subsequent_continuation():
    g = geometry(*[(i * .1, 0) for i in range(10)])
    cfg = ProjectionConfig(.1, 2, 1, 2)
    r = project_progress(Pose(x=.8), ProgressState(g, Pose()), cfg)
    assert r.switch_count == r.segment_index == 2
    assert r.switch_limit_reached
    assert r.committed_s == pytest.approx(.3)
    r2 = project_progress(Pose(x=.8), r.next_state, cfg)
    assert r2.segment_index == 4
    assert r2.switch_count == 2
    assert r2.committed_s >= r.committed_s


def test_clamp_prevents_premature_segment_commit():
    g = geometry((0, 0), (1, 0), (2, 0))
    initial = Pose(x=1.5)
    r = project_progress(initial, ProgressState(g, initial), ProjectionConfig(.2, 10, .1, 3))
    assert r.projected_s == 1.5
    assert r.committed_s == .1
    assert r.progress_clamped
    assert r.segment_index == 0
    assert r.switch_count == 0


def test_displacement_plus_slack_and_forward_window_caps():
    g = geometry((0, 0), (10, 0))
    r = project_progress(Pose(x=5), ProgressState(g, Pose(x=4)), ProjectionConfig(.2, 10, .1, 3))
    assert r.committed_s == pytest.approx(1.1)
    assert r.progress_clamped
    r = project_progress(Pose(x=5), ProgressState(g, Pose()), ProjectionConfig(.2, .5, 0, 3))
    assert r.projected_s == 5
    assert r.committed_s == .5
    assert r.progress_clamped


def test_distance_tie_uses_lower_arc():
    g = geometry((0, 0), (1, 0), (1, 1))
    state = ProgressState(g, Pose(x=1), 1, 1, 1)
    # Use an exactly representable symmetric point to avoid a floating near-tie.
    r = project_progress(Pose(x=.875, z=.125), state, CONFIG)
    assert r.projected_segment_index == 0
    assert r.projected_s == .875
    assert r.committed_s == 1


@pytest.mark.parametrize("points", [
    [(0, 0), (2, 0), (2, 1), (0, 1)],
    [(0, 0), (1, 1), (2, 0), (3, 1), (4, 0)],
    [(0, 0), (2, 2), (0, 2), (2, 0)],
])
def test_topological_walk_monotonic_progress_and_no_remote_switch(points):
    g = geometry(*points)
    state = ProgressState(g, g.points[0])
    for i, length in enumerate(g.lengths):
        for j in range(1, 21):
            point = g.point_at_s(g.cumulative_s[i] + length * j / 20)
            r = project_progress(point, state, CONFIG)
            assert r.committed_s >= state.committed_s
            assert state.segment_index <= r.segment_index <= i + 1
            assert r.committed_s - state.committed_s <= math.hypot(
                point.x - state.previous_position.x, point.z - state.previous_position.z) + CONFIG.progress_slack + 1e-12
            state = r.next_state
    assert state.committed_s == pytest.approx(g.total_length)
    assert state.segment_index == len(g.lengths) - 1


def test_reset_and_interleaved_missions_have_no_hidden_state():
    g = geometry((0, 0), (2, 0))
    initial = ProgressState(g, Pose())
    first = project_progress(Pose(x=1), initial, CONFIG)
    project_progress(Pose(x=2), first.next_state, CONFIG)
    assert project_progress(Pose(x=1), initial, CONFIG) == first
