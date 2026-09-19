"""Fixed LOS signs, corner behavior, explicit state and terminal eligibility."""
import ast
import math
from pathlib import Path as FilePath

import pytest

from core import Path, Pose
from guidance import LOSConfig, PathGeometry, ProgressState, ProjectionConfig, compute_los


PROJECTION = ProjectionConfig(.2, 10, .1, 3)
LOS = LOSConfig(.5, .3, .2)


def geometry(*points):
    return PathGeometry(Path(tuple(Pose(x=x, z=z) for x, z in points)))


@pytest.mark.parametrize("end,expected", [((2, 0), 0), ((2, 2), math.pi / 4),
                                         ((2, -2), -math.pi / 4)])
def test_pitch_enu_sign(end, expected):
    g = geometry((0, 0), end)
    command = compute_los(Pose(), ProgressState(g, Pose()), PROJECTION, LOS)
    assert command.pitch_ref == pytest.approx(expected)
    assert command.pitch_rate_ref == 0
    assert command.surge_speed_ref == .3
    assert not command.terminal_ready


@pytest.mark.parametrize("z", [-.5, .5])
def test_cross_track_correction_sign(z):
    g = geometry((0, 0), (3, 0))
    point = Pose(x=1, z=z)
    command = compute_los(point, ProgressState(g, point, committed_s=1), PROJECTION, LOS)
    assert command.cross_track_error == z
    assert command.pitch_ref == pytest.approx(math.atan2(-z, .5))


def test_lookahead_same_segment_and_across_corner():
    g = geometry((0, 0), (1, 0), (1, 1))
    for x, expected in [(0, Pose(x=.5)), (.75, Pose(x=1, z=.25))]:
        point = Pose(x=x)
        c = compute_los(point, ProgressState(g, point, committed_s=x), PROJECTION, LOS)
        assert c.lookahead_point == expected
    assert c.pitch_ref == pytest.approx(math.pi / 4)


def test_endpoint_clipping_and_terminal_ready_are_not_success():
    g = geometry((0, 0), (1, 0))
    p = Pose(x=.9)
    c = compute_los(p, ProgressState(g, p, committed_s=.9), PROJECTION, LOS)
    assert c.lookahead_point == Pose(x=1)
    assert c.terminal_ready
    assert not hasattr(c, "success")
    assert c.surge_speed_ref == LOS.nominal_speed


@pytest.mark.parametrize("previous,expected", [(None, math.pi / 4), (.3, .3)])
def test_zero_target_displacement_previous_command_or_final_tangent(previous, expected):
    g = geometry((0, 0), (1, 1))
    point = Pose(x=1, z=1)
    c = compute_los(point, ProgressState(g, point, committed_s=g.total_length),
                    PROJECTION, LOS, previous)
    assert c.pitch_ref == pytest.approx(expected)
    assert c.terminal_ready


def test_interior_zero_displacement_is_defined_even_when_progress_lags():
    g = geometry((0, 0), (1, 0), (1, 1))
    point = Pose(x=.5)
    c = compute_los(point, ProgressState(g, point), ProjectionConfig(.2, 10, 0, 3), LOS)
    assert c.lookahead_point == point
    assert c.pitch_ref == pytest.approx(math.pi / 2)
    assert not c.terminal_ready


@pytest.mark.parametrize("position", [Pose(pitch=.7), Pose(x=1, z=1)])
def test_singleton_is_finite_terminal_only(position):
    g = geometry((0, 0))
    c = compute_los(position, ProgressState(g, position), PROJECTION, LOS)
    assert c.terminal_ready
    assert math.isfinite(c.pitch_ref)
    if position.x == position.z == 0:
        assert c.pitch_ref == .7


def test_terminal_requires_final_segment_remaining_arc_and_actual_distance():
    # Goal spatially close to start must not shortcut the remaining U-shaped path.
    g = geometry((0, 0), (2, 0), (2, .1), (0, .1))
    c = compute_los(Pose(), ProgressState(g, Pose()), PROJECTION, LOS)
    assert c.distance_to_goal < LOS.terminal_entry_distance
    assert not c.terminal_ready
    g = geometry((0, 0), (1, 0))
    point = Pose(x=1, z=1)
    c = compute_los(point, ProgressState(g, point, committed_s=1), PROJECTION, LOS)
    assert not c.terminal_ready
    point = Pose(x=1)
    c = compute_los(point, ProgressState(g, point), ProjectionConfig(.1, 10, 0, 2), LOS)
    assert not c.terminal_ready


def test_raw_los_allows_unfiltered_corner_change_with_zero_q_reference():
    g = geometry((0, 0), (1, 0), (1, 1))
    p = Pose(x=.9)
    config = LOSConfig(.05, .3, .1)
    first = compute_los(p, ProgressState(g, p, committed_s=.9), PROJECTION, config)
    second = compute_los(Pose(x=1, z=.1), first.projection.next_state,
                         PROJECTION, config, first.pitch_ref)
    assert first.pitch_ref == 0
    assert second.pitch_ref == pytest.approx(math.pi / 2)
    assert first.pitch_rate_ref == second.pitch_rate_ref == 0


def test_explicit_state_is_deterministic_and_commands_finite():
    g = geometry((0, 0), (1, 1), (2, 0), (3, 1))
    state = ProgressState(g, Pose())
    previous = None
    for j in range(51):
        point = g.point_at_s(g.total_length * j / 50)
        c = compute_los(point, state, PROJECTION, LOS, previous)
        assert c == compute_los(point, state, PROJECTION, LOS, previous)
        assert c.progress_s >= state.committed_s
        assert all(math.isfinite(v) for v in (c.pitch_ref, c.pitch_rate_ref, c.surge_speed_ref,
            c.progress_s, c.projected_s, c.cross_track_error, c.along_track_error, c.distance_to_goal))
        state, previous = c.projection.next_state, c.pitch_ref


@pytest.mark.parametrize("kwargs", [dict(lookahead=0), dict(nominal_speed=-1),
    dict(terminal_entry_distance=float("nan")), dict(lookahead=float("inf"))])
def test_invalid_los_config(kwargs):
    values = dict(lookahead=.5, nominal_speed=.3, terminal_entry_distance=.2)
    values.update(kwargs)
    with pytest.raises(ValueError):
        LOSConfig(**values)


def test_invalid_los_api_types_and_previous_command():
    g = geometry((0, 0), (1, 0))
    state = ProgressState(g, Pose())
    with pytest.raises(TypeError):
        LOSConfig(True, .3, .2)
    with pytest.raises(TypeError):
        compute_los(Pose(), state, PROJECTION, object())
    with pytest.raises(ValueError):
        compute_los(Pose(), state, PROJECTION, LOS, float("nan"))


def test_phase_a_imports_only_core_standard_library_and_guidance():
    root = FilePath(__file__).resolve().parents[2] / "guidance"
    allowed = {"core", "dataclasses", "math", "numbers", "bisect"}
    for source in root.glob("*.py"):
        for node in ast.walk(ast.parse(source.read_text(encoding="utf-8"))):
            if isinstance(node, ast.Import):
                assert all(alias.name.split(".")[0] in allowed for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and not node.level:
                assert node.module.split(".")[0] in allowed
