"""Check diagnostic isolation and phase geometry, not desired vehicle outcomes."""
from math import atan2
import pytest
from core import Path, Pose
from validation.startup_corner_diagnosis import fixtures, phase_metrics


def test_design_keeps_alignment_a_single_factor_and_corner_run_in_explicit():
    design = list(fixtures())
    assert len(design) == 20
    indexed = {item[0][0]: item for item in design}
    for speed in (.2, .3, .4, .5):
        original, turn, _ = indexed[f'detour_{speed}_original']
        aligned, aligned_turn, _ = indexed[f'detour_{speed}_aligned']
        assert original[1:3] == aligned[1:3]
        assert original[4:] == aligned[4:]
        assert original[3].twist == aligned[3].twist
        assert original[3].pose.x == aligned[3].pose.x
        assert original[3].pose.z == aligned[3].pose.z
        assert turn == aligned_turn
        corner, corner_time, _ = indexed[f'corner_{speed}']
        assert corner_time == pytest.approx(8/speed)
        assert corner[3].twist.u == speed
        assert not corner[2].obstacles
        a, b = corner[1][0], corner[1][1]
        assert corner[3].pose.pitch == pytest.approx(atan2(b.z-a.z, b.x-a.x))


def test_phase_window_includes_final_endpoint_and_reports_segment_clearance():
    states = [dict(timestamp=t, pose=dict(x=t, z=z, pitch=0)) for t, z in ((0, 0), (1, 2), (2, 0))]
    controls = [dict(time=t, segment_clearance=.4-t*.1,
        applied=dict(tau_m=4), controller_diagnostics=dict(pitch_error=.2,
        effective_limits=dict(maximum_tau_m=8))) for t in (0, 1)]
    history = dict(states=states, original_references=states, controls=controls)
    path = Path.from_list([Pose(), Pose(x=2)])
    metrics = phase_metrics(history, path, 0, 1, .3)
    assert metrics['transitions'] == 1
    assert metrics['max_path_distance'] == 2
    assert metrics['minimum_net_margin'] == pytest.approx(.1)
    assert metrics['torque_saturation_fraction'] == 0
    assert phase_metrics(history, path, 2, 3, .3) is None
