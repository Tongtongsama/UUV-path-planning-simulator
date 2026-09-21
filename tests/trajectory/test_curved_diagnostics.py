"""Read-only diagnostics reconstruct the original recorded controller output."""
import pytest
from validation.planner_curved_report import moment_terms, progress_plateaus


def test_moment_sum_and_mismatch():
    d=dict(pitch_error=.2,pitch_pd=1.,integral_after={'pitch_error_integral':.4},
           restoring_feedforward=.3,requested={'tau_m':1.5},output={'tau_m':1.})
    c=dict(controller_diagnostics=d,applied={'tau_m':1.})
    actual=moment_terms(c,dict(pitch_kp=2.,pitch_ki=.5))
    assert actual['P']==pytest.approx(.4)
    assert actual['D']==pytest.approx(.6)
    assert actual['I']==pytest.approx(.2)
    d['requested']['tau_m']=2.
    with pytest.raises(ValueError):moment_terms(c,dict(pitch_kp=2.,pitch_ki=.5))


def test_fine_segments_reduce_gate_even_when_global_radius_is_large():
    route=[dict(x=0,z=0),dict(x=.1,z=0),dict(x=.2,z=0)]
    history=dict(controls=[dict(time=i*.1,path_progress_s=.1,
        guidance_command={'projection':{'segment_index':0}}) for i in range(12)],
        states=[{'pose':dict(x=.12,z=.06)} for i in range(13)])
    result=progress_plateaus(history,route,.2)
    assert len(result)==1
    assert result[0]['gate_radius']==pytest.approx(.045)
    assert result[0]['end_cross_track']==pytest.approx(.06)
