import copy
import math
import pytest
from controller.guidance import BodyGuidance3DOF
from ros2.twin_axial_pid.control import AllocatedPID
from ros2.twin_axial_pid.protocol import target,duration
from ros2.twin_axial_pid.acceptance import step_metrics,analyze_run
from ros2.twin_axial_v2.numerical import initial_state
from ros2.twin_axial_v2.protocol import feedback


def record():
    s=initial_state()
    return {'valid':True,'sequence':0,'solver_state_ned_frd':s,**feedback(s)}


def test_small_signal_uses_baseline_pid_and_no_aw_correction():
    c=AllocatedPID()
    raw,d=c.command(record(),BodyGuidance3DOF(.1,.01))
    assert raw['X']==pytest.approx(25*.1+3*.1*.05)
    assert raw['M']==pytest.approx(20*.01+1*.01*.05)
    assert d['allocation']['alpha']==1
    assert all(abs(v)<1e-15 for v in d['backcalculation'].values())
    assert d['integral_after']==pytest.approx(d['integral_candidate'],abs=1e-14)


def test_joint_saturation_corrects_both_channels_from_allocated_wrench():
    c=AllocatedPID()
    raw,d=c.command(record(),BodyGuidance3DOF(.6,math.pi/3))
    a=d['allocation']; assert a['alpha']<1
    assert abs(a['allocated_wrench']['X'])+4*abs(a['allocated_wrench']['M'])==pytest.approx(30)
    assert d['backcalculation']['surge_error_integral']==pytest.approx(.05*(a['allocated_wrench']['X']-raw['X'])/3)
    assert d['backcalculation']['pitch_error_integral']==pytest.approx(.05*(a['allocated_wrench']['M']-raw['M']))
    assert all(v<0 for v in d['backcalculation'].values())


def test_persistent_saturation_memory_bounded_and_reset():
    c=AllocatedPID()
    for i in range(300):
        c.command(record(),BodyGuidance3DOF(.6,math.pi/3))
        assert abs(c.memory.surge_error_integral)<=4
        assert abs(c.memory.pitch_error_integral)<=1
    c.reset()
    assert c.memory.surge_error_integral==c.memory.pitch_error_integral==0


def test_actual_state_changes_next_control():
    r=record(); a=AllocatedPID().command(r,BodyGuidance3DOF(.4,.2))[0]
    r['solver_state_ned_frd']['u']=.3
    r['solver_state_ned_frd']['q']=.1
    b=AllocatedPID().command(r,BodyGuidance3DOF(.4,.2))[0]
    assert b['X']<a['X'] and b['M']<a['M']


def test_invalid_feedback_cannot_keep_integral_state():
    c=AllocatedPID(); c.command(record(),BodyGuidance3DOF(.4,0))
    r=record();r['valid']=False
    with pytest.raises(ValueError): c.command(r,BodyGuidance3DOF(.4,0))
    assert c.memory.surge_error_integral==0


def test_positive_negative_pitch_are_symmetric():
    positive=AllocatedPID().command(record(),BodyGuidance3DOF(.4,.35))[0]
    negative=AllocatedPID().command(record(),BodyGuidance3DOF(.4,-.35))[0]
    assert positive['X']==negative['X']
    assert positive['M']==-negative['M']


def test_protocol_keeps_fixed_thirty_second_steps():
    for case in ('speed_step','pitch_positive','pitch_negative'):
        assert target(case,1.99).u==0
        assert target(case,2).u==.4
        assert duration(case)-2==30
    assert target('pitch_positive',2).pitch==pytest.approx(math.radians(20))
    assert target('pitch_negative',2).pitch==pytest.approx(math.radians(-20))
    assert target('allocation_recovery',10).pitch==0


def test_settling_requires_staying_in_band_not_first_crossing():
    m=step_metrics([2,3,4,5,6],[0,.4,.6,.4,.4],.4,2,.04)
    assert m['settling_s']==3
    assert m['overshoot']==pytest.approx(.2)
    m=step_metrics([2,3,4],[0,.4,.5],.4,2,.04)
    assert m['settling_s'] is None


def test_missing_records_fail_closed(tmp_path):
    assert not analyze_run(tmp_path)['passed']


def test_recompute_comparison_allows_roundoff_not_control_changes():
    from ros2.twin_axial_pid.acceptance import numerically_equal
    assert numerically_equal({'X':1.0},{'X':1.0+4e-16})
    assert not numerically_equal({'X':1.0},{'X':1.0+1e-8})
    assert not numerically_equal({'X':float('nan')},{'X':float('nan')})
