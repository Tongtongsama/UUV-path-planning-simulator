import copy
import math
import pytest
from ros2.twin_axial_motion.protocol import guidance,desired_depth,duration,CASES,OUTER
from ros2.twin_axial_motion.safety import FeedbackGuard
from ros2.twin_axial_motion.acceptance import analyze_run
from ros2.twin_axial_v2.numerical import initial_state
from ros2.twin_axial_v2.protocol import feedback


def record(seq=0,t=.05):
    s=initial_state();s['t']=t
    return {'valid':True,'sequence':seq,'elapsed_end':t,'solver_state_ned_frd':s,**feedback(s)}


def test_depth_targets_and_pitch_sign():
    r=record(t=8)
    assert desired_depth('depth_up',7.95)==30
    assert desired_depth('depth_up',8)==28
    assert desired_depth('depth_down',8)==32
    up,du=guidance('depth_up',r); down,dd=guidance('depth_down',r)
    assert up.u==down.u==.4
    assert up.pitch==-down.pitch==pytest.approx(OUTER['maximum_pitch_rad'])
    assert du['depth_error_up_m']==2 and dd['depth_error_up_m']==-2


def test_depth_outer_uses_real_depth_and_world_vertical_speed():
    r=record(t=9);r['solver_state_ned_frd'].update(z=28,u=.4,w=-.2)
    desired,d=guidance('depth_up',r)
    assert d['vertical_up_mps']==pytest.approx(.2)
    assert desired.pitch<0  # brake rising motion at the target depth
    r['solver_state_ned_frd']['w']=0
    assert guidance('depth_up',r)[0].pitch==0


def test_straight_does_not_use_position_or_path_reference():
    r=record(t=10);a=guidance('straight',r)
    r['solver_state_ned_frd']['y']=100
    assert guidance('straight',r)==a
    assert a[0].pitch==0


def test_guard_fresh_states_and_wall_expiry_even_without_sim_advance():
    g=FeedbackGuard();g.receive(record(),10)
    g.receive(record(1,.1),10.05)
    assert g.check_timeout(10.30) is None
    assert g.check_timeout(10.301)=='state_timeout'
    g.receive(record(2,.15),10.31)
    assert g.latest['sequence']==1  # latched, no automatic restart


@pytest.mark.parametrize('fault',['same','backwards','gap','time_jump','invalid'])
def test_guard_rejects_stale_or_corrupt_feedback(fault):
    g=FeedbackGuard();g.receive(record(),0)
    r=record(1,.1)
    if fault=='same':r=record()
    if fault=='backwards':r=record(-1,0)
    if fault=='gap':r['sequence']=2
    if fault=='time_jump':r['solver_state_ned_frd']['t']=1
    if fault=='invalid':r['valid']=False
    g.receive(r,.05)
    assert g.reason is not None
    assert g.latest['sequence']==0


def test_suite_duration_and_missing_records():
    assert duration('straight')==42
    assert duration('depth_up')==duration('depth_down')==68
    assert all(duration(c)==12 for c in ('state_loss','stale_state','command_loss'))


def test_missing_evidence_cannot_pass(tmp_path):
    assert not analyze_run(tmp_path)['passed']


def test_shutdown_zero_stamp_cannot_regress_after_observation_tail():
    from ros2.twin_axial_motion.safety import zero_command_stamp
    assert zero_command_stamp(32,record(t=32.15))==32.15
    assert zero_command_stamp(32,record(t=31.9))==32
    bad=record(t=33);bad['valid']=False
    assert zero_command_stamp(32,bad)==32
