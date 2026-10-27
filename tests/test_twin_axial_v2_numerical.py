import copy
import math
import numpy as np
import pytest
from ros2.twin_axial_v2.numerical import (
    STEPS,schedule,initial_state,reduced,energy3,energy6,reference_run,assess,decode,reanalyze,
)


def test_exact_switch_times_and_physical_wrench():
    from ros2.twin_axial_v2.allocation import wrench
    assert wrench(schedule(1.999))=={'X':0,'Z':0,'M':0}
    assert wrench(schedule(2))=={'X':4,'Z':0,'M':0}
    assert wrench(schedule(7))=={'X':0,'Z':0,'M':.5}
    assert wrench(schedule(12))=={'X':0,'Z':0,'M':0}
    for dt in STEPS:
        for boundary in (2,7,12,20,22): assert boundary/dt==pytest.approx(round(boundary/dt))


def test_energy_units_and_coordinate_projection():
    s=initial_state()
    assert reduced(s)==pytest.approx([2,-30,0,0,0,0])
    assert energy6(s)==0
    s.update(u=.4,w=.2,q=.1)
    assert energy6(s)==pytest.approx(.5*(25*.4**2+28*.2**2+10*.1**2))
    y=np.array([2,-30,.2,.4,.2,.1])
    assert energy3(y)==pytest.approx(.5*(25*.4**2+28*.2**2+10*.1**2)+6*(1-math.cos(.2)))


def test_decoder_enforces_actual_requested_step():
    r={k:[v] for k,v in initial_state().items()}; r['t']=[.0125]
    assert decode(r,0,.0125)['t']==.0125
    with pytest.raises(ValueError,match='step/time'): decode(r,0,.05)
    r['qr']=[2]
    with pytest.raises(ValueError,match='quaternion'): decode(r,0,.0125)


@pytest.fixture(scope='module')
def matched_reference():
    runs={}
    for dt in STEPS:
        rows=reference_run(dt)
        twin=copy.deepcopy(rows)
        for r in twin: r['full_state']={'x':0,'v':0,'p':0,'r':0}
        runs[str(dt)]={'python':rows,'xdyn':twin}
    return runs


def test_reference_decay_convergence_and_identical_series(matched_reference):
    report=assess(matched_reference)
    assert report['passed']
    assert report['consistency']['0.05']['max_errors']['position_m']==0


@pytest.mark.parametrize('fault',['energy_increase','response_mismatch','missing_sample','nonfinite'])
def test_acceptance_rejects_bad_evidence(matched_reference,fault):
    runs=copy.deepcopy(matched_reference); rows=runs['0.05']['xdyn']
    if fault=='energy_increase': rows[-1]['energy_J']=10
    if fault=='response_mismatch': rows[200]['state'][0]+=.2
    if fault=='missing_sample': rows.pop(100)
    if fault=='nonfinite': rows[100]['state'][0]=float('nan')
    if fault in ('missing_sample','nonfinite'):
        with pytest.raises(ValueError): assess(runs)
    else: assert not assess(runs)['passed']


def test_missing_files_never_pass(tmp_path):
    assert not reanalyze(tmp_path)['passed']


def test_revision_gate_rejects_old_model_sized_residual(matched_reference):
    from ros2.twin_axial_v2.numerical import diagnose
    runs=copy.deepcopy(matched_reference)
    ref=runs['0.05']['python']
    assert diagnose(runs,ref)['passed']
    runs['0.05']['xdyn'][100]['state'][0]+=.02
    assert assess(runs)['passed']  # old broad response tolerance alone is insufficient
    assert not diagnose(runs,ref)['passed']
