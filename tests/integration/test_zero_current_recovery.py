"""Prefix whitelist, repeated classification inputs, and recovery censoring."""
from copy import deepcopy
import numpy as np
import pytest
from validation.zero_current_recovery import (
    load_config,compare_prefix,recovery_events,statistics,peaks,runs,simulate,analyze,
)
from validation import envelope_measurement as v1


def fixture():
    states=[dict(timestamp=0.,pose=dict(x=7.9,z=-40.)),dict(timestamp=.05,pose=dict(x=8.,z=-40.))]
    step=dict(time=0.,elapsed=0.,boundary_margin=1.,projection=dict(segment_index=1,projected_segment_index=1,
              mu=.2,lambda_raw=.2,committed_s=17.9,projection_point=dict(x=7.9,z=-40.)),
              diagnostics=dict(integral_after=dict(i=.1)),applied=dict(tau_x=2.))
    old=dict(states=states,steps=[step],summary=dict(status='MEASUREMENT_COMPLETE',elapsed=.05,
             case=dict(angle=0),initial_controller_memory=dict(i=0.),final_controller_memory=dict(i=.1)))
    new=dict(states=deepcopy(states),steps=[deepcopy(step)],initial_controller_memory=dict(i=0.))
    return old,new


def test_exact_prefix_and_explicit_whitelist():
    old,new=fixture(); new['steps'][0]['projection']['mu']=.01; new['steps'][0]['projection']['lambda_raw']=.01
    new['steps'][0]['boundary_margin']=300.
    assert compare_prefix(old,new,load_config())['passed']


@pytest.mark.parametrize('mutation',['state','control','memory','clock','progress','point','length'])
def test_prefix_rejects_nonwhitelisted_difference(mutation):
    old,new=fixture()
    if mutation=='state': new['states'][1]['pose']['x']+=.01
    if mutation=='control': new['steps'][0]['applied']['tau_x']+=.01
    if mutation=='memory': new['steps'][0]['diagnostics']['integral_after']['i']+=.01
    if mutation=='clock': new['states'][1]['timestamp']+=.01
    if mutation=='progress': new['steps'][0]['projection']['committed_s']+=.01
    if mutation=='point': new['steps'][0]['projection']['projection_point']['z']+=.01
    if mutation=='length': new['states'].pop()
    with pytest.raises(ValueError): compare_prefix(old,new,load_config())


def test_recovery_then_reexit_not_stable_exit():
    t=np.arange(0.,10.,.1); mask=(t>=1)&(t<=3) | (t>=7)&(t<=8.5)
    r=recovery_events(t,mask,t,1.,3.)
    assert r['first_recovery_d_exit']==pytest.approx(1.)
    assert r['last_reentry_d_exit']==pytest.approx(7.)
    assert r['reexit_count']==2
    assert not r['stable_through_window_end']
    assert r['longest_in_band_duration']==pytest.approx(2.)


def test_sustained_end_and_insufficient_hold():
    t=np.arange(10.)
    r=recovery_events(t,t>=2,t,1.,5.)
    assert r['stable_through_window_end'] and r['reexit_count']==0
    r=recovery_events(t,t==9,t,1.,5.)
    assert r['first_recovery_d_exit'] is None and r['last_reentry_d_exit']==9


def test_runs_endpoint_duration():
    assert runs([True,True,False,True])==[(0,1),(3,3)]
    r=recovery_events(np.array([0.,.5,1.]),[True,True,False],np.arange(3.),1.,1.)
    assert r['first_recovery_time'] is None


def test_statistics_signed_and_absolute_trend():
    t=np.arange(4.); r=statistics(t,-t)
    assert r['trend']==pytest.approx(-1.) and r['absolute_trend']==pytest.approx(1.)


def test_peak_plateau_prominence_and_separation():
    t=np.arange(11.)*.1; y=np.array([2,0,3,3,0,2,0,4,0,2,1.])
    r=peaks(t,y,.1,.5)
    assert r['candidates'][0]['index']==2
    assert any(v['reason']=='SEPARATION' for v in r['candidates'])
    assert all(v['period']>=.5-1e-9 for v in r['ratios'])


def test_monotone_signal_no_fake_peaks():
    assert not peaks(np.arange(10.),np.arange(10.),.01,.5)['ratios']


def test_short_measurement_repeat_and_no_terminal():
    cfg=deepcopy(load_config()); cfg['budget_seconds']=.1
    case=next(v1.cases(v1.load_config(),'pilot'))
    a=simulate(case,cfg); b=simulate(case,cfg)
    assert v1.encode(a)==v1.encode(b)
    assert len(a['states'])==len(a['steps'])+1 and not a['terminal_entered']
    r=analyze(a,cfg,dict(end_timestamp=10.))
    assert r['labels']==['SEGMENT_NOT_ENTERED','INCONCLUSIVE']
