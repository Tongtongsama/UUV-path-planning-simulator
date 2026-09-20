"""Measurement semantics, not acceptance of a particular success percentage."""
from copy import deepcopy
import json
import numpy as np
import pytest
from validation.envelope_measurement import (
    SCHEMA, audit, cases, corner_fixture, encode, execute, load_config,
    segment_distance, validate_config,
)


def test_frozen_schema_and_counts():
    cfg=load_config()
    assert len(list(cases(cfg,'pilot')))==30
    assert len(list(cases(cfg,'full')))==150
    assert len(list(cases(cfg,'stopping')))==106
    assert len(list(cases(cfg,'refinement')))==8


@pytest.mark.parametrize('key,value',[('dt',True),('repeats',1),('lookahead',float('nan')),('extra',1)])
def test_reject_changed_configuration(key,value):
    cfg=load_config(); cfg[key]=value
    with pytest.raises(ValueError): validate_config(cfg,json.loads(SCHEMA.read_text()))


def test_finite_segment_geometry():
    actual=segment_distance(np.array([[-1.,0.],[.5,2.],[3.,0.]]),np.array([0.,0.]),np.array([1.,0.]))
    assert actual==pytest.approx([1.,2.,2.])


def test_mirrored_corner_and_terminal_separation():
    cfg=load_config(); samples=list(cases(cfg,'pilot'))[:2]
    a,_=corner_fixture(samples[0],cfg); b,_=corner_fixture(samples[1],cfg)
    assert a[-1].z+b[-1].z==pytest.approx(2*cfg['fixture']['z_origin'])
    assert cfg['fixture']['outgoing_length']-cfg['fixture']['measurement_outgoing']>cfg['lookahead']


def test_short_window_censored_deterministic_no_terminal():
    cfg=deepcopy(load_config()); cfg['turn_budget_seconds']=.2
    case=next(cases(cfg,'pilot')); a=execute(case,cfg); b=execute(case,cfg)
    assert encode(a)==encode(b)
    assert a['summary']['status']=='RIGHT_CENSORED'
    assert a['summary']['d_exit'] is None
    assert not a['summary']['eligible_for_settled_entry_envelope']
    assert all(audit(a,cfg).values())


def test_ground_speed_not_water_relative_stop():
    cfg=deepcopy(load_config()); cfg['stop']['warmup_seconds']=.1; cfg['stop']['budget_seconds']=.1
    case=next(cases(cfg,'stopping')); case['w_offset']=.05
    result=execute(case,cfg)
    assert result['summary']['status']=='RIGHT_CENSORED'
    assert result['summary']['final_ground_speed']>cfg['stop']['ground_speed']
    assert all(audit(result,cfg).values())


def test_censored_distance_is_not_certified_stop():
    cfg=deepcopy(load_config()); cfg['stop']['warmup_seconds']=.1; cfg['stop']['budget_seconds']=.1
    result=execute(next(cases(cfg,'stopping')),cfg)
    assert result['summary']['maximum_forward_displacement']>0
    assert result['summary']['censor_reason']=='MEASUREMENT_BUDGET'
    assert not result['summary']['eligible_for_settled_entry_envelope']


def test_nonfinite_json_rejected():
    with pytest.raises(ValueError): encode({'x':np.float64('nan')})
