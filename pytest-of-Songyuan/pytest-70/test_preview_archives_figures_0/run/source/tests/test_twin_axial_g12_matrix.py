from copy import deepcopy
import pytest

from ros2.twin_axial_g12_matrix import cli


def states(count=4):
    return [dict(t=(i+1)*.05,state=[0.,-10.,0.,0.,0.,0.]) for i in range(count)]


def accepted(arrival=.2):
    return dict(passed=True,record_verified=True,metrics=dict(arrival_time=arrival,
                min_clearance=1.,rms_error=.1,max_error=.2))


def test_repeat_rejects_failed_repeat_even_when_positions_identical():
    bad=accepted();bad['passed']=False
    r=cli.repeatability(states(),states(),accepted(),bad)
    assert not r['passed'] and not r['checks']['repeat_run_passed']


def test_repeat_rejects_arrival_difference_despite_same_recorded_positions():
    r=cli.repeatability(states(),states(),accepted(.2),accepted(.301))
    assert not r['passed'] and not r['checks']['arrival_time']


def test_repeat_rejects_truncated_run_despite_identical_overlap():
    r=cli.repeatability(states(10),states(4),accepted(),accepted())
    assert not r['passed'] and not r['checks']['terminal_time']


def test_repeat_rejects_position_limit_and_duplicate_times():
    b=states()
    for row in b:row['state'][0]=.021
    assert not cli.repeatability(states(),b,accepted(),accepted())['passed']
    b=states();b[-1]['t']=b[-2]['t']
    with pytest.raises(ValueError):cli.repeatability(states(),b,accepted(),accepted())


def test_repeat_all_original_limits_and_both_acceptances_pass():
    r=cli.repeatability(states(),states(),accepted(),accepted())
    assert r['passed'] and r['arrival_time_difference_s']==0 and r['terminal_time_difference_s']==0


def test_failed_repeat_blocks_group_and_gate(monkeypatch,tmp_path):
    rr={key:accepted() for key in cli.KEYS}
    monkeypatch.setattr(cli,'rows',lambda folder:states())
    rr['S1/F0/P0_repeat']['passed']=False
    s=cli.summarize(tmp_path,rr)
    assert len(rr)==54 and not s['groups']['S1/F0']['passed'] and not s['all_54_passed']
    assert not cli.gate_report(s,True)['python_prerequisite_passed']
    assert not cli.gate_report(dict(all_54_passed=True),False)['python_prerequisite_passed']


def test_source_mutation_is_detected(monkeypatch,tmp_path):
    live=tmp_path/'live';live.mkdir();(live/'module.py').write_text('frozen')
    run=tmp_path/'run';(run/'source').mkdir(parents=True)
    (run/'source/module.py').write_text('frozen')
    files={'module.py':cli.sha(live/'module.py')}
    digest=cli.hashlib.sha256(cli.json.dumps(files,sort_keys=True).encode()).hexdigest()
    cli.write(run/'source.json',dict(files=files,aggregate_sha256=digest))
    monkeypatch.setattr(cli,'ROOT',live)
    assert cli.check_source(run)==digest
    (live/'module.py').write_text('changed')
    with pytest.raises(ValueError,match='Frozen source changed'):cli.check_source(run)


def test_g12_matches_frozen_candidate_and_matrix_has_54_unique_entries():
    candidate=cli.read(cli.ROOT/'artifacts/twin_axial_s3_transient_v2/candidate_G12_20261107.json')
    assert cli.g12_config()==candidate['configuration']
    assert len(cli.KEYS)==len(set(cli.KEYS))==54
    assert sum(key.endswith('P0_repeat') for key in cli.KEYS)==9
