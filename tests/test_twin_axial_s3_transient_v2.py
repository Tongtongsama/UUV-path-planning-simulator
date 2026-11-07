import json
import math

import numpy as np
import pytest

from ros2.twin_axial_s3_transient_v2 import cli


def test_original_rates_are_isolated_and_restored():
    previous = dict(cli.study.FIXED)
    base = dict(cli.study.base_guidance.CONFIG)
    with pytest.raises(RuntimeError):
        with cli.original_rates():
            assert len(cli.GROUPS) == 12
            for group in cli.GROUPS:
                config = cli.study.config_for_group(group)
                assert config['pitch_rate_limit'] == .12
                assert config['speed_rate_limit'] == .04
                assert config['turn_speed'] == .12
                assert config['pitch_limit_deg'] == 70
            raise RuntimeError('interrupted')
    assert cli.study.FIXED == previous
    assert cli.study.base_guidance.CONFIG == base


def stationary_record(folder):
    points, _ = cli.build_path(cli.SCENES['S3'], (2,-10))
    guide = cli.study.base_guidance.Guide(points, [0,0])
    pid = cli.AllocatedPID(math.radians(70))
    rows = []
    for i in range(2):
        t = (i+1)*.05
        target, nav = guide.update(np.array([2,-10,0,0,0,0]), t)
        _, control = pid.command(dict(valid=True, sequence=i, elapsed_end=t, pitch_nose_up_rad=0,
                   solver_state_ned_frd=dict(t=t,y=2,z=30,u=0,w=0,q=0)), target,.05)
        rows.append(dict(t=t,state=[2,-10,0,0,0,0],clearance=2,nav=nav,control=control,applied_force=[0,0,0]))
    metrics = dict(arrival_time=None,min_clearance=2,rms_error=0,max_error=0,saturation_fraction=0)
    cli.write(folder/'plan.json',dict(path=points.tolist(),scene='S3',flow='F0',case='P0',config=cli.study.BASE_CONFIG))
    cli.write(folder/'report.json',dict(passed=False,checks=cli.study.case_checks(metrics,False,None),metrics=metrics))
    (folder/'states.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
    cli.write(folder/'manifest.json',{name:cli.sha(folder/name) for name in ('plan.json','report.json','states.jsonl')})
    return rows


def test_replay_detects_resealed_state_tampering(tmp_path):
    rows = stationary_record(tmp_path)
    result = cli.verify_run(tmp_path,replay=True)
    assert result['record_verified'] and not result['passed']
    cli.write(tmp_path/'verified.json',result)
    assert cli.read(tmp_path/'verified.json')['checks']['envelope'] is True
    rows[-1]['state'][4] = 2.
    (tmp_path/'states.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
    manifest = cli.read(tmp_path/'manifest.json')
    manifest['states.jsonl'] = cli.sha(tmp_path/'states.jsonl')
    cli.write(tmp_path/'manifest.json',manifest)
    with pytest.raises(AssertionError,match='dynamics'):
        cli.verify_run(tmp_path,replay=True)


def test_manifest_cannot_escape_record_directory(tmp_path):
    stationary_record(tmp_path)
    manifest = cli.read(tmp_path/'manifest.json')
    manifest['../outside.json'] = '0'*64
    cli.write(tmp_path/'manifest.json',manifest)
    with pytest.raises(AssertionError):
        cli.verify_manifest(tmp_path)
