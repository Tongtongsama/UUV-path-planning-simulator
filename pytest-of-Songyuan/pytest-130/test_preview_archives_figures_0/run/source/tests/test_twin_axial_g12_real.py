import json
from pathlib import Path
from types import SimpleNamespace
import pytest

from ros2.twin_axial_g12_real import gate,mission,cli,acceptance
from ros2.twin_axial_g12_real.trial import run_trial
from ros2.twin_axial_s3_transient_v2.cli import ROOT,read,write

PYTHON=ROOT/'artifacts/twin_axial_g12_matrix/python_20261110_01'


@pytest.mark.parametrize('scene',['S1','S2','S3'])
def test_all_plans_and_models_preserve_frozen_python_geometry(scene,tmp_path):
    before=dict(mission.study.base_guidance.CONFIG)
    for flow in ('F0','F1','F2'):
        for case in mission.CASES:
            saved=read(PYTHON/scene/flow/case/'plan.json')
            plan=mission.make_plan(case,scene,flow)
            assert plan['path']==saved['path'] and plan['config']==saved['config']
            assert mission.obstacle_extents(mission.environment_from_world(mission.world_xml(case,scene,True)))==mission.SCENES[scene]
    out=mission.prepare(tmp_path/'prepared','P2',scene,True,'F2')
    assert read(out/'model.yml')==mission.model_config('P2','F2')
    assert mission.study.base_guidance.CONFIG==before


def test_s3_steady_rejection_is_preserved_without_changing_geometry():
    plan=mission.make_plan('P0','S3','F2')
    assert not plan['steady_certificate']['passed']
    assert plan['geometry']['planning_clearance_m']>=1.5
    mission.Mission(plan,mission.world_xml('P0','S3',True))
    plan['config']['cruise_speed']=.47
    with pytest.raises(ValueError):mission.Mission(plan,mission.world_xml('P0','S3',True))


@pytest.mark.parametrize('key',['S3/F2/P2','S3/F0/P3'])
def test_real_mission_matches_full_archived_feedback(key):
    result=gate.feedback_equivalence(PYTHON/key)
    assert result['passed']
    assert result['max_coordinate_roundtrip_error']<1e-9
    assert result['max_archived_diagnostic_roundoff']<1e-9


@pytest.fixture
def release(tmp_path,monkeypatch):
    monkeypatch.setattr(gate,'RELEASE',tmp_path/'release.json')
    gate.seal_release(PYTHON)
    return read(gate.RELEASE)


def test_contract_matches_frozen_evidence_and_refuses_overwrite(release):
    assert gate.check_contract(PYTHON)['python_source_sha256']==gate.APPROVED_PYTHON_SOURCE
    with pytest.raises(ValueError,match='already exists'):gate.seal_release(PYTHON)


@pytest.mark.parametrize('relative',[
    'S1/F0/P0/states.jsonl','S2/F2/P4/report.json',
    'S3/F2/P0_repeat/manifest.json','matrix.json','preregistration.json'])
def test_resealed_or_changed_evidence_is_rejected(release,monkeypatch,relative):
    original=gate.sha
    def altered(path):
        return '0'*64 if Path(path).resolve()==(PYTHON/relative).resolve() else original(path)
    monkeypatch.setattr(gate,'sha',altered)
    with pytest.raises(ValueError,match='Evidence changed'):gate.check_contract(PYTHON)


def test_configuration_drift_is_rejected(release,monkeypatch):
    monkeypatch.setattr(gate,'GUIDE_CONFIG',{**gate.GUIDE_CONFIG,'lookahead':2.2})
    with pytest.raises(ValueError,match='configuration drift'):gate.check_contract(PYTHON)


def test_adapter_source_drift_is_rejected(release,monkeypatch):
    original=gate.sha
    monkeypatch.setattr(gate,'sha',lambda p:'0'*64 if Path(p).resolve()==Path(mission.__file__).resolve() else original(p))
    with pytest.raises(ValueError,match='Adapter source changed'):gate.check_contract(PYTHON)


def test_failed_gate_cannot_create_run_directory_or_launch(monkeypatch,tmp_path):
    import subprocess
    def reject(*a,**kw):raise ValueError('fixture gate rejection')
    def forbidden(*a,**kw):pytest.fail('Process launch is forbidden')
    monkeypatch.setattr(cli,'evaluate',reject)
    monkeypatch.setattr(subprocess,'Popen',forbidden)
    output=tmp_path/'real'
    assert cli.main(['run','--python-matrix',str(PYTHON),'--output',str(output)])==2
    assert not output.exists()


def test_check_only_success_cannot_create_run_directory_or_launch(monkeypatch,tmp_path):
    import subprocess
    def forbidden(*a,**kw):pytest.fail('Process launch is forbidden')
    monkeypatch.setattr(subprocess,'Popen',forbidden)
    monkeypatch.setattr(cli,'evaluate',lambda *a,**kw:SimpleNamespace(report=dict(passed=True,ready_to_run=True,real_started=False)))
    output=tmp_path/'real'
    assert cli.main(['real-matrix','--check-only','--python-matrix',str(PYTHON),'--output',str(output)])==0
    assert not output.exists()


def test_direct_trial_cannot_use_cached_json_as_proof(monkeypatch,tmp_path):
    import socket
    monkeypatch.setattr(socket,'socket',lambda *a,**kw:pytest.fail('Ports touched before gate'))
    with pytest.raises(ValueError,match='in-process evidence proof'):
        run_trial(SimpleNamespace(gate_proof={'passed':True},gui=True,output=tmp_path/'real'))
    assert not (tmp_path/'real').exists()


@pytest.mark.parametrize('repeat_pass,arrival_delta',[(False,0.),(True,.2),(True,0.)])
def test_real_suite_requires_repeat_acceptance_and_arrival_time(monkeypatch,tmp_path,repeat_pass,arrival_delta):
    for name in (*mission.CASES,'P0_repeat'):
        folder=tmp_path/name;(folder/'source').mkdir(parents=True)
        (folder/'source/a.py').write_text('same')
        write(folder/'metadata.json',dict(scene='S3',flow='F2'))
        (folder/'transactions.jsonl').write_text(''.join(json.dumps(dict(valid=True,elapsed_end=t,position_enu_m=[0,0,0]))+'\n' for t in (.05,.10)))
    def analyze(folder):
        repeat=Path(folder).name=='P0_repeat'
        return dict(passed=repeat_pass if repeat else True,checks=dict(safety=True),
                    arrival_time_s=10.+(arrival_delta if repeat else 0),error=None)
    monkeypatch.setattr(acceptance,'analyze_run',analyze)
    result=acceptance.analyze_suite(tmp_path)
    assert result['passed']==(repeat_pass and arrival_delta<=.1)


def test_physical_acceptance_body_unchanged_except_evidence_binding():
    old=(ROOT/'ros2/twin_axial_feasible_matrix/acceptance.py').read_text(encoding='utf-8')
    new=Path(acceptance.__file__).read_text(encoding='utf-8')
    old=old[old.index('def _analyze'):old.index('def source_snapshot')]
    new=new[new.index('def _analyze'):new.index('def source_snapshot')]
    new=new.replace("    from .gate import verify_record_binding\n    checks['frozen_evidence_binding']=verify_record_binding(directory)\n",'')
    assert old==new
