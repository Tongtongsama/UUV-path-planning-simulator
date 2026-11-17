import hashlib
import pytest
from scripts import run_feasible_flow_suite as runner
from ros2.twin_axial_feasible import verify_python

def fixture_gate(monkeypatch,tmp_path,flow='F1',passed=True):
    monkeypatch.setattr(verify_python,'suite',lambda _:dict(passed=passed,runs={'P0':{'flow':flow}}))
    root=tmp_path/'live';root.mkdir();(root/'file.py').write_text('unchanged')
    snapshot=tmp_path/'record/P0/source';snapshot.mkdir(parents=True);(snapshot/'file.py').write_text('unchanged')
    monkeypatch.setattr(runner,'ROOT',root)
    return tmp_path/'record',root

def test_matching_flow_gate(monkeypatch,tmp_path):
    path,_=fixture_gate(monkeypatch,tmp_path)
    assert runner.verify_gate(path,'F1')['passed']

def test_other_flow_does_not_authorize_run(monkeypatch,tmp_path):
    path,_=fixture_gate(monkeypatch,tmp_path,flow='F2')
    with pytest.raises(ValueError,match='matching Python'):runner.verify_gate(path,'F1')

def test_failed_flow_rejected(monkeypatch,tmp_path):
    path,_=fixture_gate(monkeypatch,tmp_path,flow='F0',passed=False)
    with pytest.raises(ValueError,match='no real trial'):runner.verify_gate(path,'F0')

def test_modified_source_rejected(monkeypatch,tmp_path):
    path,root=fixture_gate(monkeypatch,tmp_path)
    (root/'file.py').write_text('changed')
    with pytest.raises(ValueError,match='differs'):runner.verify_gate(path,'F1')
