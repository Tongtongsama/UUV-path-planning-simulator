from pathlib import Path
import json
import pytest
from ros2.twin_axial_g12_v3_joint import gate

def test_joint_contract_preserves_frozen_v3():
    assert gate.contract()['v3_release_sha256']==gate.sha(gate.v3.RELEASE)

@pytest.mark.parametrize('name',['ros2/twin_axial_g12_v3_joint/probe.py','ros2/twin_axial_g12_v3_joint/gate.py','ros2/twin_axial_g12_real_v3/gateway.py'])
def test_source_drift_blocks(name,monkeypatch):
    original=gate.sha
    monkeypatch.setattr(gate,'sha',lambda p:'changed' if Path(p).resolve()==(gate.ROOT/name).resolve() else original(p))
    with pytest.raises(ValueError):gate.contract()

def test_seal_never_overwrites():
    with pytest.raises(ValueError):gate.seal()

def test_missing_raw_channels_rejected(tmp_path):
    (tmp_path/'manifest.json').write_text('{}')
    with pytest.raises(ValueError,match='Incomplete'):gate.analyze(tmp_path)

def test_probe_existing_output_refused_before_any_launch(monkeypatch,tmp_path):
    import subprocess,sys
    from ros2.twin_axial_g12_v3_joint import probe
    monkeypatch.setattr(sys,'argv',['probe','--output',str(tmp_path)])
    monkeypatch.setattr(subprocess,'Popen',lambda *a,**k:pytest.fail('launch'))
    with pytest.raises(ValueError,match='exists'):probe.main()
