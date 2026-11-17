import json,math,hashlib
from pathlib import Path
from types import SimpleNamespace
import pytest
from ros2.twin_axial_g12_real_v3.gateway import Gate
from ros2.twin_axial_g12_real_v3.ipc import JsonlReader
from ros2.twin_axial_g12_real_v3 import gate
from ros2.twin_axial_g12_real_v3.trial import run_trial
from ros2.twin_axial_s3_transient_v2.cli import ROOT
PYTHON=ROOT/'artifacts/twin_axial_g12_matrix/python_20261110_01'

@pytest.mark.parametrize('name',['ipc_07_test','ipc_08_test','ipc_09_test'])
def test_recorded_roundoff_failure(name):
    p=ROOT/f'artifacts/twin_axial_g12_real/{name}/S3/F0/P0/transactions.jsonl'
    r=next(json.loads(line) for line in p.read_text().splitlines() if json.loads(line)['sequence']==161)
    g=Gate();g.update({'X':1.,'M':0.},r['command_simulation_stamp'],r['command_sequence'],r['command_received_wall'])
    assert r['watchdog_zero']
    assert not g.applied(r['start_time'],r['wall_monotonic'])[2]

@pytest.mark.parametrize('sim,stamp,now,expired',[(8.1,8.100000000000001,10.1,False),(8.1,8.100000001,10.1,True),(8.1,8.15,10.1,True),(8.35,8.1,10.1,False),(8.350002,8.1,10.1,True),(8.1,8.1,11.000001,True),(8.1,8.1,11.,False),(2.25,1.6,10.01,True)])
def test_deadlines_not_relaxed(sim,stamp,now,expired):
    g=Gate();g.update({'X':1,'M':0},stamp,0,10.)
    assert g.applied(sim,now)[2]==expired
    if expired:assert all(v==0 for v in g.applied(sim,now)[0].values())

@pytest.mark.parametrize('stamp,seq',[(1.,0),(.9,1),(float('nan'),1),(1.,True)])
def test_invalid_order(stamp,seq):
    g=Gate();g.update({'X':1,'M':0},1.,0,10.)
    with pytest.raises(ValueError):g.update({'X':1,'M':0},stamp,seq,10.)

@pytest.mark.parametrize('cut',range(1,30))
def test_partial_utf8_and_multiline_exactly_once(tmp_path,cut):
    p=tmp_path/'pipe';reader=JsonlReader(p)
    payload=(json.dumps({'sequence':0,'text':'艇'},ensure_ascii=False)+'\n'+json.dumps({'sequence':1})+'\n').encode()
    p.write_bytes(payload[:cut]);first=reader.read()
    with p.open('ab') as f:f.write(payload[cut:])
    records=first+reader.read()
    assert [v['sequence'] for v in records]==[0,1]
    assert reader.read()==[]

def test_bad_complete_line_fails_without_advancing(tmp_path):
    p=tmp_path/'pipe';p.write_bytes(b'{"ok":1}\nBROKEN\n');r=JsonlReader(p)
    with pytest.raises(ValueError):r.read()
    assert r.offset==0

def test_truncation_and_disappearance_rejected(tmp_path):
    p=tmp_path/'pipe';p.write_bytes(b'{"x":1}\n');r=JsonlReader(p);r.read();p.write_bytes(b'')
    with pytest.raises(ValueError,match='truncated'):r.read()
    p.unlink()
    with pytest.raises(ValueError,match='disappeared'):r.read()

def test_no_live_evidence_blocks():
    assert not gate.transport_check(None)['passed']
    with pytest.raises(ValueError):gate.guard({'passed':True})

def test_trial_rejected_before_launch_or_output(monkeypatch,tmp_path):
    import subprocess
    monkeypatch.setattr(subprocess,'Popen',lambda *a,**kw:pytest.fail('launch'))
    with pytest.raises(ValueError):run_trial(SimpleNamespace(gate_proof={'passed':True},gui=True,output=tmp_path/'run'))
    assert not (tmp_path/'run').exists()

@pytest.fixture
def release(tmp_path,monkeypatch):
    monkeypatch.setattr(gate,'RELEASE',tmp_path/'release.json')
    return gate.seal(PYTHON)

def test_release_includes_v2_v3_and_rejects_drift(release,monkeypatch):
    files=release['adapter_files']
    assert 'ros2/twin_axial_g12_real_v2/gate.py' in files
    assert 'ros2/twin_axial_g12_real_v3/gateway.py' in files
    assert gate.contract(PYTHON)['revision']=='g12_real_v3'
    with pytest.raises(ValueError):gate.seal(PYTHON)
    original=gate.sha
    monkeypatch.setattr(gate,'sha',lambda p:'bad' if Path(p).as_posix().endswith('v3/gateway.py') else original(p))
    with pytest.raises(ValueError):gate.contract(PYTHON)

def test_legacy_record_cannot_pass_binding():
    with pytest.raises((ValueError,KeyError)):
        gate.verify_record_binding(ROOT/'artifacts/twin_axial_g12_real/real_ipc_20261111_03/S3/F0/P0')

def test_missing_snapshot_is_rejected(release,tmp_path):
    p=tmp_path/'run';p.mkdir()
    (p/'adapter_release.json').write_bytes(gate.RELEASE.read_bytes())
    (p/'evidence_gate.json').write_text(json.dumps(dict(passed=True,ready_to_run=True,transport=dict(passed=True),adapter_release_sha256=gate.sha(gate.RELEASE))))
    with pytest.raises(ValueError,match='snapshot'):gate.verify_record_binding(p)

def test_physical_acceptance_unchanged():
    old=(ROOT/'ros2/twin_axial_g12_real_v2/acceptance.py').read_bytes()
    new=(ROOT/'ros2/twin_axial_g12_real_v3/acceptance.py').read_bytes()
    assert old==new

@pytest.fixture
def probe(release,tmp_path):
    import time
    p=tmp_path/'probe';p.mkdir()
    meta=dict(origin='actual_integration_probe',gui=True,adapter_release_sha256=gate.sha(gate.RELEASE),runtime_files=release['runtime_files'],completed_unix=time.time())
    rows=[]
    for channel in ('state','command','pose'):
        for i in range(30):rows.append(dict(channel=channel,sequence=i,sent={'sequence':i},received={'sequence':i},sender_pid=1,receiver_pid=2,sent_monotonic=i*.05,received_monotonic=i*.05+.01,message_type='lotusim_msgs/msg/VesselPositionArray'))
    rows.append(dict(channel='action',accepted=True,status=4,results=[True]))
    rows.append(dict(channel='termination',zero_applied=True,last_step=30,step_after_cleanup=30,cleanup_completed=True))
    (p/'probe.json').write_text(json.dumps(meta));(p/'events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    (p/'manifest.json').write_text(json.dumps({n:gate.sha(p/n) for n in ('probe.json','events.jsonl')}))
    return p

def test_transport_contract_valid_fixture_only(probe):
    assert gate.transport_check(probe)['passed']  # synthetic unit fixture, never live evidence

@pytest.mark.parametrize('fault',['missing_pose','bad_action','continued_steps','expired','source','payload','delay'])
def test_transport_gate_rejects_bad_raw_evidence(probe,fault):
    meta=json.loads((probe/'probe.json').read_text());rows=[json.loads(x) for x in (probe/'events.jsonl').read_text().splitlines()]
    if fault=='missing_pose':rows=[r for r in rows if r['channel']!='pose']
    if fault=='bad_action':rows[-2]['status']=5
    if fault=='continued_steps':rows[-1]['step_after_cleanup']=31
    if fault=='expired':meta['completed_unix']=0
    if fault=='source':meta['adapter_release_sha256']='old'
    if fault=='payload':rows[0]['received']={'sequence':9}
    if fault=='delay':rows[0]['received_monotonic']=1
    (probe/'probe.json').write_text(json.dumps(meta));(probe/'events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    (probe/'manifest.json').write_text(json.dumps({n:gate.sha(probe/n) for n in ('probe.json','events.jsonl')}))
    with pytest.raises(ValueError):gate.transport_check(probe)
