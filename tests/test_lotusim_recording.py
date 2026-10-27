import csv
import hashlib
import json
from pathlib import Path
import pytest
from ros2.lotusim_replay.lotusim_replay.recording import Recorder
from ros2.lotusim_replay.lotusim_replay.core import load_verified_payload,select_scene,lotusim_pose
from ros2.lotusim_replay.lotusim_replay.gazebo_scene import scene_sdf

@pytest.fixture(scope='module')
def payload(): return load_verified_payload(Path(__file__).parents[1])

def make(path,payload):
    scene=select_scene(payload,'double_baffle_detour')
    return Recorder(path,payload,scene,{'APPROACH':scene['arms']['APPROACH']},{'arms':['APPROACH']},scene_sdf(scene))

def test_recording_roundtrip_and_manifest(tmp_path,payload):
    r=make(tmp_path/'run',payload)
    r.command(0,'APPROACH','boat',lotusim_pose([3,1,-5,.2]))
    r.observe('APPROACH','boat',100,(1,0,-5),(0,0,0,1))
    r.finish('completed'); r.finish('failed')
    summary=json.loads((r.path/'summary.json').read_text())
    assert summary['status']=='completed' and summary['observations']==1
    with (r.path/'commands.csv').open() as f:
        row=list(csv.DictReader(f))[0]
    assert float(row['source_time_s'])==3 and float(row['z'])==-5
    for name,digest in json.loads((r.path/'manifest.json').read_text()).items():
        assert hashlib.sha256((r.path/name).read_bytes()).hexdigest()==digest

def test_missing_telemetry_and_failure_are_explicit(tmp_path,payload):
    r=make(tmp_path/'run',payload); r.finish('failed','timeout')
    summary=json.loads((r.path/'summary.json').read_text())
    assert not summary['gazebo_observed'] and summary['error']=='timeout'
    with pytest.raises(FileExistsError): make(r.path,payload)

def test_historical_controls_are_exact_and_not_live(tmp_path,payload):
    r=make(tmp_path/'run',payload); r.finish('interrupted')
    scene=select_scene(payload,'double_baffle_detour')
    with (r.path/'APPROACH_controls.csv').open() as f: data=list(csv.reader(f))
    assert data[0]==payload['control_columns']
    assert len(data)-1==len(scene['arms']['APPROACH']['controls'])
    assert float(data[1][1])==scene['arms']['APPROACH']['controls'][0][1]
