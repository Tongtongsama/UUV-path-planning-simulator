import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path
from xml.etree import ElementTree as ET
import pytest
from ros2.recorded_replay.core import (Playback,display_world,fit_camera,load_recording,load_suite,
                                       sample,suite_unchanged,unchanged,visual_model)

def fixture_record(tmp_path,x0=1,x1=3,z=-30):
    (tmp_path/'worlds').mkdir()
    (tmp_path/'worlds/minimal.world').write_text('''<sdf><world><model name="test"><plugin name="bad"/><lotus_param><uri>ws://127.0.0.1:12356</uri></lotus_param><link name="base_link"><sensor name="bad"/><visual name="body"><geometry><box><size>1 .2 .2</size></box></geometry></visual></link></model></world></sdf>''')
    (tmp_path/'metadata.json').write_text(json.dumps({'model':'test'}))
    (tmp_path/'run_status.json').write_text(json.dumps({'status':'completed'}))
    rows=[{'time':10.,'position':[x0,0,z],'quaternion_xyzw':[0,0,0,1]},
          {'time':12.,'position':[x1,0,z],'quaternion_xyzw':[0,math.sin(.2),0,math.cos(.2)]}]
    (tmp_path/'gazebo_poses.jsonl').write_text('\n'.join(map(json.dumps,rows)))
    files=[p for p in tmp_path.rglob('*') if p.is_file()]
    (tmp_path/'manifest.json').write_text(json.dumps({p.relative_to(tmp_path).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}))
    return load_recording(tmp_path)


def fixture_suite(tmp_path):
    (tmp_path/'beta').mkdir(); (tmp_path/'alpha').mkdir()
    fixture_record(tmp_path/'beta',1,3)
    fixture_record(tmp_path/'alpha',2,8,z=-28)
    (tmp_path/'suite.json').write_text(json.dumps({'passed':True,'runs':{'beta':{'exit_code':0},'alpha':{'exit_code':0}}}))
    return load_suite(tmp_path)

def test_snapshot_and_safe_display_scene(tmp_path):
    record=fixture_record(tmp_path)
    xml=visual_model(record,'display')
    assert 'plugin' not in xml and 'lotus_param' not in xml and 'sensor' not in xml and 'ws://' not in xml
    world=ET.fromstring(display_world('unique',[1,0,-30]))
    plugins=[p.get('filename') for p in world.findall('./world/plugin')]
    assert len(plugins)==2 and not any('physics' in x.lower() for x in plugins)
    assert unchanged(record)
    (tmp_path/'metadata.json').write_text('{}')
    assert not unchanged(record)
    with pytest.raises(ValueError,match='hash mismatch'): load_recording(tmp_path)

def test_interpolation_position_and_shortest_rotation(tmp_path):
    record=fixture_record(tmp_path); pose=sample(record,1)
    assert pose['position']==[2,0,-30]
    assert pose['quaternion_xyzw']==pytest.approx([0,math.sin(.1),0,math.cos(.1)])
    assert sample(record,100)['time']==12
    record['poses'][1]['quaternion_xyzw']=[-v for v in record['poses'][1]['quaternion_xyzw']]
    assert sample(record,1)['quaternion_xyzw']==pytest.approx(pose['quaternion_xyzw'])

def test_pause_slow_restart_and_loop():
    c=Playback(8,.25,True)
    c.advance(30); assert c.elapsed==0
    c.command('p'); c.advance(4); assert c.elapsed==1
    c.command('pause'); c.advance(20); assert c.elapsed==1
    c.command('speed 0.5'); c.command('play'); c.advance(2); assert c.elapsed==2
    c.command('r'); assert c.elapsed==0 and not c.paused
    c.command('l'); c.advance(18); assert c.elapsed==1 and c.cycles==1
    c.command('l'); c.advance(20); assert c.elapsed==8 and c.paused and c.cycles==2
    with pytest.raises(ValueError): c.command('speed nan')
    with pytest.raises(ValueError): c.command('speed 0')

def test_manifest_cannot_escape_recording(tmp_path):
    fixture_record(tmp_path)
    manifest=json.loads((tmp_path/'manifest.json').read_text()); manifest['../outside']='invalid'
    (tmp_path/'manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError,match='Invalid manifest path'): load_recording(tmp_path)


def test_suite_verifies_every_run_in_manifest_order(tmp_path):
    suite=fixture_suite(tmp_path)
    assert suite['order']==['beta','alpha']
    assert [len(suite['records'][name]['poses']) for name in suite['order']]==[2,2]
    assert suite_unchanged(suite)
    (tmp_path/'alpha'/'metadata.json').write_text('{}')
    assert not suite_unchanged(suite)
    with pytest.raises(ValueError,match='Invalid suite case alpha: Recording hash mismatch'): load_suite(tmp_path)
    (tmp_path/'suite.json').unlink()
    with pytest.raises(ValueError,match='suite.json is missing'): load_suite(tmp_path)


def test_suite_rejects_case_names_outside_suite(tmp_path):
    for name in ('../escape','a/b','a\\b','..',''):
        (tmp_path/'suite.json').write_text(json.dumps({'runs':{name:{'exit_code':0}}}))
        with pytest.raises(ValueError,match='Invalid suite case name'): load_suite(tmp_path)


def test_fit_camera_frames_all_suite_poses(tmp_path):
    suite=fixture_suite(tmp_path); camera=fit_camera(suite['records'].values())
    cx,cy,cz=camera[:3]; hfov=math.radians(60); vfov=2*math.atan(math.tan(hfov/2)/(16/9))
    points=[p['position'] for record in suite['records'].values() for p in record['poses']]
    assert cy<min(p[1] for p in points)
    for x,_,z in points:
        assert abs(math.atan2(abs(x-cx),-cy))<=hfov/2+1e-9
        assert abs(math.atan2(abs(z-cz),math.hypot(x-cx,cy)))<=vfov/2+1e-9


def test_display_world_accepts_framed_camera_and_title():
    world=ET.fromstring(display_world('unique',[1,0,-30],camera=[4,-10,-30,0,0,1.5],title='Suite TEST'))
    plugins=world.findall('./world/plugin')
    assert len(plugins)==2 and not any('physics' in p.get('filename').lower() for p in plugins)
    assert world.findtext('./world/gui/plugin[@filename="MinimalScene"]/gz-gui/title')=='Suite TEST'
    assert world.findtext('./world/gui/plugin[@filename="MinimalScene"]/camera_pose')=='4 -10 -30 0 0 1.5'


def test_cli_suite_validate_only_needs_no_ros(tmp_path):
    fixture_suite(tmp_path)
    repo=Path(__file__).resolve().parents[1]
    result=subprocess.run([sys.executable,'-m','ros2.recorded_replay.cli',str(tmp_path),'--validate-only'],
                          cwd=repo,capture_output=True,text=True,timeout=60)
    assert result.returncode==0,result.stderr
    assert 'Verified suite' in result.stdout and 'beta' in result.stdout and 'alpha' in result.stdout


def test_cli_prepare_only_writes_verified_scene(tmp_path):
    suite_root=tmp_path/'suite'; suite_root.mkdir(); fixture_suite(suite_root)
    out=tmp_path/'prepared'
    repo=Path(__file__).resolve().parents[1]
    result=subprocess.run([sys.executable,'-m','ros2.recorded_replay.cli',str(suite_root),'--prepare-only','--output',str(out)],
                          cwd=repo,capture_output=True,text=True,timeout=60)
    assert result.returncode==0,result.stderr
    assert (out/'state/worlds/replay.world').is_file() and (out/'vehicle.sdf').is_file()
    summary=json.loads((out/'summary.json').read_text(encoding='utf-8'))
    assert summary['status']=='prepared' and summary['dynamics_launched'] is False and summary['source_unchanged'] is True
    assert set(summary['cases'])=={'beta','alpha'}
    source=json.loads((out/'source_record.json').read_text(encoding='utf-8'))
    assert set(source['cases'])=={'beta','alpha'} and len(source['suite_sha256'])==64
    world=(out/'state/worlds/replay.world').read_text(encoding='utf-8')
    assert 'camera_pose' in world and 'multi_agent_system_plugin' in world and 'physics_interface_plugin' not in world
