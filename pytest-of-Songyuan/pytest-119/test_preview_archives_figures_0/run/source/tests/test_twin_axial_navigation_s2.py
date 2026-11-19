"""The S2 map must bind planning and acceptance to the displayed SDF boxes."""
import json
from xml.etree import ElementTree as ET

import pytest

from ros2.twin_axial_navigation.mission import (
    Mission, environment_from_world, obstacle_extents, prepare,
    scene_obstacles, segment_clearance, world_xml,
)
from ros2.twin_axial_navigation.gui_replay import marker, parse_command, replay_world, sample_at
from ros2.twin_axial_navigation.current_model import FLOWS,model_for_flow
from ros2.twin_axial_navigation.acceptance import source_snapshot_consistent
from ros2.twin_axial_v2.model import model as zero_current_model


def test_s2_plan_uses_two_real_sdf_obstacles(tmp_path):
    output=prepare(tmp_path/'s2','P0','S2')
    plan=json.loads((output/'plan.json').read_text())
    xml=(output/'worlds/minimal.world').read_text()
    mission=Mission(plan,xml)
    assert plan['config']['scene']=='double_baffle_scaled_v1'
    assert 'obstacle' not in plan['config']
    assert obstacle_extents(mission.world)==scene_obstacles('S2')
    assert len(mission.path)>2
    assert min(segment_clearance(mission.world,a,b)
               for a,b in zip(mission.path,mission.path.poses[1:]))>=1.5
    assert len([m for m in ET.fromstring(xml).findall('./world/model') if m.findtext('static')=='true'])==2


def test_s2_rejects_obstacle_mismatch(tmp_path):
    output=prepare(tmp_path/'s2','P0','S2')
    plan=json.loads((output/'plan.json').read_text())
    altered=(output/'worlds/minimal.world').read_text().replace('6.5 0 -33.5','6.7 0 -33.5')
    assert obstacle_extents(environment_from_world(altered))!=scene_obstacles('S2')
    with pytest.raises(ValueError,match='obstacle geometry'):
        Mission(plan,altered)


def test_s1_world_geometry_stays_fixed():
    assert obstacle_extents(environment_from_world(world_xml('P0')))==scene_obstacles('S1')


def test_s2_gui_camera_keeps_obstacle_geometry():
    xml=world_xml('P0','S2',gui=True)
    assert obstacle_extents(environment_from_world(xml))==scene_obstacles('S2')
    assert ET.fromstring(xml).findtext('./world/gui/plugin/camera_pose')=='10 -42 -30 0 0 1.5707963267948966'
    root=ET.fromstring(xml)
    visuals=root.findall("./world/model[@name='uuv_twin_axial_v2']/link/visual")
    assert {v.get('name') for v in visuals}=={'body','nose','upper_thruster','lower_thruster'}
    assert len({v.findtext('material/diffuse') for v in visuals})==4
    assert root.find("./world/model[@name='uuv_twin_axial_v2']/link/visual[@name='tracking_marker_display_only']") is None
    baffles=[m.findtext('link/visual/material/diffuse') for m in root.findall('./world/model') if m.get('name','').startswith('double_baffle')]
    assert len(set(baffles))==2


def test_s2_gui_replay_is_visual_only_and_controllable():
    root=ET.fromstring(replay_world())
    assert root.find("./world/model[@name='uuv_twin_axial_v2']") is None
    assert all(p.get('filename')!='physics_interface_plugin' for p in root.findall('./world/plugin'))
    assert obstacle_extents(environment_from_world(replay_world()))==scene_obstacles('S2')
    assert ET.fromstring(marker('actual','0.1 0.85 1 1')).findtext('./model/link/visual/material/diffuse')=='0.1 0.85 1 1'
    assert sample_at([(0,0,0,0),(2,4,2,1)],1)==(1,2,1,.5)
    assert parse_command('4',1,False,10,0)==(4,False,10,False)
    assert parse_command('p',4,False,10,0)==(4,True,10,False)
    assert parse_command('r',4,True,10,0)==(4,False,0,False)


def test_uniform_current_model_preserves_f0_and_uses_relative_water_forces():
    assert model_for_flow('F0')==zero_current_model()
    assert FLOWS['F1']==(-.05,0) and FLOWS['F2']==(0,.05)
    for flow in ('F1','F2'):
        data=model_for_flow(flow)
        assert data['environment models']==zero_current_model()['environment models']
        forces=data['bodies'][0]['external forces']
        damping=next(f for f in forces if f.get('name')=='uniform_current_relative_damping')
        added=next(f for f in forces if f.get('name')=='added_mass_coriolis')
        assert 'u(t)-' in damping['X'] and 'w(t)-' in damping['Z']
        assert 'abs(' in damping['X']
        assert 'q(t)' in added['Z'] and 'sin(theta(t))' in added['X']
        assert '+5*(-' not in added['X'] and '+5*(-' not in added['Y']
        assert '+8*(-' not in added['Z']


def test_suite_source_snapshot_detects_mid_suite_edits(tmp_path):
    for name in ('P0','P1','P2','P3','P4','P0_repeat'):
        folder=tmp_path/name/'source'/'pkg'
        folder.mkdir(parents=True)
        (folder/'dynamic.py').write_text('gain=1\n')
    assert source_snapshot_consistent(tmp_path)
    (tmp_path/'P4'/'source'/'pkg'/'dynamic.py').write_text('gain=2\n')
    assert not source_snapshot_consistent(tmp_path)
    (tmp_path/'P4'/'source'/'pkg'/'dynamic.py').write_text('gain=1\n')
    (tmp_path/'P2'/'source'/'pkg'/'extra.py').write_text('added=1\n')
    assert not source_snapshot_consistent(tmp_path)


def test_suite_source_snapshot_rejects_empty_capture(tmp_path):
    for name in ('P0','P1','P2','P3','P4','P0_repeat'):
        (tmp_path/name/'source').mkdir(parents=True)
    assert not source_snapshot_consistent(tmp_path)
