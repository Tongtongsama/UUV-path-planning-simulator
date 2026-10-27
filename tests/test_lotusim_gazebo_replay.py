"""Display contracts independent of ROS and the frozen algorithm."""
import math
from pathlib import Path
from xml.etree import ElementTree as ET
import pytest
from ros2.lotusim_replay.lotusim_replay.core import load_verified_payload, select_scene, lotusim_pose, interpolate
from ros2.lotusim_replay.lotusim_replay.gazebo_scene import scene_sdf, vehicle_sdf
from ros2.lotusim_replay.lotusim_replay.gazebo_player import main

@pytest.fixture(scope='module')
def scene():
    return select_scene(load_verified_payload(Path(__file__).parents[1]),'double_baffle_detour')

def test_baffles_match_frozen_geometry(scene):
    root=ET.fromstring(scene_sdf(scene))
    for i,o in enumerate(scene['obstacles']):
        v=root.find(f".//visual[@name='obstacle_{i}']")
        x,y,z,*_=map(float,v.findtext('pose').split())
        sx,sy,sz=map(float,v.findtext('geometry/box/size').split())
        assert (x-sx/2,x+sx/2,z-sz/2,z+sz/2)==pytest.approx((o['x_min'],o['x_max'],o['z_min'],o['z_max']))
    assert len(root.findall(".//visual"))==len(scene['obstacles'])+4+len(scene['path'])-1+2

def test_no_competing_motion_or_collision_engine(scene):
    for xml in [scene_sdf(scene),vehicle_sdf('uuv','1 0 0 1')]:
        root=ET.fromstring(xml)
        assert not root.findall('.//plugin')
        assert not root.findall('.//collision')
        assert not root.findall('.//lotus_param')

@pytest.mark.parametrize('pitch',[0,math.pi/2,-math.pi/2,2*math.pi+.3])
def test_transformed_nose_matches_physics_forward_direction(pitch):
    pose=lotusim_pose([0,0,0,pitch])
    # Rotate local +X by quaternion. World z must increase for positive pitch.
    assert (1-2*pose.qy**2,-2*pose.qw*pose.qy)==pytest.approx((math.cos(pitch),math.sin(pitch)))

def test_arms_share_time_but_keep_independent_terminal_state(scene):
    short=scene['arms']['APPROACH']; long=scene['arms']['P1']
    assert short['status']=='SUCCESS' and long['status']=='TIMEOUT'
    assert interpolate(short['samples'],110)==short['samples'][-1]
    assert interpolate(long['samples'],110)[0]==pytest.approx(110)

@pytest.mark.parametrize('value',['nan','inf','0','-1'])
def test_bad_clock_arguments_fail_before_loading(value):
    with pytest.raises(SystemExit): main(['--rate',value,'--validate-only'])
