import json
import math
import pytest
from core import Pose
from ros2.twin_axial_navigation.mission import (
    prepare,Mission,world_xml,environment_from_world,segment_clearance,model_config,
    initial,CONFIG,path_distance,
)
from ros2.twin_axial_v2.model import model
from ros2.twin_axial_v2.numerical import initial_state
from ros2.twin_axial_v2.protocol import feedback

@pytest.fixture
def mission(tmp_path):
    p=prepare(tmp_path/'run','P0')
    return Mission(json.loads((p/'plan.json').read_text()),(p/'worlds/minimal.world').read_text())

def record(x,z,t):
    s=initial_state();s.update(y=x,z=20-z,t=t)
    return {'valid':True,'sequence':round(t/.05),'elapsed_end':t,'solver_state_ned_frd':s,**feedback(s)}

def test_scene_geometry_and_swept_collision_between_safe_endpoints():
    world=environment_from_world(world_xml('P0'))
    a,b=Pose(x=8,z=-10),Pose(x=11,z=-10)
    assert world.clearance(a)>=.9 and world.clearance(b)>=.9
    assert segment_clearance(world,a,b)==0
    assert segment_clearance(world,Pose(x=.5,z=-6),Pose(x=2,z=-6))==.5
    assert world.clearance(Pose(x=9.5,z=-7))==1

def test_plan_is_not_direct_collision_line(mission):
    assert len(mission.path)>2
    assert max(p.z for p in mission.path)>-7
    assert all(segment_clearance(mission.world,a,b)>=1.5-1e-6 for a,b in zip(mission.path,mission.path.poses[1:]))
    assert path_distance(mission.path,mission.path.poses[1])==0

def test_only_initial_conditions_change_from_v2():
    for case in ('P0','P1','P2','P3','P4'):
        new=model_config(case);old=model()
        new['bodies'][0]['initial position of body frame relative to NED']=old['bodies'][0]['initial position of body frame relative to NED']
        assert new==old
    assert initial('P1').pose.pitch==pytest.approx(math.radians(3))

def test_arrival_requires_continuous_two_seconds(mission):
    mission.previous_pose=Pose(x=18,z=-10)
    mission.update(record(18,-10,10));assert not mission.arrived
    mission.update(record(18,-10,11.95));assert not mission.arrived
    mission.update(record(17.4,-10,12));assert not mission.arrived
    mission.update(record(18,-10,12.05));assert not mission.arrived
    mission.update(record(18,-10,14.05));assert mission.arrived

def test_online_safety_stops_instead_of_accepting_unsafe_state(mission):
    with pytest.raises(RuntimeError,match='safety'):
        mission.update(record(9.5,-10,3))

def test_guidance_reads_live_position(mission):
    target,nav=mission.update(record(2,-10,2))
    assert nav['position_xz']==[2,-10]
    assert 0<=target.u<=.4
    assert abs(target.pitch)<=math.pi/3
