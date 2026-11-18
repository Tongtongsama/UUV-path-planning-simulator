import json
from xml.etree import ElementTree as ET

import pytest

from core import Pose
from planner import PlanningConstraints,PlanningRequest,EnvironmentPlanningSpace,validate_path
from ros2.twin_axial_current_double.mission import (CONFIG,Mission,config_for,environment_from_world,initial,
                                                    prepare,world_xml)


def test_double_baffle_world_and_path(tmp_path):
    out=prepare(tmp_path/'run','P0','F2')
    xml=(out/'worlds/minimal.world').read_text(encoding='utf-8')
    root=ET.fromstring(xml)
    names={m.get('name') for m in root.findall('./world/model')}
    assert {'baffle_lower','baffle_upper'}<=names
    plan=json.loads((out/'plan.json').read_text(encoding='utf-8'))
    assert plan['config']==config_for('F2')
    assert len(plan['config']['obstacles'])==2
    assert len(plan['path'])>=3
    current=json.loads((out/'current.json').read_text(encoding='utf-8'))
    mission=Mission(plan,xml,current)
    assert len(mission.world.obstacles)==2
    request=PlanningRequest(initial('P0'),Pose(x=18,z=-10),PlanningConstraints(.7,.8,goal_tolerance=0.))
    assert validate_path(mission.path,EnvironmentPlanningSpace(mission.world),request.constraints,request=request).valid


def test_plan_geometry_is_asserted_by_the_recording_config(tmp_path):
    out=prepare(tmp_path/'run','P0','F0')
    xml=(out/'worlds/minimal.world').read_text(encoding='utf-8')
    world=environment_from_world(xml)
    assert len(world.obstacles)==2
    assert world.obstacles[0].bounds==(6.,-13.,7.,-10.)
    assert world.obstacles[1].bounds==(13.,-10.,14.,-7.)
    assert plan_obstacles(out)==CONFIG['obstacles']


def plan_obstacles(out):
    plan=json.loads((out/'plan.json').read_text(encoding='utf-8'))
    return plan['config']['obstacles']


def test_plan_and_current_must_match(tmp_path):
    out=prepare(tmp_path/'run','P0','F2')
    xml=(out/'worlds/minimal.world').read_text(encoding='utf-8')
    current=json.loads((out/'current.json').read_text(encoding='utf-8'))
    plan=json.loads((out/'plan.json').read_text(encoding='utf-8'))
    bad=dict(plan); bad['config']=config_for('F0')
    with pytest.raises(ValueError,match='Plan/current mismatch'):
        Mission(bad,xml,current)
