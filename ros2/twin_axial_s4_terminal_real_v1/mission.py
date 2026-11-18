import copy
import json
from unittest.mock import patch
from xml.etree import ElementTree as ET
from ros2.twin_axial_g12_real_v3 import mission as parent
from ros2.twin_axial_g12_real_v3.mission import CONFIG,CASES,model_config,initial,planning_pose,path_distance,segment_clearance
from ros2.twin_axial_navigation.mission import world_xml as baseline_world
from ros2.twin_axial_s4_terminal_validation_v1.cli import plan as python_plan
from ros2.twin_axial_s4_planning_v1.study import SPEC

def world_xml(case,scene='S4',gui=True):
    if scene!='S4':raise ValueError('Only S4 supported')
    root=ET.fromstring(baseline_world(case,'S2',gui));world=root.find('world')
    old=[n for n in world.findall('model') if n.get('name','').startswith('double_baffle_obstacle_')]
    template=copy.deepcopy(old[0])
    for n in old:world.remove(n)
    for i,(a,b,c,d) in enumerate(SPEC['obstacles_xmin_xmax_zmin_zmax']):
        node=copy.deepcopy(template);node.set('name',f'double_baffle_obstacle_{i}')
        node.find('pose').text=f'{(a+b)/2} 0 {(c+d)/2-20} 0 0 0'
        for size in node.findall('./link/*/geometry/box/size'):size.text=f'{b-a} 4 {d-c}'
        world.append(node)
    title=world.find('./gui/plugin/gz-gui/title')
    if title is not None:title.text='S4 terminal candidate: live UUV dynamics'
    return ET.tostring(root,encoding='unicode')

def make_plan(case,scene,flow):
    if scene!='S4' or case not in CASES or flow not in parent.FLOWS:raise ValueError('Unknown S4 case/flow')
    p=python_plan(case)
    return {'case':case,'scene':scene,'flow':flow,'path':p['path'],'geometry':p['geometry'],
            'config':dict(parent.GUIDE_CONFIG),'feasibility_basis':'18 bound Python records; no real acceptance yet'}

class Mission(parent.Mission):
    def __init__(self,plan,xml):
        with patch.object(parent,'make_plan',make_plan),patch.dict(parent.SCENES,{'S4':SPEC['obstacles_xmin_xmax_zmin_zmax']}):
            super().__init__(plan,xml)
    # update and Guide are inherited unchanged.

def prepare(directory,case,scene='S4',gui=True,flow='F0'):
    p=make_plan(case,scene,flow);xml=world_xml(case,scene,gui)
    assert parent.obstacle_extents(parent.environment_from_world(xml))==SPEC['obstacles_xmin_xmax_zmin_zmax']
    out=parent.vessel_prepare(directory)
    (out/'model.yml').write_text(json.dumps(model_config(case,flow),indent=2),encoding='utf-8')
    (out/'worlds/minimal.world').write_text(xml,encoding='utf-8')
    (out/'plan.json').write_text(json.dumps(p,indent=2),encoding='utf-8')
    return out
