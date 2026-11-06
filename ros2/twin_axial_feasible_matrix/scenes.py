"""Documented S1/S2/S3 geometries without mutating baseline registries."""
import math
from xml.etree import ElementTree as ET
from ros2.twin_axial_navigation.mission import world_xml as baseline_world
from ros2.twin_axial_navigation.mission import INITIALS,environment_from_world,obstacle_extents
from ros2.twin_axial_navigation.current_model import FLOWS
from .guidance import build_path,executable_path,bottleneck,CONFIG

SCENES={'S1':[[9,10,-20,-8]],'S2':[[6,7,-20,-7],[13,14,-13,0]],
        'S3':[[6,7,-20,-5.5],[13,14,-14.5,0]]}

def world_xml(case,scene='S2',gui=False):
    root=ET.fromstring(baseline_world(case,'S2',gui));world=root.find('world')
    nodes=[n for n in world.findall('model') if n.get('name','').startswith('double_baffle_obstacle_')]
    for i,node in enumerate(nodes):
        if i>=len(SCENES[scene]):world.remove(node);continue
        xmin,xmax,zmin,zmax=SCENES[scene][i]
        node.find('pose').text=f'{(xmin+xmax)/2} 0 {(zmin+zmax)/2-20} 0 0 0'
        for size in node.findall('./link/*/geometry/box/size'):size.text=f'{xmax-xmin} 4 {zmax-zmin}'
    title=world.find('./gui/plugin/gz-gui/title')
    if title is not None:title.text=f'{scene} live UUV dynamics'
    return ET.tostring(root,encoding='unicode')

def preflight(scene,case,flow):
    obstacles=obstacle_extents(environment_from_world(world_xml(case,scene)))
    assert obstacles==SCENES[scene]
    report=dict(scene=scene,case=case,flow=flow,config=CONFIG,passed=False,dynamics_started=False,obstacles=obstacles)
    if len(obstacles)==2:
        bound=bottleneck(obstacles);theta=-math.radians(70);vx,vz=FLOWS[flow]
        bound['max_steady_descent_at_cruise_deg']=-math.degrees(theta-math.atan2(vx*math.sin(theta)-vz*math.cos(theta),CONFIG['cruise_speed']))
        report['bottleneck']=bound
    try:
        points,geometry=build_path(obstacles,INITIALS[case][:2]);report['geometry']=geometry;report['path']=points.tolist()
        report['steady_trim']=executable_path(points,FLOWS[flow]);report['passed']=True
    except ValueError as e:report['reason']=str(e)
    return report
