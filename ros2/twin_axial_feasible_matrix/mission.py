"""Independent mission adapter for the unchanged real transport/PID protocol."""
import json,math
import numpy as np
from core import Path,Pose
from ros2.twin_axial_navigation.mission import (CASES,CONFIG,model_config,initial,
    planning_pose,path_distance,segment_clearance,environment_from_world,obstacle_extents)
from ros2.twin_axial_navigation.current_model import FLOWS
from ros2.twin_axial_v2.model import prepare as vessel_prepare
from .scenes import world_xml,SCENES
from .guidance import CONFIG as GUIDE_CONFIG,build_path,executable_path,Guide

def prepare(directory,case,scene='S2',gui=False,flow='F2'):
    if scene not in SCENES:raise ValueError('Unknown scene')
    xml=world_xml(case,scene,gui);world=environment_from_world(xml)
    pos=initial(case).pose
    points,geometry=build_path(obstacle_extents(world),(pos.x,pos.z))
    feasibility=executable_path(points,FLOWS[flow])
    output=vessel_prepare(directory)
    (output/'model.yml').write_text(json.dumps(model_config(case,flow),indent=2))
    (output/'worlds/minimal.world').write_text(xml)
    plan=dict(path=points.tolist(),case=case,scene=scene,flow=flow,config=GUIDE_CONFIG,
              geometry=geometry,feasibility=feasibility)
    (output/'plan.json').write_text(json.dumps(plan,indent=2))
    return output

class Mission:
    def __init__(self,plan,xml):
        if plan['config']!=GUIDE_CONFIG or plan['scene'] not in SCENES:raise ValueError('Unknown revision/configuration')
        self.scene=plan['scene'];self.flow=plan['flow'];self.world=environment_from_world(xml)
        if obstacle_extents(self.world)!=SCENES[self.scene]:raise ValueError('Scene geometry mismatch')
        pos=initial(plan['case']).pose
        expected,_=build_path(obstacle_extents(self.world),(pos.x,pos.z))
        if not np.array_equal(expected,np.array(plan['path'])):raise ValueError('Unexpected generated path')
        executable_path(expected,FLOWS[self.flow])
        self.path=Path(tuple(Pose(x=x,z=z) for x,z in plan['path']))
        self.config={**GUIDE_CONFIG,'current':list(FLOWS[self.flow])}
        self.guide=Guide(expected,FLOWS[self.flow]);self.previous_pose=pos
        self.arrived=False;self.arrival_time=None;self.inside_since=None

    def update(self,record):
        p=planning_pose(record);s=record['solver_state_ned_frd'];t=record['elapsed_end']
        clearance=segment_clearance(self.world,self.previous_pose,p);self.previous_pose=p
        if clearance<.9-1e-6:raise RuntimeError('Swept safety clearance violated')
        if abs(p.pitch)>math.radians(75) or max(abs(s['u']),abs(s['w']))>1.5 or abs(s['q'])>.8:raise RuntimeError('State envelope violated')
        target,nav=self.guide.update([p.x,p.z,p.pitch,s['u'],s['w'],s['q']],t)
        if self.guide.arrived and not self.arrived:self.arrived=True;self.arrival_time=t
        self.inside_since=self.guide.hold
        return target,{**nav,'phase':'ARRIVED' if self.arrived else 'FEASIBLE_GUIDANCE',
            'elapsed':t,'position_xz':[p.x,p.z],'swept_clearance_m':clearance}
