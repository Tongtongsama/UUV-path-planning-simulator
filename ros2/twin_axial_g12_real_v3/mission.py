"""Real feedback adapter for the frozen G12 guide; dynamics remain in xdyn."""
import json
import math
import numpy as np
from core import Path, Pose
from ros2.twin_axial_navigation.mission import (
    CASES, CONFIG, model_config, initial, planning_pose, path_distance,
    segment_clearance, environment_from_world, obstacle_extents)
from ros2.twin_axial_navigation.current_model import FLOWS
from ros2.twin_axial_v2.model import prepare as vessel_prepare
from ros2.twin_axial_feasible_matrix.scenes import SCENES, world_xml, preflight
from ros2.twin_axial_feasible_matrix.guidance import build_path
from ros2.twin_axial_s3_transient_v2.validate import g12_config
from ros2.twin_axial_s3_transient import study

GUIDE_CONFIG = g12_config()


class Guide(study.base_guidance.Guide):
    def update(self,state,t):
        with study.patched_config(GUIDE_CONFIG):
            return super().update(state,t)


def make_plan(case,scene,flow):
    if scene not in SCENES or case not in CASES or flow not in FLOWS:
        raise ValueError('Unknown scene, initial or flow')
    with study.patched_config(GUIDE_CONFIG):
        pos=initial(case).pose
        points,geometry=build_path(SCENES[scene],(pos.x,pos.z))
        certificate=preflight(scene,case,flow)
    return dict(path=points.tolist(),case=case,scene=scene,flow=flow,
                config=dict(GUIDE_CONFIG),geometry=geometry,
                steady_certificate={k:v for k,v in certificate.items() if k not in ('path','config')},
                feasibility_basis='Frozen full 54-run dynamic Python evidence; steady certificate retained as diagnostic')


def prepare(directory,case,scene='S3',gui=True,flow='F0'):
    plan=make_plan(case,scene,flow)
    xml=world_xml(case,scene,gui)
    if obstacle_extents(environment_from_world(xml))!=SCENES[scene]:
        raise ValueError('SDF geometry mismatch')
    output=vessel_prepare(directory)
    (output/'model.yml').write_text(json.dumps(model_config(case,flow),indent=2),encoding='utf-8')
    (output/'worlds/minimal.world').write_text(xml,encoding='utf-8')
    (output/'plan.json').write_text(json.dumps(plan,indent=2),encoding='utf-8')
    return output


class Mission:
    def __init__(self,plan,xml):
        expected=make_plan(plan['case'],plan['scene'],plan['flow'])
        if plan!=expected:
            raise ValueError('Plan/configuration differs from frozen G12')
        self.scene=plan['scene'];self.flow=plan['flow'];self.world=environment_from_world(xml)
        if obstacle_extents(self.world)!=SCENES[self.scene]:
            raise ValueError('Scene geometry mismatch')
        self.path=Path(tuple(Pose(x=x,z=z) for x,z in plan['path']))
        self.config={**GUIDE_CONFIG,'current':list(FLOWS[self.flow])}
        self.guide=Guide(plan['path'],FLOWS[self.flow])
        self.previous_pose=initial(plan['case']).pose
        self.arrived=False;self.arrival_time=None;self.inside_since=None

    def update(self,record):
        p=planning_pose(record);s=record['solver_state_ned_frd'];t=record['elapsed_end']
        clearance=segment_clearance(self.world,self.previous_pose,p);self.previous_pose=p
        if clearance<.9-1e-6:raise RuntimeError('Swept safety clearance violated')
        if abs(p.pitch)>math.radians(75) or max(abs(s['u']),abs(s['w']))>1.5 or abs(s['q'])>.8:
            raise RuntimeError('State envelope violated')
        target,nav=self.guide.update([p.x,p.z,p.pitch,s['u'],s['w'],s['q']],t)
        if self.guide.arrived and not self.arrived:
            self.arrived=True;self.arrival_time=t
        self.inside_since=self.guide.hold
        return target,{**nav,'phase':'ARRIVED' if self.arrived else 'FEASIBLE_GUIDANCE',
                      'elapsed':t,'position_xz':[p.x,p.z],'swept_clearance_m':clearance}
