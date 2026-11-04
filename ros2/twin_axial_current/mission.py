"""Unchanged S1 planning/LOS targets; add current model and diagnostics only."""
import json
from copy import deepcopy
from ros2.twin_axial_navigation.mission import (
    CASES,CONFIG,initial,planning_pose,path_distance,segment_clearance,
    prepare as base_prepare,Mission as BaseMission)
from .model import model_config,configuration,telemetry

def prepare(directory,case,flow):
    output=base_prepare(directory,case)
    (output/'model.yml').write_text(json.dumps(model_config(case,flow),indent=2),encoding='utf-8')
    (output/'current.json').write_text(json.dumps(configuration(flow),indent=2),encoding='utf-8')
    plan=json.loads((output/'plan.json').read_text())
    plan['config']['current']=configuration(flow)['planning_xz_mps']
    (output/'plan.json').write_text(json.dumps(plan,indent=2),encoding='utf-8')
    return output

class Mission(BaseMission):
    def __init__(self,plan,xml,current):
        if current!=configuration(current['flow']):raise ValueError('Invalid current configuration')
        if plan['config']['current']!=current['planning_xz_mps']:raise ValueError('Plan/current mismatch')
        # Base LOS is current-unaware; only its fixed geometric settings matter.
        base_plan=deepcopy(plan);base_plan['config']['current']=CONFIG['current']
        super().__init__(base_plan,xml)
        self.flow=current['flow']

    def update(self,record):
        target,nav=super().update(record)
        nav['water']=telemetry(record['solver_state_ned_frd'],self.flow)
        return target,nav
