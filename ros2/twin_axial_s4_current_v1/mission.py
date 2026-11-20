"""S4 D path with the v10 measured-current schedule."""
import json
import ros2.twin_axial_s4_terminal_real_v1.mission as s4parent
import ros2.twin_axial_g12_real_v3.mission as v3
from ros2.twin_axial_g12_current_v10 import mission as v10
from ros2.twin_axial_g12_current_v10 import schedule
from ros2.twin_axial_g12_current_v10.mission import model_config as measured_model_config
from ros2.twin_axial_s4_terminal_real_v1.mission import Mission as S4Mission, make_plan, world_xml, CONFIG, model_config as s4_model_config
from ros2.twin_axial_navigation.mission import CASES, initial, planning_pose, path_distance, segment_clearance


def model_config(case, flow):
    return measured_model_config(case, flow) if flow == 'F2' else s4_model_config(case, flow)


class Mission(S4Mission):
    def __init__(self, plan, xml):
        super().__init__(plan, xml)
        if plan['flow'] == 'F2':
            self.config = {**self.config, 'current': list(schedule.current(0))}
            self.guide = v10.Guide(plan['path'], schedule.current(0), 'F2')

def prepare(directory,case,scene='S4',gui=True,flow='F2'):
    p=make_plan(case,scene,flow); xml=world_xml(case,scene,gui)
    out=v3.vessel_prepare(directory)
    (out/'model.yml').write_text(json.dumps(model_config(case,flow),indent=2),encoding='utf-8')
    (out/'worlds/minimal.world').write_text(xml,encoding='utf-8')
    (out/'plan.json').write_text(json.dumps(p,indent=2),encoding='utf-8')
    if flow == 'F2':
        (out/'current_schedule.json').write_text(json.dumps({'true_current':schedule.manifest(),'controller_current':schedule.manifest(),'estimated_current':schedule.manifest()},indent=2),encoding='utf-8')
    return out
