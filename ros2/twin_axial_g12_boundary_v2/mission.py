"""Frozen G12 mission with registered current/model configuration only."""
import json
from .config import CASES as BOUNDARY_CASES,flow_context,model_config
from ros2.twin_axial_g12_real_v3 import mission as parent
from ros2.twin_axial_g12_real_v3.mission import CONFIG,initial,planning_pose,path_distance,segment_clearance
from ros2.twin_axial_v2.model import prepare as vessel_prepare
CASES=('P0',)

def make_plan(case,scene,flow):
 if case!='P0' or scene!='S2' or flow not in BOUNDARY_CASES:raise ValueError('Outside registered five cases')
 with flow_context(flow):plan=parent.make_plan(case,scene,flow)
 plan['boundary_configuration']=BOUNDARY_CASES[flow]
 plan['feasibility_basis']='Separate boundary Python evidence required; no nominal-matrix substitution'
 if BOUNDARY_CASES[flow]['damping_scale']!=1:
  plan['steady_certificate']['scope']='Nominal-damping diagnostic only; not a perturbed-damping certificate'
 return plan

def prepare(directory,case,scene='S2',gui=True,flow='U10'):
 plan=make_plan(case,scene,flow);output=vessel_prepare(directory)
 (output/'model.yml').write_text(json.dumps(model_config(case,flow),indent=2))
 (output/'plan.json').write_text(json.dumps(plan,indent=2))
 (output/'worlds/minimal.world').write_text(parent.world_xml(case,scene,gui))
 return output

class Mission(parent.Mission):
 def __init__(self,plan,xml):
  if plan!=make_plan(plan['case'],plan['scene'],plan['flow']):raise ValueError('Boundary plan drift')
  with flow_context(plan['flow']):
   original=parent.make_plan(plan['case'],plan['scene'],plan['flow'])
   super().__init__(original,xml)
 # update, guidance, limits, goal, PID and allocation are inherited unchanged.
