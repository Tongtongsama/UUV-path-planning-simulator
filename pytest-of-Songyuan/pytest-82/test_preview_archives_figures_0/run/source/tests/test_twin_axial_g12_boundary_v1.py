import ast
from pathlib import Path
import numpy as np
import pytest
from ros2.twin_axial_g12_boundary_v1.config import CASES,parameters,model_config,flow_context
from ros2.twin_axial_g12_boundary_v1.mission import make_plan,Mission
from ros2.twin_axial_g12_real_v3 import mission as parent
from ros2.twin_axial_navigation.current_model import FLOWS
from ros2.twin_axial_v2.model import model
from simulation.vehicle_config import load_synthetic_parameters

@pytest.mark.parametrize('name',CASES)
def test_registered_model_changes_only_current_or_all_damping(name):
 original=model();actual=model_config('P0',name)
 if name.startswith('D'):
  forces=actual['bodies'][0]['external forces'];base=original['bodies'][0]['external forces']
  assert forces[2:]==base[2:]
  key='damping matrix at the center of gravity projected in the body frame'
  for i in (0,1):
   for row,v in base[i][key].items():
    assert forces[i][key][row]==[x*CASES[name]['damping_scale'] for x in v]
  actual['bodies'][0]['external forces'][:2]=base[:2]
  assert actual==original
 else:
  # Current uses the existing relative-water force builder; all other model fields match.
  a=actual['bodies'][0];b=original['bodies'][0]
  assert a['external forces'][1:4]==b['external forces'][2:5]
  a['external forces']=b['external forces'];assert actual==original
 p=parameters(name);q=load_synthetic_parameters()
 assert np.array_equal(p.mass,q.mass)
 assert np.array_equal(p.damping_linear,q.damping_linear*CASES[name]['damping_scale'])
 assert np.array_equal(p.damping_quadratic,q.damping_quadratic*CASES[name]['damping_scale'])

@pytest.mark.parametrize('name',CASES)
def test_plan_unchanged_geometry_and_g12_and_no_registry_leak(name):
 before=FLOWS.copy();baseline=parent.make_plan('P0','S2','F0');plan=make_plan('P0','S2',name)
 assert plan['path']==baseline['path'] and plan['config']==baseline['config']
 mission=Mission(plan,parent.world_xml('P0','S2',True))
 assert mission.config['current']==CASES[name]['current']
 assert FLOWS==before and name not in FLOWS
 with pytest.raises(ValueError):make_plan('P1','S2',name)

def test_live_control_and_cleanup_ast_unchanged():
 def functions(path):
  return {n.name:ast.dump(n,include_attributes=False) for n in ast.walk(ast.parse(Path(path).read_text())) if isinstance(n,ast.FunctionDef)}
 a=functions('ros2/twin_axial_g12_link_v6/trial.py');b=functions('ros2/twin_axial_g12_boundary_v1/trial.py')
 for n in ('process','publish','stop_process','state_cb','pose_cb','wait_ready','solver_ready'):assert a[n]==b[n]
 assert Mission.update is parent.Mission.update

def test_scope_restores_on_exception():
 before=FLOWS.copy()
 with pytest.raises(RuntimeError):
  with flow_context('U10'):raise RuntimeError('test')
 assert FLOWS==before

def test_unknown_configs_refused():
 with pytest.raises(ValueError):model_config('P1','D80')
 with pytest.raises(ValueError):
  with flow_context('U25'):pass
