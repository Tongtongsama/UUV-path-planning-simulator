"""Independent perturbations; never write or replace a frozen model."""
from contextlib import contextmanager
from dataclasses import replace
from unittest.mock import patch
from ros2.twin_axial_navigation import current_model
from ros2.twin_axial_navigation.mission import model_config as nominal_model_config
from simulation.vehicle_config import load_synthetic_parameters

CASES = {
 'U10': {'current': [0., .10], 'damping_scale': 1.},
 'U15': {'current': [0., .15], 'damping_scale': 1.},
 'U20': {'current': [0., .20], 'damping_scale': 1.},
 'D80': {'current': [0., 0.], 'damping_scale': .8},
 'D120': {'current': [0., 0.], 'damping_scale': 1.2},
}

def parameters(case):
 p=load_synthetic_parameters();scale=CASES[case]['damping_scale']
 return replace(p,damping_linear=p.damping_linear*scale,damping_quadratic=p.damping_quadratic*scale)

@contextmanager
def flow_context(case):
 # Only an isolated experiment process gets a new name. Existing F0/F1/F2 stay identical.
 if case not in CASES:raise ValueError('Unregistered boundary case')
 with patch.dict(current_model.FLOWS,{case:tuple(CASES[case]['current'])}):yield

def model_config(initial,case):
 if initial!='P0':raise ValueError('Only registered P0 is permitted')
 with flow_context(case):
  data=nominal_model_config(initial,case if CASES[case]['current']!=[0.,0.] else 'F0')
 scale=CASES[case]['damping_scale']
 if scale!=1:
  for force in data['bodies'][0]['external forces'][:2]:
   key='damping matrix at the center of gravity projected in the body frame'
   force[key]={row:[v*scale for v in values] for row,values in force[key].items()}
 return data
