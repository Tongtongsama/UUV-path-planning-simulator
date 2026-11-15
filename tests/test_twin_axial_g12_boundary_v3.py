import pytest
from ros2.twin_axial_g12_boundary_v3.gate import replay_equal

def test_cross_platform_roundoff_only():
 assert replay_equal({'checks':{'passed':False},'x':[.3,0.]},{'checks':{'passed':False},'x':[.3+1e-14,1e-14]})
 assert not replay_equal({'checks':{'passed':False}},{'checks':{'passed':True}})
 assert not replay_equal([.35],[.35001])
 assert not replay_equal(float('nan'),float('nan'))
 assert not replay_equal('abc','def')

def test_v3_retains_control_and_original_acceptance_limits():
 import ast
 from pathlib import Path
 def nodes(p):return {n.name:ast.dump(n,include_attributes=False) for n in ast.walk(ast.parse(Path(p).read_text())) if isinstance(n,ast.FunctionDef)}
 a=nodes('ros2/twin_axial_g12_link_v6/trial.py');b=nodes('ros2/twin_axial_g12_boundary_v3/trial.py')
 for n in ('process','publish','stop_process'):assert a[n]==b[n]
 a=nodes('ros2/twin_axial_g12_boundary_v2/acceptance.py');b=nodes('ros2/twin_axial_g12_boundary_v3/acceptance.py')
 assert a==b
