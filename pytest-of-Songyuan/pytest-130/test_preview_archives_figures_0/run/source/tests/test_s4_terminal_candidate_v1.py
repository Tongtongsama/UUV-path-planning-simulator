import numpy as np
from ros2.twin_axial_s4_terminal_candidate_v1.cli import make_plan,SPEC

def test_terminal_constraint_survives_shortcuts():
    p=make_plan();assert p['anchors'][-2:]==[[14.,-10.],[18.,-10.]]
    i=p['path'].index([14.,-10.]);terminal=np.array(p['path'][i:])
    assert np.all(terminal[:,1]==-10.)
    assert np.all(np.diff(terminal[:,0])>0)
    assert terminal[-1,0]-terminal[0,0]==4.
    assert p['geometry']['min_clearance_m']>=1.5

def test_original_goal_and_segment_sampling_preserved():
    p=np.array(make_plan()['path'])
    assert p[0].tolist()==[2.,-10.] and p[-1].tolist()==[18.,-10.]
    assert np.max(np.linalg.norm(np.diff(p,axis=0),axis=1))<=.05+1e-12
    assert SPEC['order']==['F0','F1','F2']
