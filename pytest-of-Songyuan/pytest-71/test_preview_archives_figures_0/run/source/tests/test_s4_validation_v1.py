import numpy as np
from ros2.twin_axial_s4_validation_v1.cli import plan,NEW,KEYS,read,REFERENCE
from ros2.twin_axial_g12_matrix.cli import repeatability
from ros2.twin_axial_navigation.mission import INITIALS

def test_exact_seventeen_new_entries():
    assert len(NEW)==17 and len(set(NEW))==17 and len(KEYS)==18
    assert 'F0/P0' not in NEW
    assert all(f'{f}/P0_repeat' in NEW for f in ('F0','F1','F2'))

def test_b_reproduces_frozen_reference_geometry():
    assert plan('P0')['path']==read(REFERENCE/'B_safe_shortcut/plan.json')['path']
    assert plan('P0_repeat')==plan('P0')

def test_initial_positions_and_planning_clearance():
    for case,pose in INITIALS.items():
        result=plan(case)
        assert np.allclose(result['path'][0],pose[:2],rtol=0,atol=0)
        assert result['path'][-1]==[18.,-10.]
        assert result['geometry']['min_clearance_m']>=1.5

def test_identical_failed_repeat_cannot_pass():
    records=[{'t':.05,'state':[2.,-10.,0.,0.,0.,0.]}]
    report={'passed':False,'record_verified':True,'metrics':{'arrival_time':None}}
    assert not repeatability(records,records,report,report)['passed']
