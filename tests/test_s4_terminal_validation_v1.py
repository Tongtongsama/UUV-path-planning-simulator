from ros2.twin_axial_s4_terminal_validation_v1.cli import plan,NEW,KEYS,PARENT,read,candidate
from ros2.twin_axial_navigation.mission import INITIALS

def test_exact_fifteen_new_runs():
    assert len(NEW)==len(set(NEW))==15 and len(KEYS)==18
    assert all(not k.endswith('/P0') for k in NEW)
    assert all(f'{f}/P0_repeat' in NEW for f in ('F0','F1','F2'))

def test_reference_path_and_restoration():
    original=candidate.initial
    assert plan('P0')['path']==read(PARENT/'plan.json')['path']
    assert plan('P0_repeat')==plan('P0')
    plan('P2')
    assert candidate.initial is original

def test_all_initials_retain_terminal_constraint():
    for case,state in INITIALS.items():
        p=plan(case)
        assert p['path'][0]==list(state[:2])
        assert p['anchors'][-2:]==[[14.,-10.],[18.,-10.]]
        i=p['path'].index([14.,-10.])
        assert all(z==-10. for x,z in p['path'][i:])
        assert p['geometry']['min_clearance_m']>=1.5
