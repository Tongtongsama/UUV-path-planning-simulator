import math
from xml.etree import ElementTree as ET
import numpy as np
import pytest
from ros2.twin_axial_v2.allocation import allocate, wrench
from ros2.twin_axial_v2.model import model, world, KEYS
from ros2.twin_axial_v2.gateway import Gate
from ros2.twin_axial_v2.protocol import feedback, check_request, decode_reply, pulse, FIELDS, DT

def state():
    return dict(zip(FIELDS,[0,0,2,30,math.sqrt(.5),0,0,math.sqrt(.5),0,0,0,0,0,0]))

def test_allocation_joint_domain_and_direction():
    for x in [-100,-30,-2,0,2,30,100]:
        for m in [-30,-7.5,-.5,0,.5,7.5,30]:
            a=allocate({'X':x,'M':m}); w=wrench(a['allocated_commands'])
            assert max(map(abs,a['allocated_commands'].values()))<=15+1e-12
            assert w['Z']==0
            assert w['X']==pytest.approx(x*a['alpha'])
            assert w['M']==pytest.approx(m*a['alpha'])
            assert abs(w['X'])+4*abs(w['M'])<=30+1e-9
    assert allocate({'X':20,'M':5})['alpha']==.75

@pytest.mark.parametrize('command',[{}, {'X':0,'M':0,'Z':1},{'X':float('nan'),'M':0},{'X':True,'M':0},{'X':1e308,'M':1e308}])
def test_invalid_request_rejected(command):
    with pytest.raises(ValueError): allocate(command)

def test_model_actual_input_rank_and_moment_arms():
    f={v.get('name'):v for v in model()['bodies'][0]['external forces']}
    assert 'fore' not in f and 'surge' not in f
    columns=[]
    for name in ['upper','lower']:
        assert all(f[name]['reference frame'][a]['value']==0 for a in ['x','y','z'])
        columns.append([eval(f[name][axis],{'__builtins__':{}},{'T':1}) for axis in ['X','Z','M']])
    b=np.array(columns).T
    assert np.linalg.matrix_rank(b)==2
    assert np.allclose(b,[[1,1],[0,0],[-.25,.25]])
    tree=ET.fromstring(world())
    assert tree.findtext('.//model/pose')=='2 0 -30 0 0 0'
    assert {e.text for e in tree.findall('.//control_surfaces/*')}==set(KEYS)

def test_watchdog_frozen_stamp_and_wall_time():
    g=Gate(); assert g.applied(0,0)[2]
    g.update({'X':2,'M':0},1,1,10)
    assert not g.applied(1.25,10.25)[2]
    assert g.applied(1.30,10.30)[2]
    g.update({'X':2,'M':0},1,2,10.31)
    assert g.applied(1.35,10.32)[2]  # repeated packets do not renew simulation authority
    assert g.applied(1.1,11.32)[2]   # paused sim / wall-clock expiry
    assert g.applied(.9,10.32)[2]    # future-dated command
    with pytest.raises(ValueError): g.update({'X':2,'M':0},1,2,10.4)
    with pytest.raises(ValueError): g.update({'X':2,'M':0},.9,3,10.4)

def test_frame_mapping_east_heading_and_nose_up():
    s=state(); f=feedback(s)
    assert f['position_enu_m']==[2,0,-30]
    assert f['position_planning_xz_m']==[2,-10]
    assert abs(f['orientation_enu_flu_wxyz'][0])==pytest.approx(1)
    # NED orientation yaw=pi/2, pitch=0.2; ENU/FLU pitch has opposite sign.
    c=math.cos(.1)*math.sqrt(.5); q=math.sin(.1)*math.sqrt(.5)
    s.update(qr=c,qi=-q,qj=q,qk=c)
    f=feedback(s); assert f['pitch_nose_up_rad']==pytest.approx(.2)
    w,x,y,z=f['orientation_enu_flu_wxyz']
    assert math.asin(2*(w*y-z*x))==pytest.approx(-.2)

def test_time_and_invalid_solver_evidence():
    s=state(); check_request({'states':[s],'Dt':DT},0)
    with pytest.raises(ValueError): check_request({'states':[s],'Dt':DT},.1)
    r={k:[v] for k,v in s.items()}; r['t']=[DT]
    decode_reply(r,0)
    r['u']=[float('nan')]
    with pytest.raises(ValueError): decode_reply(r,0)

def test_schedules_and_missing_evidence(tmp_path):
    from ros2.twin_axial_v2.acceptance import analyze_run
    assert not analyze_run(tmp_path)['passed']
    for name in ['surge_positive','pitch_positive','saturation']:
        assert pulse(name,1)=={'X':0,'M':0}
        assert pulse(name,4)=={'X':0,'M':0}
    assert wrench(allocate(pulse('pitch_positive',3))['allocated_commands'])=={'X':0,'Z':0,'M':.5}

@pytest.mark.parametrize('fault',['none','time_jump','wrong_gui_attitude','uncommanded_motion'])
def test_acceptance_uses_physical_evidence_not_only_hashes(tmp_path,fault):
    import json,hashlib
    from ros2.twin_axial_v2.acceptance import analyze_run
    rows=[]; observations=[]
    for i in range(600):
        s=state(); s['t']=(i+1)*DT
        r={'valid':True,'start_time':i*DT,'elapsed_start':i*DT,'elapsed_end':s['t'],
           'solver_state_ned_frd':s,'applied_commands':dict.fromkeys(KEYS,0.0),
           'applied_wrench':{'X':0.0,'Z':0.0,'M':0.0},'watchdog_zero':True,
           'allocation':allocate({'X':0,'M':0}),'command_simulation_stamp':-1e30,
           'command_received_wall':-1e30,'wall_monotonic':i*DT,**feedback(s)}
        rows.append(r)
        observations.append({'time':i*DT,'position':[2,0,-30],'quaternion_xyzw':[0,0,0,1]})
    if fault=='time_jump': rows[100]['solver_state_ned_frd']['t']+=.02
    if fault=='wrong_gui_attitude':
        for o in observations: o['quaternion_xyzw']=[math.sqrt(.5),0,0,math.sqrt(.5)]
    if fault=='uncommanded_motion': rows[100]['solver_state_ned_frd']['w']=.1
    (tmp_path/'transactions.jsonl').write_text('\n'.join(map(json.dumps,rows)))
    (tmp_path/'gazebo_poses.jsonl').write_text('\n'.join(map(json.dumps,observations)))
    (tmp_path/'commands.jsonl').write_text('')
    (tmp_path/'model.yml').write_text('{}')
    (tmp_path/'metadata.json').write_text(json.dumps({'case':'zero'}))
    (tmp_path/'run_status.json').write_text(json.dumps({'status':'completed','cleanup_error':None,
        'process_exit_codes':[{'returncode':0}]*3}))
    (tmp_path/'manifest.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in tmp_path.iterdir()}))
    report=analyze_run(tmp_path)
    assert report['checks']['recording_hashes']
    assert report['passed']==(fault=='none')


def test_added_mass_wrench_matches_cross_product_and_does_no_work():
    forces=model()['bodies'][0]['external forces']
    force=next(f for f in forces if f.get('name')=='added_mass_coriolis')
    assert 'commands' not in force
    rng=np.random.default_rng(46)
    for nu in rng.normal(size=(100,6)):
        velocity=nu[:3]; omega=nu[3:]
        momentum=np.array([5,5,8])*velocity
        angular=2*omega
        expected=np.r_[-np.cross(omega,momentum),
                       -np.cross(velocity,momentum)-np.cross(omega,angular)]
        env=dict(zip('uvwpqr',nu))
        actual=np.array([eval(force[a].replace('(t)',''),{'__builtins__':{}},env) for a in ('X','Y','Z','K','M','N')])
        assert np.allclose(actual,expected,atol=1e-12)
        assert abs(nu@actual)<1e-12


def test_longitudinal_coupling_matches_original_python():
    from physics.uuv_3dof import added_mass_coriolis
    from simulation.vehicle_config import load_synthetic_parameters
    force=next(f for f in model()['bodies'][0]['external forces'] if f.get('name')=='added_mass_coriolis')
    for u,w,q in [(0.5,.2,-.1),(-.3,.4,.2),(.1,-.2,.3)]:
        env={'u':u,'v':0,'w':w,'p':0,'q':q,'r':0}
        actual=[eval(force[a].replace('(t)',''),{'__builtins__':{}},env) for a in ('X','Z','M')]
        expected=-added_mass_coriolis(load_synthetic_parameters(),np.array([u,w,q]))@np.array([u,w,q])
        assert np.allclose(actual,expected)


def test_v1_stays_distinct_and_no_extra_actuator():
    from ros2.twin_axial.model import model as old_model
    old=old_model()['bodies'][0]; new=model()['bodies'][0]
    assert old['name']=='uuv_twin_axial_v1'
    assert new['name']=='uuv_twin_axial_v2'
    assert not any(f.get('name')=='added_mass_coriolis' for f in old['external forces'])
    assert [f['commands'] for f in new['external forces'] if 'commands' in f]==[['T'],['T']]
