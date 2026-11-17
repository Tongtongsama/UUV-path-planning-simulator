import json,math
import numpy as np
import pytest
from ros2.twin_axial_current.model import (
    FLOWS,model_config,configuration,telemetry,correction_expressions,rotation)
from ros2.twin_axial_navigation.mission import model_config as original_model
from ros2.twin_axial_navigation.mission import Mission as OriginalMission
from ros2.twin_axial_current.mission import prepare,Mission
from ros2.twin_axial_v2.numerical import initial_state
from ros2.twin_axial_v2.protocol import feedback
from physics.frames import world_current_to_body

def quaternion(phi,theta,psi):
    c1,c2,c3=[math.cos(x/2) for x in (phi,theta,psi)]
    s1,s2,s3=[math.sin(x/2) for x in (phi,theta,psi)]
    return dict(zip(('qr','qi','qj','qk'),(c1*c2*c3+s1*s2*s3,s1*c2*c3-c1*s2*s3,
                                        c1*s2*c3+s1*c2*s3,c1*c2*s3-s1*s2*c3)))

@pytest.mark.parametrize('flow',FLOWS)
def test_original_model_is_exact_prefix(flow):
    extended=model_config('P0',flow);original=original_model('P0')
    correction=extended['bodies'][0]['external forces'].pop()
    assert extended==original
    assert correction['name']=='constant_current_correction'

@pytest.mark.parametrize('flow',FLOWS)
@pytest.mark.parametrize('pitch',[-math.pi/3,0,math.pi/3])
def test_body_relative_velocity_and_coordinate_roundtrip(flow,pitch):
    s=initial_state();s.update(quaternion(0,pitch,math.pi/2));s.update(u=.3,w=-.1,q=.2)
    info=telemetry(s,flow)
    expected=world_current_to_body(np.array(FLOWS[flow]),pitch)
    assert np.allclose(np.array(info['current_body_frd_mps'])[[0,2]],expected,atol=1e-14)
    assert np.allclose(np.array(info['relative_water_body_frd_mps'])[[0,2]],[.3,-.1]-expected,atol=1e-14)
    assert np.allclose(rotation(s)@info['current_body_frd_mps'],configuration(flow)['ned_mps'],atol=1e-14)
    assert np.allclose(np.array(info['current_body_derivative_mps2'])[[0,2]],[-.2*expected[1],.2*expected[0]])

@pytest.mark.parametrize('flow',FLOWS)
def test_solver_expressions_against_independent_vector_formula(flow):
    # Exercise roll/yaw too: catches transposed rotations and missing rotating-current term.
    phi,theta,psi=.12,-.4,1.3
    state=initial_state();state.update(quaternion(phi,theta,psi));state.update(u=.3,v=-.07,w=-.1,p=.03,q=.2,r=-.05)
    env={k:state[k] for k in 'uvwpqr'};env.update(phi=phi,theta=theta,psi=psi,sin=math.sin,cos=math.cos,abs=abs)
    actual=np.array([eval(e.replace('(t)',''),{'__builtins__':{}},env) for e in correction_expressions(flow).values()])
    nu=np.array([state[k] for k in 'uvw']);omega=np.array([state[k] for k in 'pqr'])
    cur=rotation(state).T@configuration(flow)['ned_mps'];relative=nu-cur
    ma=np.array([5,5,8]);linear=np.array([4,6,6]);quad=np.array([1,1.5,1.5])
    translation=-np.cross(omega,ma*relative)+np.cross(omega,ma*nu)
    translation+=linear*(nu-relative)+quad*(abs(nu)*nu-abs(relative)*relative)-ma*np.cross(omega,cur)
    moment=-np.cross(relative,ma*relative)+np.cross(nu,ma*nu)
    assert np.allclose(actual,np.r_[translation,moment],atol=1e-12)

@pytest.mark.parametrize('flow',FLOWS)
def test_los_targets_unchanged_and_flow_saved(tmp_path,flow):
    output=prepare(tmp_path/'trial','P0',flow)
    plan=json.loads((output/'plan.json').read_text());xml=(output/'worlds/minimal.world').read_text()
    current=json.loads((output/'current.json').read_text())
    assert plan['config']['current']==list(FLOWS[flow])
    mission=Mission(plan,xml,current)
    plan['config']['current']=[0,0];base=OriginalMission(plan,xml)
    s=initial_state();s['t']=2
    record={'sequence':40,'elapsed_end':2,'solver_state_ned_frd':s,**feedback(s)}
    target,nav=mission.update(record);expected,old_nav=base.update(record)
    assert target==expected
    nav.pop('water');assert nav==old_nav

def test_invalid_current_label_rejected():
    with pytest.raises(KeyError):configuration('F3')
