import math
import json
from xml.etree import ElementTree as ET
import pytest
from ros2.minimal_xdyn.model import DT,KEYS,model,world
from ros2.minimal_xdyn.protocol import clamp,check_request,decode_reply,feedback,pulse,FIELDS
from ros2.minimal_xdyn.gateway import Gate

def state():
    return dict(zip(FIELDS,[0,0,0,30,1,0,0,0,0,0,0,0,0,0]))

def test_model_wrench_and_no_competing_integrator():
    body=model()['bodies'][0]
    forces={f.get('name'):f for f in body['external forces']}
    assert forces['fore']['M']=='-0.5*T' and forces['aft']['M']=='0.5*T'
    assert all(forces[n]['reference frame']['x']['value']==0 for n in ['surge','fore','aft'])
    root=ET.fromstring(world())
    assert root.find('.//waypoint_follower') is None
    assert root.findtext('.//max_step_size')=='0.05'
    assert root.find('.//underwater/uri').text.endswith(':12346')

def test_request_time_and_step_are_strict():
    req={'Dt':DT,'states':[state()]}
    assert check_request(req,0)['z']==30
    with pytest.raises(ValueError): check_request(req,.1)
    req['Dt']=.2
    with pytest.raises(ValueError): check_request(req)

def test_invalid_solver_response_never_becomes_feedback():
    reply={k:[v] for k,v in state().items()}; reply['t']=[DT]
    assert decode_reply(reply,0)['t']==DT
    reply['t']=[0]
    with pytest.raises(ValueError): decode_reply(reply,0)
    reply['t']=[DT]; reply['u']=[float('nan')]
    with pytest.raises(ValueError): decode_reply(reply,0)

def test_clamping_rejects_bad_and_missing_commands():
    assert list(clamp(dict(zip(KEYS,[100,-100,100]))).values())==[5,-2,2]
    with pytest.raises(ValueError): clamp({})
    with pytest.raises(ValueError): clamp(dict.fromkeys(KEYS,float('inf')))

def test_independent_watchdog_sim_and_wall_time():
    gate=Gate(); command=dict(zip(KEYS,[2,0,0]))
    assert gate.applied(0,0)[2]
    gate.update(command,10,1,100)
    assert gate.applied(10.1,100.1)==(command,1,False)
    assert all(v==0 for v in gate.applied(10.3,100.1)[0].values())
    assert all(v==0 for v in gate.applied(10.1,101.1)[0].values())
    with pytest.raises(ValueError): gate.update(command,9,2,101)

def test_pulses_are_finite_duration_and_pure_pitch_pair():
    for t in [0,1.999,4,8]: assert all(v==0 for v in pulse('pitch_positive',t).values())
    c=pulse('pitch_positive',3)
    assert c['fore(T)']+c['aft(T)']==0
    assert .5*(c['aft(T)']-c['fore(T)'])==.5
    down=pulse('heave_positive',3)
    assert down['fore(T)']+down['aft(T)']==1
    assert down['aft(T)']-down['fore(T)']==0

def test_ned_to_enu_heading_and_pitch():
    s=state(); s.update(x=2,y=4,z=30)
    f=feedback(s)
    assert f['position_enu_m']==[4,2,-30]
    q=f['orientation_enu_flu_wxyz']
    assert abs(2*(q[0]*q[3]+q[1]*q[2]))==pytest.approx(1) # NED north becomes ENU +Y
    s.update(qr=math.cos(.1),qj=math.sin(.1))
    assert feedback(s)['pitch_nose_up_rad']==pytest.approx(.2)

def test_missing_evidence_cannot_pass(tmp_path):
    from ros2.minimal_xdyn.acceptance import analyze_run
    assert not analyze_run(tmp_path)['passed']
    (tmp_path/'manifest.json').write_text(json.dumps({'missing.log':'anything'}))
    assert not analyze_run(tmp_path)['checks']['recording_hashes']

def test_repeatability_rejects_trajectory_change(tmp_path):
    from ros2.minimal_xdyn.acceptance import compare_repeat
    def write(name,offset):
        path=tmp_path/name; path.mkdir()
        records=[{'elapsed_end':i*.05,'valid':True,'position_enu_m':[i*.001+offset,0,-30],
            'solver_state_ned_frd':dict.fromkeys(('u','v','w','p','q','r'),0)} for i in range(1,161)]
        (path/'transactions.jsonl').write_text('\n'.join(map(json.dumps,records)))
        return path
    first=write('first',0)
    assert compare_repeat(first,write('same',0))['passed']
    assert not compare_repeat(first,write('different',.03))['passed']
