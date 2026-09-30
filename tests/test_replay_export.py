"""Replay is a projection of evidence, never a second simulation."""
from copy import deepcopy
import pytest
from visualization.replay_export import pack_arm


def fixture():
    states=[dict(timestamp=t,pose=dict(x=t,z=-5,pitch=t),twist=dict(u=.2,w=0,q=1)) for t in (0.,.05,.1)]
    controls=[dict(time=t,applied=dict(tau_x=1,tau_m=2),guidance_phase='PATH_FOLLOWING',
                   capture_phase='DISABLED',startup_phase='DISABLED',collision=False,in_bounds=True,segment_clearance=.4) for t in (0.,.05)]
    return dict(history=dict(states=states,controls=controls),summary=dict(status='SUCCESS',metrics={}))


def test_preserves_original_samples_and_does_not_mutate_evidence():
    data=fixture();old=deepcopy(data);out=pack_arm(data)
    assert out['samples'][1]==[.05,.05,-5,.05,.2,0,1]
    assert data==old and out['end']==.1


def test_collision_is_detected_at_step_end_not_invented_contact_time():
    data=fixture();data['history']['controls'][0]['collision']=True
    out=pack_arm(data)
    assert out['collision']==dict(step=0,interval=[0.,.05],detected_at=.05)
    assert out['end']==.05 and len(out['samples'])==2 and len(out['controls'])==1
    assert out['status']=='COLLISION'


@pytest.mark.parametrize('fault',['length','nan','time','clock'])
def test_invalid_log_rejected(fault):
    data=fixture()
    if fault=='length':data['history']['controls'].pop()
    if fault=='nan':data['history']['states'][0]['pose']['pitch']=float('nan')
    if fault=='time':data['history']['states'][1]['timestamp']=0.
    if fault=='clock':data['history']['controls'][0]['time']=.03
    with pytest.raises(ValueError):pack_arm(data)
