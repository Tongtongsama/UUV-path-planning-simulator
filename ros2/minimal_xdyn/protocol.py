"""Checked co-simulation transactions and ideal actuator limits, no integrator."""
import math
from .model import DT, KEYS, LIMITS

FIELDS=('t','x','y','z','qr','qi','qj','qk','u','v','w','p','q','r')


def finite(value):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
        raise ValueError('Expected finite numeric value')
    return float(value)


def clamp(commands):
    if set(commands)!=set(KEYS): raise ValueError('Exactly surge(T), fore(T), aft(T) required')
    return {k:max(-limit,min(limit,finite(commands[k]))) for k,limit in zip(KEYS,LIMITS)}


def check_request(request, previous_end=None):
    dt=finite(request['Dt'])
    if abs(dt-DT)>1e-8: raise ValueError(f'Expected Dt={DT}, received {dt}')
    if len(request['states'])!=1: raise ValueError('Single-body bench requires one state')
    state={k:finite(request['states'][0][k]) for k in FIELDS}
    if previous_end is not None and abs(state['t']-previous_end)>1e-6:
        raise ValueError('Non-contiguous request simulation time; restart the whole trial')
    return state


def decode_reply(reply, start):
    if 'error' in reply: raise ValueError(str(reply['error']))
    values={k:finite(reply[k][-1]) for k in FIELDS}
    if abs(values['t']-(start+DT))>1e-6: raise ValueError('Solver end time is not start + Dt')
    norm=sum(values[k]**2 for k in ('qr','qi','qj','qk'))
    if abs(norm-1)>1e-3: raise ValueError('Solver quaternion not unit length')
    if not 15 < values['z'] < 45: raise ValueError('Depth outside bench envelope')
    if max(abs(values[k]) for k in ('u','v','w'))>3 or max(abs(values[k]) for k in ('p','q','r'))>1:
        raise ValueError('Velocity outside bench envelope')
    return values


def feedback(state):
    # q_ENU_FLU = q_ENU_NED * q_NED_FRD * q_FRD_FLU (Hamilton wxyz).
    def mul(a,b):
        w,x,y,z=a; W,X,Y,Z=b
        return [w*W-x*X-y*Y-z*Z,w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W]
    s=math.sqrt(.5)
    quat=mul(mul([0,s,s,0],[state[k] for k in ('qr','qi','qj','qk')]),[0,1,0,0])
    return {'position_enu_m':[state['y'],state['x'],-state['z']],
        'orientation_enu_flu_wxyz':quat,
        'linear_velocity_body_frd_mps':[state[k] for k in ('u','v','w')],
        'angular_velocity_body_frd_radps':[state[k] for k in ('p','q','r')],
        'pitch_nose_up_rad':math.asin(max(-1,min(1,2*(state['qr']*state['qj']-state['qk']*state['qi']))))}


def pulse(case, elapsed):
    commands=dict.fromkeys(KEYS,0.0)
    if 2 <= elapsed < 4:
        if case=='surge_positive': commands[KEYS[0]]=2.0
        elif case=='surge_negative': commands[KEYS[0]]=-2.0
        elif case=='pitch_positive': commands[KEYS[1]]=-.5; commands[KEYS[2]]=.5
        elif case=='pitch_negative': commands[KEYS[1]]=.5; commands[KEYS[2]]=-.5
        elif case=='heave_positive': commands[KEYS[1]]=.5; commands[KEYS[2]]=.5
        elif case=='heave_negative': commands[KEYS[1]]=-.5; commands[KEYS[2]]=-.5
    return commands

CASES=('zero','surge_positive','surge_negative','heave_positive','heave_negative','pitch_positive','pitch_negative','watchdog')
