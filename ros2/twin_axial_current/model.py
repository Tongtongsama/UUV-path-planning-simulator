"""Independent constant-current extension; original v2 forces stay intact.

The xdyn builtin constant UW current is horizontal only. This extension keeps
that environment at zero and adds the exact change of hydrodynamic wrench
inside the solver, evaluated at every RK stage. Never add a second builtin flow.
"""
import math
import numpy as np
from ros2.twin_axial_navigation.mission import model_config as base_model
from ros2.twin_axial_v2.model import frame

FLOWS={'F0':(0.,0.),'F1':(-.05,0.),'F2':(0.,.05)}

def configuration(flow):
    x,z=FLOWS[flow]
    return {'revision':'constant_current_v1','flow':flow,'planning_xz_mps':[x,z],
            'enu_mps':[x,0.,z],'ned_mps':[0.,x,-z],
            'implementation':'xdyn RK-stage maneuvering correction; builtin UW current remains zero',
            'velocity_feedback':'body ground-relative; water-relative velocity is diagnostic only'}

def correction_expressions(flow):
    # R_body_to_NED.T @ [0, vx, -vz], full roll/pitch/yaw transformation.
    x,z=FLOWS[flow]
    a=f'({x}*cos(theta(t))*sin(psi(t))+{z}*sin(theta(t)))'
    b=f'({x}*(sin(phi(t))*sin(theta(t))*sin(psi(t))+cos(phi(t))*cos(psi(t)))-{z}*sin(phi(t))*cos(theta(t)))'
    c=f'({x}*(cos(phi(t))*sin(theta(t))*sin(psi(t))-sin(phi(t))*cos(psi(t)))-{z}*cos(phi(t))*cos(theta(t)))'
    u,v,w=[f'({k}(t)-{cur})' for k,cur in zip('uvw',(a,b,c))]
    old=['5*v(t)*r(t)-8*w(t)*q(t)','8*w(t)*p(t)-5*u(t)*r(t)',
         '5*u(t)*q(t)-5*v(t)*p(t)','-3*v(t)*w(t)','3*u(t)*w(t)','0']
    new=[f'5*{v}*r(t)-8*{w}*q(t)',f'8*{w}*p(t)-5*{u}*r(t)',
         f'5*{u}*q(t)-5*{v}*p(t)',f'-3*{v}*{w}',f'3*{u}*{w}','0']
    dot=[f'5*(r(t)*{b}-q(t)*{c})',f'5*(p(t)*{c}-r(t)*{a})',f'8*(q(t)*{a}-p(t)*{b})']
    expressions=[]
    for i,(o,n) in enumerate(zip(old,new)):
        e=f'({n})-({o})'
        if i<3:
            k='uvw'[i]+'(t)';r=(u,v,w)[i];d=(4,6,6)[i];q=(1,1.5,1.5)[i]
            e+=f'+{d}*({k}-{r})+{q}*(abs({k})*{k}-abs({r})*{r})+{dot[i]}'
        expressions.append(e)
    return dict(zip(('X','Y','Z','K','M','N'),expressions))

def model_config(case,flow):
    data=base_model(case)
    data['bodies'][0]['external forces'].append({'model':'maneuvering','name':'constant_current_correction',
        'reference frame':frame(),**correction_expressions(flow)})
    return data

def rotation(state):
    """Body FRD to NED, independently from quaternion feedback."""
    w,x,y,z=[state[k] for k in ('qr','qi','qj','qk')]
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])

def telemetry(state,flow):
    cfg=configuration(flow);r=rotation(state)
    current=r.T@np.array(cfg['ned_mps']);velocity=np.array([state[k] for k in 'uvw'])
    derivative=-np.cross([state[k] for k in 'pqr'],current)
    relative=velocity-current
    return {'flow':flow,'current_ned_mps':cfg['ned_mps'],
        'current_body_frd_mps':current.tolist(),'ground_body_frd_mps':velocity.tolist(),
        'relative_water_body_frd_mps':relative.tolist(),
        'current_body_derivative_mps2':derivative.tolist(),
        'relative_water_ned_mps':(r@relative).tolist()}
