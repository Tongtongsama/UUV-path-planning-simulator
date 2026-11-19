"""Time-varying current model for the smooth segmented G12 revision."""
import math
from ros2.twin_axial_navigation.mission import model_config as base_model
from ros2.twin_axial_v2.model import frame
from . import schedule


def _correction(x,z):
    a=f'({x}*cos(theta(t))*sin(psi(t))+{z}*sin(theta(t)))'
    b=f'({x}*(sin(phi(t))*sin(theta(t))*sin(psi(t))+cos(phi(t))*cos(psi(t)))-{z}*sin(phi(t))*cos(theta(t)))'
    c=f'({x}*(cos(phi(t))*sin(theta(t))*sin(psi(t))-sin(phi(t))*cos(psi(t)))-{z}*cos(phi(t))*cos(theta(t)))'
    u,v,w=[f'({k}(t)-{cur})' for k,cur in zip('uvw',(a,b,c))]
    old=['5*v(t)*r(t)-8*w(t)*q(t)','8*w(t)*p(t)-5*u(t)*r(t)','5*u(t)*q(t)-5*v(t)*p(t)',
         '-3*v(t)*w(t)','3*u(t)*w(t)','0']
    new=[f'5*{v}*r(t)-8*{w}*q(t)',f'8*{w}*p(t)-5*{u}*r(t)',f'5*{u}*q(t)-5*{v}*p(t)',
         f'-3*{v}*{w}',f'3*{u}*{w}','0']
    dot=[f'5*(r(t)*{b}-q(t)*{c})',f'5*(p(t)*{c}-r(t)*{a})',f'8*(q(t)*{a}-p(t)*{b})']
    out=[]
    for i,(o,n) in enumerate(zip(old,new)):
        e=f'({n})-({o})'
        if i<3:
            k='uvw'[i]+'(t)'; r=(u,v,w)[i]; d=(4,6,6)[i]; q=(1,1.5,1.5)[i]
            e+=f'+{d}*({k}-{r})+{q}*(abs({k})*{k}-abs({r})*{r})+{dot[i]}'
        out.append(e)
    return dict(zip(('X','Y','Z','K','M','N'),out))


def model_config(case,flow):
    data=base_model(case)
    if flow=='F2':
        x=schedule.expression_x(); z=schedule.expression_z()
    elif flow in ('F0','F1'):
        x='0.0'; z='0.0'
    else:
        raise ValueError('Unknown flow')
    data['bodies'][0]['external forces'].append({'model':'maneuvering','name':'smooth_segmented_current',
        'reference frame':frame(),**_correction(x,z)})
    return data
