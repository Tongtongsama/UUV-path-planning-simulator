"""Uniform ENU x/z current as relative-water forces in pinned xdyn."""
from ros2.twin_axial_v2.model import model as zero_current_model,frame


FLOWS={'F0':(0.0,0.0),'F1':(-0.05,0.0),'F2':(0.0,0.05)}


def model_for_flow(flow):
    if flow not in FLOWS: raise ValueError(f'Unknown flow: {flow}')
    data=zero_current_model()
    if flow=='F0': return data
    cx,cz=FLOWS[flow]
    # NED current=(0,cx,-cz). R_NED,body = Rz(psi) Ry(theta) Rx(phi).
    # These body components include attitude changes during every RK4 stage.
    cu=f'({cx}*sin(psi(t))*cos(theta(t))+{cz}*sin(theta(t)))'
    cv=f'({cx}*(sin(psi(t))*sin(theta(t))*sin(phi(t))+cos(psi(t))*cos(phi(t)))-{cz}*cos(theta(t))*sin(phi(t)))'
    cw=f'({cx}*(sin(psi(t))*sin(theta(t))*cos(phi(t))-cos(psi(t))*sin(phi(t)))-{cz}*cos(theta(t))*cos(phi(t)))'
    ur=f'(u(t)-{cu})';vr=f'(v(t)-{cv})';wr=f'(w(t)-{cw})'
    def drag(v,linear,quadratic): return f'-{linear}*{v}-{quadratic}*{v}*abs({v})'
    forces=data['bodies'][0]['external forces']
    # Do not enable xdyn's built-in UW current: the pinned version also adds
    # it to kinematics, while its damping already subtracts it. Keep one
    # absolute-state owner and supply the relative-water force explicitly.
    forces[:2]=[{'model':'maneuvering','name':'uniform_current_relative_damping',
        'reference frame':frame(),'X':drag(ur,4,1),'Y':drag(vr,6,1.5),
        'Z':drag(wr,6,1.5),'K':drag('p(t)',3,.5),
        'M':drag('q(t)',3,.5),'N':drag('r(t)',3,.5)}]
    forces[-1]={'model':'maneuvering','name':'added_mass_coriolis',
        'reference frame':frame(),
        'X':f'5*{vr}*r(t)-8*{wr}*q(t)+5*(r(t)*{cv}-q(t)*{cw})',
        'Y':f'8*{wr}*p(t)-5*{ur}*r(t)+5*(p(t)*{cw}-r(t)*{cu})',
        'Z':f'5*{ur}*q(t)-5*{vr}*p(t)+8*(q(t)*{cu}-p(t)*{cv})',
        'K':f'-3*{vr}*{wr}','M':f'3*{ur}*{wr}','N':'0'}
    return data
