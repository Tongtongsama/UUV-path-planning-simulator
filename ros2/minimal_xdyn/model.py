"""Synthetic six-DOF model, ideal reversible thrust inputs in newtons."""
import json
from pathlib import Path

NAME = 'uuv_minimal_xdyn_v1'
DT = 0.05
KEYS = ('surge(T)', 'fore(T)', 'aft(T)')
LIMITS = (5.0, 2.0, 2.0)
LEVER = 0.5


def quantity(v, unit): return {'value': v, 'unit': unit}


def frame():
    return {'frame': NAME, **{a:quantity(0,'m') for a in ('x','y','z')},
            **{a:quantity(0,'rad') for a in ('phi','theta','psi')}}


def matrix(diagonal):
    return {f'row {i+1}':[v if i==j else 0 for j in range(6)] for i,v in enumerate(diagonal)}


def model():
    forces=[{'model':'linear damping',
        'damping matrix at the center of gravity projected in the body frame':matrix([4,6,6,3,3,3])},
        {'model':'quadratic damping',
        'damping matrix at the center of gravity projected in the body frame':matrix([1,1.5,1.5,.5,.5,.5])}]
    # Equivalent COM wrench for point thrusters. Reference frame at COM prevents
    # a second lever-arm transform by xdyn. No RPM conversion or motor lag.
    for name,x,z,m in [('surge','T','0','0'),('fore','0','T','-0.5*T'),('aft','0','T','0.5*T')]:
        forces.append({'model':'maneuvering','name':name,'reference frame':frame(),
                       'commands':['T'],'X':x,'Y':'0','Z':z,'K':'0','M':m,'N':'0'})
    forces.append({'model':'maneuvering','name':'restoring','reference frame':frame(),
                   'X':'0','Y':'0','Z':'0','K':'-6*sin(phi(t))','M':'-6*sin(theta(t))','N':'0'})
    return {'rotations convention':['psi',"theta'","phi''"],
        'environmental constants':{'g':quantity(9.81,'m/s^2'),'rho':quantity(1000,'kg/m^3'),'nu':quantity(1e-6,'m^2/s')},
        'environment models':[{'model':'no waves','constant sea elevation in NED frame':quantity(0,'m')}, {'model':'no wind'}],
        'bodies':[{'name':NAME,
            'position of body frame relative to mesh':{**frame(),'frame':'mesh'},
            'initial position of body frame relative to NED':{**frame(),'frame':'NED','z':quantity(30,'m')},
            'initial velocity of body frame relative to NED':{'frame':NAME,**{k:quantity(0,'m/s' if k in 'uvw' else 'rad/s') for k in 'uvwpqr'}},
            'dynamics':{
                'hydrodynamic forces calculation point in body frame':{a:quantity(0,'m') for a in ('x','y','z')},
                'centre of inertia':{'frame':NAME,**{a:quantity(0,'m') for a in ('x','y','z')}},
                'rigid body inertia matrix at the center of gravity and projected in the body frame':matrix([20,20,20,8,8,8]),
                'added mass matrix at the center of gravity and projected in the body frame':matrix([5,5,8,2,2,2])},
            'external forces':forces}]}


def world():
    return f'''<sdf version="1.7"><world name="uuv_minimal">
<physics name="bench" type="ode"><max_step_size>{DT}</max_step_size><real_time_factor>1</real_time_factor></physics>
<spherical_coordinates><surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation><latitude_deg>0</latitude_deg><longitude_deg>0</longitude_deg><elevation>0</elevation><heading_deg>0</heading_deg></spherical_coordinates>
<gravity>0 0 0</gravity><scene><ambient>1 1 1</ambient><background>0.15 0.2 0.3</background></scene>
<plugin filename="physics_interface_plugin" name="lotusim::gazebo::PhysicsInterfacePlugin"/>
<plugin filename="multi_agent_system_plugin" name="lotusim::gazebo::MultiAgentSystem"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<model name="{NAME}"><pose>0 0 -30 0 0 0</pose><link name="base_link">
<visual name="body"><geometry><box><size>1 .2 .2</size></box></geometry><material><ambient>1 .5 0 1</ambient></material></visual>
<visual name="nose"><pose>.55 0 0 0 0 0</pose><geometry><sphere><radius>.08</radius></sphere></geometry></visual>
</link><lotus_param><physics_engine_interface><init_state>Underwater</init_state><underwater>
<interface_type>XDynWebSocket</interface_type><uri>ws://127.0.0.1:12346</uri>
<control_surfaces><surge>surge(T)</surge><fore>fore(T)</fore><aft>aft(T)</aft></control_surfaces>
</underwater></physics_engine_interface></lotus_param></model></world></sdf>'''


def prepare(directory):
    path=Path(directory); path.mkdir(parents=True,exist_ok=False)
    (path/'model.yml').write_text(json.dumps(model(),indent=2),encoding='utf-8')
    (path/'worlds').mkdir()
    (path/'worlds'/'minimal.world').write_text(world(),encoding='utf-8')
    return path
