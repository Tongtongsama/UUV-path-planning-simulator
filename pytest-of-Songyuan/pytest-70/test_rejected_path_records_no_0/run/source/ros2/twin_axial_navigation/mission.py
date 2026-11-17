"""Existing A*/LOS integration; no new planner, PID or dynamics algorithm."""
import json
import math
from dataclasses import asdict
from xml.etree import ElementTree as ET
from core import Pose,Twist,VehicleState,Path
from environment import Boundary,Obstacle,WorldModel
from planner import AStarPlanner,AStarConfig,PlanningConstraints,PlanningRequest,EnvironmentPlanningSpace,validate_path
from planner.candidates import simplify_path
from trajectory import generate_trajectory,TrajectoryConfig
from navigation.los_adapter import LOSNavigationAdapter
from guidance import LOSConfig,ProjectionConfig
from controller.guidance import BodyGuidance3DOF
from ros2.twin_axial_v2 import model as vessel
from .current_model import FLOWS,model_for_flow

CASES=('P0','P1','P2','P3','P4')
INITIALS={'P0':(2,-10,0),'P1':(2,-9.9,3),'P2':(2,-10.1,-3),
          'P3':(2.1,-10,-3),'P4':(1.9,-10,3)}
CONFIG={'revision':'safe_shortcut_v2','scene':'single_baffle_v1','current':[0,0],'boundary':[0,20,-20,0],
        'obstacle':[9,10,-20,-8],'goal':[18,-10],
        'grid_resolution':.5,'planning_clearance':1.5,'safety_clearance':.9,
        'lookahead':1.2,'nominal_speed':.4,'goal_radius':.5,'goal_hold_s':2.,
        'timeout_s':300.,'rms_limit_m':.35,'max_error_m':.8,'max_saturation_fraction':.2}
SCENES={
    'S1': CONFIG,
    'S2': {**{key:value for key,value in CONFIG.items() if key!='obstacle'},
           'scene':'double_baffle_scaled_v1',
           'obstacles':[[6,7,-20,-7],[13,14,-13,0]],
           'projection_switch_radius':.8,'nominal_speed':.2,'pitch_limit_deg':70},
}


def scene_config(scene):
    return SCENES[scene]


def mission_config(scene,flow='F0'):
    if flow not in FLOWS: raise ValueError('Unknown flow')
    config=scene_config(scene)
    if flow=='F0': return config
    return {**config,'flow':flow,'current':list(FLOWS[flow])}


def scene_obstacles(scene):
    config=scene_config(scene)
    return [config['obstacle']] if scene=='S1' else config['obstacles']


def obstacle_extents(world):
    return [[xmin,xmax,zmin,zmax] for obstacle in world.obstacles
            for xmin,zmin,xmax,zmax in [obstacle.bounds]]

def initial(case):
    x,z,pitch=INITIALS[case]
    return VehicleState(Pose(x=x,z=z,pitch=math.radians(pitch)),Twist.zero(),0.)

def model_config(case,flow='F0'):
    data=model_for_flow(flow);s=initial(case)
    p=data['bodies'][0]['initial position of body frame relative to NED']
    p['y']['value']=s.pose.x;p['z']['value']=20-s.pose.z;p['theta']['value']=s.pose.pitch
    return data

def world_xml(case,scene='S1',gui=False):
    root=ET.fromstring(vessel.world());world=root.find('world');s=initial(case)
    world.find('model/pose').text=f'{s.pose.x} 0 {s.pose.z-20} 0 {-s.pose.pitch} 0'
    if gui and scene=='S2':
        interface=ET.SubElement(world,'gui',fullscreen='false')
        camera=ET.SubElement(interface,'plugin',filename='MinimalScene',name='S2 navigation')
        ET.SubElement(ET.SubElement(camera,'gz-gui'),'title').text='S2/F0 live UUV dynamics'
        for key,value in [('engine','ogre2'),('scene','scene'),('ambient_light','1 1 1'),
                          ('background_color','0.12 0.16 0.22'),
                          ('camera_pose','10 -42 -30 0 0 1.5707963267948966')]:
            ET.SubElement(camera,key).text=value
        for filename in ('GzSceneManager','InteractiveViewControl','CameraTracking','EntityContextMenuPlugin','EntityTree'):
            ET.SubElement(interface,'plugin',filename=filename,name=filename)
        # The base vessel SDF only specifies body ambient color. Ogre2 needs
        # diffuse material as well, and the other parts need explicit colors.
        colors={'body':'0.95 0.55 0.08 1','nose':'1 0.95 0.25 1',
                'upper_thruster':'0.15 0.7 0.95 1','lower_thruster':'0.2 0.9 0.55 1'}
        for visual in world.findall('./model/link/visual'):
            material=visual.find('material')
            if material is None: material=ET.SubElement(visual,'material')
            for tag in ('ambient','diffuse'):
                node=material.find(tag)
                if node is None: node=ET.SubElement(material,tag)
                node.text=colors[visual.get('name')]
    for index,(xmin,xmax,zmin,zmax) in enumerate(scene_obstacles(scene)):
        name='single_baffle_obstacle' if scene=='S1' else f'double_baffle_obstacle_{index}'
        obstacle=ET.SubElement(world,'model',name=name)
        ET.SubElement(obstacle,'static').text='true'
        x=(xmin+xmax)/2; z=(zmin+zmax)/2-20
        ET.SubElement(obstacle,'pose').text=f'{x:g} 0 {z:g} 0 0 0'
        link=ET.SubElement(obstacle,'link',name='base_link')
        for kind in ('visual','collision'):
            node=ET.SubElement(link,kind,name='baffle')
            ET.SubElement(ET.SubElement(ET.SubElement(node,'geometry'),'box'),'size').text=f'{xmax-xmin:g} 4 {zmax-zmin:g}'
            if kind=='visual':
                material=ET.SubElement(node,'material')
                rgba=('0.16 0.53 0.92 1' if index==0 else '0.8 0.25 0.65 1') if scene=='S2' else '.5 .5 .5 1'
                ET.SubElement(material,'ambient').text=rgba
                ET.SubElement(material,'diffuse').text=rgba
    return ET.tostring(root,encoding='unicode')

def environment_from_world(xml):
    """Derive the planner obstacle from the exact SDF loaded by LOTUSim."""
    nodes=[node for node in ET.fromstring(xml).findall('./world/model')
           if node.get('name','').endswith('_obstacle') or node.get('name','').startswith('double_baffle_obstacle_')]
    if not nodes: raise ValueError('No navigation obstacle in world')
    obstacles=[]
    for node in nodes:
        pose=list(map(float,node.findtext('pose').split()))
        size=list(map(float,node.findtext('link/collision/geometry/box/size').split()))
        if pose[3:]!=[0,0,0] or pose[1]!=0 or size[1]<1.4: raise ValueError('Unsupported obstacle geometry')
        x,z=pose[0],pose[2]+20
        obstacles.append(Obstacle.rectangle(node.get('name'),x_min=x-size[0]/2,x_max=x+size[0]/2,
                                            z_min=z-size[2]/2,z_max=z+size[2]/2))
    return WorldModel(Boundary.rectangle(x_min=0,x_max=20,z_min=-20,z_max=0),
        obstacles)

def prepare(directory,case,scene='S1',gui=False,flow='F0'):
    output=vessel.prepare(directory)
    (output/'model.yml').write_text(json.dumps(model_config(case,flow),indent=2),encoding='utf-8')
    xml=world_xml(case,scene,gui);(output/'worlds/minimal.world').write_text(xml,encoding='utf-8')
    world=environment_from_world(xml)
    expected=scene_obstacles(scene)
    if obstacle_extents(world)!=expected: raise ValueError('SDF obstacle geometry does not match scene')
    space=EnvironmentPlanningSpace(world)
    constraints=PlanningConstraints(.7,.8,goal_tolerance=0.)
    request=PlanningRequest(initial(case),Pose(x=18,z=-10),constraints)
    config=mission_config(scene,flow)
    result=AStarPlanner(AStarConfig(resolution=config['grid_resolution'])).plan(request,space)
    if not result.success: raise RuntimeError('Planning failed: '+str(result.message))
    processed=generate_trajectory(result.path,space,constraints,
        TrajectoryConfig(nominal_speed=config['nominal_speed'],sample_spacing=.25,shortcut=True),request=request)
    # Reuse the existing collision-checked geometric shortcut, not its timing.
    path=simplify_path(processed.shortcut_path)
    if not validate_path(path,space,constraints,request=request).valid: raise RuntimeError('Invalid planned path')
    plan={'config':config,'case':case,'initial':asdict(initial(case)),
        'planner':'existing AStarPlanner + generate_trajectory safe shortcut + simplify_path',
        'planning_constraints':asdict(constraints),'expanded_nodes':result.expanded_nodes,
        'raw_path':[[p.x,p.z] for p in result.path],'path':[[p.x,p.z] for p in path]}
    (output/'plan.json').write_text(json.dumps(plan,indent=2),encoding='utf-8')
    return output

def planning_pose(record):
    s=record['solver_state_ned_frd']
    return Pose(x=s['y'],z=20-s['z'],pitch=record['pitch_nose_up_rad'])

def segment_clearance(world,a,b):
    # Rectangular boundary: minimum over each segment is attained at an endpoint.
    boundary=min(a.x,20-a.x,a.z+20,-a.z,b.x,20-b.x,b.z+20,-b.z)
    return min(boundary,world.segment_clearance(a,b))

def path_distance(path,p):
    ds=[]
    for a,b in zip(path,path.poses[1:]):
        dx,dz=b.x-a.x,b.z-a.z
        t=max(0,min(1,((p.x-a.x)*dx+(p.z-a.z)*dz)/(dx*dx+dz*dz)))
        ds.append(math.hypot(p.x-a.x-t*dx,p.z-a.z-t*dz))
    return min(ds)

class Mission:
    def __init__(self,plan,xml):
        config=plan['config'].copy()
        # Development recordings before the S2 config cleanup included S1's
        # unused singular obstacle field. Preserve read-only review of them.
        if config.get('scene')=='double_baffle_scaled_v1' and config.get('obstacle')==CONFIG['obstacle']:
            config.pop('obstacle')
        scene=next((key for key in SCENES if any(mission_config(key,flow)==config for flow in FLOWS)),None)
        if scene is None: raise ValueError('Unknown navigation configuration')
        flow=next(flow for flow in FLOWS if mission_config(scene,flow)==config)
        self.scene=scene;self.flow=flow; self.config=mission_config(scene,flow)
        self.path=Path(tuple(Pose(x=x,z=z) for x,z in plan['path']))
        self.world=environment_from_world(xml)
        if obstacle_extents(self.world)!=scene_obstacles(scene):
            raise ValueError('Recorded SDF obstacle geometry does not match scene')
        self.adapter=LOSNavigationAdapter(ProjectionConfig(self.config.get('projection_switch_radius',.3),4,.05,8),
                                          LOSConfig(self.config['lookahead'],self.config['nominal_speed'],1.5))
        self.progress=self.adapter.initialize(self.path,initial(plan['case']))
        self.previous_pitch=None;self.previous_pose=initial(plan['case']).pose
        self.inside_since=None;self.arrived=False;self.braking=False;self.arrival_time=None

    def update(self,record):
        p=planning_pose(record);t=record['elapsed_end']
        clearance=segment_clearance(self.world,self.previous_pose,p)
        self.previous_pose=p
        if clearance<CONFIG['safety_clearance']-1e-6: raise RuntimeError('Swept safety clearance violated')
        distance=math.hypot(p.x-18,p.z+10)
        if distance<=.5:
            if self.inside_since is None:self.inside_since=t
        else:self.inside_since=None
        held=0 if self.inside_since is None else t-self.inside_since
        if held>=2-1e-9 and not self.arrived:self.arrived=True;self.arrival_time=t
        state=VehicleState(p,Twist(u=record['solver_state_ned_frd']['u'],w=record['solver_state_ned_frd']['w'],q=record['solver_state_ned_frd']['q']),record['solver_state_ned_frd']['t'])
        cmd=self.adapter.update(state,self.progress,self.previous_pitch)
        self.progress=cmd.projection.next_state;self.previous_pitch=cmd.pitch_ref
        if distance<=.35:self.braking=True
        raw=self.adapter.to_body(cmd)
        pitch_limit=math.radians(self.config.get('pitch_limit_deg',60))
        pitch=max(-pitch_limit,min(pitch_limit,raw.pitch))
        speed=min(raw.u,max(.08,self.config['nominal_speed']*distance/1.5))
        phase='LOS'
        if t<2:speed=0;pitch=0;phase='ARMING'
        elif self.braking:speed=0;pitch=0;phase='GOAL_BRAKE'
        if self.arrived:speed=0;pitch=0;phase='ARRIVED'
        return BodyGuidance3DOF(speed,pitch),{'phase':phase,'elapsed':t,'position_xz':[p.x,p.z],
            'goal_distance_m':distance,'goal_held_s':held,'swept_clearance_m':clearance,
            'path_distance_m':path_distance(self.path,p),'progress_s':cmd.progress_s,
            'segment_index':cmd.segment_index,'lookahead_xz':[cmd.lookahead_point.x,cmd.lookahead_point.z],
            'raw_los_pitch':raw.pitch,'raw_los_speed':raw.u,'target_pitch':pitch,'target_speed':speed}
