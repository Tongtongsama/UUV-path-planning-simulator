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

CASES=('P0','P1','P2','P3','P4')
INITIALS={'P0':(2,-10,0),'P1':(2,-9.9,3),'P2':(2,-10.1,-3),
          'P3':(2.1,-10,-3),'P4':(1.9,-10,3)}
CONFIG={'revision':'safe_shortcut_v2','scene':'single_baffle_v1','current':[0,0],'boundary':[0,20,-20,0],
        'obstacle':[9,10,-20,-8],'goal':[18,-10],
        'grid_resolution':.5,'planning_clearance':1.5,'safety_clearance':.9,
        'lookahead':1.2,'nominal_speed':.4,'goal_radius':.5,'goal_hold_s':2.,
        'timeout_s':300.,'rms_limit_m':.35,'max_error_m':.8,'max_saturation_fraction':.2}

def initial(case):
    x,z,pitch=INITIALS[case]
    return VehicleState(Pose(x=x,z=z,pitch=math.radians(pitch)),Twist.zero(),0.)

def model_config(case):
    data=vessel.model();s=initial(case)
    p=data['bodies'][0]['initial position of body frame relative to NED']
    p['y']['value']=s.pose.x;p['z']['value']=20-s.pose.z;p['theta']['value']=s.pose.pitch
    return data

def world_xml(case):
    root=ET.fromstring(vessel.world());world=root.find('world');s=initial(case)
    world.find('model/pose').text=f'{s.pose.x} 0 {s.pose.z-20} 0 {-s.pose.pitch} 0'
    obstacle=ET.SubElement(world,'model',name='single_baffle_obstacle')
    ET.SubElement(obstacle,'static').text='true'
    ET.SubElement(obstacle,'pose').text='9.5 0 -34 0 0 0'
    link=ET.SubElement(obstacle,'link',name='base_link')
    for kind in ('visual','collision'):
        node=ET.SubElement(link,kind,name='baffle')
        ET.SubElement(ET.SubElement(ET.SubElement(node,'geometry'),'box'),'size').text='1 4 12'
        if kind=='visual':
            ET.SubElement(ET.SubElement(node,'material'),'ambient').text='.5 .5 .5 1'
    return ET.tostring(root,encoding='unicode')

def environment_from_world(xml):
    """Derive the planner obstacle from the exact SDF loaded by LOTUSim."""
    node=ET.fromstring(xml).find(".//model[@name='single_baffle_obstacle']")
    pose=list(map(float,node.findtext('pose').split()))
    size=list(map(float,node.findtext('link/collision/geometry/box/size').split()))
    if pose[3:]!=[0,0,0] or pose[1]!=0 or size[1]<1.4: raise ValueError('Unsupported obstacle geometry')
    x,z=pose[0],pose[2]+20
    return WorldModel(Boundary.rectangle(x_min=0,x_max=20,z_min=-20,z_max=0),
        [Obstacle.rectangle('single_baffle_obstacle',x_min=x-size[0]/2,x_max=x+size[0]/2,z_min=z-size[2]/2,z_max=z+size[2]/2)])

def prepare(directory,case):
    output=vessel.prepare(directory)
    (output/'model.yml').write_text(json.dumps(model_config(case),indent=2),encoding='utf-8')
    xml=world_xml(case);(output/'worlds/minimal.world').write_text(xml,encoding='utf-8')
    space=EnvironmentPlanningSpace(environment_from_world(xml))
    constraints=PlanningConstraints(.7,.8,goal_tolerance=0.)
    request=PlanningRequest(initial(case),Pose(x=18,z=-10),constraints)
    result=AStarPlanner(AStarConfig(resolution=CONFIG['grid_resolution'])).plan(request,space)
    if not result.success: raise RuntimeError('Planning failed: '+str(result.message))
    processed=generate_trajectory(result.path,space,constraints,
        TrajectoryConfig(nominal_speed=.4,sample_spacing=.25,shortcut=True),request=request)
    # Reuse the existing collision-checked geometric shortcut, not its timing.
    path=simplify_path(processed.shortcut_path)
    if not validate_path(path,space,constraints,request=request).valid: raise RuntimeError('Invalid planned path')
    plan={'config':CONFIG,'case':case,'initial':asdict(initial(case)),
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
        if plan['config']!=CONFIG: raise ValueError('Unknown navigation configuration')
        self.path=Path(tuple(Pose(x=x,z=z) for x,z in plan['path']))
        self.world=environment_from_world(xml)
        self.adapter=LOSNavigationAdapter(ProjectionConfig(.3,4,.05,8),LOSConfig(CONFIG['lookahead'],.4,1.5))
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
        pitch=max(-math.pi/3,min(math.pi/3,raw.pitch))
        speed=min(raw.u,max(.08,.4*distance/1.5))
        phase='LOS'
        if t<2:speed=0;pitch=0;phase='ARMING'
        elif self.braking:speed=0;pitch=0;phase='GOAL_BRAKE'
        if self.arrived:speed=0;pitch=0;phase='ARRIVED'
        return BodyGuidance3DOF(speed,pitch),{'phase':phase,'elapsed':t,'position_xz':[p.x,p.z],
            'goal_distance_m':distance,'goal_held_s':held,'swept_clearance_m':clearance,
            'path_distance_m':path_distance(self.path,p),'progress_s':cmd.progress_s,
            'segment_index':cmd.segment_index,'lookahead_xz':[cmd.lookahead_point.x,cmd.lookahead_point.z],
            'raw_los_pitch':raw.pitch,'raw_los_speed':raw.u,'target_pitch':pitch,'target_speed':speed}
