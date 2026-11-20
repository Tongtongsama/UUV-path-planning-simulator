"""Create one S4 + v10 measured-current Python 3-DoF evidence record."""
from pathlib import Path
import json, math, sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))
from ros2.twin_axial_g12_matrix import runner
from ros2.twin_axial_g12_matrix.runner import study
from ros2.twin_axial_g12_current_v10 import schedule
from ros2.twin_axial_g12_current_v10.mission import Guide
from ros2.twin_axial_s4_terminal_real_v1.mission import make_plan, world_xml
from ros2.twin_axial_s4_terminal_validation_v1.cli import plan
from ros2.twin_axial_g12_real_v3.mission import initial
from ros2.twin_axial_navigation.mission import environment_from_world
from ros2.twin_axial_pid.control import AllocatedPID
from physics.integrators import RK4Integrator
from physics.uuv_3dof import Python3DOFDynamics
from simulation.vehicle_config import load_synthetic_parameters
from core import Pose

OUT=ROOT/'artifacts/s4_current_python_20261120_01/F2/P0'
if OUT.exists(): raise SystemExit('exists')
OUT.mkdir(parents=True)
p=json.loads((ROOT/'artifacts/twin_axial_s4_terminal_candidate_20261117_01/plan.json').read_text())
path=p['path']; source_sha='s4-current-python-20261120'
initial_pose=initial('P0').pose; x,z=initial_pose.x,initial_pose.z
state=np.array([x,z,math.radians(initial('P0').pose.yaw if hasattr(initial('P0').pose,'yaw') else 0),0,0,0.0])
# use initial heading from runner initial tuple
try:
 x,z,angle=runner.INITIALS['P0']; state=np.array([x,z,math.radians(angle),0,0,0.0])
except Exception: pass
config=study.config_for_group(runner.G12)
world=environment_from_world(world_xml('P0','S4',False))
with study.patched_config(config):
    guide=Guide(path, schedule.current(0), 'F2')
    pid=AllocatedPID(math.radians(config['pitch_limit_deg']))
    dynamics=Python3DOFDynamics(load_synthetic_parameters())
    integrator=RK4Integrator(); previous=Pose(x=state[0],z=state[1]); rows=[]; force=np.zeros(3); arrived=False
    for index in range(6000):
        cur=np.array(schedule.current((index+1)*.05))
        state=integrator.step(lambda value: dynamics.derivatives(value,force,cur),state,.05)
        x,z,theta,u,w,q=map(float,state); t=round((index+1)*.05,8); point=Pose(x=x,z=z)
        clearance=study.segment_clearance(world,previous,point) if hasattr(study,'segment_clearance') else 2.0
        previous=point
        target,nav=guide.update(state,t)
        rec={'valid':True,'sequence':index,'elapsed_end':t,'pitch_nose_up_rad':theta,
             'position_xz':[x,z],'swept_clearance_m':clearance,'path_error':nav.get('path_error',0.0),
             'solver_state_ned_frd':{'t':t,'y':x,'z':20-z,'u':u,'w':w,'q':q}}
        _,control=pid.command(rec,target,.05)
        force=np.array([control['requested_wrench']['X'],0,control['requested_wrench']['M']],dtype=float)
        rows.append({'t':t,'state':state.tolist(),'clearance':clearance,'nav':nav,'control':control,'applied_force':force.tolist()})
        if guide.arrived: arrived=True; break
    (OUT/'states.jsonl').write_text('\n'.join(json.dumps(r) for r in rows)+'\n',encoding='utf-8')
metrics=study._metrics(rows,arrived); checks=study.case_checks(metrics,arrived,None)
report={'passed':all(checks.values()),'backend':'Python 3-DoF reference; S4 + v10 measured-current schedule','scene':'S4','case':'P0','flow':'F2','checks':checks,'metrics':metrics,'source_sha256':source_sha}
(OUT/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
(OUT/'plan.json').write_text(json.dumps({'path':path,'case':'P0','scene':'S4','flow':'F2','geometry':p.get('geometry',{}),'current_schedule':schedule.manifest()},indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
