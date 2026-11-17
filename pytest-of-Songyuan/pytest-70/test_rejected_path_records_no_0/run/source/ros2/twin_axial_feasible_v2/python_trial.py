"""Independent full-trajectory reference test; never labels Python as LOTUSim."""
import json,math,hashlib
from pathlib import Path
import numpy as np
from physics.uuv_3dof import Python3DOFDynamics
from physics.integrators import RK4Integrator
from simulation.vehicle_config import load_synthetic_parameters
from ros2.twin_axial_pid.control import AllocatedPID
from ros2.twin_axial_navigation.mission import INITIALS,world_xml,environment_from_world,segment_clearance
from core import Pose
from .guidance import Guide,build_path,executable_path,CONFIG

def run(output,case='P0',flow='F2'):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    current={'F0':[0,0],'F1':[-.05,0],'F2':[0,.05]}[flow]
    x,z,angle=INITIALS[case];state=np.array([x,z,math.radians(angle),0,0,0.])
    points,planning=build_path([[6,7,-20,-7],[13,14,-13,0]],(x,z))
    planning.update(executable_path(points,current))
    (output/'plan.json').write_text(json.dumps(dict(path=points.tolist(),config=CONFIG,planning=planning,case=case,flow=flow),indent=2))
    guide=Guide(points,current);pid=AllocatedPID(math.radians(70));dyn=Python3DOFDynamics(load_synthetic_parameters());rk=RK4Integrator()
    world=environment_from_world(world_xml(case,'S2'));prev=Pose(x=x,z=z)
    force=np.zeros(3);rows=[];error=None
    for i in range(6000):
        state=rk.step(lambda s:dyn.derivatives(s,force,np.array(current)),state,.05)
        x,z,theta,u,w,q=map(float,state);t=round((i+1)*.05,8);p=Pose(x=x,z=z)
        clearance=segment_clearance(world,prev,p);prev=p
        target,nav=guide.update(state,t)
        record=dict(valid=True,sequence=i,elapsed_end=t,pitch_nose_up_rad=theta,
                    solver_state_ned_frd=dict(t=t,y=x,z=20-z,u=u,w=w,q=q))
        raw,control=pid.command(record,target,.05)
        row=dict(t=t,state=state.tolist(),clearance=clearance,nav=nav,control=control,applied_force=force.tolist());rows.append(row)
        allocated=control['allocation']['allocated_wrench'];force=np.array([allocated['X'],0,allocated['M']])
        if clearance<.9-1e-6:error='safety';break
        if abs(theta)>math.radians(75) or abs(q)>.8 or max(abs(u),abs(w))>1.5:error='envelope';break
        if guide.arrived:break
    errors=[r['nav']['path_error'] for r in rows];sat=sum(r['control']['allocation']['alpha']<.999 for r in rows)/len(rows)
    metrics=dict(arrival_time=rows[-1]['t'] if guide.arrived else None,min_clearance=min(r['clearance'] for r in rows),
                 rms_error=math.sqrt(sum(e*e for e in errors)/len(errors)),max_error=max(errors),saturation_fraction=sat)
    checks=dict(arrived=guide.arrived,timeout=guide.arrived and rows[-1]['t']<=300,safety=metrics['min_clearance']>=.9-1e-6,
                rms=metrics['rms_error']<=.35,max_error=metrics['max_error']<=.8,saturation=sat<=.2,envelope=error!='envelope')
    report=dict(passed=all(checks.values()),backend='Python reference; no LOTUSim/xdyn validation',case=case,flow=flow,checks=checks,metrics=metrics,error=error)
    (output/'states.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    root=Path(__file__).resolve().parents[2]
    for folder in ('ros2/twin_axial_feasible_v2','ros2/twin_axial_navigation','ros2/twin_axial_pid','ros2/twin_axial_v2','controller','physics','simulation','core','config/vehicles'):
        for source in (root/folder).rglob('*'):
            if not source.is_file() or '__pycache__' in source.parts or source.suffix not in ('.py','.yaml'):continue
            target=output/'source'/source.relative_to(root);target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(source.read_bytes())
    (output/'report.json').write_text(json.dumps(report,indent=2))
    (output/'manifest.json').write_text(json.dumps({p.relative_to(output).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in output.rglob('*') if p.is_file()},indent=2))
    print(json.dumps(report),flush=True);return report

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--case',default='P0',choices=INITIALS);p.add_argument('--flow',default='F2',choices=['F0','F1','F2']);a=p.parse_args()
    result=run(a.output,a.case,a.flow)
    raise SystemExit(0 if result['passed'] else 2)
