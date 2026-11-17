"""Read-only independent metrics and deterministic control replay of Python runs."""
import json,hashlib,math
from pathlib import Path
import numpy as np
from core import Pose
from ros2.twin_axial_navigation.mission import INITIALS,environment_from_world,segment_clearance
from ros2.twin_axial_pid.control import AllocatedPID
from ros2.twin_axial_navigation.acceptance import numerically_equal
from .scenes import world_xml,preflight,SCENES
from .guidance import Guide,CONFIG,build_path,executable_path

def analyze(directory):
    try:return _analyze(Path(directory))
    except Exception as e:return dict(passed=False,error=str(e))

def _analyze(directory):
    manifest=json.loads((directory/'manifest.json').read_text())
    for name,h in manifest.items():
        p=(directory/name).resolve()
        if not p.is_relative_to(directory.resolve()) or hashlib.sha256(p.read_bytes()).hexdigest()!=h:raise ValueError('Hash mismatch: '+name)
    report=json.loads((directory/'report.json').read_text())
    if report.get('status')=='preflight_rejected':
        if not {'report.json','preflight.json'}.issubset(manifest):raise ValueError('Missing rejection evidence')
        fresh=preflight(report['scene'],report['case'],report['flow'])
        saved=json.loads((directory/'preflight.json').read_text())
        if fresh!=saved or fresh['passed']:raise ValueError('Rejection cannot be reproduced')
        return {**report,'checks':{'recording_hashes':True,'reproduced_rejection':True,'path_preflight':False}}
    if not {'states.jsonl','plan.json','report.json'}.issubset(manifest):raise ValueError('Incomplete manifest')
    plan=json.loads((directory/'plan.json').read_text());case=plan['case'];flow=plan['flow'];scene=plan['scene']
    current={'F0':[0,0],'F1':[-.05,0],'F2':[0,.05]}[flow]
    x,z,angle=INITIALS[case];points,_=build_path(preflight(scene,case,flow)['obstacles'],(x,z))
    if plan['config']!=CONFIG or not np.array_equal(points,plan['path']):raise ValueError('Configuration/path mismatch')
    executable_path(points,current)
    guide=Guide(points,current);pid=AllocatedPID(math.radians(70))
    world=environment_from_world(world_xml(case,scene));prev=Pose(x=x,z=z)
    records=[json.loads(l) for l in (directory/'states.jsonl').read_text().splitlines()];errors=[];clears=[];sat=[];envelope=True;forces=np.zeros(3)
    from physics.uuv_3dof import Python3DOFDynamics
    from physics.integrators import RK4Integrator
    from simulation.vehicle_config import load_synthetic_parameters
    dyn=Python3DOFDynamics(load_synthetic_parameters());rk=RK4Integrator()
    prior=np.array([x,z,math.radians(angle),0,0,0.]);arrival=None
    for i,r in enumerate(records):
        s=np.array(r['state']);t=r['t']
        if not np.isfinite(s).all() or abs(t-(i+1)*.05)>1e-6:raise ValueError('Invalid state/time')
        expected=rk.step(lambda y:dyn.derivatives(y,forces,np.array(current)),prior,.05)
        if not np.allclose(expected,s,atol=1e-9,rtol=1e-10):raise ValueError('Dynamics replay mismatch')
        prior=s
        if not np.allclose(forces,r['applied_force'],atol=1e-12):raise ValueError('Applied force mismatch')
        pose=Pose(x=s[0],z=s[1]);clears.append(segment_clearance(world,prev,pose));prev=pose
        desired,nav=guide.update(s,t)
        record=dict(valid=True,sequence=i,elapsed_end=t,pitch_nose_up_rad=float(s[2]),solver_state_ned_frd=dict(t=t,y=float(s[0]),z=float(20-s[1]),u=float(s[3]),w=float(s[4]),q=float(s[5])))
        _,control=pid.command(record,desired,.05)
        if not numerically_equal(control,r['control']) or not numerically_equal(nav,r['nav']):raise ValueError('Guidance/PID replay mismatch')
        allocated=control['allocation']['allocated_wrench'];forces=np.array([allocated['X'],0,allocated['M']])
        if max(abs(v) for v in control['allocation']['allocated_commands'].values())>15+1e-9:raise ValueError('Thruster limit')
        errors.append(nav['path_error']);sat.append(control['allocation']['alpha']<.999)
        envelope &= abs(s[2])<=math.radians(75) and max(abs(s[3]),abs(s[4]))<=1.5 and abs(s[5])<=.8
        if guide.arrived and arrival is None:arrival=t
    if not records:raise ValueError('Empty recording')
    rms=math.sqrt(sum(e*e for e in errors)/len(errors));fraction=sum(sat)/len(sat)
    checks=dict(arrived=arrival is not None,timeout=arrival is not None and arrival<=300,
                safety=bool(min(clears)>=.9-1e-6),rms=rms<=.35,max_error=max(errors)<=.8,
                saturation=fraction<=.2,envelope=bool(envelope),hashes=True,dynamics_replay=True,controller_replay=True)
    return dict(passed=all(checks.values()),checks=checks,arrival_time=arrival,min_clearance=min(clears),rms_error=rms,max_error=max(errors),saturation_fraction=fraction,case=case,flow=flow,scene=scene)

def suite(directory):
    directory=Path(directory);names=[*INITIALS,'P0_repeat'];reports={n:analyze(directory/n) for n in names}
    try:
        a=[json.loads(l) for l in (directory/'P0/states.jsonl').read_text().splitlines()]
        b=[json.loads(l) for l in (directory/'P0_repeat/states.jsonl').read_text().splitlines()]
        repeat=len(a)==len(b) and max(math.dist(x['state'][:2],y['state'][:2]) for x,y in zip(a,b))<=.02 and abs(a[-1]['t']-b[-1]['t'])<=.1
        snapshots=[{p.relative_to(directory/n/'source').as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in (directory/n/'source').rglob('*') if p.is_file()} for n in names]
        consistent=bool(snapshots[0]) and all(s==snapshots[0] for s in snapshots)
    except Exception:repeat=False;consistent=False
    count=sum(reports[n]['passed'] for n in INITIALS)
    safe=all(r.get('checks',{}).get('safety',False) for r in reports.values())
    return dict(passed=count>=4 and reports['P0']['passed'] and safe and repeat and consistent,
                successful_initials=count,all_safe=safe,repeatability=repeat,source_consistent=consistent,runs=reports,
                scope='Python only; excludes 6DOF planar checks, transport faults and Gazebo mirror')

def matrix(directory):
    directory=Path(directory);reports={};snapshots=[]
    for scene in SCENES:
        for flow in ('F0','F1','F2'):
            key=scene+'/'+flow;reports[key]=suite(directory/scene/flow)
            base=directory/scene/flow/'P0/source'
            snapshots.append({p.relative_to(base).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in base.rglob('*') if p.is_file()})
    same=bool(snapshots[0]) and all(s==snapshots[0] for s in snapshots)
    all_runs=all(r['passed'] and all(v.get('passed',False) and v.get('scene')+'/'+v.get('flow')==key for v in r['runs'].values()) for key,r in reports.items())
    rejected=sum(v.get('status')=='preflight_rejected' for r in reports.values() for v in r['runs'].values())
    return dict(passed=all_runs and same,all_54_passed=all_runs,cross_scene_source_identical=same,preflight_rejections=rejected,groups=reports)


def gate(directory):
    result=matrix(directory)
    if not result['passed']:raise ValueError('Complete S1/S2/S3 x F0/F1/F2 Python matrix must pass before any real run')
    root=Path(__file__).resolve().parents[2];base=Path(directory)/'S1/F0/P0/source'
    for p in base.rglob('*'):
        if p.is_file():
            live=root/p.relative_to(base)
            if not live.is_file() or hashlib.sha256(live.read_bytes()).digest()!=hashlib.sha256(p.read_bytes()).digest():raise ValueError('Source changed since Python gate: '+str(live))
    return result
