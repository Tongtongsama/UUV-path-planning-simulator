"""Pre-navigation frame/sign and M6 current response gates, no PID."""
import json,math,hashlib,shutil
from pathlib import Path
import numpy as np
from physics.uuv_3dof import Python3DOFDynamics
from physics.integrators import RK4Integrator
from simulation.vehicle_config import load_synthetic_parameters
from ros2.twin_axial_v2.numerical import initial_state,schedule,errors
from ros2.twin_axial_v2.allocation import wrench
from ros2.twin_axial_v2.model import KEYS
from .model import FLOWS,model_config,configuration,telemetry,rotation
from .solver import xdyn_run

def reference(dt,flow):
    dynamics=Python3DOFDynamics(load_synthetic_parameters());rk=RK4Integrator()
    state=np.array([2.,-30.,0.,0.,0.,0.]);rows=[{'t':0.,'state':state.tolist()}]
    for i in range(round(20/dt)):
        force=wrench(schedule(round(i*dt,10)));control=np.array([force['X'],0.,force['M']])
        state=rk.step(lambda s:dynamics.derivatives(s,control,np.array(FLOWS[flow])),state,dt)
        rows.append({'t':round((i+1)*dt,10),'state':state.tolist()})
    return rows

def within(error):
    return error['position_m']<=.05 and error['pitch_deg']<=1 and error['velocity_mps']<=.02

def assess(data):
    checks={};metrics={}
    for flow in FLOWS:
        runs=data['responses'][flow]
        for step,rows in runs.items():
            py=reference(float(step),flow)
            err=errors([r['state'] for r in rows],[r['state'] for r in py])
            checks[f'{flow}_M6_dt_{step}']=within(err);metrics[f'{flow}_M6_dt_{step}']=err
        coarse=runs['0.05'];fine=runs['0.025'][::2]
        err=errors([r['state'] for r in coarse],[r['state'] for r in fine])
        checks[f'{flow}_step_convergence']=err['position_m']<=.02 and err['pitch_deg']<=.5 and err['velocity_mps']<=.01
        metrics[f'{flow}_step_convergence']=err
        for label,rows in data['probes'][flow].items():
            first,last=rows[0]['full_state'],rows[-1]['full_state'];dt=last['t']-first['t']
            if label=='rest':
                vc=configuration(flow)['ned_mps'];delta=np.array([last[k]-first[k] for k in ('x','y','z')])
                checks[f'{flow}_rest_drift_direction']=float(delta@vc)>0 if flow!='F0' else np.linalg.norm(delta)<1e-9
            elif label=='comoving':
                vc=configuration(flow)['ned_mps']
                err=max(abs(last[k]-first[k]-dt*c) for k,c in zip(('x','y','z'),vc))
                checks[f'{flow}_comoving_no_drag']=err<1e-8 and max(abs(last[k]-first[k]) for k in 'uvwpqr')<1e-8
            else:
                pitch=float(label);theta=math.radians(pitch)
                nu=np.array([first[k] for k in ('u','w','q')])
                derivative=Python3DOFDynamics(load_synthetic_parameters()).derivatives(np.array([2.,-30.,theta,*nu]),np.zeros(3),np.array(FLOWS[flow])).nu_dot
                measured=np.array([(last[k]-first[k])/dt for k in ('u','w','q')])
                error=float(np.max(np.abs(measured-derivative)))
                checks[f'{flow}_tilted_derivative_{label}']=error<1e-4
                metrics[f'{flow}_tilted_derivative_{label}']=error
        frame_error=0.
        for rows in runs.values():
            for row in rows:
                s=row['full_state'];t=telemetry(s,flow);r=rotation(s)
                c=np.array(t['current_body_frd_mps']);vc=np.array(t['current_ned_mps'])
                frame_error=max(frame_error,float(np.max(np.abs(r@c-vc))))
        checks[f'{flow}_frame_roundtrip']=frame_error<1e-9
        metrics[f'{flow}_frame_roundtrip']=frame_error
    checks={k:bool(v) for k,v in checks.items()}
    return {'passed':all(checks.values()),'checks':checks,'metrics':metrics,
            'scope':'Constant uniform F0/F1/F2, vertical-plane M6 and sign/relative-speed gates; no PID'}

def run(directory):
    directory=Path(directory).resolve();directory.mkdir(parents=True,exist_ok=False)
    data={'responses':{},'probes':{}}
    for flow in FLOWS:
        config=model_config('P0',flow);data['responses'][flow]={};data['probes'][flow]={}
        for dt in (.05,.025):
            data['responses'][flow][str(dt)]=xdyn_run(directory/f'{flow}_{dt}',dt,config,initial_state(),schedule,20)
        for label in ('rest','comoving','-30','30'):
            state=initial_state();dt=.05;horizon=5
            if label=='comoving':
                body=rotation(state).T@np.array(configuration(flow)['ned_mps'])
                state.update(zip('uvw',body.tolist()))
            elif label not in ('rest','comoving'):
                th=math.radians(float(label));c,s=math.cos(th/2),math.sin(th/2);a=math.sqrt(.5)
                state.update(qr=a*c,qi=-a*s,qj=a*s,qk=a*c,u=.3,w=-.1,q=.2)
                dt=1e-5;horizon=dt
            data['probes'][flow][label]=xdyn_run(directory/f'{flow}_{label}',dt,config,state,lambda t:dict.fromkeys(KEYS,0.),horizon)
    (directory/'data.json').write_text(json.dumps(data),encoding='utf-8')
    for folder in ('ros2/twin_axial_current','ros2/twin_axial_v2','physics','simulation'):
        for source in Path(folder).rglob('*.py'):
            dest=directory/'source'/source;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest)
    (directory/'manifest.json').write_text(json.dumps({p.relative_to(directory).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in directory.rglob('*') if p.is_file()},indent=2))
    result=assess(data);(directory/'acceptance.json').write_text(json.dumps(result,indent=2));return result

def analyze(directory):
    directory=Path(directory).resolve()
    for name,digest in json.loads((directory/'manifest.json').read_text()).items():
        p=(directory/name).resolve()
        if not p.is_relative_to(directory) or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:raise ValueError('Record changed: '+name)
    for flow in FLOWS:
        for suffix in ('0.05','0.025','rest','comoving','-30','30'):
            if json.loads((directory/f'{flow}_{suffix}'/'model.yml').read_text())!=model_config('P0',flow):
                raise ValueError('Preflight model differs from current implementation')
    return assess(json.loads((directory/'data.json').read_text()))
