"""No-current M4/M5/M6 checks against the real pinned xdyn solver.

Direct co-simulation protocol harness: deterministic open-loop inputs, no ROS
timing jitter, no Gazebo, no controller, no change to the reference algorithms.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import time

import numpy as np
from .model import model, KEYS
from .protocol import FIELDS, feedback
from .allocation import wrench
from physics.uuv_3dof import Python3DOFDynamics, added_mass_coriolis
from physics.derivative import StateDerivative3DOF
from physics.integrators import RK4Integrator
from simulation.vehicle_config import load_synthetic_parameters

STEPS=(.05,.025,.0125)
HORIZON=22.0  # M5/M6 end at20; M4 needs10 seconds after input release at12.
PORT=12358


def schedule(t):
    if t<2: return dict.fromkeys(KEYS,0.)
    if t<7: return dict.fromkeys(KEYS,2.)
    if t<12: return dict(zip(KEYS,(-1.,1.)))
    return dict.fromkeys(KEYS,0.)


def initial_state():
    return dict(zip(FIELDS,(0.,0.,2.,30.,math.sqrt(.5),0.,0.,math.sqrt(.5),0.,0.,0.,0.,0.,0.)))


def reduced(state):
    f=feedback(state)
    return [state['y'],-state['z'],f['pitch_nose_up_rad'],state['u'],state['w'],state['q']]


def energy6(state):
    f=feedback(state)
    kinetic=.5*sum(m*state[k]**2 for k,m in zip(('u','v','w','p','q','r'),(25,25,28,10,10,10)))
    return kinetic+6*(1-math.cos(f['pitch_nose_up_rad']))+6*(1-math.cos(f['roll_enu_rad']))


def energy3(state):
    return float(.5*(25*state[3]**2+28*state[4]**2+10*state[5]**2)+6*(1-math.cos(state[2])))


def reference_run(dt,diagnostic_without_added_coriolis=False):
    dynamics=Python3DOFDynamics(load_synthetic_parameters())
    integrator=RK4Integrator(); state=np.array([2.,-30.,0.,0.,0.,0.])
    rows=[{'t':0.,'state':state.tolist(),'energy_J':energy3(state)}]
    for i in range(round(HORIZON/dt)):
        t=round(i*dt,10); applied=schedule(t); force=wrench(applied)
        control=np.array([force['X'],0.,force['M']])
        def derivative(s):
            d=dynamics.derivatives(s,control,np.zeros(2))
            if not diagnostic_without_added_coriolis: return d
            # Diagnostic ablation only. Does not alter the reference or the acceptance target.
            correction=np.linalg.solve(dynamics.parameters.mass,added_mass_coriolis(dynamics.parameters,s[3:])@s[3:])
            return StateDerivative3DOF(d.eta_dot,d.nu_dot+correction)
        state=integrator.step(derivative,state,dt)
        rows.append({'t':round((i+1)*dt,10),'state':state.tolist(),'energy_J':energy3(state)})
    return rows


def decode(reply,t,dt):
    if 'error' in reply: raise ValueError(str(reply['error']))
    state={k:float(reply[k][-1]) for k in FIELDS}
    if not all(math.isfinite(v) for v in state.values()): raise ValueError('Nonfinite solver state')
    if abs(state['t']-t-dt)>1e-8: raise ValueError('Wrong solver step/time')
    if abs(sum(state[k]**2 for k in ('qr','qi','qj','qk'))-1)>1e-6: raise ValueError('Invalid quaternion')
    return state


def xdyn_run(directory,dt):
    from websockets.sync.client import connect
    with socket.socket() as port_test:
        port_test.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        port_test.bind(('127.0.0.1',PORT))
    directory.mkdir()
    config=directory/'model.yml'; config.write_text(json.dumps(model(),indent=2),encoding='utf-8')
    process=None
    with (directory/'xdyn.log').open('w',encoding='utf-8') as log, (directory/'transactions.jsonl').open('w',encoding='utf-8',buffering=1) as transactions:
        try:
            process=subprocess.Popen(['xdyn-for-cs','--yml',str(config),'--solver','rk4','--address','127.0.0.1','--port',str(PORT),'--dt',str(dt)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+30
            while True:
                if process.poll() is not None: raise RuntimeError('xdyn exited during startup; see xdyn.log')
                try:
                    ws=connect(f'ws://127.0.0.1:{PORT}',open_timeout=1); break
                except OSError:
                    if time.monotonic()>deadline: raise RuntimeError('xdyn startup timeout')
                    time.sleep(.1)
            with ws:
                state=initial_state()
                rows=[{'t':0.,'state':reduced(state),'full_state':state,'energy_J':energy6(state)}]
                for i in range(round(HORIZON/dt)):
                    t=round(i*dt,10)
                    if abs(state['t']-t)>1e-8: raise ValueError('Non-contiguous solver state')
                    request={'Dt':dt,'states':[state],'commands':schedule(t)}
                    ws.send(json.dumps(request)); reply=json.loads(ws.recv(timeout=5))
                    result=decode(reply,t,dt)
                    transactions.write(json.dumps({'request':request,'reply':reply},allow_nan=False)+'\n')
                    state=result
                    rows.append({'t':round((i+1)*dt,10),'state':reduced(state),'full_state':state,'energy_J':energy6(state)})
            return rows
        finally:
            if process is not None:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGTERM)
                    try: process.wait(timeout=5)
                    except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGKILL); process.wait()
                (directory/'process.json').write_text(json.dumps({'command':process.args,'returncode':process.returncode},indent=2),encoding='utf-8')


def errors(a,b):
    a=np.asarray(a,dtype=float); b=np.asarray(b,dtype=float)
    return {'position_m':float(np.max(np.linalg.norm(a[:,:2]-b[:,:2],axis=1))),
            'pitch_deg':float(np.max(np.abs(np.arctan2(np.sin(a[:,2]-b[:,2]),np.cos(a[:,2]-b[:,2]))))*180/math.pi),
            'velocity_mps':float(np.max(np.abs(a[:,3:5]-b[:,3:5]))),
            'pitch_rate_radps':float(np.max(np.abs(a[:,5]-b[:,5])))}


def at(rows,t):
    matches=[r for r in rows if abs(r['t']-t)<1e-8]
    if len(matches)!=1: raise ValueError(f'Missing or duplicate sample at {t}')
    return matches[0]


def assess(runs):
    report={'scope':'No-current numerical backend validation only; no PID or current validation','energy':{},'convergence':{},'consistency':{}}
    passes=[]
    for backend in ('python','xdyn'):
        for dt in STEPS:
            rows=runs[str(dt)][backend]
            if len(rows)!=round(HORIZON/dt)+1: raise ValueError('Incomplete trajectory')
            if not all(abs(r['t']-i*dt)<1e-8 for i,r in enumerate(rows)): raise ValueError('Non-contiguous trajectory')
            if not all(math.isfinite(v) for r in rows for v in [*r['state'],r['energy_J']]): raise ValueError('Invalid trajectory')
            e0=at(rows,12)['energy_J']; ef=at(rows,22)['energy_J']
            ok=e0>1e-12 and ef<=e0*1.01
            report['energy'][backend+'_'+str(dt)]={'passed':ok,'release_J':e0,'after_10s_J':ef,
                'ratio':ef/e0 if e0 else None,'max_after_release_J':max(r['energy_J'] for r in rows if r['t']>=12)}
            passes.append(ok)
        pairs=[]
        for coarse,fine in zip(STEPS,STEPS[1:]):
            a=runs[str(coarse)][backend]; b=runs[str(fine)][backend]
            endpoint=errors([at(a,20)['state']],[at(b,20)['state']])
            common=[r for r in a if r['t']<=20]; lookup={round(r['t'],8):r for r in b}
            trajectory=errors([r['state'] for r in common],[lookup[round(r['t'],8)]['state'] for r in common])
            pairs.append({'steps':[coarse,fine],'endpoint':endpoint,'max_common_time':trajectory})
        limits={'position_m':.02,'pitch_deg':.5,'velocity_mps':.01}
        # Numerical noise floors, orders of magnitude below acceptance limits.
        floors={'position_m':1e-8,'pitch_deg':1e-6,'velocity_mps':1e-8}
        ok=all(p['endpoint'][k]<=v for p in pairs for k,v in limits.items())
        ok=ok and all(pairs[1]['endpoint'][k]<=pairs[0]['endpoint'][k]+floors[k] for k in limits)
        report['convergence'][backend]={'passed':ok,'pairs':pairs,'limits':limits,'noise_floors':floors}; passes.append(ok)
    for dt in STEPS:
        a=[r for r in runs[str(dt)]['python'] if r['t']<=20]
        b=[r for r in runs[str(dt)]['xdyn'] if r['t']<=20]
        err=errors([r['state'] for r in a],[r['state'] for r in b])
        limits={'position_m':.05,'pitch_deg':1.,'velocity_mps':.02}
        ok=all(err[k]<=v for k,v in limits.items())
        # A planar comparison is invalid if full solver motion escaped the plane.
        plane=max(max(abs(r['full_state']['x']),abs(r['full_state']['v']),abs(r['full_state']['p']),abs(r['full_state']['r'])) for r in b)
        ok=ok and plane<1e-6
        report['consistency'][str(dt)]={'passed':ok,'max_errors':err,'limits':limits,'out_of_plane_max':plane}; passes.append(ok)
    report['passed']=all(passes)
    report['interpretation']='M4/M5/M6 plus a stricter restored-coupling response gate; scope is the tested no-current longitudinal motion.'
    return report


def diagnose(runs,ablated):
    # Extra revision-specific gate; original M4/M5/M6 thresholds remain intact.
    limits={'position_m':1e-6,'pitch_deg':1e-5,'velocity_mps':1e-6,'pitch_rate_radps':1e-6}
    matched={}
    for dt in STEPS:
        series=runs[str(dt)]
        err=errors([r['state'] for r in series['python'] if r['t']<=20],
                   [r['state'] for r in series['xdyn'] if r['t']<=20])
        matched[str(dt)]=err
    ablation=errors([r['state'] for r in ablated if r['t']<=20],
                    [r['state'] for r in runs[str(STEPS[0])]['xdyn'] if r['t']<=20])
    return {'passed':all(e[k]<=v for e in matched.values() for k,v in limits.items()),
            'purpose':'Verify restored added-mass coupling against unchanged Python reference',
            'limits':limits,'matched_errors':matched,
            'diagnostic_python_without_added_coriolis_vs_corrected_xdyn':ablation,
            'scope':'No-current longitudinal test only; not general six-DOF or current equivalence',
            'source_revision':'naval-group/lxdyn@363668b2b1f97e3177d2d617586fee6c4a727262'}


def reanalyze(directory):
    directory=Path(directory).resolve()
    try:
        manifest=json.loads((directory/'manifest.json').read_text(encoding='utf-8'))
        required={'protocol.json','diagnostic_reference_no_added_coriolis.json',*(f'states_{dt}.json' for dt in STEPS)}
        if not required.issubset(manifest): raise ValueError('Required evidence missing from manifest')
        for name,h in manifest.items():
            p=(directory/name).resolve()
            if not p.is_relative_to(directory) or not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=h:
                raise ValueError('Recording hash/path mismatch: '+name)
        runs={str(dt):json.loads((directory/f'states_{dt}.json').read_text(encoding='utf-8')) for dt in STEPS}
        report=assess(runs)
        ablated=json.loads((directory/'diagnostic_reference_no_added_coriolis.json').read_text(encoding='utf-8'))
        report['coupling_diagnostic']=diagnose(runs,ablated)
        report['recording_hashes']=True
        report['clean_shutdown']=all(json.loads((directory/f'dt_{dt}/process.json').read_text())['returncode'] in (0,-15) for dt in STEPS)
        report['passed']=report['passed'] and report['clean_shutdown'] and report['coupling_diagnostic']['passed']
        return report
    except Exception as exc: return {'passed':False,'error':str(exc)}


def run(output):
    output=Path(output).resolve(); output.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[2]
    sources=[*Path(__file__).parent.glob('*.py'),* (root/'physics').glob('*.py'),root/'simulation/vehicle_config.py',root/'config/vehicles/uuv_3dof_synthetic_v1.yaml']
    for source in sources:
        target=output/'source'/source.relative_to(root); target.parent.mkdir(parents=True,exist_ok=True); shutil.copy2(source,target)
    if os.environ.get('UUV_XDYN_LOCK'): shutil.copy2(os.environ['UUV_XDYN_LOCK'],output/'flake.lock')
    (output/'protocol.json').write_text(json.dumps({'steps':STEPS,'duration':HORIZON,'comparison_end':20,'release_time':12,'current':[0,0],'input':'0-2 zero;2-7 upper=lower=2;7-12 upper=-1 lower=1;12-22 zero','xdyn_executable':shutil.which('xdyn-for-cs')},indent=2),encoding='utf-8')
    runs={}; failure=None
    try:
        for dt in STEPS:
            print(f'Numerical validation dt={dt}: Python reference + real xdyn',flush=True)
            ref=reference_run(dt); ext=xdyn_run(output/('dt_'+str(dt)),dt)
            runs[str(dt)]={'python':ref,'xdyn':ext}
            (output/('states_'+str(dt)+'.json')).write_text(json.dumps(runs[str(dt)],allow_nan=False),encoding='utf-8')
        report=assess(runs)
        ablated=reference_run(STEPS[0],diagnostic_without_added_coriolis=True)
        (output/'diagnostic_reference_no_added_coriolis.json').write_text(json.dumps(ablated),encoding='utf-8')
        report['coupling_diagnostic']=diagnose(runs,ablated)
        report['clean_shutdown']=all(json.loads((output/f'dt_{dt}/process.json').read_text())['returncode'] in (0,-15) for dt in STEPS)
        report['passed']=report['passed'] and report['clean_shutdown'] and report['coupling_diagnostic']['passed']
    except Exception as exc:
        failure=str(exc); report={'passed':False,'error':failure}
    (output/'acceptance.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    manifest={p.relative_to(output).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in output.rglob('*') if p.is_file() and p.name!='manifest.json'}
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2),flush=True)
    if not report['passed']: raise SystemExit(2)
