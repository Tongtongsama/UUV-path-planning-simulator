"""M6: pair pinned xdyn and approved Python 3-DoF under F1/F2 currents."""
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
from websockets.sync.client import connect
from physics.uuv_3dof import Python3DOFDynamics
from physics.integrators import RK4Integrator
from simulation.vehicle_config import load_synthetic_parameters
from ros2.twin_axial_v2.allocation import wrench
from ros2.twin_axial_v2.protocol import FIELDS
from .current_model import FLOWS
from .mission import model_config

DT=.05
STEPS=400
PORT=12358


def commands(t):
    if t<2: return {'upper(T)':0.,'lower(T)':0.}
    if t<7: return {'upper(T)':2.,'lower(T)':2.}
    if t<12: return {'upper(T)':-1.,'lower(T)':1.}
    return {'upper(T)':0.,'lower(T)':0.}


def run_solver(directory,flow):
    config=directory/'model.yml'
    config.write_text(json.dumps(model_config('P0',flow),indent=2),encoding='utf-8')
    state=dict(zip(FIELDS,(0.,0.,2.,30.,math.sqrt(.5),0.,0.,math.sqrt(.5),0.,0.,0.,0.,0.,0.)))
    rows=[]
    with (directory/'xdyn.log').open('w',encoding='utf-8') as log:
        process=subprocess.Popen(['xdyn-for-cs','--yml',str(config),'--solver','rk4',
            '--address','127.0.0.1','--port',str(PORT),'--dt',str(DT)],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+15
            while True:
                if process.poll() is not None: raise RuntimeError('xdyn exited during startup')
                try:ws=connect(f'ws://127.0.0.1:{PORT}',open_timeout=.5);break
                except OSError:
                    if time.monotonic()>deadline: raise RuntimeError('xdyn startup timeout')
                    time.sleep(.05)
            with ws:
                for i in range(STEPS):
                    t=round(i*DT,10)
                    ws.send(json.dumps({'Dt':DT,'states':[state],'commands':commands(t)}))
                    reply=json.loads(ws.recv(timeout=5))
                    if 'error' in reply: raise RuntimeError(str(reply['error']))
                    state={k:float(reply[k][-1]) for k in FIELDS}
                    if abs(state['t']-t-DT)>1e-8 or not all(math.isfinite(v) for v in state.values()):
                        raise ValueError('Invalid xdyn response')
                    pitch=math.asin(max(-1,min(1,2*(state['qr']*state['qj']-state['qk']*state['qi']))))
                    rows.append([state['y'],-state['z'],pitch,state['u'],state['w'],state['q'],state['v'],state['p'],state['r']])
        finally:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    (directory/'xdyn_rows.json').write_text(json.dumps(rows),encoding='utf-8')
    return np.array(rows,dtype=float)


def run_reference(directory,flow):
    dynamics=Python3DOFDynamics(load_synthetic_parameters())
    integrator=RK4Integrator()
    current=np.array(FLOWS[flow],dtype=float)
    state=np.array([2.,-30.,0.,0.,0.,0.],dtype=float)
    rows=[]
    for i in range(STEPS):
        applied=wrench(commands(i*DT))
        control=np.array([applied['X'],0.,applied['M']])
        state=integrator.step(lambda s:dynamics.derivatives(s,control,current),state,DT)
        rows.append(state.tolist())
    (directory/'reference_rows.json').write_text(json.dumps(rows),encoding='utf-8')
    return np.array(rows,dtype=float)


def validate(output):
    output=Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    lock=os.environ.get('UUV_XDYN_LOCK')
    if lock: shutil.copy2(lock,output/'flake.lock')
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        probe.bind(('127.0.0.1',PORT))
    report={'duration_s':STEPS*DT,'dt_s':DT,'flows':{},'scope':'M6 fixed open-loop inputs; not navigation success'}
    for flow in ('F1','F2'):
        folder=output/flow;folder.mkdir()
        actual=run_solver(folder,flow)
        expected=run_reference(folder,flow)
        error=np.abs(actual[:,:6]-expected)
        metrics={'max_x_m':float(error[:,0].max()),'max_z_m':float(error[:,1].max()),
            'max_pitch_deg':float(np.degrees(error[:,2].max())),
            'max_u_mps':float(error[:,3].max()),'max_w_mps':float(error[:,4].max()),
            'max_q_radps':float(error[:,5].max()),
            'max_lateral_mps':float(np.abs(actual[:,6]).max()),
            'max_roll_rate_radps':float(np.abs(actual[:,7]).max()),
            'max_yaw_rate_radps':float(np.abs(actual[:,8]).max())}
        metrics['passed']=max(metrics['max_x_m'],metrics['max_z_m'])<=.05 and metrics['max_pitch_deg']<=1 and max(metrics['max_u_mps'],metrics['max_w_mps'])<=.02
        report['flows'][flow]=metrics
        (folder/'comparison.json').write_text(json.dumps(metrics,indent=2),encoding='utf-8')
    report['passed']=all(row['passed'] for row in report['flows'].values())
    (output/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    (output/'manifest.json').write_text(json.dumps({str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(output.rglob('*')) if p.is_file()},indent=2),encoding='utf-8')
    return report
