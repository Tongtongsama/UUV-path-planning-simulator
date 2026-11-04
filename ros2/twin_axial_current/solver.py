import json,os,signal,socket,subprocess,time
from ros2.twin_axial_v2.numerical import decode,reduced,energy6
PORT=12358

def xdyn_run(directory,dt,config_data,state0,input_schedule,horizon):
    from websockets.sync.client import connect
    with socket.socket() as port_test:
        port_test.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        port_test.bind(('127.0.0.1',PORT))
    directory.mkdir(parents=True)
    config=directory/'model.yml'; config.write_text(json.dumps(config_data,indent=2),encoding='utf-8')
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
                state=dict(state0)
                rows=[{'t':0.,'state':reduced(state),'full_state':state,'energy_J':energy6(state)}]
                for i in range(round(horizon/dt)):
                    t=round(i*dt,10)
                    if abs(state['t']-t)>1e-8: raise ValueError('Non-contiguous solver state')
                    request={'Dt':dt,'states':[state],'commands':input_schedule(t)}
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

