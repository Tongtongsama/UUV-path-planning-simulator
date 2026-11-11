"""Single-vessel WebSocket instrumentation and independent actuator watchdog.

Forwards integration to xdyn; never predicts or fabricates a state.
"""
import argparse
import json
import os
import threading
import time
from pathlib import Path
from ros2.twin_axial_v2.model import NAME, DT, KEYS
from ros2.twin_axial_v2.protocol import check_request, decode_reply, feedback, finite
from ros2.twin_axial_v2.allocation import allocate, wrench


class Gate:
    def __init__(self):
        self.lock=threading.Lock(); self.command=dict.fromkeys(KEYS,0.0)
        self.allocation=allocate({'X':0.0,'M':0.0})
        self.received=-1e30; self.stamp=-1e30; self.seq=-1; self.invalid=None

    def update(self, commands, stamp, seq, now):
        allocation=allocate(commands)
        values=allocation['allocated_commands']
        stamp=finite(stamp); now=finite(now)
        if isinstance(seq,bool) or not isinstance(seq,int): raise ValueError('Integer sequence required')
        with self.lock:
            if stamp < self.stamp or seq <= self.seq: raise ValueError('Command stamp/sequence went backward or repeated')
            self.allocation=allocation; self.command=values; self.stamp=stamp; self.seq=seq; self.received=now

    def applied(self, sim_time, now):
        with self.lock:
            fresh=(not self.invalid and 0 <= sim_time-self.stamp <= .25+1e-6 and now-self.received <= 1.0)
            return (dict(self.command) if fresh else dict.fromkeys(KEYS,0.0)),self.seq,not fresh,dict(self.allocation),self.stamp,self.received


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    from websockets.sync.server import serve
    from websockets.sync.client import connect
    gate=Gate(); busy=threading.Lock(); stop=threading.Event()
    command_path=args.output/'commands.ipc.jsonl'; command_pos=0
    def command_reader():
        nonlocal command_pos
        while not stop.is_set():
            if command_path.is_file():
                try:
                    with command_path.open(encoding='utf-8') as stream:
                        stream.seek(command_pos)
                        text=stream.read()
                    cut=text.rfind('\n')+1
                    block=text[:cut]
                    command_pos+=cut
                    for line in block.splitlines():
                        if not line.strip(): continue
                        data=json.loads(line)
                        gate.update(data['commands'],data['stamp'],int(data['sequence']),time.monotonic())
                except Exception as exc:
                    with gate.lock: gate.invalid=str(exc)
            time.sleep(.002)
    threading.Thread(target=command_reader,daemon=True).start()

    def handler(ws):
        if not busy.acquire(blocking=False): ws.close(1008,'One vessel only'); return
        try:
            with (args.output/'transactions.jsonl').open('a',encoding='utf-8',buffering=1) as log, \
                 (args.output/'state.jsonl').open('a',encoding='utf-8',buffering=1) as state_log:
                with connect('ws://127.0.0.1:12357',open_timeout=5) as solver:
                    previous=None; start=None; sequence=0; previous_state=None
                    for message in ws:
                        record={'sequence':sequence,'valid':False,'wall_monotonic':time.monotonic()}
                        try:
                            request=json.loads(message)
                            state=check_request(request,previous)
                            if previous_state is not None:
                                # Prove LOTUSim passed the previous solver state back, not a replay/reset.
                                mismatch=max(abs(state[k]-previous_state[k]) for k in ('x','y','z','u','v','w','p','q','r'))
                                dot=abs(sum(state[k]*previous_state[k] for k in ('qr','qi','qj','qk')))
                                if mismatch>1e-5 or abs(dot-1)>1e-5: raise ValueError('Gazebo state feedback discontinuity')
                            if start is None: start=state['t']
                            commands,cmd_seq,watchdog,allocation,stamp,received=gate.applied(state['t'],time.monotonic())
                            record.update(request_state=state,request_commands=request.get('commands'),
                                applied_commands=commands,applied_wrench=wrench(commands),allocation=allocation,
                                command_simulation_stamp=stamp,command_received_wall=received,
                                command_sequence=cmd_seq,watchdog_zero=watchdog,
                                start_time=state['t'],elapsed_start=state['t']-start,dt=DT)
                            request['commands']=commands
                            solver.send(json.dumps(request))
                            raw=solver.recv(timeout=3)
                            result=decode_reply(json.loads(raw),state['t'])
                            record.update(valid=True,solver_state_ned_frd=result,elapsed_end=result['t']-start,**feedback(result))
                            # Same untouched reply goes to LOTUSim. State feedback is solver ground truth,
                            # not a claim that GUI or sensors have consumed the result already.
                            ws.send(raw)
                            log.write(json.dumps(record,allow_nan=False)+'\n')
                            # Compact state only: keep the full transaction in
                            # transactions.jsonl for audit, but keep the ROS
                            # control message below the 1 KB UDP transport
                            # limit.  The controller only needs these fields.
                            compact={key:record[key] for key in
                                     ('sequence','valid','elapsed_end','solver_state_ned_frd',
                                      'pitch_nose_up_rad','watchdog_zero','applied_commands')}
                            compact['error']=None
                            state_log.write(json.dumps(compact,allow_nan=False)+'\n')
                            previous=result['t']; previous_state=result; sequence+=1
                        except Exception as exc:
                            record.update(valid=False,error=str(exc))
                            log.write(json.dumps(record,allow_nan=False)+'\n')
                            compact={'sequence':sequence,'valid':False,'error':str(exc),
                                     'watchdog_zero':True,'applied_commands':dict.fromkeys(KEYS,0.0)}
                            state_log.write(json.dumps(compact)+'\n')
                            ws.close(1011,'Invalid dynamics transaction; restart trial')
                            return
        except Exception as exc:
            (args.output/'gateway_error.txt').write_text(str(exc),encoding='utf-8')
        finally: busy.release()
    try:
        with serve(handler,'127.0.0.1',12356) as server:
            (args.output/'gateway.ready').write_text('ready',encoding='utf-8')
            print('Gateway ready at :12356 -> xdyn :12357',flush=True)
            server.serve_forever()
    finally:
        stop.set()

if __name__=='__main__': main()
