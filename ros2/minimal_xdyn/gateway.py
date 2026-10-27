"""Single-vessel WebSocket instrumentation and independent actuator watchdog.

Forwards integration to xdyn; never predicts or fabricates a state.
"""
import argparse
import json
import threading
import time
from pathlib import Path
from .model import NAME, DT, KEYS
from .protocol import check_request, decode_reply, feedback, clamp


class Gate:
    def __init__(self):
        self.lock=threading.Lock(); self.command=dict.fromkeys(KEYS,0.0)
        self.received=-1e30; self.stamp=-1e30; self.seq=-1; self.invalid=None

    def update(self, commands, stamp, seq, now):
        values=clamp(commands)
        with self.lock:
            if stamp < self.stamp: raise ValueError('Command simulation stamp went backward')
            self.command=values; self.stamp=stamp; self.seq=seq; self.received=now

    def applied(self, sim_time, now):
        with self.lock:
            fresh=(not self.invalid and 0 <= sim_time-self.stamp <= .25+1e-6 and now-self.received <= 1.0)
            return (dict(self.command) if fresh else dict.fromkeys(KEYS,0.0)),self.seq,not fresh


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from lotusim_msgs.msg import VesselCmdArray
    from websockets.sync.server import serve
    from websockets.sync.client import connect
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('uuv_minimal_gateway')
    publisher=node.create_publisher(String,'/uuv_minimal/state',20)
    gate=Gate(); busy=threading.Lock()
    def command_cb(message):
        try:
            for cmd in message.cmds:
                if cmd.vessel_name==NAME:
                    stamp=cmd.header.stamp.sec+cmd.header.stamp.nanosec*1e-9
                    gate.update(json.loads(cmd.cmd_string),stamp,int(cmd.header.frame_id),time.monotonic())
        except Exception as exc:
            with gate.lock: gate.invalid=str(exc)
    sub=node.create_subscription(VesselCmdArray,'/uuv_minimal/vessel_cmd_array',command_cb,20)
    spinner=threading.Thread(target=lambda:rclpy.spin(node),daemon=True); spinner.start()

    def handler(ws):
        if not busy.acquire(blocking=False): ws.close(1008,'One vessel only'); return
        try:
            with (args.output/'transactions.jsonl').open('a',encoding='utf-8',buffering=1) as log:
                with connect('ws://127.0.0.1:12347',open_timeout=5) as solver:
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
                            commands,cmd_seq,watchdog=gate.applied(state['t'],time.monotonic())
                            record.update(request_state=state,request_commands=request.get('commands'),
                                applied_commands=commands,command_sequence=cmd_seq,watchdog_zero=watchdog,
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
                            msg=String(); msg.data=json.dumps(record,allow_nan=False); publisher.publish(msg)
                            previous=result['t']; previous_state=result; sequence+=1
                        except Exception as exc:
                            record.update(valid=False,error=str(exc))
                            log.write(json.dumps(record,allow_nan=False)+'\n')
                            msg=String(); msg.data=json.dumps(record); publisher.publish(msg)
                            ws.close(1011,'Invalid dynamics transaction; restart trial')
                            return
        except Exception as exc:
            (args.output/'gateway_error.txt').write_text(str(exc),encoding='utf-8')
        finally: busy.release()
    try:
        with serve(handler,'127.0.0.1',12346) as server:
            (args.output/'gateway.ready').write_text('ready',encoding='utf-8')
            print('Gateway ready at :12346 -> xdyn :12347',flush=True)
            server.serve_forever()
    finally:
        rclpy.shutdown(); spinner.join(timeout=2)

if __name__=='__main__': main()
