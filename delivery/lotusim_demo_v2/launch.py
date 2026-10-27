"""LOTUSim interaction demo: explicit connection, live feedback and optional dynamics."""
import argparse
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid
from .verify import verify

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--connect-existing',action='store_true',help='Use existing MAS server; do not start or stop LOTUSim')
    p.add_argument('--namespace',default='/lotusim')
    p.add_argument('--headless',action='store_true')
    p.add_argument('--auto-close',action='store_true')
    p.add_argument('--compare',action='store_true')
    p.add_argument('--rate',type=float,default=1)
    p.add_argument('--start-delay',type=float,default=15)
    p.add_argument('--dynamics',action='store_true',help='Independent real xdyn actuator test, not trajectory replay')
    p.add_argument('--case',default='pitch_positive',choices=['zero','surge_positive','pitch_positive','heave_positive'])
    args=p.parse_args()
    import math
    if not math.isfinite(args.rate) or args.rate<=0 or not math.isfinite(args.start_delay) or args.start_delay<0: p.error('Invalid rate or delay')
    if args.connect_existing and args.dynamics: p.error('Dynamics owns its isolated world; cannot attach to an existing world')
    root=Path(__file__).resolve().parents[1]; verify(root)
    out=root/'runs'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:6])
    if args.dynamics:
        from ros2.minimal_xdyn.trial import run_trial
        from argparse import Namespace
        os.environ['UUV_XDYN_LOCK']=str(root/'flake.lock')
        print('REAL DYNAMICS: ideal thrust -> xdyn -> LOTUSim pose and full state. No PID.',flush=True)
        run_trial(Namespace(case=args.case,output=out,gui=not args.headless,keep_open=not args.auto_close)); return
    out.mkdir(parents=True); env=os.environ.copy()
    if not args.connect_existing:
        (out/'state/worlds').mkdir(parents=True)
        world_name='uuv_demo_'+uuid.uuid4().hex[:10]
        (out/'state/worlds/lotusim.world').write_text((root/'lotusim.world').read_text().replace('world name="lotusim"','world name="'+world_name+'"'))
        args.namespace='/'+world_name
        env.update(LOTUSIM_STATE_HOME=str(out/'state'),GZ_PARTITION='lu_demo_'+out.name,ROS_DOMAIN_ID='73')
    print('DISPLAY REPLAY: CREATE UUV + obstacles -> MOVE pose -> receive LOTUSim /poses.',flush=True)
    print('Output: '+str(out)+'; ROS_DOMAIN_ID='+env.get('ROS_DOMAIN_ID','0'),flush=True)
    server=None; replay=None; gui=None; code=1
    staged_gui=not args.headless and not args.connect_existing
    ready=out/'scene-ready.json'; gate=out/'start-playback'
    with (out/'gazebo.log').open('w') as log:
        try:
            if not args.connect_existing:
                if not shutil.which('lotusim'): raise RuntimeError('lotusim executable missing; use run.sh or --connect-existing')
                server=subprocess.Popen(['lotusim','run','lotusim.world'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,cwd=root)
            cmd=[sys.executable,'-m','demo.gazebo_player','--repo-root',str(root),'--record-dir',str(out/'recording'),'--namespace',args.namespace,'--rate',str(args.rate),'--start-delay',str(args.start_delay)]
            if args.compare: cmd+=['--arm','P1','--arm','APPROACH']
            if args.auto_close: cmd+=['--auto-close']
            if staged_gui: cmd+=['--scene-ready',str(ready),'--playback-gate',str(gate)]
            replay=subprocess.Popen(cmd,env=env,cwd=root,start_new_session=True)
            while replay.poll() is None:
                if server is not None and server.poll() is not None: raise RuntimeError('LOTUSim exited; see gazebo.log')
                if staged_gui and gui is None and ready.exists():
                    print('All UUVs created and state feedback received. Opening Gazebo with the complete scene...',flush=True)
                    # GUI-only process, same partition as the server. No second server.
                    gui=subprocess.Popen(['lotusim','-g','-v','1'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,cwd=root)
                    gate.touch()
                if gui is not None and gui.poll() is not None:
                    raise RuntimeError('Gazebo GUI closed; see gazebo.log')
                time.sleep(.2)
            code=replay.returncode
        except KeyboardInterrupt: code=130; print('Cleaning up demo entities...',flush=True)
        finally:
            for child in (replay,gui,server):
                if child is not None and child.poll() is None:
                    os.killpg(child.pid,signal.SIGINT)
                    try: child.wait(timeout=20)
                    except subprocess.TimeoutExpired: os.killpg(child.pid,signal.SIGKILL); child.wait()
    summary=out/'recording/summary.json'
    if summary.exists():
        result=json.loads(summary.read_text()); print(json.dumps(result,indent=2),flush=True)
        if code in (0,130) and result['status']=='completed' and result['gazebo_observed']: code=0
        elif code==0: code=2
    elif code==0: code=2
    raise SystemExit(code)

if __name__=='__main__': main()
