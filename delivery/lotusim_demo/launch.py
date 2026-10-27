"""One terminal launches isolated LOTUSim and the packaged replay."""
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
    p.add_argument('--headless',action='store_true')
    p.add_argument('--auto-close',action='store_true')
    p.add_argument('--rate',type=float,default=1)
    p.add_argument('--compare',action='store_true')
    p.add_argument('--start-delay',type=float,default=10)
    p.add_argument('--validate-only',action='store_true')
    args=p.parse_args()
    import math
    if not math.isfinite(args.rate) or args.rate<=0 or not math.isfinite(args.start_delay) or args.start_delay<0:
        p.error('rate must be positive; start-delay must be nonnegative; both must be finite')
    root=Path(__file__).resolve().parents[1]; verify(root)
    if args.validate_only:
        from .gazebo_player import main as player
        player(['--repo-root',str(root),'--validate-only']); return
    out=root/'runs'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:6])
    (out/'state/worlds').mkdir(parents=True)
    shutil.copy2(root/'lotusim.world',out/'state/worlds/lotusim.world')
    env=os.environ.copy(); env.update(LOTUSIM_STATE_HOME=str(out/'state'),GZ_PARTITION='lu_demo_'+out.name,ROS_DOMAIN_ID='73')
    print('Recording and logs: '+str(out),flush=True)
    server=None; replay=None; exit_code=1
    with (out/'gazebo.log').open('w') as log:
        try:
            server=subprocess.Popen(['lotusim','run']+([] if args.headless else ['--gui'])+['lotusim.world'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,cwd=root)
            cmd=[sys.executable,'-m','demo.gazebo_player','--repo-root',str(root),'--record-dir',str(out/'recording'),'--rate',str(args.rate),'--start-delay',str(args.start_delay)]
            if args.compare: cmd+=['--arm','P1','--arm','APPROACH']
            if args.auto_close: cmd+=['--auto-close']
            replay=subprocess.Popen(cmd,env=env,cwd=root,start_new_session=True)
            while replay.poll() is None:
                if server.poll() is not None: raise RuntimeError('LOTUSim exited; see gazebo.log')
                time.sleep(.2)
            exit_code=replay.returncode
        except KeyboardInterrupt:
            print('Closing this demo only...',flush=True); exit_code=130
        finally:
            for process in (replay,server):
                if process is not None and process.poll() is None:
                    os.killpg(process.pid,signal.SIGINT)
                    try: process.wait(timeout=20)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid,signal.SIGKILL); process.wait()
    summary=out/'recording/summary.json'
    if summary.exists():
        result=json.loads(summary.read_text()); print(json.dumps(result,indent=2),flush=True)
        if result['status']=='completed' and result['gazebo_observed'] and exit_code in (0,130): exit_code=0
        elif exit_code==0: exit_code=2
    elif exit_code==0: exit_code=2
    raise SystemExit(exit_code)

if __name__=='__main__': main()
