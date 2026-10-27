"""Launch isolated processes and send pulses scheduled by solver feedback time."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time
from .model import prepare,NAME,KEYS
from .protocol import pulse


def run_trial(args):
    # Check ports before starting anything; never terminate someone else's process.
    for port in (12346,12347):
        with socket.socket() as s:
            s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            s.bind(('127.0.0.1',port))
    output=prepare(args.output.resolve())
    source_dir=Path(__file__).parent
    (output/'source').mkdir()
    for source in source_dir.glob('*.py'): shutil.copy2(source,output/'source'/source.name)
    if os.environ.get('UUV_XDYN_LOCK'): shutil.copy2(os.environ['UUV_XDYN_LOCK'],output/'flake.lock')
    metadata={'case':args.case,'schema':1,'model':NAME,'dt':.05,'duration':8.0,
        'inputs':'ideal reversible thrust N, not RPM','dynamics':'xdyn full 6DOF; no state clamping',
        'restoring':'neutral buoyancy, artificial -6*sin(angle) roll/pitch torque',
        'frames':'solver NED/FRD; feedback also includes ENU/FLU pose',
        'executables':{k:shutil.which(k) for k in ('xdyn-for-cs','lotusim','python3')}}
    (output/'metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
    env=os.environ.copy()
    env['LOTUSIM_STATE_HOME']=str(output)
    env['GZ_PARTITION']='uuv_minimal_'+output.name
    # Isolate ROS from the usual replay world; child processes inherit this.
    env['ROS_DOMAIN_ID']='72'
    os.environ['ROS_DOMAIN_ID']='72'
    processes=[]; files=[]; node=None
    status='failed'; failure=None; cleanup_error=None
    def launch(command,filename):
        f=(output/filename).open('w',encoding='utf-8'); files.append(f)
        process=subprocess.Popen(command,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process); return process
    def wait_ready(test,process,timeout=20):
        end=time.monotonic()+timeout
        while not test():
            if process.poll() is not None: raise RuntimeError(f'Process exited: {process.args}; see logs')
            if time.monotonic()>end: raise RuntimeError('Startup timeout; see process logs')
            time.sleep(.1)
    def solver_ready():
        try:
            with socket.create_connection(('127.0.0.1',12347),timeout=.2): return True
        except OSError: return False
    try:
        solver=launch(['xdyn-for-cs','--yml',str(output/'model.yml'),'--solver','rk4','--address','127.0.0.1','--port','12347','--dt','0.05'],'xdyn.log')
        wait_ready(solver_ready,solver)
        gateway=launch([sys.executable,'-m','ros2.minimal_xdyn.gateway','--output',str(output)],'gateway.log')
        wait_ready(lambda:(output/'gateway.ready').exists(),gateway)
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import String
        from lotusim_msgs.msg import VesselCmd,VesselCmdArray,VesselPositionArray
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node=rclpy.create_node('uuv_minimal_open_loop')
        pub=node.create_publisher(VesselCmdArray,'/uuv_minimal/vessel_cmd_array',20)
        latest=None; seen=None; seq=0
        observed=(output/'gazebo_poses.jsonl').open('w',encoding='utf-8',buffering=1); files.append(observed)
        sent=(output/'commands.jsonl').open('w',encoding='utf-8',buffering=1); files.append(sent)
        def state_cb(msg):
            nonlocal latest,seen
            latest=json.loads(msg.data); seen=time.monotonic()
        def pose_cb(msg):
            for v in msg.vessels:
                if v.vessel_name==NAME:
                    p=v.pose.position; q=v.pose.orientation
                    observed.write(json.dumps({'time':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                        'position':[p.x,p.y,p.z],'quaternion_xyzw':[q.x,q.y,q.z,q.w]})+'\n')
        sub=node.create_subscription(String,'/uuv_minimal/state',state_cb,20)
        sub_pose=node.create_subscription(VesselPositionArray,'/uuv_minimal/poses',pose_cb,20)
        sim=launch(['lotusim','run']+(['--gui'] if args.gui else [])+['--debug','minimal.world'],'gazebo.log')
        print(f'Trial {args.case}; recording to {output}',flush=True)
        deadline=time.monotonic()+90; last_sequence=-1
        def publish(commands,stamp):
            nonlocal seq
            cmd=VesselCmd(); cmd.vessel_name=NAME; cmd.cmd_string=json.dumps(commands)
            cmd.header.frame_id=str(seq)
            ns=round(stamp*1e9); cmd.header.stamp.sec=ns//10**9; cmd.header.stamp.nanosec=ns%10**9
            message=VesselCmdArray()
            message.cmds=[cmd]; pub.publish(message)
            sent.write(json.dumps({'sequence':seq,'simulation_stamp':stamp,'commands':commands})+'\n'); seq+=1
        while True:
            rclpy.spin_once(node,timeout_sec=.05)
            if any(p.poll() is not None for p in processes): raise RuntimeError('A child process exited; inspect logs')
            if time.monotonic()>deadline: raise RuntimeError('Trial wall-clock timeout (paused or stalled)')
            if seen is not None and time.monotonic()-seen>3: raise RuntimeError('State feedback stale for >3 seconds')
            if latest is None or latest['sequence']==last_sequence: continue
            if not latest['valid']: raise RuntimeError(latest.get('error','Invalid solver state'))
            last_sequence=latest['sequence']; elapsed=latest['elapsed_end']
            if args.case=='watchdog':
                # Send nonzero only during [2,3), then go silent; gateway must zero it.
                if elapsed<3: publish(dict(zip(KEYS,[2.0 if elapsed>=2 else 0.0,0.0,0.0])),latest['solver_state_ned_frd']['t'])
            else:
                publish(pulse(args.case,elapsed),latest['solver_state_ned_frd']['t'])
            if elapsed>=8:
                publish(dict.fromkeys(KEYS,0.0),latest['solver_state_ned_frd']['t'])
                status='completed'; break
        if args.keep_open:
            print('Pulse complete, commands zero. Ctrl+C closes this trial.',flush=True)
            while True: rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:
        status='interrupted' if status!='completed' else status
    except Exception as exc:
        failure=str(exc); print(f'FAILED: {failure}',file=sys.stderr,flush=True)
    finally:
        # Remove this bench's own entity while the server still runs, so the
        # xdyn socket is released before Gazebo destroys its static maps.
        if node is not None and status=='completed':
            client=None
            try:
                from rclpy.action import ActionClient
                from lotusim_msgs.action import MASCmdArray
                from lotusim_msgs.msg import MASCmd
                client=ActionClient(node,MASCmdArray,'/uuv_minimal/mas_cmd_array')
                if not client.wait_for_server(timeout_sec=2): raise RuntimeError('MAS cleanup unavailable')
                command=MASCmd(); command.cmd_type=MASCmd.DELETE_CMD; command.vessel_name=NAME
                goal=MASCmdArray.Goal(); goal.cmd=[command]
                future=client.send_goal_async(goal)
                rclpy.spin_until_future_complete(node,future,timeout_sec=2)
                if not future.done() or not future.result().accepted: raise RuntimeError('MAS cleanup not accepted')
                future=future.result().get_result_async()
                rclpy.spin_until_future_complete(node,future,timeout_sec=2)
                if not future.done() or future.result().status!=4: raise RuntimeError('MAS cleanup failed')
                time.sleep(.15)
            except Exception as exc: cleanup_error=str(exc)
            finally:
                if client is not None: client.destroy()
        # Own process groups only. Stop Gazebo first: no further integration.
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGTERM)
                try: p.wait(timeout=4)
                except subprocess.TimeoutExpired: os.killpg(p.pid,signal.SIGKILL); p.wait()
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        for f in files: f.close()
        (output/'run_status.json').write_text(json.dumps({'status':status,'error':failure,'cleanup_error':cleanup_error,
            'process_exit_codes':[{ 'command':p.args,'returncode':p.returncode} for p in processes]},indent=2),encoding='utf-8')
        (output/'manifest.json').write_text(json.dumps({str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(output.rglob('*')) if p.is_file() and not p.is_symlink()
            and p.name not in ('manifest.json','acceptance.json')},indent=2),encoding='utf-8')
    from .acceptance import analyze_run
    report=analyze_run(output)
    (output/'acceptance.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2),flush=True)
    if not report['passed']: raise SystemExit(2)
