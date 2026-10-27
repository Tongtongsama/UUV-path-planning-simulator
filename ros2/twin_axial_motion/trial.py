"""Run PID from actual solver feedback through ROS, allocation and LOTUSim."""
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
from ros2.twin_axial_v2.model import prepare,NAME,KEYS
from .protocol import guidance, duration, FAULT_CASES, FAULT_TIME, OUTER, CRITERIA
from .safety import FeedbackGuard, zero_command_stamp
from ros2.twin_axial_pid.control import AllocatedPID, parameters
from dataclasses import asdict


def run_trial(args):
    # Check ports before starting anything; never terminate someone else's process.
    for port in (12356,12357):
        with socket.socket() as s:
            s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            s.bind(('127.0.0.1',port))
    output=prepare(args.output.resolve())
    root=Path(__file__).resolve().parents[2]
    for folder in ('ros2/twin_axial_motion','ros2/twin_axial_pid','ros2/twin_axial_v2','controller','core'):
        for source in (root/folder).glob('*.py'):
            destination=output/'source'/source.relative_to(root)
            destination.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(source,destination)
    (output/'controller.json').write_text(json.dumps({'parameters':asdict(parameters()),
        'tracking_time_s':1.0,'outer':OUTER,'criteria':CRITERIA,'allocation':'unchanged v2 joint direction-preserving scaling'},indent=2))
    if os.environ.get('UUV_XDYN_LOCK'): shutil.copy2(os.environ['UUV_XDYN_LOCK'],output/'flake.lock')
    metadata={'case':args.case,'schema':1,'experiment':'no-current motion and closed-loop fault injection','model':NAME,'dt':.05,'duration':duration(args.case),
        'inputs':'requested X (N), M (Nm) -> direction-preserving allocation -> upper/lower ideal thrust N; no direct heave','dynamics':'xdyn full 6DOF; no state clamping',
        'restoring':'neutral buoyancy, artificial -6*sin(angle) roll/pitch torque',
        'frames':'solver NED/FRD; feedback also includes ENU/FLU pose',
        'executables':{k:shutil.which(k) for k in ('xdyn-for-cs','lotusim','python3')}}
    (output/'metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
    env=os.environ.copy()
    env['LOTUSIM_STATE_HOME']=str(output)
    env['GZ_PARTITION']='uuv_twin_axial_'+output.name
    # Isolate ROS from the usual replay world; child processes inherit this.
    env['ROS_DOMAIN_ID']='74'
    os.environ['ROS_DOMAIN_ID']='74'
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
            with socket.create_connection(('127.0.0.1',12357),timeout=.2): return True
        except OSError: return False
    try:
        solver=launch(['xdyn-for-cs','--yml',str(output/'model.yml'),'--solver','rk4','--address','127.0.0.1','--port','12357','--dt','0.05'],'xdyn.log')
        wait_ready(solver_ready,solver)
        gateway=launch([sys.executable,'-m','ros2.twin_axial_v2.gateway','--output',str(output)],'gateway.log')
        wait_ready(lambda:(output/'gateway.ready').exists(),gateway)
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import String
        from lotusim_msgs.msg import VesselCmd,VesselCmdArray,VesselPositionArray
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node=rclpy.create_node('uuv_twin_axial_motion')
        pub=node.create_publisher(VesselCmdArray,'/uuv_twin_axial/vessel_cmd_array',20)
        latest=None; seen=None; seq=0; injected=False; halted=False
        guard=FeedbackGuard()
        observed=(output/'gazebo_poses.jsonl').open('w',encoding='utf-8',buffering=1); files.append(observed)
        sent=(output/'commands.jsonl').open('w',encoding='utf-8',buffering=1); files.append(sent)
        events=(output/'events.jsonl').open('w',encoding='utf-8',buffering=1); files.append(events)
        deliveries=(output/'feedback_delivery.jsonl').open('w',encoding='utf-8',buffering=1); files.append(deliveries)
        def event(event_type,**details):
            events.write(json.dumps({'event':event_type,'wall_monotonic':time.monotonic(),
                'elapsed':latest['elapsed_end'] if latest else None,**details})+'\n')
        def state_cb(msg):
            nonlocal latest,seen,injected
            latest=json.loads(msg.data); seen=time.monotonic()
            if args.case in FAULT_CASES and latest.get('elapsed_end',0)>=FAULT_TIME and not injected:
                injected=True
                event('fault_injected',kind=args.case,last_feedback_stamp=(guard.latest['solver_state_ned_frd']['t'] if guard.latest else None))
            delivered=latest
            if injected and args.case=='state_loss': delivered=None
            if injected and args.case=='stale_state': delivered=guard.latest
            deliveries.write(json.dumps({'wall_monotonic':seen,'observer_sequence':latest['sequence'],
                'observer_elapsed':latest.get('elapsed_end'),
                'delivered_sequence':delivered['sequence'] if delivered else None,
                'delivered_stamp':delivered['solver_state_ned_frd']['t'] if delivered and delivered.get('valid') else None})+'\n')
            if delivered is not None: guard.receive(delivered,seen)
        def pose_cb(msg):
            for v in msg.vessels:
                if v.vessel_name==NAME:
                    p=v.pose.position; q=v.pose.orientation
                    observed.write(json.dumps({'time':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                        'position':[p.x,p.y,p.z],'quaternion_xyzw':[q.x,q.y,q.z,q.w]})+'\n')
        sub=node.create_subscription(String,'/uuv_twin_axial/state',state_cb,20)
        sub_pose=node.create_subscription(VesselPositionArray,'/uuv_twin_axial/poses',pose_cb,20)
        sim=launch(['lotusim','run','--debug','minimal.world'],'gazebo.log')
        print(f'Trial {args.case}; recording to {output}',flush=True)
        deadline=time.monotonic()+360; last_sequence=-1; last_stamp=None
        controller=AllocatedPID()
        def publish(commands,stamp,diagnostics=None):
            nonlocal seq
            cmd=VesselCmd(); cmd.vessel_name=NAME; cmd.cmd_string=json.dumps(commands)
            cmd.header.frame_id=str(seq)
            ns=round(stamp*1e9); cmd.header.stamp.sec=ns//10**9; cmd.header.stamp.nanosec=ns%10**9
            message=VesselCmdArray()
            message.cmds=[cmd]
            delivered=not (injected and args.case=='command_loss')
            if delivered: pub.publish(message)
            sent.write(json.dumps({'sequence':seq,'simulation_stamp':stamp,'commands':commands,'controller':diagnostics,'delivered':delivered,'wall_monotonic':time.monotonic()})+'\n'); seq+=1
        while True:
            rclpy.spin_once(node,timeout_sec=.02)
            if any(p.poll() is not None for p in processes): raise RuntimeError('A child process exited; inspect logs')
            if time.monotonic()>deadline: raise RuntimeError('Trial wall-clock timeout')
            if latest is None: continue
            if not latest['valid']: raise RuntimeError(latest.get('error','Invalid solver state'))
            # Observer is used only for experiment termination and safety monitoring.
            # Feedback-loss tests never substitute this state into the PID.
            elapsed=latest['elapsed_end']
            guard.check_timeout(time.monotonic())
            if elapsed>2.2 and latest['watchdog_zero']:
                guard.trip('command_watchdog')
            if guard.reason and not halted:
                before=asdict(controller.memory); controller.reset(); halted=True
                event('controller_halted',reason=guard.reason,integral_before=before,
                      integral_after=asdict(controller.memory),last_controller_stamp=last_stamp)
                if last_stamp is not None: publish({'X':0.0,'M':0.0},last_stamp)
                expected={'state_loss':'state_timeout','stale_state':'stale_state','command_loss':'command_watchdog'}
                # State loss may hit the independent simulation-time watchdog first.
                allowed=({expected[args.case],'command_watchdog'} if args.case=='state_loss' else {expected.get(args.case)})
                if not injected or guard.reason not in allowed:
                    raise RuntimeError('Unexpected closed-loop safety stop: '+guard.reason)
            if elapsed>=duration(args.case):
                if not halted:
                    publish({'X':0.0,'M':0.0},latest['solver_state_ned_frd']['t'])
                    controller.reset()
                if elapsed>=duration(args.case)+.15:
                    if args.case in FAULT_CASES and not halted: raise RuntimeError('Fault did not stop controller')
                    status='fault_observed' if args.case in FAULT_CASES else 'completed'
                    break
                continue
            if halted: continue
            accepted=guard.latest
            if accepted is None or accepted['sequence']==last_sequence: continue
            last_sequence=accepted['sequence']; stamp=accepted['solver_state_ned_frd']['t']; last_stamp=stamp
            desired,outer=guidance(args.case,accepted)
            commands,diagnostics=controller.command(accepted,desired,.05)
            diagnostics['outer']=outer
            publish(commands,stamp,diagnostics)
    except KeyboardInterrupt:
        status='interrupted' if status!='completed' else status
    except Exception as exc:
        failure=str(exc); print(f'FAILED: {failure}',file=sys.stderr,flush=True)
    finally:
        # On failure/stale state, reset memory and send only zero, never old thrust.
        if node is not None and 'controller' in locals():
            controller.reset()
            if last_stamp is not None:
                try:
                    publish({'X':0.0,'M':0.0},zero_command_stamp(last_stamp,latest))
                    rclpy.spin_once(node,timeout_sec=.05)
                except Exception: pass
        # Remove this bench's own entity while the server still runs, so the
        # xdyn socket is released before Gazebo destroys its static maps.
        if node is not None:
            client=None
            try:
                from rclpy.action import ActionClient
                from lotusim_msgs.action import MASCmdArray
                from lotusim_msgs.msg import MASCmd
                client=ActionClient(node,MASCmdArray,'/uuv_twin_axial/mas_cmd_array')
                if not client.wait_for_server(timeout_sec=2): raise RuntimeError('MAS cleanup unavailable')
                command=MASCmd(); command.cmd_type=MASCmd.DELETE_CMD; command.vessel_name=NAME
                goal=MASCmdArray.Goal(); goal.cmd=[command]
                future=client.send_goal_async(goal)
                rclpy.spin_until_future_complete(node,future,timeout_sec=2)
                if not future.done() or not future.result().accepted: raise RuntimeError('MAS cleanup not accepted')
                future=future.result().get_result_async()
                rclpy.spin_until_future_complete(node,future,timeout_sec=2)
                if not future.done() or future.result().status!=4 or not all(future.result().result.result): raise RuntimeError('MAS cleanup failed')
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
