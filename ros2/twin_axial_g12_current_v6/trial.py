"""Run PID from actual solver feedback through ROS, allocation and LOTUSim."""
from .ipc import JsonlReader
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time
from ros2.twin_axial_v2.model import NAME,KEYS
from ros2.twin_axial_g12_current_v6.mission import prepare,Mission,CONFIG
from ros2.twin_axial_motion.safety import zero_command_stamp
from ros2.twin_axial_pid.control import AllocatedPID, parameters
from dataclasses import asdict


def run_trial(args):
    from .gate import pilot_evaluate, guard, RELEASE
    proof = getattr(args, "gate_proof", None)
    if proof is None:
        proof = pilot_evaluate(args.normal_probe,args.repeat_probe,args.abort_probe)
    guard(proof, runtime=True, gui=args.gui)
    args.real_started=False
    # Check ports before starting anything; never terminate someone else's process.
    for port in (12356,12357):
        with socket.socket() as s:
            s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            s.bind(('127.0.0.1',port))
    output=prepare(args.output.resolve(),args.case,args.scene,args.gui,args.flow)
    (output/'evidence_gate.json').write_text(json.dumps(proof.report,indent=2),encoding='utf-8')
    shutil.copy2(RELEASE,output/'adapter_release.json')
    for label,path in proof.probes.items():shutil.copytree(path,output/'transport_evidence'/label,symlinks=True)
    mission=Mission(json.loads((output/'plan.json').read_text()),(output/'worlds/minimal.world').read_text())
    root=Path(__file__).resolve().parents[2]
    release=json.loads(RELEASE.read_text(encoding='utf-8'))
    for name in release['files']:
        destination=output/'source'/name
        destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(root/name,destination)
    for folder in ('ros2/twin_axial_g12_real_v3','ros2/twin_axial_g12_real_v2','ros2/twin_axial_g12_real','ros2/twin_axial_g12_matrix','ros2/twin_axial_s3_transient','ros2/twin_axial_s3_transient_v2','simulation', 'config/vehicles','ros2/twin_axial_feasible_matrix','ros2/twin_axial_feasible_v2','ros2/twin_axial_navigation','ros2/twin_axial_motion','ros2/twin_axial_pid','ros2/twin_axial_v2','controller','core','planner','guidance','navigation','environment','trajectory','physics'):
        for source in (root/folder).rglob('*.py'):
            destination=output/'source'/source.relative_to(root)
            destination.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(source,destination)
    (output/'controller.json').write_text(json.dumps({'parameters':asdict(parameters()),
        'tracking_time_s':1.0,'max_pitch_target_deg':mission.config.get('pitch_limit_deg',60),
        'allocation':'unchanged v2 joint direction-preserving scaling'},indent=2))
    if os.environ.get('UUV_XDYN_LOCK'): shutil.copy2(os.environ['UUV_XDYN_LOCK'],output/'flake.lock')
    metadata={'case':args.case,'scene':args.scene,'flow':args.flow,'current_enu_xz_mps':mission.config['current'],
        'schema':1,'experiment':f'{args.scene} {args.flow} LOS navigation','model':NAME,'dt':.05,'duration':CONFIG['timeout_s'],
        'inputs':'requested X (N), M (Nm) -> direction-preserving allocation -> upper/lower ideal thrust N; no direct heave','dynamics':'xdyn full 6DOF; no state clamping',
        'restoring':'neutral buoyancy, artificial -6*sin(angle) roll/pitch torque',
        'frames':'solver NED/FRD; feedback also includes ENU/FLU pose',
        'executables':{k:shutil.which(k) for k in ('xdyn-for-cs','lotusim','python3')}}
    (output/'metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
    profile=str(Path(__file__).with_name('loopback.xml').resolve())
    os.environ.update(FASTRTPS_DEFAULT_PROFILES_FILE=profile,FASTDDS_DEFAULT_PROFILES_FILE=profile,FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
    env=os.environ.copy()
    env['LOTUSIM_STATE_HOME']=str(output)
    env['GZ_PARTITION']='uuv_twin_axial_'+output.name
    # Isolate ROS from the usual replay world; child processes inherit this.
    env['ROS_DOMAIN_ID']='74'
    env['FASTDDS_BUILTIN_TRANSPORTS']='UDPv4'
    os.environ['ROS_DOMAIN_ID']='74'
    os.environ['FASTDDS_BUILTIN_TRANSPORTS']='UDPv4'
    processes=[]; files=[]; node=None; mas_client=None
    status='failed'; failure=None; cleanup_error=None
    zero_confirmed=False; termination_time=None
    def launch(command,filename):
        f=(output/filename).open('w',encoding='utf-8'); files.append(f)
        process=subprocess.Popen(command,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        args.real_started=True
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
        gateway=launch([sys.executable,'-m','ros2.twin_axial_g12_current_v6.gateway','--output',str(output)],'gateway.log')
        wait_ready(lambda:(output/'gateway.ready').exists(),gateway)
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import String
        from lotusim_msgs.msg import VesselCmd,VesselCmdArray,VesselPositionArray
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node=rclpy.create_node('uuv_twin_axial_navigation')
        print('DOMAIN',rclpy.get_default_context().get_domain_id(),
              'RMW',rclpy.get_rmw_implementation_identifier(),
              'LOCALHOST_ONLY',os.environ.get('ROS_LOCALHOST_ONLY'),
              'HOST',socket.gethostname(),flush=True)
        time.sleep(1)
        print('STATE_PUBLISHERS',node.count_publishers('/uuv_twin_axial/state'),flush=True)
        pub=node.create_publisher(VesselCmdArray,'/uuv_twin_axial/vessel_cmd_array',20)
        latest=None; seen=None; seq=0; pose_seen=False; state_seen=0
        observed=(output/'gazebo_poses.jsonl').open('w',encoding='utf-8',buffering=1); files.append(observed)
        sent=(output/'commands.jsonl').open('w',encoding='utf-8',buffering=1); files.append(sent)
        command_ipc=(output/'commands.ipc.jsonl').open('w',encoding='utf-8',buffering=1); files.append(command_ipc)
        navlog=(output/'navigation.jsonl').open('w',encoding='utf-8',buffering=1); files.append(navlog)
        def state_cb(msg):
            nonlocal latest,seen,state_seen
            state_seen+=1
            if state_seen<=3: print('STATE_CB',len(msg.data),flush=True)
            latest=json.loads(msg.data); seen=time.monotonic()
        def pose_cb(msg):
            nonlocal pose_seen
            for v in msg.vessels:
                if v.vessel_name==NAME:
                    p=v.pose.position; q=v.pose.orientation
                    pose_seen=True
                    observed.write(json.dumps({'time':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                        'position':[p.x,p.y,p.z],'quaternion_xyzw':[q.x,q.y,q.z,q.w]})+'\n')
        sub=node.create_subscription(String,'/uuv_twin_axial/state',state_cb,20)
        sub_pose=node.create_subscription(VesselPositionArray,'/uuv_twin_axial/poses',pose_cb,20)
        sim=launch(['lotusim','run']+(['--gui'] if args.gui else [])+['--debug','minimal.world'],'gazebo.log')
        print(f'Trial {args.case}; recording to {output}',flush=True)
        # No blocking DDS diagnostic loop while dynamics are producing states.
        state_reader=JsonlReader(output/'state.jsonl')
        read_states=state_reader.read
        pending=[]
        deadline=time.monotonic()+600; last_sequence=-1; last_stamp=None
        controller=AllocatedPID(max_pitch_target=math.radians(mission.config.get('pitch_limit_deg',60)))
        def publish(commands,stamp,diagnostics=None):
            nonlocal seq
            cmd=VesselCmd(); cmd.vessel_name=NAME; cmd.cmd_string=json.dumps(commands)
            cmd.header.frame_id=str(seq)
            ns=round(stamp*1e9); cmd.header.stamp.sec=ns//10**9; cmd.header.stamp.nanosec=ns%10**9
            command_ipc.write(json.dumps({'sequence':seq,'stamp':stamp,'commands':commands})+'\n')
            sent.write(json.dumps({'sequence':seq,'simulation_stamp':stamp,'commands':commands,'controller':diagnostics})+'\n'); seq+=1
        def process(latest):
            nonlocal last_sequence,last_stamp,status,zero_confirmed,termination_time,state_seen
            if latest['sequence']==last_sequence: return False
            if not latest['valid']: raise RuntimeError(latest.get('error','Invalid solver state'))
            if last_sequence>=0 and latest['sequence']!=last_sequence+1:
                raise RuntimeError('Skipped or reordered state feedback; controller stopped')
            last_sequence=latest['sequence']; elapsed=latest['elapsed_end']
            stamp=latest['solver_state_ned_frd']['t']
            if last_stamp is not None and abs(stamp-last_stamp-.05)>1e-6:
                raise RuntimeError('Non-contiguous controller simulation time')
            last_stamp=stamp
            desired,navigation=mission.update(latest)
            navlog.write(json.dumps({'sequence':latest['sequence'],**navigation})+'\n')
            if elapsed>=CONFIG['timeout_s'] and not mission.arrived:
                raise RuntimeError('Navigation simulation timeout')
            if elapsed>2.2 and latest['watchdog_zero']:
                raise RuntimeError('Unexpected watchdog stop')
            if mission.arrived:
                publish({'X':0.0,'M':0.0},stamp)
                controller.reset()
                applied=latest.get('applied_commands') or {}
                if applied and all(abs(value)<=1e-12 for value in applied.values()):
                    zero_confirmed=True
                if elapsed>=mission.arrival_time+.15:
                    for extra in list(pending):
                        if extra.get('valid'):
                            _, extra_nav=mission.update(extra)
                            navlog.write(json.dumps({'sequence':extra['sequence'],**extra_nav})+'\n')
                    pending.clear()
                    status='completed'; termination_time=elapsed; return True
                return False
            commands,diagnostics=controller.command(latest,desired,.05)
            diagnostics['navigation']=navigation
            publish(commands,stamp,diagnostics)
            return False

        while True:
            rclpy.spin_once(node,timeout_sec=.001)
            if any(p.poll() is not None for p in processes): raise RuntimeError('A child process exited; inspect logs')
            if time.monotonic()>deadline: raise RuntimeError('Trial wall-clock timeout (paused or stalled)')
            pending.extend(read_states())
            if not pending:
                if seen is not None and time.monotonic()-seen>.25: raise RuntimeError('State feedback stale for >0.25 wall seconds')
                time.sleep(.005)
                continue
            while pending:
                latest=pending.pop(0); seen=time.monotonic(); state_seen+=1
                if process(latest): break
            if status=='completed': break
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
        # A completed run with confirmed zero input stops its own integration
        # before any potentially blocking MAS cleanup.  Gateway first, then
        # xdyn; Gazebo stays alive for the DELETE action.  This is a controlled
        # stop, not a deleted tail: every state recorded up to this point is
        # retained and the normal acceptance checks still apply.
        def stop_process(process):
            if process is None or process.poll() is not None:
                return
            os.killpg(process.pid,signal.SIGTERM)
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
                process.wait()

        if True:  # Both success and failure stop integration before MAS waiting.
            stop_process(locals().get('gateway'))
            stop_process(locals().get('solver'))
        # Remove this bench's own entity while the server still runs.
        if node is not None:
            client=mas_client
            try:
                from lotusim_msgs.msg import MASCmd
                if client is None:
                    from rclpy.action import ActionClient
                    from lotusim_msgs.action import MASCmdArray
                    client=ActionClient(node,MASCmdArray,'/uuv_twin_axial/mas_cmd_array')
                if not client.wait_for_server(timeout_sec=2): raise RuntimeError('MAS cleanup unavailable')
                command=MASCmd(); command.cmd_type=MASCmd.DELETE_CMD; command.vessel_name=NAME
                goal=MASCmdArray.Goal(); goal.cmd=[command]
                future=client.send_goal_async(goal)
                rclpy.spin_until_future_complete(node,future,timeout_sec=2)
                if not future.done() or not future.result().accepted: raise RuntimeError('MAS cleanup not accepted')
                future=future.result().get_result_async()
                rclpy.spin_until_future_complete(node,future,timeout_sec=2)
                if not future.done() or future.result().status!=4 or not future.result().result.result or not all(future.result().result.result): raise RuntimeError('MAS cleanup failed')
                time.sleep(.15)
            except Exception as exc: cleanup_error=str(exc)
            finally:
                if client is not None: client.destroy()
        # Own process groups only.  Stop the remaining Gazebo process; no
        # further integration is possible because gateway/xdyn are already
        # gone on the completed path.
        for p in reversed(processes):
            stop_process(p)
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        for f in files: f.close()
        (output/'run_status.json').write_text(json.dumps({'status':status,'error':failure,'cleanup_error':cleanup_error,'real_started':args.real_started,
            'arrival_time':mission.arrival_time,'termination':{'zero_confirmed':zero_confirmed,'termination_time':termination_time,
            'stopped_before_mas_cleanup':bool(locals().get('gateway') is not None and gateway.poll() is not None)},
            'process_exit_codes':[{  'command':p.args,'returncode':p.returncode} for p in processes]},indent=2),encoding='utf-8')
        (output/'manifest.json').write_text(json.dumps({str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(output.rglob('*')) if p.is_file() and not p.is_symlink()
            and p.name not in ('manifest.json','acceptance.json')},indent=2),encoding='utf-8')
    from .acceptance import analyze_run
    report=analyze_run(output)
    (output/'acceptance.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2),flush=True)
    if not report['passed']: raise SystemExit(2)
