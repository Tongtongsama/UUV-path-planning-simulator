"""Gazebo replay via acknowledged LOTUSim MAS actions, not renderer topics."""
import argparse
import json
import math
import time
import uuid
from pathlib import Path
from datetime import datetime, timezone
from .recording import Recorder
from .core import load_verified_payload, select_scene, select_arms, interpolate, lotusim_pose
from .gazebo_scene import scene_sdf, vehicle_sdf


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo-root', type=Path, default=Path.cwd())
    parser.add_argument('--scene', default='double_baffle_detour', choices=['double_baffle_detour'])
    parser.add_argument('--arm', action='append', choices=['P1','APPROACH'])
    parser.add_argument('--rate', type=float, default=1)
    parser.add_argument('--publish-hz', type=float, default=10)
    parser.add_argument('--namespace', default='/lotusim')
    parser.add_argument('--validate-only', action='store_true')
    parser.add_argument('--export-dir', type=Path)
    parser.add_argument('--record-dir', type=Path, help='New output directory; defaults to artifacts/lotusim_recordings/<UTC time + ID>')
    args = parser.parse_args(argv)
    if any(not math.isfinite(v) or v <= 0 for v in [args.rate,args.publish_hz]):
        parser.error('rate and publish-hz must be finite and positive')
    payload = load_verified_payload(args.repo_root)
    scene = select_scene(payload,args.scene)
    arms = select_arms(scene,args.arm or ['APPROACH'])
    sdf = scene_sdf(scene)
    summary = {a: {'samples':len(d['samples']), 'end':d['end'], 'status':d['status']} for a,d in arms.items()}
    print(json.dumps(summary, indent=2))
    if args.export_dir:
        args.export_dir.mkdir(parents=True, exist_ok=False)
        (args.export_dir/'scene.sdf').write_text(sdf,encoding='utf-8')
        (args.export_dir/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    if args.validate_only:
        return
    try:
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.signals import SignalHandlerOptions
        from rclpy.type_support import check_for_type_support
        from lotusim_msgs.action import MASCmdArray
        from lotusim_msgs.msg import MASCmd, VesselPositionArray
        from rclpy.qos import qos_profile_sensor_data
        # Python class import alone does not load nested messages or C support.
        check_for_type_support(MASCmdArray)
        check_for_type_support(MASCmd)
        check_for_type_support(VesselPositionArray)
    except ImportError as exc:
        raise SystemExit(f'Missing ROS dependencies ({exc}): run through the updated ros2/lotusim_replay/run-nix.sh') from exc
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('uuv_frozen_gazebo_replay')
    client = ActionClient(node,MASCmdArray,args.namespace.rstrip('/')+'/mas_cmd_array')
    owned = []
    prefix = 'replay_'+uuid.uuid4().hex[:8]
    output=args.record_dir or args.repo_root/'artifacts'/'lotusim_recordings'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+prefix)
    recorder=Recorder(output,payload,scene,arms,{'rate':args.rate,'requested_hz':args.publish_hz,'namespace':args.namespace,'arms':list(arms)},sdf)
    print('Recording to '+str(output.resolve()),flush=True)
    names={}
    def observe(message):
        for vessel in message.vessels:
            arm=next((a for a,n in names.items() if n==vessel.vessel_name),None)
            if arm is None: continue
            p=vessel.pose.position; q=vessel.pose.orientation
            stamp=message.header.stamp
            recorder.observe(arm,vessel.vessel_name,stamp.sec+stamp.nanosec*1e-9,
                (p.x,p.y,p.z),(q.x,q.y,q.z,q.w))
    subscription=node.create_subscription(VesselPositionArray,args.namespace.rstrip('/')+'/poses',observe,qos_profile_sensor_data)
    status='interrupted'; failure=None

    def command(kind,name,xml='',pose=None):
        c = MASCmd(); c.cmd_type=kind; c.vessel_name=name; c.sdf_string=xml
        c.vessel_position.orientation.w=1.0
        if pose:
            c.vessel_position.position.x=pose.x
            c.vessel_position.position.y=pose.y
            c.vessel_position.position.z=pose.z
            c.vessel_position.orientation.y=pose.qy
            c.vessel_position.orientation.w=pose.qw
        return c

    def wait(future):
        rclpy.spin_until_future_complete(node,future,timeout_sec=15)
        if not future.done():
            raise RuntimeError('MAS timeout. Ensure Gazebo is playing. A timed-out command may still execute; restart the world before retrying.')
        return future.result()

    def send(commands,creating=False):
        goal=MASCmdArray.Goal(); goal.cmd=commands
        handle=wait(client.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('MAS rejected replay command')
        result=wait(handle.get_result_async()).result
        if creating:
            owned.extend(name for name,ok in zip(result.name,result.result) if ok)
        if len(result.result)!=len(commands) or not all(result.result):
            raise RuntimeError(f'MAS command failed: {result.result}')
        return result

    try:
        if not client.wait_for_server(timeout_sec=15):
            raise RuntimeError('No LOTUSim MAS action server. Start lotusim.world first.')
        send([command(MASCmd.CREATE_CMD,prefix+'_scene',sdf)],True)
        for arm,data in arms.items():
            name=prefix+'_'+arm.lower()
            result=send([command(MASCmd.CREATE_CMD,name,
                vehicle_sdf(name,'0 0.6 1 1' if arm=='P1' else '1 0.4 0 1'),
                lotusim_pose(data['samples'][0]))],True)
            names[arm]=result.name[0]
            recorder.event('entity_created',{'arm':arm,'name':names[arm]})
            (recorder.path/f'{arm}_vehicle.sdf').write_text(vehicle_sdf(name,'0 0.6 1 1' if arm=='P1' else '1 0.4 0 1'),encoding='utf-8')
        print('Focus Entity Tree on '+prefix+'_scene. Orange=APPROACH; blue=P1; white end=nose.',flush=True)
        print('Wall-clock replay; Gazebo must remain unpaused. Ctrl+C removes only this replay.',flush=True)
        start=min(d['samples'][0][0] for d in arms.values())
        end=max(d['samples'][-1][0] for d in arms.values())
        epoch=time.monotonic()
        sequence=0
        while rclpy.ok():
            tick=time.monotonic()
            source_time=min(end,start+(tick-epoch)*args.rate)
            commands=[]
            for a,d in arms.items():
                pose=lotusim_pose(interpolate(d['samples'],source_time))
                recorder.command(sequence,a,names[a],pose)
                commands.append(command(MASCmd.MOVE_CMD,names[a],pose=pose))
            send(commands)
            recorder.event('batch_acknowledged',{'sequence':sequence})
            sequence+=1
            if source_time>=end:
                break
            deadline=tick+1/args.publish_hz
            while time.monotonic()<deadline and rclpy.ok():
                rclpy.spin_once(node,timeout_sec=max(0,deadline-time.monotonic()))
        status='completed' if source_time>=end else 'interrupted'
        # Drain final state telemetry before closing the recording, not indefinitely during hold.
        deadline=time.monotonic()+1
        while time.monotonic()<deadline and rclpy.ok(): rclpy.spin_once(node,timeout_sec=.1)
        recorder.finish(status)
        print('Replay complete; final poses held. Ctrl+C to remove.',flush=True)
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.5)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        status='failed'; failure=str(exc)
        raise
    finally:
        recorder.finish(status,failure)
        if rclpy.ok() and owned:
            try:
                send([command(MASCmd.DELETE_CMD,name) for name in owned])
            except Exception as exc:
                print(f'Cleanup incomplete: {exc}; replay entities: {owned}')
        elif owned:
            print(f'ROS stopped; remove these replay entities or restart Gazebo: {owned}')
        client.destroy(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()
