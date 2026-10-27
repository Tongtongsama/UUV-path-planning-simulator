"""Visualize saved Gazebo poses; never launch xdyn or run a controller."""
import argparse
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time
import uuid
from .core import (Playback,display_world,fit_camera,load_recording,load_suite,sample,
                   suite_unchanged,unchanged,visual_model)


def acceptance_passed(record):
    path=record['root']/'acceptance.json'
    if not path.is_file(): return None
    try: return json.loads(path.read_text(encoding='utf-8')).get('passed')
    except ValueError: return None


def describe_case(record):
    return (f"{len(record['poses'])} poses, {record['duration']:.2f} s, "
            f"source {record['poses'][0]['time']:g}-{record['poses'][-1]['time']:g} s")


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('recording',type=Path,help='A completed recording, or a directory with suite.json')
    p.add_argument('--case',help='Initial suite case (default: first run in suite.json)')
    p.add_argument('--rate',type=float,default=.25)
    p.add_argument('--loop',action='store_true')
    p.add_argument('--play',action='store_true',help='Start playing; GUI defaults to paused')
    p.add_argument('--headless',action='store_true')
    p.add_argument('--auto-close',action='store_true')
    p.add_argument('--camera',choices=('auto','fit','close'),default='auto',help='auto: fit for suites, close for one recording')
    p.add_argument('--output',type=Path)
    p.add_argument('--validate-only',action='store_true')
    p.add_argument('--prepare-only',action='store_true',help='Write the display scene and source record without launching Gazebo')
    args=p.parse_args()
    if args.auto_close and args.loop: p.error('--auto-close cannot be combined with --loop')
    suite=load_suite(args.recording) if (args.recording/'suite.json').is_file() else None
    if suite is not None:
        cases=suite['order']; records=suite['records']
        if args.case:
            if args.case not in records: p.error(f"unknown suite case {args.case!r}; use --validate-only to list cases")
            current=args.case
        else: current=cases[0]
        record=records[current]
        print(f"Verified suite {suite['root'].name}: {len(cases)} completed runs, suite passed={suite['suite'].get('passed')}.",flush=True)
        for name in cases:
            print(f"  {name}: {describe_case(records[name])}, acceptance={acceptance_passed(records[name])}",flush=True)
        print(f"Active case {cases.index(current)+1}/{len(cases)}: {current}",flush=True)
    else:
        if args.case: p.error('--case requires a suite directory')
        cases=[]; records={}; current=None
        record=load_recording(args.recording)
        print(f"Verified {len(record['poses'])} recorded poses, {record['duration']:.2f} s. Visualization only; no dynamics.",flush=True)
    if args.validate_only: return
    clock=Playback(record['duration'],args.rate,not(args.play or args.headless or args.auto_close),args.loop)
    tag=uuid.uuid4().hex[:10]; namespace='recorded_'+tag; name='recorded_uuv_'+tag
    model=visual_model(record,name)
    if suite is not None:
        for case in cases:
            if visual_model(records[case],name)!=model:
                p.error('Suite cases do not share one inline visual model; replay them individually')
    mode=('fit' if suite is not None else 'close') if args.camera=='auto' else args.camera
    camera=fit_camera(list(records.values()) if suite is not None else [record]) if mode=='fit' else None
    title=None if suite is None else f"Twin-axial recorded suite — {suite['root'].name} — visualization only"
    out=(args.output or Path('artifacts/twin_axial_pid_replays' if suite is not None else 'artifacts/twin_axial_replays')/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+tag)).resolve()
    if out.is_relative_to(record['root']) or (suite is not None and out.is_relative_to(suite['root'])):
        p.error('Replay output must be outside the original recording')
    if out.exists(): p.error('Replay output already exists; choose a new directory')
    (out/'state/worlds').mkdir(parents=True)
    (out/'state/worlds/replay.world').write_text(display_world(namespace,record['poses'][0]['position'],camera,title),encoding='utf-8')
    (out/'vehicle.sdf').write_text(model,encoding='utf-8')
    if suite is None:
        source={'path':str(record['root']),'hashes':record['hashes'],'pose_source':'gazebo_poses.jsonl','first_source_time':record['times'][0],'last_source_time':record['times'][-1]}
    else:
        source={'path':str(suite['root']),'suite_sha256':suite['hashes']['suite.json'],'suite':suite['suite'],'pose_source':'gazebo_poses.jsonl','cases':{}}
        for case in cases:
            item=records[case]
            source['cases'][case]={'path':str(item['root']),'hashes':item['hashes'],'poses':len(item['poses']),
                                   'first_source_time':item['times'][0],'last_source_time':item['times'][-1]}
    (out/'source_record.json').write_text(json.dumps(source,indent=2),encoding='utf-8')
    if args.prepare_only:
        preserved=suite_unchanged(suite) if suite is not None else unchanged(record)
        report={'status':'prepared','error':None,'source_unchanged':preserved,'commands':0,'observations':0,'cycles':0,
                'dynamics_launched':False,'pose_source':'gazebo_poses.jsonl','output':str(out),'prepared_only':True}
        if suite is not None:
            report.update(suite=str(suite['root']),suite_passed=suite['suite'].get('passed'),active_case=current)
            report['cases']={case:{'poses':len(records[case]['poses']),'duration':records[case]['duration'],'acceptance':acceptance_passed(records[case])} for case in cases}
        (out/'summary.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        print('Prepared display scene: '+str(out),flush=True)
        print(json.dumps(report,indent=2),flush=True)
        if not preserved: raise SystemExit(2)
        return
    env=os.environ.copy(); env.update(LOTUSIM_STATE_HOME=str(out/'state'),GZ_PARTITION=namespace,ROS_DOMAIN_ID='83')
    os.environ['ROS_DOMAIN_ID']='83'
    processes=[]; node=None; client=None; ros=None; created=False; count=0; observed=0; status='stopped'; error=None
    inbox=queue.Queue()
    def keyboard():
        for line in sys.stdin: inbox.put(line)
    threading.Thread(target=keyboard,daemon=True).start()
    print('Replay output: '+str(out),flush=True)
    if suite is None:
        print('Terminal commands + Enter: p=pause/resume, r=restart/play, l=loop on/off, speed 0.25, speed 1, q=quit.',flush=True)
    else:
        print('Terminal commands + Enter: p=pause/resume, r=restart, l=loop on/off, speed 0.25, next, prev, case NAME, list, q=quit.',flush=True)
    print('Use these terminal controls; do not pause the Gazebo server.',flush=True)
    with (out/'gazebo.log').open('w',encoding='utf-8') as log, (out/'commands.jsonl').open('w',encoding='utf-8',buffering=1) as commands, (out/'observations.jsonl').open('w',encoding='utf-8',buffering=1) as observations, (out/'events.jsonl').open('w',encoding='utf-8',buffering=1) as events:
        def event(kind,**data):
            events.write(json.dumps({'event':kind,'wall_monotonic':time.monotonic(),'case':current,'elapsed':clock.elapsed,'paused':clock.paused,'rate':clock.rate,'loop':clock.loop,'cycles':clock.cycles,**data})+'\n')
        def launch(command):
            child=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append(child); return child
        def print_cases():
            for index,case in enumerate(cases,1):
                mark='*' if case==current else ' '
                print(f"{mark} {index}. {case}: {describe_case(records[case])}, acceptance={acceptance_passed(records[case])}",flush=True)
        def select_case(case):
            nonlocal current,record,clock
            if case not in records: raise ValueError(f"Unknown case {case!r}; use list")
            clock=Playback(records[case]['duration'],clock.rate,clock.paused,clock.loop)
            current=case; record=records[case]
            event('select_case',case=case)
            print(f"Case {cases.index(case)+1}/{len(cases)}: {case} ({describe_case(record)}); {'PAUSED' if clock.paused else 'PLAYING'}",flush=True)
        try:
            server=launch(['lotusim','run','replay.world'])
            import rclpy as ros
            from rclpy.signals import SignalHandlerOptions
            from rclpy.action import ActionClient
            from rclpy.qos import qos_profile_sensor_data
            from lotusim_msgs.action import MASCmdArray
            from lotusim_msgs.msg import MASCmd,VesselPositionArray
            ros.init(signal_handler_options=SignalHandlerOptions.NO)
            node=ros.create_node('recorded_pose_player_'+tag)
            client=ActionClient(node,MASCmdArray,'/'+namespace+'/mas_cmd_array')
            def observe(message):
                nonlocal observed
                for vessel in message.vessels:
                    if vessel.vessel_name!=name: continue
                    observed+=1; pos=vessel.pose.position; q=vessel.pose.orientation
                    observations.write(json.dumps({'wall_monotonic':time.monotonic(),'display_sim_time':message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                        'position':[pos.x,pos.y,pos.z],'quaternion_xyzw':[q.x,q.y,q.z,q.w]})+'\n')
            sub=node.create_subscription(VesselPositionArray,'/'+namespace+'/poses',observe,qos_profile_sensor_data)
            def wait(future):
                ros.spin_until_future_complete(node,future,timeout_sec=10)
                if not future.done(): raise RuntimeError('Display MAS timeout; keep Gazebo server unpaused')
                return future.result()
            def send(kind,pose=None,xml=''):
                c=MASCmd(); c.cmd_type=kind; c.vessel_name=name; c.sdf_string=xml
                c.vessel_position.orientation.w=1.
                if pose:
                    for axis,value in zip('xyz',pose['position']): setattr(c.vessel_position.position,axis,float(value))
                    for axis,value in zip('xyzw',pose['quaternion_xyzw']): setattr(c.vessel_position.orientation,axis,float(value))
                goal=MASCmdArray.Goal(); goal.cmd=[c]
                handle=wait(client.send_goal_async(goal))
                if not handle.accepted: raise RuntimeError('Display MAS command rejected')
                result=wait(handle.get_result_async())
                if result.status!=4 or list(result.result.result)!=[True]: raise RuntimeError('Display MAS command failed')
            if not client.wait_for_server(timeout_sec=30): raise RuntimeError('Display server unavailable; see gazebo.log')
            send(MASCmd.CREATE_CMD,sample(record,0),model); created=True
            deadline=time.monotonic()+10
            while not observed:
                if time.monotonic()>deadline: raise RuntimeError('No initial display state feedback')
                ros.spin_once(node,timeout_sec=.1)
            # Avoid the dynamic-creation/initial-GUI-snapshot race: GUI starts only after model exists.
            gui=launch(['lotusim','-g','-v','1']) if not args.headless else None
            event('ready',model=name)
            print(f'Model {name} ready. '+('PAUSED: enter p to play.' if clock.paused else 'Playing.'),flush=True)
            if suite is not None: print_cases()
            previous=time.monotonic(); last_print=previous; last_cycles=0
            while True:
                tick=time.monotonic(); clock.advance(tick-previous); previous=tick
                if server.poll() is not None: raise RuntimeError('Display server exited')
                if gui is not None and gui.poll() is not None:
                    if gui.returncode: raise RuntimeError('Display GUI failed; see gazebo.log')
                    break
                quit_requested=False
                while not inbox.empty():
                    line=inbox.get(); raw=line.strip().split(); words=[word.lower() for word in raw]
                    if words==['q']: quit_requested=True; break
                    if suite is not None:
                        target=None
                        if words in (['next'],['n']): target=cases[(cases.index(current)+1)%len(cases)]
                        elif words in (['prev'],['b']): target=cases[(cases.index(current)-1)%len(cases)]
                        elif len(words)==2 and words[0]=='case': target=raw[1]
                        elif words==['list']:
                            print_cases(); event('list_cases'); continue
                        if target is not None:
                            try: select_case(target)
                            except ValueError as exc: print(str(exc),flush=True); event('invalid_control',message=str(exc))
                            last_cycles=0
                            continue
                    try: clock.command(line); event('control',command=line.strip())
                    except ValueError as exc: print(str(exc),flush=True); event('invalid_control',message=str(exc))
                if quit_requested: break
                pose=sample(record,clock.elapsed)
                send(MASCmd.MOVE_CMD,pose)
                count+=1
                commands.write(json.dumps({'wall_monotonic':time.monotonic(),'case':current,'elapsed':clock.elapsed,'paused':clock.paused,'rate':clock.rate,'cycle':clock.cycles,**pose})+'\n')
                if clock.cycles!=last_cycles: event('cycle_end'); last_cycles=clock.cycles
                if args.auto_close and clock.cycles:
                    status='completed'
                    deadline=time.monotonic()+.5
                    while time.monotonic()<deadline: ros.spin_once(node,timeout_sec=.05)
                    break
                if tick-last_print>=2:
                    prefix=f'[{current}] ' if suite is not None else ''
                    print(f"{prefix}{'PAUSED' if clock.paused else 'PLAY'} {clock.elapsed:.2f}/{clock.duration:.2f}s, {clock.rate:g}x, loop={clock.loop}, cycles={clock.cycles}",flush=True)
                    last_print=tick
                deadline=tick+.05
                while time.monotonic()<deadline: ros.spin_once(node,timeout_sec=max(0,min(.02,deadline-time.monotonic())))
        except KeyboardInterrupt: pass
        except Exception as exc: status='failed'; error=str(exc); print('Replay failed: '+error,flush=True)
        finally:
            if created and node is not None:
                try: send(MASCmd.DELETE_CMD)
                except Exception as exc: event('cleanup_warning',message=str(exc))
            if client is not None: client.destroy()
            if node is not None: node.destroy_node()
            if ros is not None and ros.ok(): ros.shutdown()
            for child in reversed(processes):
                if child.poll() is None:
                    os.killpg(child.pid,signal.SIGINT)
                    try: child.wait(timeout=8)
                    except subprocess.TimeoutExpired: os.killpg(child.pid,signal.SIGKILL); child.wait()
    preserved=suite_unchanged(suite) if suite is not None else unchanged(record)
    report={'status':status,'error':error,'source_unchanged':preserved,'commands':count,'observations':observed,'cycles':clock.cycles,
            'dynamics_launched':False,'pose_source':'gazebo_poses.jsonl','output':str(out)}
    if suite is not None:
        report.update(suite=str(suite['root']),suite_passed=suite['suite'].get('passed'),active_case=current)
        report['cases']={case:{'poses':len(records[case]['poses']),'duration':records[case]['duration'],'acceptance':acceptance_passed(records[case])} for case in cases}
    (out/'summary.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2),flush=True)
    if error or not preserved: raise SystemExit(2)


if __name__=='__main__': main()
