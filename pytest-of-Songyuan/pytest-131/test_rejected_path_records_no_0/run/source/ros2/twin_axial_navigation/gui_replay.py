"""Display-only Gazebo replay of a verified S2 navigation history."""
import argparse
import json
import math
import os
from pathlib import Path
import secrets
import select
import signal
import subprocess
import sys
import time
from xml.etree import ElementTree as ET

from visualization.twin_axial_navigation_export import digest
from .mission import world_xml


def load_history(directory,allow_failed=False):
    directory=Path(directory).resolve()
    manifest=json.loads((directory/'manifest.json').read_text(encoding='utf-8'))
    for name,expected in manifest.items():
        path=(directory/name).resolve()
        if not path.is_relative_to(directory) or not path.is_file() or digest(path)!=expected:
            raise ValueError(f'Recording hash mismatch: {name}')
    acceptance=json.loads((directory/'acceptance.json').read_text(encoding='utf-8'))
    metadata=json.loads((directory/'metadata.json').read_text(encoding='utf-8'))
    if metadata['scene']!='S2':
        raise ValueError('GUI replay needs an S2 recording')
    from .acceptance import analyze_run
    rechecked=analyze_run(directory)
    if acceptance['passed']:
        if not rechecked['passed']: raise ValueError('Read-only acceptance recheck failed')
    elif not allow_failed:
        raise ValueError('Failed recording needs --include-failed-reference')
    else:
        status=json.loads((directory/'run_status.json').read_text(encoding='utf-8'))
        if status['status']!='failed' or status['error']!=acceptance.get('error') or rechecked.get('error')!=status['error']:
            raise ValueError('Failed reference does not match its recorded stop reason')
    rows=[json.loads(line) for line in (directory/'transactions.jsonl').read_text(encoding='utf-8').splitlines() if line]
    samples=[(r['elapsed_end'],r['position_enu_m'][0],r['position_enu_m'][2],r['pitch_nose_up_rad'])
             for r in rows if r.get('valid')]
    if len(samples)<2 or any(b[0]<=a[0] for a,b in zip(samples,samples[1:])):
        raise ValueError('Invalid replay time series')
    return samples


def replay_world():
    root=ET.fromstring(world_xml('P0','S2',gui=True))
    world=root.find('world')
    vessel=world.find("model[@name='uuv_twin_axial_v2']")
    world.remove(vessel)
    for plugin in list(world.findall('plugin')):
        if plugin.get('filename')=='physics_interface_plugin': world.remove(plugin)
    return ET.tostring(root,encoding='unicode')


def marker(name,rgba):
    root=ET.Element('sdf',version='1.7')
    model=ET.SubElement(root,'model',name=name)
    ET.SubElement(model,'static').text='true'
    link=ET.SubElement(model,'link',name='base_link')
    visual=ET.SubElement(link,'visual',name='colored_sphere')
    ET.SubElement(ET.SubElement(ET.SubElement(visual,'geometry'),'sphere'),'radius').text='0.36'
    material=ET.SubElement(visual,'material')
    for tag in ('ambient','diffuse'):
        ET.SubElement(material,tag).text=rgba
    return ET.tostring(root,encoding='unicode')


def sample_at(samples,t):
    if t<=samples[0][0]: return samples[0]
    if t>=samples[-1][0]: return samples[-1]
    lo,hi=0,len(samples)-1
    while hi-lo>1:
        mid=(hi+lo)//2
        if samples[mid][0]<=t: lo=mid
        else: hi=mid
    a,b=samples[lo],samples[hi]
    f=(t-a[0])/(b[0]-a[0])
    return tuple(x+f*(y-x) for x,y in zip(a,b))


def parse_command(line,rate,paused,t,start):
    command=line.strip().lower()
    if command in ('q','quit','exit'): return rate,paused,t,True
    if command in ('r','restart'):
        print('从头重播。',flush=True)
        return rate,False,start,False
    if command in ('p','pause','play'):
        print('已继续。' if paused else '已暂停。',flush=True)
        return rate,not paused,t,False
    if command in ('0.25','0.5','1','2','4','8'):
        print(f'回放速度 {command}×。',flush=True)
        return float(command),paused,t,False
    print('指令: p 暂停/继续, r 从头重播, 0.25/0.5/1/2/4/8 倍速, q 退出',flush=True)
    return rate,paused,t,False


def run(args):
    samples=load_history(args.recording)
    if args.reference:
        reference=load_history(args.reference,args.include_failed_reference)
    else: reference=None
    if args.validate_only:
        print(json.dumps({'samples':len(samples),'duration_s':samples[-1][0],
                          'reference_samples':0 if reference is None else len(reference)}))
        return
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.signals import SignalHandlerOptions
    from lotusim_msgs.action import MASCmdArray
    from lotusim_msgs.msg import MASCmd,VesselPositionArray
    output=args.output.resolve()
    output.mkdir(parents=True,exist_ok=False)
    (output/'worlds').mkdir()
    (output/'worlds/minimal.world').write_text(replay_world(),encoding='utf-8')
    env=os.environ.copy();env['LOTUSIM_STATE_HOME']=str(output)
    env['GZ_PARTITION']='uuv_s2_replay_'+output.name
    domain=args.ros_domain_id if args.ros_domain_id is not None else 80+secrets.randbelow(140)
    env['ROS_DOMAIN_ID']=str(domain);os.environ['ROS_DOMAIN_ID']=str(domain)
    print(f'GUI replay ROS_DOMAIN_ID={domain}',flush=True)
    log=(output/'gazebo.log').open('w',encoding='utf-8')
    sim=subprocess.Popen(['lotusim','run','--gui','--debug','minimal.world'],env=env,
                         stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('uuv_s2_display_replay')
    client=ActionClient(node,MASCmdArray,'/uuv_twin_axial/mas_cmd_array')
    names=[]
    observed={}
    def observe(msg):
        for vessel in msg.vessels:
            observed[vessel.vessel_name]=(vessel.pose.position.x,vessel.pose.position.z)
    poses=node.create_subscription(VesselPositionArray,'/uuv_twin_axial/poses',observe,20)
    def command(kind,name,xml='',pose=None):
        c=MASCmd();c.cmd_type=kind;c.vessel_name=name;c.sdf_string=xml
        c.vessel_position.orientation.w=1.0
        if pose:
            _,x,z,pitch=pose
            c.vessel_position.position.x=x;c.vessel_position.position.z=z
            c.vessel_position.orientation.y=math.sin(-pitch/2)
            c.vessel_position.orientation.w=math.cos(-pitch/2)
        return c
    def send(commands):
        goal=MASCmdArray.Goal();goal.cmd=commands
        future=client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        if not future.done() or not future.result().accepted: raise RuntimeError('Gazebo replay action unavailable')
        future=future.result().get_result_async()
        rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        if not future.done() or not all(future.result().result.result): raise RuntimeError('Gazebo replay action failed')
        return future.result().result
    try:
        if not client.wait_for_server(timeout_sec=25): raise RuntimeError('LOTUSim GUI did not start; inspect gazebo.log')
        sources=[('actual',samples,'0.1 0.85 1 1')]
        if reference:
            if args.include_failed_reference: print('橙色参考记录在原试验失败处结束，之后保持在失败位置。',flush=True)
            sources.append(('reference',reference,'1 0.55 0.08 1'))
        for name,history,color in sources:
            result=send([command(MASCmd.CREATE_CMD,'s2_replay_'+name,marker(name,color),history[0])])
            names.append(result.name[0])
        deadline=time.monotonic()+8
        while not set(names).issubset(observed):
            if time.monotonic()>deadline: raise RuntimeError('Replay markers were not observed after creation')
            rclpy.spin_once(node,timeout_sec=.1)
        # The MAS entity index trails its first pose publication by a render
        # tick; allow it to settle before the first MOVE action.
        time.sleep(1)
        print('青色=当前记录；橙色=参考记录。p 暂停/继续，r 从头重播，0.25/0.5/1/2/4/8 倍速，q 退出。输入指令后按 Enter。',flush=True)
        start=samples[0][0];end=max(h[-1][0] for _,h,_ in sources)
        t=start;rate=args.rate;paused=False;last=time.monotonic();loops=0
        while rclpy.ok():
            now=time.monotonic();dt=now-last;last=now
            if sys.stdin.isatty() and select.select([sys.stdin],[],[],0)[0]:
                rate,paused,t,quit_now=parse_command(sys.stdin.readline(),rate,paused,t,start)
                if quit_now: break
            if not paused:
                t=min(end,t+dt*rate)
            send([command(MASCmd.MOVE_CMD,name,pose=sample_at(history,t))
                  for name,(_,history,_) in zip(names,sources)])
            if t>=end and not paused:
                loops+=1
                if args.loops and loops>=args.loops:
                    for name,(_,history,_) in zip(names,sources):
                        target=sample_at(history,t)
                        deadline=time.monotonic()+3
                        while time.monotonic()<deadline and math.dist(observed.get(name,(-1e9,-1e9)),(target[1],target[2]))>.5:
                            rclpy.spin_once(node,timeout_sec=.1)
                        position=observed.get(name)
                        print(f'{name}: Gazebo pose={position}, recorded target={(target[1],target[2])}',flush=True)
                        if position is None or math.dist(position,(target[1],target[2]))>.5:
                            raise RuntimeError(f'GUI marker did not reach recorded endpoint: {name}')
                    break
                paused=True
                print('已到终点。输入 r 从头重播，或输入其他指令。',flush=True)
            time.sleep(max(0,1/args.hz-(time.monotonic()-now)))
    finally:
        if rclpy.ok() and names:
            try: send([command(MASCmd.DELETE_CMD,name) for name in names])
            except Exception as exc: print(f'清理回放标记失败: {exc}',file=sys.stderr)
        node.destroy_subscription(poses)
        client.destroy();node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        if sim.poll() is None:
            os.killpg(sim.pid,signal.SIGTERM)
            try:sim.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(sim.pid,signal.SIGKILL);sim.wait()
        log.close()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording',type=Path)
    parser.add_argument('--reference',type=Path,help='Second accepted S2 recording as an orange comparison marker')
    parser.add_argument('--include-failed-reference',action='store_true')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--rate',type=float,default=1)
    parser.add_argument('--hz',type=float,default=10)
    parser.add_argument('--loops',type=int,default=0,help='Exit after this many plays; 0 is interactive')
    parser.add_argument('--ros-domain-id',type=int,choices=range(80,220))
    parser.add_argument('--validate-only',action='store_true')
    args=parser.parse_args(argv)
    if not math.isfinite(args.rate) or args.rate<=0 or not math.isfinite(args.hz) or args.hz<=0:
        parser.error('rate and hz must be finite and positive')
    if not args.validate_only and args.output is None: parser.error('--output is required')
    run(args)


if __name__=='__main__': main()
