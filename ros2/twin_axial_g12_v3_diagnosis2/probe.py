"""Actual Gazebo/xdyn + v3 IPC probe; zero-input, never a navigation trial."""
import argparse,json,os,signal,socket,subprocess,sys,time,shutil,traceback
from pathlib import Path
from ros2.twin_axial_g12_real_v3.mission import prepare
from ros2.twin_axial_g12_real_v3.ipc import JsonlReader
from ros2.twin_axial_v2.model import NAME
from ros2.twin_axial_s3_transient_v2.cli import sha,read,write,ROOT
from .gate import RELEASE,contract,analyze

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--transport',choices=['default','UDPv4'],default='default');args=p.parse_args();out=args.output.resolve()
 if out.exists():raise ValueError('Output exists')
 os.environ['ROS_DOMAIN_ID']='74';os.environ.pop('FASTDDS_BUILTIN_TRANSPORTS',None)
 if args.transport!='default':os.environ['FASTDDS_BUILTIN_TRANSPORTS']=args.transport
 release=contract();
 for port in (12356,12357):
  with socket.socket() as sock:sock.bind(('127.0.0.1',port))
 from ros2.twin_axial_g12_real.gate import runtime_check
 runtime=runtime_check(True)
 if not runtime['passed']:raise ValueError('Nix runtime check failed: '+str(runtime))
 prepare(out,'P0','S3',True,'F0')
 write(out/'runtime.json',runtime);shutil.copy2(RELEASE,out/'joint_release.json')
 for name in release['files']:
  dst=out/'source'/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/name,dst)
 processes=[];logs=[];node=None;client=None;status='failed';error=None;action={};end={};started=False
 stage='setup';last=-1;exception={}
 diagnostics=(out/'diagnostics.jsonl').open('w',buffering=1);logs.append(diagnostics)
 def mark(name,**data):
  nonlocal stage
  stage=name;diagnostics.write(json.dumps(dict(stage=name,wall=time.monotonic(),sequence=last,**data))+'\n')
 env=os.environ.copy();env['ROS_DOMAIN_ID']='74';env.pop('FASTDDS_BUILTIN_TRANSPORTS',None)
 if args.transport!='default':env['FASTDDS_BUILTIN_TRANSPORTS']=args.transport
 env['LOTUSIM_STATE_HOME']=str(out);env['GZ_PARTITION']='g12_joint_'+out.name
 os.environ['ROS_DOMAIN_ID']='74'
 if args.transport!='default':os.environ['FASTDDS_BUILTIN_TRANSPORTS']=args.transport
 def launch(cmd,name):
  nonlocal started
  log=(out/name).open('w');logs.append(log)
  proc=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(proc);started=True;return proc
 def stop(proc):
  if proc and proc.poll() is None:
   os.killpg(proc.pid,signal.SIGTERM)
   try:proc.wait(timeout=4)
   except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
 def wait(test,proc):
  until=time.monotonic()+20
  while not test():
   if proc.poll() is not None or time.monotonic()>until:raise RuntimeError('Startup timeout/process exit')
   time.sleep(.05)
 def solver_ready():
  try:
   with socket.create_connection(('127.0.0.1',12357),timeout=.1):return True
  except OSError:return False
 def tx_count():
  path=out/'transactions.jsonl'
  return len(path.read_bytes().splitlines()) if path.exists() else 0
 try:
  import rclpy
  from rclpy.signals import SignalHandlerOptions
  from rclpy.action import ActionClient
  from lotusim_msgs.msg import VesselPositionArray,MASCmd
  from lotusim_msgs.action import MASCmdArray
  rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
  node=rclpy.create_node('g12_joint_probe')
  graph=(out/'graph.jsonl').open('w',buffering=1);logs.append(graph)
  def record_graph():
   mark('graph_query')
   endpoints=[dict(node=e.node_name,namespace=e.node_namespace,type=e.topic_type,gid=list(e.endpoint_gid),qos=str(e.qos_profile)) for e in node.get_publishers_info_by_topic('/uuv_twin_axial/poses')]
   diagnostics.write(json.dumps(dict(stage='pose_endpoints',wall=time.monotonic(),endpoints=endpoints,subscription_qos=str(sub.qos_profile)))+'\n')
   graph.write(json.dumps(dict(wall=time.monotonic(),domain=rclpy.get_default_context().get_domain_id(),rmw=rclpy.get_rmw_implementation_identifier(),nodes=node.get_node_names_and_namespaces(),pose_publishers=node.count_publishers('/uuv_twin_axial/poses'),action_ready=client.server_is_ready() if client else False))+'\n')
  # Never attach to an already visible bench in this namespace.
  for _ in range(10):rclpy.spin_once(node,timeout_sec=.05)
  if node.count_publishers('/uuv_twin_axial/poses'):raise RuntimeError('Existing bench publisher; refusing interference')
  poses=(out/'poses.jsonl').open('w',buffering=1);logs.append(poses)
  def callback(msg):
   mark('pose_callback',names=[v.vessel_name for v in msg.vessels],count=len(msg.vessels))
   for v in msg.vessels:
    if v.vessel_name==NAME:
     q=v.pose.orientation;a=v.pose.position
     poses.write(json.dumps(dict(time=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,position=[a.x,a.y,a.z],quaternion_xyzw=[q.x,q.y,q.z,q.w],received_monotonic=time.monotonic(),receiver_pid=os.getpid(),message_type='lotusim_msgs/msg/VesselPositionArray'))+'\n')
  sub=node.create_subscription(VesselPositionArray,'/uuv_twin_axial/poses',callback,20)
  client=ActionClient(node,MASCmdArray,'/uuv_twin_axial/mas_cmd_array')
  solver=launch(['xdyn-for-cs','--yml',str(out/'model.yml'),'--solver','rk4','--address','127.0.0.1','--port','12357','--dt','0.05'],'xdyn.log');wait(solver_ready,solver)
  gateway=launch([sys.executable,'-m','ros2.twin_axial_g12_v3_diagnosis2.gateway','--output',str(out)],'gateway.log');wait(lambda:(out/'gateway.ready').exists(),gateway)
  commands=(out/'commands.ipc.jsonl').open('w',buffering=1);logs.append(commands)
  sent=(out/'command_sent.jsonl').open('w',buffering=1);logs.append(sent)
  received=(out/'state_received.jsonl').open('w',buffering=1);logs.append(received)
  reader=JsonlReader(out/'state.jsonl');last=-1
  sim=launch(['lotusim','run','--gui','--debug','minimal.world'],'gazebo.log')
  deadline=time.monotonic()+70
  while last<259:
   mark('spin_once')
   rclpy.spin_once(node,timeout_sec=.001)
   if time.monotonic()>deadline:raise RuntimeError('Probe state timeout')
   if any(proc.poll() is not None for proc in processes):raise RuntimeError('Child exited')
   mark('state_read',offset=reader.offset,path=str(reader.path))
   rows=reader.read()
   if not rows:time.sleep(.002)
   for row in rows:
    if not row['valid'] or row['sequence']!=last+1:raise RuntimeError('Invalid/noncontiguous state')
    last=row['sequence'];now=time.monotonic()
    if last%20==0:record_graph()
    mark('state_received_write')
    received.write(json.dumps(dict(payload=row,received_monotonic=now,receiver_pid=os.getpid()))+'\n')
    command=dict(sequence=last,stamp=row['solver_state_ned_frd']['t'],commands={'X':0.,'M':0.})
    mark('command_audit_write')
    sent.write(json.dumps(dict(payload=command,sent_monotonic=time.monotonic(),sender_pid=os.getpid()))+'\n')
    mark('command_ipc_write')
    commands.write(json.dumps(command)+'\n')
  # Same termination ordering as v3: zero is actually applied; stop own
  # gateway/solver before waiting for MAS. Preserve every transaction.
  mark('stop_gateway')
  stop(gateway)
  mark('stop_solver')
  stop(solver)
  end=dict(last_step=tx_count(),stop_monotonic=time.monotonic(),gateway_returncode=gateway.returncode,solver_returncode=solver.returncode)
  # Drain pending pose messages while Gazebo remains available.
  for _ in range(10):rclpy.spin_once(node,timeout_sec=.02)
  action=dict(endpoint='/uuv_twin_axial/mas_cmd_array',vessel=NAME,sent_monotonic=time.monotonic())
  mark('action_wait_server')
  if not client.wait_for_server(timeout_sec=2):raise RuntimeError('MAS action unavailable')
  command=MASCmd();command.cmd_type=MASCmd.DELETE_CMD;command.vessel_name=NAME
  goal=MASCmdArray.Goal();goal.cmd=[command]
  mark('action_send_goal')
  future=client.send_goal_async(goal);rclpy.spin_until_future_complete(node,future,timeout_sec=2)
  if not future.done():raise RuntimeError('MAS goal response timeout')
  action['accepted']=future.result().accepted
  if not action['accepted']:raise RuntimeError('MAS rejected goal')
  mark('action_get_result')
  result=future.result().get_result_async();rclpy.spin_until_future_complete(node,result,timeout_sec=2)
  if not result.done():raise RuntimeError('MAS result timeout')
  response=result.result();action.update(status=response.status,results=list(response.result.result),names=list(response.result.name),received_monotonic=time.monotonic())
  if response.status!=4 or not action['results'] or not all(action['results']):raise RuntimeError('MAS delete failed')
  time.sleep(.15)
  end.update(step_after_cleanup=tx_count(),cleanup_completed=True)
  status='completed'
 except Exception as exc:
  error=repr(exc);exception=dict(error=error,errno=getattr(exc,'errno',None),stage=stage,sequence=last,traceback=traceback.format_exc());print(exception['traceback'],flush=True)
 finally:
  for proc in reversed(processes):stop(proc)
  if client is not None:client.destroy()
  if node is not None:node.destroy_node();rclpy.shutdown()
  for f in logs:f.close()
  end.setdefault('step_after_cleanup',tx_count());end.setdefault('cleanup_completed',False)
  write(out/'exception.json',exception)
  write(out/'action.json',action);write(out/'termination.json',end)
  write(out/'probe.json',dict(origin='actual_integration_probe',scope='13 s zero-input real GUI integration; not navigation',gui=True,adapter_release_sha256=sha(RELEASE),v3_release_sha256=release['v3_release_sha256'],runtime_files=runtime['runtime_files'],completed_unix=time.time(),status=status,error=error,real_started=started,formal_trials_started=False,processes=[dict(pid=x.pid,args=x.args,returncode=x.returncode) for x in processes],collector_pid=os.getpid(),environment={k:env.get(k) for k in ('ROS_DOMAIN_ID','GZ_PARTITION','FASTDDS_BUILTIN_TRANSPORTS','RMW_IMPLEMENTATION','ROS_LOCALHOST_ONLY','ROS_AUTOMATIC_DISCOVERY_RANGE','ROS_STATIC_PEERS','FASTDDS_DEFAULT_PROFILES_FILE','FASTRTPS_DEFAULT_PROFILES_FILE','RMW_FASTRTPS_USE_QOS_FROM_XML')}))
  names=['probe.json','runtime.json','action.json','termination.json','transactions.jsonl','state.jsonl','state_received.jsonl','commands.ipc.jsonl','command_sent.jsonl','poses.jsonl','model.yml','plan.json','worlds/minimal.world','joint_release.json','gateway.log','xdyn.log','gazebo.log','graph.jsonl','diagnostics.jsonl','exception.json','ipc_exception.json']
  write(out/'manifest.json',{n:sha(out/n) for n in names if (out/n).exists()})
 try:report=analyze(out)
 except Exception as exc:report=dict(passed=False,error=repr(exc))
 write(out/'analysis.json',report);print(json.dumps(report,indent=2),flush=True)
 return 0 if report.get('passed') else 2
if __name__=='__main__':raise SystemExit(main())
