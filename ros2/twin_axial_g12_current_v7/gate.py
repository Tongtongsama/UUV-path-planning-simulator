"""Read-only joint gate over authentic IPC and actual Gazebo observations."""
import argparse,json,math,time
from dataclasses import dataclass
from pathlib import Path
from ros2.twin_axial_g12_real_v3 import gate as v3
from ros2.twin_axial_g12_real import gate as base
from ros2.twin_axial_s3_transient_v2.cli import ROOT,read,write,sha
RELEASE=Path(__file__).with_name('release_current_10.json')
PYTHON=ROOT/'artifacts/twin_axial_g12_matrix/python_20261110_01'
require=base.require

def seal():
 require(not RELEASE.exists(),'Release already exists')
 previous=v3.contract(PYTHON)
 files={**previous['python_files'],**previous['adapter_files']}
 for p in Path(__file__).parent.iterdir():
  if p.is_file() and p.suffix in ('.py','.xml','.sh'):files[p.relative_to(ROOT).as_posix()]=sha(p)
 script=Path(__file__).with_name('run-nix.sh');files[script.relative_to(ROOT).as_posix()]=sha(script)
 data=dict(revision='g12_current_v7_20261118',v3_release_sha256=sha(v3.RELEASE),files=files,runtime_files=previous['runtime_files'])
 write(RELEASE,data);return data

def contract():
 v3.contract(PYTHON);release=read(RELEASE)
 require(release['v3_release_sha256']==sha(v3.RELEASE),'Frozen v3 release mismatch')
 for name,digest in release['files'].items():require(sha(ROOT/name)==digest,'Joint source drift: '+name)
 return release

def rows(path):return [json.loads(s) for s in path.read_text().splitlines() if s]

def analyze(directory,fresh=False):
 directory=Path(directory);release=contract();manifest=read(directory/'manifest.json')
 needed={'probe.json','runtime.json','action.json','termination.json','transactions.jsonl','state.jsonl','state_received.jsonl','commands.ipc.jsonl','command_sent.jsonl','poses.jsonl','model.yml','plan.json','worlds/minimal.world','joint_release.json','gateway.log','xdyn.log','gazebo.log','graph.jsonl','diagnostics.jsonl','exception.json','ipc_exception.json','prestop_action.json','ipc_retries.jsonl'}
 require(set(manifest)==needed,'Incomplete raw evidence manifest')
 for name,digest in manifest.items():require(sha(directory/name)==digest,'Evidence changed: '+name)
 require(sha(directory/'joint_release.json')==sha(RELEASE),'Probe release mismatch')
 for name,digest in release['files'].items():require(sha(directory/'source'/name)==digest,'Snapshot mismatch: '+name)
 meta=read(directory/'probe.json');require(meta['origin']=='actual_integration_probe' and meta['gui'],'Wrong scope')
 require(meta['adapter_release_sha256']==sha(RELEASE) and meta['v3_release_sha256']==sha(v3.RELEASE),'Wrong revision')
 require(meta['runtime_files']==release['runtime_files'],'Runtime mismatch')
 if fresh:require(0<=time.time()-meta['completed_unix']<=600,'Live evidence expired/future')
 from ros2.twin_axial_g12_current_v2.mission import make_plan,model_config,world_xml
 require(read(directory/'plan.json')==make_plan('P0','S3','F0'),'Plan/config changed')
 require(read(directory/'model.yml')==model_config('P0','F0'),'Model changed')
 require((directory/'worlds/minimal.world').read_text()==world_xml('P0','S3',True),'Scene changed')
 tr=rows(directory/'transactions.jsonl');received=rows(directory/'state_received.jsonl');states=rows(directory/'state.jsonl');sent=rows(directory/'command_sent.jsonl');ipc=rows(directory/'commands.ipc.jsonl');poses=rows(directory/'poses.jsonl')
 checks=dict(completed=meta['status']=='completed' and meta['error'] is None,runtime=read(directory/'runtime.json')['passed'],states_valid=len(tr)>=260 and all(r['valid'] for r in tr))
 require(checks['states_valid'],'Insufficient/invalid real steps')
 checks['state_sequence']=[r['sequence'] for r in tr]==list(range(len(tr)))
 checks['state_ipc_payload']=len(states)==len(tr) and all(states[i]==r['payload'] for i,r in enumerate(received))
 checks['state_ipc_coverage']=len(received)>=len(tr)-1 and [r['payload']['sequence'] for r in received]==list(range(len(received)))
 expected_keys=('sequence','valid','elapsed_end','solver_state_ned_frd','pitch_nose_up_rad','watchdog_zero','applied_commands')
 checks['actual_solver_feedback']=all(s=={**{k:r[k] for k in expected_keys},'error':None} for s,r in zip(states,tr))
 delays=[r['received_monotonic']-tr[i]['wall_monotonic'] for i,r in enumerate(received)]
 checks['state_delivery_bound']=bool(delays) and all(0<=d<=.25 for d in delays)
 checks['command_ipc_payload']=[c['payload'] for c in sent]==ipc
 checks['command_sequence']=[c['sequence'] for c in ipc]==list(range(len(ipc)))
 by={c['sequence']:c for c in ipc};by_sent={c['payload']['sequence']:c for c in sent};ages=[];bad=[];applied=set()
 from ros2.twin_axial_g12_real_v3.timing import command_age
 from ros2.twin_axial_v2.allocation import allocate,wrench
 for r in tr:
  seq=r['command_sequence']
  if seq<0:continue
  c=by.get(seq)
  if c is None or c['commands']!=r['allocation']['requested_wrench'] or c['stamp']!=r['command_simulation_stamp']:bad.append(r['sequence']);continue
  applied.add(seq);ages.append(r['command_received_wall']-by_sent[seq]['sent_monotonic'])
  if allocate(c['commands'])!=r['allocation'] or wrench(r['applied_commands'])!=r['applied_wrench']:bad.append(r['sequence'])
 checks['command_actual_application']=len(applied)>=30 and not bad
 checks['command_delivery_bound']=bool(ages) and all(0<=d<=.25 for d in ages)
 checks['watchdog']=all(not r['watchdog_zero'] and 0<=command_age(r['start_time'],r['command_simulation_stamp'])<=.25+1e-6 and r['wall_monotonic']-r['command_received_wall']<=1.0 for r in tr if r['elapsed_start']>=2.2)
 covered=set()
 for o in poses:
  for i,r in enumerate(tr):
   if abs(o['time']-r['start_time'])>.05+1e-6:continue
   q=r['orientation_enu_flu_wxyz'];v=o['quaternion_xyzw'];dot=abs(sum(a*b for a,b in zip(q,[v[3],*v[:3]])))
   if math.dist(o['position'],r['position_enu_m'])<=.01 and 2*math.acos(min(1,dot))<=math.radians(1):covered.add(i)
 checks['pose_actual_type']=len(poses)>=30 and all(p['message_type']=='lotusim_msgs/msg/VesselPositionArray' for p in poses)
 checks['pose_mirror']=len(covered)/len(tr)>=.95
 act=read(directory/'action.json');end=read(directory/'termination.json')
 checks['action']=bool(act.get('accepted') and act.get('status')==4 and act.get('results') and all(act['results']))
 checks['action_after_stop']=bool(act.get('sent_monotonic',0)>=end.get('stop_monotonic',math.inf))
 checks['termination']=bool(end.get('cleanup_completed') and end.get('last_step')==end.get('step_after_cleanup')==len(tr) and end.get('gateway_returncode') in (0,-15) and end.get('solver_returncode') in (0,-15))
 checks['zero_actually_applied']=all(all(v==0 for v in r['applied_commands'].values()) for r in tr[-4:])
 checks['no_native_crash']=not any(marker in (directory/'gazebo.log').read_text(errors='replace') for marker in ('Segmentation fault','core dumped','terminate called'))
 checks['clean_process_exit']=len(meta['processes'])==3 and all(p['returncode'] in (0,-15) for p in meta['processes'])
 checks['distinct_processes']=len({p['pid'] for p in meta['processes']}|{meta['collector_pid']})==4
 return dict(passed=all(checks.values()),checks=checks,steps=len(tr),state_samples=len(received),commands_applied=len(applied),pose_samples=len(poses),pose_coverage=len(covered)/len(tr),max_state_delivery_upper_bound_s=max(delays),max_command_delivery_s=max(ages) if ages else None,manifest_sha256=sha(directory/'manifest.json'),scope='Actual 13 s zero-input integration only; no navigation or four-case acceptance',real_started=meta['real_started'],formal_trials_started=False)

def evaluate(directory):
 contract();proof=base.evaluate(PYTHON,runtime=True,gui=True)
 try:transport=analyze(directory,fresh=True)
 except Exception as exc:transport=dict(passed=False,error=repr(exc))
 report={**proof.report,'joint_release_sha256':sha(RELEASE),'v3_release_sha256':sha(v3.RELEASE),'transport':transport,'passed':proof.report['passed'] and transport['passed'],'ready_to_run':proof.report['ready_to_run'] and transport['passed'],'formal_trials_started':False}
 report['scope']='Joint launch conditions at check time; probe is not a navigation acceptance'
 return report


def abort_evidence(directory):
 directory=Path(directory);release=contract();manifest=read(directory/'manifest.json')
 for name,digest in manifest.items():require(sha(directory/name)==digest,'Abort evidence drift: '+name)
 require(sha(directory/'joint_release.json')==sha(RELEASE),'Abort release mismatch')
 for name,digest in release['files'].items():require(sha(directory/'source'/name)==digest,'Abort snapshot mismatch: '+name)
 meta=read(directory/'probe.json');exc=read(directory/'exception.json');end=read(directory/'termination.json');act=read(directory/'action.json');tr=rows(directory/'transactions.jsonl')
 require(meta.get('injected_abort') and meta['status']=='failed' and exc.get('sequence')==120 and 'Injected collector abort' in exc.get('error',''),'Missing deliberate abort')
 checks=dict(cleanup=end.get('cleanup_completed') and end['last_step']==end['step_after_cleanup']==len(tr),
             actual_zero=bool(tr) and all(all(v==0 for v in r['applied_commands'].values()) for r in tr[-4:]),
             action=act.get('accepted') and act.get('status')==4 and act.get('results')==[True] and act.get('names')==['uuv_twin_axial_v2'],
             no_native_crash=not any(s in (directory/'gazebo.log').read_text(errors='replace') for s in ('Segmentation fault','core dumped','terminate called')),
             own_process_exit=len(meta['processes'])==3 and all(p['returncode'] in (0,-15) for p in meta['processes']))
 return dict(passed=all(checks.values()),checks=checks,manifest_sha256=sha(directory/'manifest.json'),scope='Expected failed probe; exception cleanup evidence only')


@dataclass(frozen=True)
class PilotProof:
 report: dict
 probes: dict
 release_sha256: str


def pilot_evaluate(normal,repeat,abort):
 paths={k:Path(p).resolve() for k,p in dict(normal=normal,repeat=repeat,abort=abort).items()}
 require(len(set(paths.values()))==3,'Three distinct probe records required')
 proof=base.evaluate(PYTHON,runtime=True,gui=True)
 reports={k:analyze(paths[k]) for k in ('normal','repeat')}
 reports['abort']=abort_evidence(paths['abort'])
 # At least the repeat must be fresh at launch; retain the earlier normal/abort evidence.
 age=time.time()-read(paths['repeat']/'probe.json')['completed_unix']
 passed=proof.report['passed'] and all(r['passed'] for r in reports.values()) and 0<=age<=600
 report={**proof.report,'passed':passed,'ready_to_run':passed,'joint_release_sha256':sha(RELEASE),'transport':reports,'formal_trials_started':False,'scope':'Four pilot launch conditions only; no navigation result'}
 return PilotProof(report,paths,sha(RELEASE))


def guard(proof,runtime=True,gui=True):
 require(isinstance(proof,PilotProof) and gui,'Bound GUI pilot proof required')
 require(proof.report['passed'] and proof.report['ready_to_run'],'Pilot launch blocked')
 require(sha(RELEASE)==proof.release_sha256,'Pilot release changed')
 contract()
 for label,path in proof.probes.items():
  current=abort_evidence(path) if label=='abort' else analyze(path,fresh=label=='repeat')
  require(current==proof.report['transport'][label],'Probe changed since check')
 if runtime:require(base.runtime_check(True)['passed'],'Runtime no longer ready')


def verify_record_binding(directory):
 directory=Path(directory);proof=read(directory/'evidence_gate.json');release=read(directory/'adapter_release.json')
 require(proof['passed'] and proof['ready_to_run'],'Missing launch proof')
 require(sha(directory/'adapter_release.json')==sha(RELEASE)==proof['joint_release_sha256'],'Trial release mismatch')
 for name,digest in release['files'].items():require(sha(directory/'source'/name)==digest,'Trial snapshot mismatch: '+name)
 for label in ('normal','repeat','abort'):
  path=directory/'transport_evidence'/label
  report=abort_evidence(path) if label=='abort' else analyze(path)
  require(report==proof['transport'][label] and report['passed'],'Trial probe binding mismatch')
 return True

def main():
 p=argparse.ArgumentParser();p.add_argument('mode',choices=['seal','runtime','check','analyze']);p.add_argument('--probe',type=Path);p.add_argument('--report',type=Path);args=p.parse_args()
 if args.report and args.report.exists():raise ValueError('Report exists')
 if args.mode=='seal':result=seal()
 elif args.mode=='runtime':result=base.runtime_check(True)
 else:
  try:result=evaluate(args.probe) if args.mode=='check' else analyze(args.probe)
  except Exception as exc:result=dict(passed=False,ready_to_run=False,error=repr(exc),formal_trials_started=False)
 if args.report:write(args.report,result)
 print(json.dumps({k:v for k,v in result.items() if k not in ('adapter_equivalence','files')},indent=2),flush=True)
 return 0 if result.get('passed',args.mode=='seal') else 2
if __name__=='__main__':raise SystemExit(main())
