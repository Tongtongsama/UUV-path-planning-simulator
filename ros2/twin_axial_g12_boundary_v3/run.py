import json,subprocess,sys,traceback,contextlib
from pathlib import Path
from types import SimpleNamespace
from ros2.twin_axial_g12_boundary_v2.config import CASES
from ros2.twin_axial_g12_boundary_v2.cli import write
from ros2.twin_axial_s3_transient_v2.cli import ROOT,read
from .gate import contract,pilot_evaluate
def real(out):
 from .trial import run_trial
 contract();results={};old=ROOT/'artifacts/twin_axial_g12_link_v4'
 (out/'real').mkdir(exist_ok=False)
 for name in CASES:
  if not read(out/'python'/name/'report.json')['passed']:
   result={'real_started':False,'passed':False,'classification':'python_gate_refusal','reason':'Corresponding Python case failed original limits'}
   write(out/('real_'+name+'_result.json'),result);results[name]=result;continue
  probe=out/('probe_'+name);log=out/('probe_'+name+'.log')
  with log.open('x') as f:
   process=subprocess.run([sys.executable,'-m','ros2.twin_axial_g12_link_v6.probe','--transport','UDPv4','--output',str(probe)],stdout=f,stderr=subprocess.STDOUT)
  write(out/('probe_'+name+'_exit.json'),{'returncode':process.returncode})
  args=SimpleNamespace(output=out/'real'/name,scene='S2',case='P0',flow=name,gui=True,real_started=False)
  result={}
  try:
   if process.returncode:raise RuntimeError('Fresh joint probe failed')
   args.gate_proof=pilot_evaluate(old/'probe_03',probe,old/'probe_abort_05',name,out/'python')
   write(out/('gate_'+name+'.json'),args.gate_proof.report)
   with (out/('trial_'+name+'.log')).open('x') as f,contextlib.redirect_stdout(f),contextlib.redirect_stderr(f):run_trial(args)
  except (Exception,SystemExit) as exc:result['error']=repr(exc);result['traceback']=traceback.format_exc()
  result['real_started']=args.real_started
  if (args.output/'acceptance.json').exists():result['acceptance']=read(args.output/'acceptance.json')
  result['passed']=result.get('acceptance',{}).get('passed',False)
  write(out/('real_'+name+'_result.json'),result);results[name]=result
  print(json.dumps({'case':name,**{k:v for k,v in result.items() if k not in ('acceptance','traceback')}}),flush=True)
 write(out/'real_summary.json',results)

if __name__=='__main__':real(Path(sys.argv[1]).resolve())
