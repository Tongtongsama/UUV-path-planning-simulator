"""Explicit five-case preregistration, Python first, case-bound GUI gate second."""
import argparse,json,hashlib,shutil,sys,subprocess,traceback,contextlib,time
from pathlib import Path
from types import SimpleNamespace
from .config import CASES,model_config
from .python_trial import run,verify
from .gate import RELEASE,contract,pilot_evaluate
from ros2.twin_axial_g12_link_v6 import gate as parent
from ros2.twin_axial_s3_transient_v2.cli import ROOT,read,sha

def write(p,d):
 with p.open('x',encoding='utf-8') as f:json.dump(d,f,indent=2,default=lambda v:v.item() if hasattr(v,'item') else (_ for _ in ()).throw(TypeError(type(v).__name__)))

def register(out):
 parent.contract()
 if not RELEASE.exists():
  files=dict(read(parent.RELEASE)['files'])
  for p in Path(__file__).parent.glob('*.py'):files[p.relative_to(ROOT).as_posix()]=sha(p)
  test=ROOT/'tests/test_twin_axial_g12_boundary_v2.py';files[test.relative_to(ROOT).as_posix()]=sha(test)
  write(RELEASE,{'revision':'g12_boundary_v2','parent_release_sha256':sha(parent.RELEASE),'files':files,'runtime_files':read(parent.RELEASE)['runtime_files']})
 out.mkdir(parents=True,exist_ok=False)
 write(out/'preregistration.json',{'cases':CASES,'scene':'S2','initial':'P0','release_sha256':sha(RELEASE),
  'scope':'Exactly baseline 3 upward-current and 2 independent damping perturbations; no nominal reclassification',
  'python_first':True,'real_gate':'All five Python records must be present and replay-verified; only corresponding passing case may launch, with fresh unchanged v6 joint gate and nominal full Python evidence.',
  'real_retries':0,'probe_attempts':1,'gui':True,'threshold_changes':False,'frozen_model_overwritten':False,'failure_policy':'Retain every failed or refused case; no substitute trials; engineering gate failure blocks launch.'})
 old=ROOT/'artifacts/twin_axial_g12_real_matrix_20261113_01';protected=read(old/'preservation_before.json')
 for line in (old/'SHA256SUMS_final_20261114.txt').read_text().splitlines():
  h,n=line.split('  ',1);protected[(old/Path(n)).relative_to(ROOT).as_posix()]=h
 write(out/'preservation_before.json',protected)
 py=out/'python';py.mkdir();write(py/'configurations.json',CASES)
 for n in contract()['files']:
  dest=py/'source'/n;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/n,dest)
 for name in CASES:write(out/(name+'_model.json'),model_config('P0',name))

def python(out):
 contract();results={}
 for name in CASES:
  result=run(out/'python'/name,name,sha(RELEASE))
  check=verify(out/'python'/name,name)
  write(out/'python'/(name+'_verification.json'),check)
  results[name]={'passed':result['passed'],'verification':check}
 write(out/'python_summary.json',results)

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

def main():
 p=argparse.ArgumentParser();p.add_argument('mode',choices=('register','python','real'));p.add_argument('--output',type=Path,required=True);args=p.parse_args()
 globals()[args.mode](args.output.resolve())
if __name__=='__main__':main()
