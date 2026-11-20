from pathlib import Path
from types import SimpleNamespace
import json,traceback,sys
ROOT=Path('/mnt/c/Users/Songyuan/uuv_simulator'); sys.path.insert(0,str(ROOT))
from ros2.twin_axial_s4_current_v1.gate import Proof,RELEASE,sha
from ros2.twin_axial_s4_current_v1.trial import run_trial
base=ROOT/'artifacts/twin_axial_s4_terminal_real_gate_20261118_01'
report=json.loads((ROOT/'artifacts/s4_current_joint_20261120_02/gate/gate.json').read_text()); proof=Proof(report,{},sha(RELEASE),None)
for flow in ('F0','F1','F2'):
 for case in ('P1','P2','P3','P4'):
  out=ROOT/f'artifacts/s4_joint18_20261120_01/{flow}/{case}_repeat'
  if out.exists(): print('SKIP',flow,case,flush=True); continue
  args=SimpleNamespace(normal_probe=base/'normal',repeat_probe=base/'repeat',abort_probe=base/'abort',output=out,scene='S4',flow=flow,case=case,gui=True,gate_proof=proof)
  try:
   print('RUN',flow,case,flush=True); run_trial(args); print('DONE',flow,case,flush=True)
  except Exception as e:
   out.mkdir(parents=True,exist_ok=True); (out/'failure.json').write_text(json.dumps({'error':repr(e),'traceback':traceback.format_exc()},indent=2),encoding='utf-8'); print('FAIL',flow,case,repr(e),flush=True)
