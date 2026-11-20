from pathlib import Path
from types import SimpleNamespace
import json,sys
ROOT=Path('/mnt/c/Users/Songyuan/uuv_simulator'); sys.path.insert(0,str(ROOT))
from ros2.twin_axial_s4_current_v1.gate import Proof,RELEASE,sha
from ros2.twin_axial_s4_current_v1.trial import run_trial
base=ROOT/'artifacts/twin_axial_s4_terminal_real_gate_20261118_01'
report=json.loads((ROOT/'artifacts/s4_current_joint_20261120_02/gate/gate.json').read_text())
proof=Proof(report,{},sha(RELEASE),None)
print({'passed':report.get('passed'),'ready':report.get('ready_to_run'),'release':proof.release_sha256},flush=True)
for case in ('P1','P2','P3','P4'):
 out=ROOT/f'artifacts/s4_current_joint_20261120_02/real/S4/F2/{case}_repeat_v2'
 args=SimpleNamespace(normal_probe=base/'normal',repeat_probe=base/'repeat',abort_probe=base/'abort',output=out,scene='S4',flow='F2',case=case,gui=True,gate_proof=proof)
 print('RUN',case,flush=True); run_trial(args)
