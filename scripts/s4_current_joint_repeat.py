from pathlib import Path
from types import SimpleNamespace
import json,sys
ROOT=Path('/mnt/c/Users/Songyuan/uuv_simulator'); sys.path.insert(0,str(ROOT))
from ros2.twin_axial_s4_current_v1.gate import Proof,RELEASE,sha
from ros2.twin_axial_s4_current_v1.trial import run_trial
base=ROOT/'artifacts/twin_axial_s4_terminal_real_gate_20261118_01'
report=json.loads((ROOT/'artifacts/s4_current_joint_20261120_02/gate/gate.json').read_text())
proof=Proof(report,{},sha(RELEASE),None)
out=ROOT/'artifacts/s4_current_joint_20261120_02/real/S4/F2/P0_repeat_v2'
args=SimpleNamespace(normal_probe=base/'normal',repeat_probe=base/'repeat',abort_probe=base/'abort',output=out,scene='S4',flow='F2',case='P0',gui=True,gate_proof=proof)
print('RUN P0 repeat',flush=True); run_trial(args)
