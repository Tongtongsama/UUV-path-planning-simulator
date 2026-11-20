from pathlib import Path
from types import SimpleNamespace
import json, sys
ROOT=Path('/mnt/c/Users/Songyuan/uuv_simulator'); sys.path.insert(0,str(ROOT))
from ros2.twin_axial_s4_current_v1.gate import pilot_evaluate
from ros2.twin_axial_s4_current_v1.trial import run_trial
base=ROOT/'artifacts/twin_axial_s4_terminal_real_gate_20261118_01'
p=pilot_evaluate(base/'normal',base/'repeat',base/'abort')
print({'gate_passed':p.report.get('passed'),'ready':p.report.get('ready_to_run')},flush=True)
out=ROOT/'artifacts/s4_current_joint_20261120_02/real/S4/F2/P0_v2'
args=SimpleNamespace(normal_probe=base/'normal',repeat_probe=base/'repeat',abort_probe=base/'abort',
 output=out,scene='S4',flow='F2',case='P0',gui=True,gate_proof=p)
run_trial(args)
