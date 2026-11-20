from pathlib import Path
import json, sys
ROOT=Path('/mnt/c/Users/Songyuan/uuv_simulator'); sys.path.insert(0,str(ROOT))
from ros2.twin_axial_s4_current_v1.gate import pilot_evaluate
base=ROOT/'artifacts/twin_axial_s4_terminal_real_gate_20261118_01'
p=pilot_evaluate(base/'normal',base/'repeat',base/'abort')
out=ROOT/'artifacts/s4_current_joint_20261120_02/gate'; out.mkdir(parents=True,exist_ok=True)
(out/'gate.json').write_text(json.dumps(p.report,indent=2),encoding='utf-8')
print(json.dumps({'passed':p.report.get('passed'),'ready':p.report.get('ready_to_run'),'error':p.report.get('error')},indent=2))
