from pathlib import Path
import json, sys
ROOT=Path('/mnt/c/Users/Songyuan/uuv_simulator'); sys.path.insert(0,str(ROOT))
from ros2.twin_axial_s4_current_v1.acceptance import analyze_run
out=ROOT/'artifacts/s4_current_joint_20261120_02/real/S4/F2/P0'
a=analyze_run(out); (out/'acceptance.json').write_text(json.dumps(a,indent=2),encoding='utf-8'); print(json.dumps(a,indent=2)[:1200])
