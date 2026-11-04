"""Run unchanged feasible_v1 after a matching-flow Python gate. GUI defaults on."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

def verify_gate(directory,flow):
    from ros2.twin_axial_feasible.verify_python import suite
    report=suite(directory)
    if not report['passed'] or not all(r.get('flow')==flow for r in report['runs'].values()):
        raise ValueError(f'{flow} matching Python suite did not pass; no real trial started')
    source=Path(directory)/'P0/source'
    files=[p for p in source.rglob('*') if p.is_file()]
    if not files:raise ValueError('Missing Python source snapshot')
    for p in files:
        live=ROOT/p.relative_to(source)
        if not live.is_file() or hashlib.sha256(p.read_bytes()).digest()!=hashlib.sha256(live.read_bytes()).digest():
            raise ValueError('Current source differs from Python gate: '+str(live))
    return report

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--flow',choices=['F0','F1','F2'],required=True)
    parser.add_argument('--python-suite',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--check-only',action='store_true')
    args=parser.parse_args()
    if args.output.exists():parser.error('Output exists; choose a new directory')
    try:gate=verify_gate(args.python_suite,args.flow)
    except Exception as e:
        print(json.dumps({'gate_passed':False,'error':str(e),'real_started':False}),flush=True)
        raise SystemExit(2)
    print(json.dumps({'gate_passed':True,'flow':args.flow,'gui':not args.headless}),flush=True)
    if args.check_only:return
    from ros2.twin_axial_feasible.trial import run_trial
    from ros2.twin_axial_feasible.acceptance import analyze_suite
    from ros2.twin_axial_navigation.mission import CASES
    args.output.mkdir(parents=True)
    (args.output/'python_gate.json').write_text(json.dumps(gate,indent=2))
    shutil.copy2(__file__,args.output/'flow_runner.py')
    for name in (*CASES,'P0_repeat'):
        trial=SimpleNamespace(output=args.output/name,case=name.split('_')[0],scene='S2',flow=args.flow,gui=not args.headless)
        try:run_trial(trial)
        except SystemExit as e:
            if e.code not in (0,2):raise
    report=analyze_suite(args.output)
    (args.output/'suite.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2),flush=True)
    raise SystemExit(0 if report['passed'] else 2)

if __name__=='__main__':main()
