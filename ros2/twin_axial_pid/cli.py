"""No-current PID step tests through LOTUSim/xdyn; no LOS or paths."""
import argparse
import json
import subprocess
import sys
from pathlib import Path
from .protocol import CASES

def main():
    p=argparse.ArgumentParser(description=__doc__)
    subs=p.add_subparsers(dest='mode',required=True)
    run=subs.add_parser('run'); run.add_argument('--case',choices=CASES,required=True)
    run.add_argument('--output',type=Path,required=True)
    suite=subs.add_parser('suite'); suite.add_argument('--output',type=Path,required=True)
    analyze=subs.add_parser('analyze'); analyze.add_argument('directory',type=Path)
    args=p.parse_args()
    if args.mode in ('run','suite') and args.output.exists():
        p.error('Output exists; choose a new directory. Old records are never overwritten.')
    if args.mode=='run':
        from .trial import run_trial
        run_trial(args)
    elif args.mode=='suite':
        args.output.mkdir(parents=True)
        results={}
        for case in CASES:
            result=subprocess.run([sys.executable,'-m','ros2.twin_axial_pid.cli','run',
                '--case',case,'--output',str(args.output/case)])
            results[case]={'exit_code':result.returncode}
        report={'passed':all(r['exit_code']==0 for r in results.values()),'runs':results}
        (args.output/'suite.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps(report,indent=2))
        if not report['passed']: raise SystemExit(2)
    else:
        from .acceptance import analyze_run
        if (args.directory/'suite.json').exists():
            reports={case:analyze_run(args.directory/case) for case in CASES}
            report={'passed':all(r['passed'] for r in reports.values()),'runs':reports}
        else: report=analyze_run(args.directory)
        print(json.dumps(report,indent=2))
        if not report['passed']: raise SystemExit(2)

if __name__=='__main__': main()
