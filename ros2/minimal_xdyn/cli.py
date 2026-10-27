"""Run one isolated open-loop trial. No PID, LOS, playback or Python dynamics."""
import argparse
from pathlib import Path

def main():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='mode',required=True)
    from .protocol import CASES
    run=sub.add_parser('run'); run.add_argument('--case',choices=CASES,required=True)
    run.add_argument('--output',type=Path,required=True)
    run.add_argument('--gui',action='store_true')
    run.add_argument('--keep-open',action='store_true')
    suite=sub.add_parser('suite',help='All eight cases plus a repeated forward pulse, headless')
    suite.add_argument('--output',type=Path,required=True)
    analyze=sub.add_parser('analyze'); analyze.add_argument('directory',type=Path)
    args=p.parse_args()
    if args.mode=='run':
        from .trial import run_trial
        run_trial(args)
    elif args.mode=='suite':
        import json
        import subprocess
        import sys
        from .acceptance import compare_repeat
        args.output.mkdir(parents=True,exist_ok=False)
        reports={}
        for case in (*CASES,'surge_positive_repeat'):
            result=subprocess.run([sys.executable,'-m','ros2.minimal_xdyn.cli','run','--case',
                case.removesuffix('_repeat'),'--output',str(args.output/case)])
            reports[case]={'exit_code':result.returncode}
            if result.returncode:
                (args.output/'suite.json').write_text(json.dumps({'passed':False,'runs':reports},indent=2))
                raise SystemExit(result.returncode)
        repeat=compare_repeat(args.output/'surge_positive',args.output/'surge_positive_repeat')
        report={'passed':repeat['passed'],'runs':reports,'repeatability':repeat}
        (args.output/'suite.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2))
        if not report['passed']: raise SystemExit(2)
    else:
        from .acceptance import analyze_run
        import json
        report=analyze_run(args.directory)
        print(json.dumps(report,indent=2))
        if not report['passed']: raise SystemExit(2)

if __name__=='__main__': main()
