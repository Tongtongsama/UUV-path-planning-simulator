"""S1/F0 navigation: actual LOTUSim/xdyn, existing A*/LOS/PID."""
import argparse,json,subprocess,sys
from pathlib import Path
from .mission import CASES

def main():
    p=argparse.ArgumentParser(description=__doc__);subs=p.add_subparsers(dest='mode',required=True)
    run=subs.add_parser('run');run.add_argument('--case',choices=CASES,default='P0');run.add_argument('--output',type=Path,required=True)
    suite=subs.add_parser('suite');suite.add_argument('--output',type=Path,required=True)
    analyze=subs.add_parser('analyze');analyze.add_argument('directory',type=Path)
    args=p.parse_args()
    if args.mode in ('run','suite') and args.output.exists():p.error('Output exists; choose a new directory')
    if args.mode=='run':
        from .trial import run_trial
        run_trial(args)
    elif args.mode=='suite':
        from .acceptance import analyze_suite
        args.output.mkdir(parents=True)
        for name in (*CASES,'P0_repeat'):
            subprocess.run([sys.executable,'-m','ros2.twin_axial_navigation.cli','run','--case',name.split('_')[0],'--output',str(args.output/name)])
        report=analyze_suite(args.output)
        (args.output/'suite.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps(report,indent=2))
        if not report['passed']:raise SystemExit(2)
    else:
        from .acceptance import analyze_run,analyze_suite
        report=analyze_run(args.directory) if (args.directory/'metadata.json').exists() else analyze_suite(args.directory)
        print(json.dumps(report,indent=2))
        if not report['passed']:raise SystemExit(2)

if __name__=='__main__':main()
