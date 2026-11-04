"""S2 double-baffle constant-current navigation; shares the S1 current preflight."""
import argparse,json,subprocess,sys
from pathlib import Path
from ros2.twin_axial_current.model import FLOWS
from .mission import CASES

def main():
    p=argparse.ArgumentParser(description=__doc__);subs=p.add_subparsers(dest='mode',required=True)
    for mode in ('numerics','run','suite'):
        sub=subs.add_parser(mode);sub.add_argument('--output',type=Path,required=True)
        if mode=='run':
            sub.add_argument('--flow',choices=FLOWS,required=True);sub.add_argument('--case',choices=CASES,default='P0')
        if mode in ('run','suite'):sub.add_argument('--preflight',type=Path,required=True,help='passed numerics directory')
    sub=subs.add_parser('analyze');sub.add_argument('directory',type=Path)
    args=p.parse_args()
    if args.mode!='analyze' and args.output.exists():p.error('Output exists; choose a new directory')
    if args.mode=='numerics':
        from ros2.twin_axial_current.numerics import run
        report=run(args.output)
    elif args.mode in ('run','suite'):
        from ros2.twin_axial_current.numerics import analyze
        if not analyze(args.preflight)['passed']:p.error('Numerical preflight failed; do not tune PID to hide it')
        if args.mode=='run':
            from .trial import run_trial
            return run_trial(args)
        from .acceptance import analyze_suite,RUNS
        args.output.mkdir(parents=True)
        for flow,case,label in RUNS:
            subprocess.run([sys.executable,'-m','ros2.twin_axial_current_double.cli','run','--flow',flow,'--case',case,
                '--preflight',str(args.preflight.resolve()),'--output',str(args.output/label)])
        report=analyze_suite(args.output)
        (args.output/'suite.json').write_text(json.dumps(report,indent=2))
    else:
        if (args.directory/'data.json').exists():
            from ros2.twin_axial_current.numerics import analyze
            report=analyze(args.directory)
        else:
            from .acceptance import analyze_run,analyze_suite
            report=analyze_run(args.directory) if (args.directory/'metadata.json').exists() else analyze_suite(args.directory)
    print(json.dumps(report,indent=2))
    if not report['passed']:raise SystemExit(2)

if __name__=='__main__':main()
