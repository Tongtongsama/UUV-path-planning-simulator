"""S1/S2 navigation: real LOTUSim/xdyn trials and read-only GUI replay."""
import argparse,json,subprocess,sys,math
from pathlib import Path
from .mission import CASES,SCENES
from .current_model import FLOWS

def main():
    p=argparse.ArgumentParser(description=__doc__);subs=p.add_subparsers(dest='mode',required=True)
    run=subs.add_parser('run');run.add_argument('--case',choices=CASES,default='P0');run.add_argument('--scene',choices=SCENES,default='S1');run.add_argument('--flow',choices=FLOWS,default='F0');run.add_argument('--gui',action='store_true');run.add_argument('--output',type=Path,required=True)
    suite=subs.add_parser('suite');suite.add_argument('--scene',choices=SCENES,default='S1');suite.add_argument('--flow',choices=FLOWS,default='F0');suite.add_argument('--output',type=Path,required=True)
    analyze=subs.add_parser('analyze');analyze.add_argument('directory',type=Path)
    numerics=subs.add_parser('current-numerics');numerics.add_argument('--output',type=Path,required=True)
    replay=subs.add_parser('replay');replay.add_argument('recording',type=Path)
    replay.add_argument('--reference',type=Path)
    replay.add_argument('--include-failed-reference',action='store_true')
    replay.add_argument('--output',type=Path)
    replay.add_argument('--rate',type=float,default=1)
    replay.add_argument('--hz',type=float,default=10)
    replay.add_argument('--loops',type=int,default=0)
    replay.add_argument('--ros-domain-id',type=int,choices=range(80,220))
    replay.add_argument('--validate-only',action='store_true')
    args=p.parse_args()
    if args.mode in ('run','suite','current-numerics') and args.output.exists():p.error('Output exists; choose a new directory')
    if args.mode=='run':
        from .trial import run_trial
        run_trial(args)
    elif args.mode=='suite':
        from .acceptance import analyze_suite
        args.output.mkdir(parents=True)
        for name in (*CASES,'P0_repeat'):
            subprocess.run([sys.executable,'-m','ros2.twin_axial_navigation.cli','run','--case',name.split('_')[0],'--scene',args.scene,'--flow',args.flow,'--output',str(args.output/name)])
        report=analyze_suite(args.output)
        (args.output/'suite.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps(report,indent=2))
        if not report['passed']:raise SystemExit(2)
    elif args.mode=='replay':
        from .gui_replay import run
        if not math.isfinite(args.rate) or args.rate<=0 or not math.isfinite(args.hz) or args.hz<=0:
            p.error('rate and hz must be finite and positive')
        if not args.validate_only and args.output is None:p.error('--output is required')
        if args.output is not None and args.output.exists():p.error('Output exists; choose a new directory')
        run(args)
    elif args.mode=='current-numerics':
        from .current_validation import validate
        report=validate(args.output)
        print(json.dumps(report,indent=2))
        if not report['passed']:raise SystemExit(2)
    else:
        from .acceptance import analyze_run,analyze_suite
        report=analyze_run(args.directory) if (args.directory/'metadata.json').exists() else analyze_suite(args.directory)
        print(json.dumps(report,indent=2))
        if not report['passed']:raise SystemExit(2)

if __name__=='__main__':main()
