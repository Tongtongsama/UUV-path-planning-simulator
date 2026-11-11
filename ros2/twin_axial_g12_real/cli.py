"""Read-only G12 gate, or explicitly requested real runs with GUI by default."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
from .gate import evaluate,guard
from ros2.twin_axial_s3_transient_v2.cli import write


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=('check','run','suite','real-matrix','analyze'))
    parser.add_argument('--python-matrix',type=Path)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--report',type=Path)
    parser.add_argument('--source',type=Path)
    parser.add_argument('--scene',choices=('S1','S2','S3'),default='S3')
    parser.add_argument('--flow',choices=('F0','F1','F2'),default='F0')
    parser.add_argument('--case',choices=('P0','P1','P2','P3','P4'),default='P0')
    parser.add_argument('--check-only',action='store_true')
    parser.add_argument('--runtime',action='store_true')
    parser.add_argument('--headless',action='store_true')
    args=parser.parse_args(argv)
    attempted_trials=[]
    if args.report and args.report.exists():parser.error('Report exists; choose a new file')
    if args.output and args.output.exists():parser.error('Run output exists; never overwrite records')
    if args.mode=='analyze':
        if args.source is None:parser.error('--source required')
        from .acceptance import analyze_run,analyze_suite
        result=analyze_run(args.source) if (args.source/'metadata.json').exists() else analyze_suite(args.source)
    else:
        if args.python_matrix is None:parser.error('--python-matrix required')
        if args.mode!='check' and args.output is None:parser.error('--output required')
        try:
            proof=evaluate(args.python_matrix,runtime=args.runtime or args.mode!='check',gui=not args.headless)
            result=proof.report
            if result['passed'] and args.mode!='check' and not args.check_only:
                guard(proof,runtime=True,gui=not args.headless)
                # No transport import, directory creation or launch occurs above this boundary.
                from .trial import run_trial
                from .acceptance import analyze_run,analyze_suite
                if args.mode=='run':
                    trial=SimpleNamespace(**vars(args),gui=not args.headless,gate_proof=proof)
                    attempted_trials.append(trial)
                    try:run_trial(trial)
                    except SystemExit as exc:
                        if exc.code not in (0,2):raise
                    result=analyze_run(args.output)
                else:
                    args.output.mkdir(parents=True)
                    write(args.output/'evidence_gate.json',proof.report)
                    groups=[(args.scene,args.flow)] if args.mode=='suite' else [(s,f) for s in ('S1','S2','S3') for f in ('F0','F1','F2')]
                    summaries={}
                    for scene,flow in groups:
                        folder=args.output if args.mode=='suite' else args.output/scene/flow
                        folder.mkdir(parents=True,exist_ok=True)
                        for name in ('P0','P1','P2','P3','P4','P0_repeat'):
                            trial=SimpleNamespace(output=folder/name,scene=scene,flow=flow,case=name.split('_')[0],
                                                  gui=not args.headless,python_matrix=args.python_matrix,gate_proof=proof)
                            attempted_trials.append(trial)
                            try:run_trial(trial)
                            except SystemExit as exc:
                                if exc.code not in (0,2):raise
                        summaries[f'{scene}/{flow}']=analyze_suite(folder)
                        write(folder/'suite.json',summaries[f'{scene}/{flow}'])
                    result=dict(passed=all(v['passed'] and v['all_runs_passed'] for v in summaries.values()),groups=summaries)
                    write(args.output/'matrix.json',result)
        except Exception as exc:
            result=dict(passed=False,ready_to_run=False,real_started=any(getattr(t,'real_started',False) for t in attempted_trials),error=repr(exc))
    if attempted_trials:
        result['real_started']=any(getattr(t,'real_started',False) for t in attempted_trials)
    if args.report:
        args.report.parent.mkdir(parents=True,exist_ok=True)
        write(args.report,result)
    print(json.dumps({k:v for k,v in result.items() if k!='adapter_equivalence'},indent=2),flush=True)
    return 0 if result.get('passed',False) else 2


if __name__=='__main__':
    raise SystemExit(main())
