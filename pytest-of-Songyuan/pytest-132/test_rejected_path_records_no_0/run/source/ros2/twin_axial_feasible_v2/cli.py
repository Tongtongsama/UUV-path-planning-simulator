"""Independent feasible navigation; a passed Python suite gates real runs."""
import argparse,json
from pathlib import Path
from ros2.twin_axial_navigation.mission import CASES

def main():
    p=argparse.ArgumentParser(description=__doc__);s=p.add_subparsers(dest='mode',required=True)
    for mode in ('python-run','python-suite','python-matrix','run','suite'):
        q=s.add_parser(mode);q.add_argument('--output',type=Path,required=True);q.add_argument('--flow',choices=['F0','F1','F2'],default='F2')
        q.add_argument('--case',choices=CASES,default='P0')
        if mode in ('run','suite'):q.add_argument('--python-matrix',type=Path,required=True);q.add_argument('--headless',action='store_true')
    for mode in ('python-analyze','analyze'):
        q=s.add_parser(mode);q.add_argument('directory',type=Path)
    a=p.parse_args()
    if hasattr(a,'output') and a.output.exists():p.error('Output exists; choose a new directory')
    if a.mode=='python-run':
        from .python_trial import run
        result=run(a.output,a.case,a.flow)
    elif a.mode=='python-matrix':
        from .python_trial import run
        from .verify_python import suite,matrix
        a.output.mkdir(parents=True)
        for flow in ('F0','F1','F2'):
            folder=a.output/flow;folder.mkdir()
            for name in (*CASES,'P0_repeat'):run(folder/name,name.split('_')[0],flow)
            (folder/'suite.json').write_text(json.dumps(suite(folder),indent=2))
        result=matrix(a.output);(a.output/'matrix.json').write_text(json.dumps(result,indent=2))
    elif a.mode=='python-suite':
        from .python_trial import run
        from .verify_python import suite
        a.output.mkdir(parents=True)
        for name in (*CASES,'P0_repeat'):run(a.output/name,name.split('_')[0],a.flow)
        result=suite(a.output);(a.output/'suite.json').write_text(json.dumps(result,indent=2))
    elif a.mode in ('run','suite'):
        from .verify_python import gate
        gate(a.python_matrix)
        a.gui=not a.headless
        from .trial import run_trial
        from .acceptance import analyze_suite,analyze_run
        a.scene='S2'
        if a.mode=='run':
            run_trial(a);result=analyze_run(a.output)
        else:
            a.output.mkdir(parents=True);parent=a.output
            for name in (*CASES,'P0_repeat'):
                a.output=parent/name;a.case=name.split('_')[0]
                try:run_trial(a)
                except SystemExit as e:
                    if e.code not in (0,2):raise
            result=analyze_suite(parent);(parent/'suite.json').write_text(json.dumps(result,indent=2))
    elif a.mode=='python-analyze':
        from .verify_python import analyze,suite,matrix
        result=matrix(a.directory) if (a.directory/'F0').is_dir() else analyze(a.directory) if (a.directory/'states.jsonl').exists() else suite(a.directory)
    else:
        from .acceptance import analyze_run,analyze_suite
        result=analyze_run(a.directory) if (a.directory/'metadata.json').exists() else analyze_suite(a.directory)
    print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 2)

if __name__=='__main__':main()
