"""Independent scene matrix with unchanged feasible_v2 guidance and strict real gate."""
import argparse,json
from pathlib import Path
from .scenes import SCENES,preflight
from ros2.twin_axial_navigation.mission import CASES

def main():
    p=argparse.ArgumentParser(description=__doc__);s=p.add_subparsers(dest='mode',required=True)
    for mode in ('preflight','python-run','python-matrix','run','suite','real-matrix'):
        q=s.add_parser(mode);q.add_argument('--output',type=Path,required=True)
        q.add_argument('--scene',choices=SCENES,default='S1');q.add_argument('--flow',choices=['F0','F1','F2'],default='F0')
        q.add_argument('--case',choices=CASES,default='P0')
        if mode in ('run','suite','real-matrix'):
            q.add_argument('--python-matrix',type=Path,required=True);q.add_argument('--headless',action='store_true');q.add_argument('--check-only',action='store_true')
    for mode in ('python-analyze','analyze'):
        q=s.add_parser(mode);q.add_argument('directory',type=Path)
    a=p.parse_args()
    if hasattr(a,'output') and a.output.exists():p.error('Output exists; use a new directory')
    if a.mode=='preflight':
        rows=[preflight(scene,case,flow) for scene in SCENES for flow in ('F0','F1','F2') for case in CASES]
        result=dict(passed=all(r['passed'] for r in rows),runs=rows,scope='Path/steady-trim checks only; no dynamics')
        a.output.mkdir(parents=True);(a.output/'preflight.json').write_text(json.dumps(result,indent=2))
    elif a.mode=='python-run':
        from .python_trial import run
        result=run(a.output,a.case,a.flow,a.scene)
    elif a.mode=='python-matrix':
        from .python_trial import run
        from .verify_python import suite,matrix
        a.output.mkdir(parents=True)
        for scene in SCENES:
            for flow in ('F0','F1','F2'):
                folder=a.output/scene/flow;folder.mkdir(parents=True)
                for name in (*CASES,'P0_repeat'):run(folder/name,name.split('_')[0],flow,scene)
                (folder/'suite.json').write_text(json.dumps(suite(folder),indent=2))
        result=matrix(a.output);(a.output/'matrix.json').write_text(json.dumps(result,indent=2))
    elif a.mode in ('run','suite','real-matrix'):
        from .verify_python import gate
        try:proof=gate(a.python_matrix)
        except Exception as e:
            print(json.dumps(dict(passed=False,real_started=False,error=str(e))),flush=True);raise SystemExit(2)
        if a.check_only:
            print(json.dumps(dict(passed=True,real_started=False,gui=not a.headless)));return
        from .trial import run_trial
        from .acceptance import analyze_run,analyze_suite
        a.gui=not a.headless
        if a.mode=='run':run_trial(a);result=analyze_run(a.output)
        else:
            parent=a.output;parent.mkdir(parents=True);(parent/'python_gate.json').write_text(json.dumps(proof,indent=2))
            groups=[(a.scene,a.flow)] if a.mode=='suite' else [(sc,fl) for sc in SCENES for fl in ('F0','F1','F2')]
            results={}
            for scene,flow in groups:
                folder=parent if a.mode=='suite' else parent/scene/flow
                folder.mkdir(parents=True,exist_ok=True);a.scene=scene;a.flow=flow
                for name in (*CASES,'P0_repeat'):
                    a.case=name.split('_')[0];a.output=folder/name
                    try:run_trial(a)
                    except SystemExit as e:
                        if e.code not in (0,2):raise
                report=analyze_suite(folder);(folder/'suite.json').write_text(json.dumps(report,indent=2));results[scene+'/'+flow]=report
            result=dict(passed=all(v['passed'] for v in results.values()),groups=results)
            (parent/'matrix.json').write_text(json.dumps(result,indent=2))
    elif a.mode=='python-analyze':
        from .verify_python import analyze,suite,matrix
        result=matrix(a.directory) if (a.directory/'S1').is_dir() else analyze(a.directory) if (a.directory/'report.json').exists() else suite(a.directory)
    else:
        from .acceptance import analyze_run,analyze_suite
        if (a.directory/'S1').is_dir():
            groups={sc+'/'+fl:analyze_suite(a.directory/sc/fl) for sc in SCENES for fl in ('F0','F1','F2')}
            result=dict(passed=all(r['passed'] for r in groups.values()),groups=groups)
        else:result=analyze_run(a.directory) if (a.directory/'metadata.json').exists() else analyze_suite(a.directory)
    if a.mode in ('preflight','python-matrix'):
        print(json.dumps({k:v for k,v in result.items() if k not in ('runs','groups')},indent=2),flush=True)
    else:print(json.dumps(result,indent=2),flush=True)
    raise SystemExit(0 if result['passed'] else 2)

if __name__=='__main__':main()
