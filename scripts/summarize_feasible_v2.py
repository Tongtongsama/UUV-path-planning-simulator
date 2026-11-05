"""Summarize the fixed 2026-11-04 S2 matrix; never run dynamics or change records."""
import hashlib
import json
import math
from pathlib import Path
from datetime import datetime, timezone

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'artifacts/twin_axial_feasible_v2'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    py=json.loads((BASE/'python_20261104_01/matrix.json').read_text())
    result={'created_utc':datetime.now(timezone.utc).isoformat(),
            'campaign':'20261104_01; F1/F2 completed on 2026-11-05 Asia/Singapore',
            'python_passed':py['passed'],'flows':{},'source_identity':True,'checks':{}}
    configs=[];sources=[];evidence={}
    for flow in ('F0','F1','F2'):
        folder=BASE/f'xdyn_{flow.lower()}_gui_20261104_01'
        suite=json.loads((folder/'suite.json').read_text())
        reports=suite['runs'];pr=py['flows'][flow]['runs']
        result['flows'][flow]={'python':{
            'passed':py['flows'][flow]['passed'],'runs_passed':sum(r['passed'] for r in pr.values()),
            'repeatability':py['flows'][flow]['repeatability'],
            'max_rms_m':max(r['rms_error'] for r in pr.values()),'max_error_m':max(r['max_error'] for r in pr.values()),
            'min_clearance_m':min(r['min_clearance'] for r in pr.values())},'xdyn':{
            'passed':suite['passed'],'runs_passed':sum(r['passed'] for r in reports.values()),
            'max_rms_m':max(r['path_rms_m'] for r in reports.values()),'max_error_m':max(r['path_max_m'] for r in reports.values()),
            'min_clearance_m':min(r['minimum_clearance_m'] for r in reports.values()),
            'repeatability':suite['repeatability'],'runs':reports}}
        for name in reports:
            run=folder/name
            python_rows=[json.loads(line) for line in (BASE/'python_20261104_01'/flow/name/'states.jsonl').read_text().splitlines()]
            real_rows={round(r['elapsed_end'],6):r for r in
                       (json.loads(line) for line in (run/'transactions.jsonl').read_text().splitlines()) if r['valid']}
            differences=[math.dist(r['state'][:2],real_rows[round(r['t'],6)]['position_planning_xz_m']) for r in python_rows if round(r['t'],6) in real_rows]
            reports[name]['python_real_max_position_difference_m']=max(differences)
            reports[name]['python_real_matched_steps']=len(differences)
            reports[name]['python_real_python_steps']=len(python_rows)
            status=json.loads((run/'run_status.json').read_text())
            assert any('--gui' in p['command'] for p in status['process_exit_codes'])
            configs.append(json.loads((run/'plan.json').read_text())['config'])
            source=run/'source'
            sources.append({p.relative_to(source).as_posix():digest(p) for p in source.rglob('*') if p.is_file()})
            manifest=json.loads((run/'manifest.json').read_text())
            for relative,expected in manifest.items():
                assert digest(run/relative)==expected,(run,relative)
            evidence[(run/'acceptance.json').relative_to(ROOT).as_posix()]=digest(run/'acceptance.json')
        evidence[(folder/'suite.json').relative_to(ROOT).as_posix()]=digest(folder/'suite.json')
    result['source_identity']=all(s==sources[0] for s in sources)
    result['same_config']=all(c==configs[0] for c in configs)
    py_source=BASE/'python_20261104_01/F0/P0/source'
    py_hashes={p.relative_to(py_source).as_posix():digest(p) for p in py_source.rglob('*') if p.is_file()}
    shared=set(py_hashes)&set(sources[0])
    result['python_real_shared_source_files']=len(shared)
    result['checks']['python_real_shared_source_identical']=bool(shared) and all(py_hashes[p]==sources[0][p] for p in shared)
    result['checks']['python_real_config_identical']=configs[0]==json.loads((BASE/'python_20261104_01/F0/P0/plan.json').read_text())['config']
    evidence[(BASE/'python_20261104_01/matrix.json').relative_to(ROOT).as_posix()]=digest(BASE/'python_20261104_01/matrix.json')
    protected=json.loads((BASE/'protected_before.json').read_text())
    result['checks']['protected_files_unchanged']=all(digest(ROOT/p)==h for p,h in protected.items())
    preservation=json.loads((BASE/'preservation_final_20261105.json').read_text())
    result['preservation']=preservation
    result['checks']['prior_records_unchanged']=preservation['passed']
    result['checks']['acceptance_logic_unchanged']=(ROOT/'ros2/twin_axial_feasible/acceptance.py').read_text()==(ROOT/'ros2/twin_axial_feasible_v2/acceptance.py').read_text()
    result['checks']['all_real_runs_passed']=all(v['xdyn']['runs_passed']==6 and v['xdyn']['passed'] for v in result['flows'].values())
    result['checks']['all_gui']=True
    result['checks']['real_recording_hashes']=True
    result['evidence_sha256']=evidence
    result['passed']=result['python_passed'] and result['source_identity'] and result['same_config'] and all(result['checks'].values())
    (BASE/'summary_20261104.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('flows','evidence_sha256')},indent=2))
    raise SystemExit(0 if result['passed'] else 2)


if __name__=='__main__':main()
