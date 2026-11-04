"""Verify the frozen 2026-10-28 minimal-dynamics evidence index (read-only)."""
import hashlib
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
INDEX=ROOT/'artifacts/milestones/lotusim_minimal_dynamics_20261028/evidence_index.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    errors=[]; checked=0
    if not INDEX.is_file():
        print(json.dumps({'passed':False,'errors':[f'missing {INDEX}']},indent=2)); return 1
    index=json.loads(INDEX.read_text(encoding='utf-8'))
    milestone=index.get('milestone',{})
    if milestone.get('id')!='lotusim-minimal-dynamics-20261028': errors.append('wrong milestone id')
    if milestone.get('status')!='frozen_within_scope': errors.append('milestone is not marked frozen_within_scope')
    for evidence in index.get('evidence',[]):
        for relative,expected in evidence.get('files',{}).items():
            path=ROOT/relative
            if not path.is_file(): errors.append(f'missing evidence file: {relative}'); continue
            checked+=1
            if sha(path)!=expected: errors.append(f'hash mismatch: {relative}')
        for relative,expected in evidence.get('runs',{}).items():
            path=ROOT/relative
            if not path.is_file(): errors.append(f'missing acceptance: {relative}'); continue
            checked+=1
            if sha(path)!=expected: errors.append(f'hash mismatch: {relative}')
        result=evidence.get('result',{})
        if 'suite_passed' in result:
            if result['suite_passed'] is not True: errors.append(f"{evidence['id']}: suite not passed")
            if result.get('runs_passed')!=result.get('runs_total'):
                errors.append(f"{evidence['id']}: not all accepted runs passed")
            if result.get('all_runs_passed') is False:
                errors.append(f"{evidence['id']}: all_runs_passed=false")
        if 'passed' in result and result['passed'] is not True:
            errors.append(f"{evidence['id']}: acceptance not passed")
        if 'status' in result and result['status']!='completed':
            errors.append(f"{evidence['id']}: replay not completed")
    protected=index.get('protected_code',{})
    if protected.get('all_unchanged') is not True: errors.append('protected_code.all_unchanged is not true')
    for name,data in protected.get('lists',{}).items():
        path=ROOT/'artifacts'/name
        if not path.is_file(): errors.append(f'missing protected list: {name}'); continue
        checked+=1
        if sha(path)!=data.get('sha256'): errors.append(f'protected list hash mismatch: {name}')
        listing=json.loads(path.read_text(encoding='utf-8'))
        if len(listing)!=data.get('entries'): errors.append(f'protected list count changed: {name}')
        for relative,expected in listing.items():
            path=ROOT/relative.replace('\\','/')
            checked+=1
            if not path.is_file(): errors.append(f'missing protected file: {relative}')
            elif sha(path)!=expected: errors.append(f'protected file changed: {relative}')
    for required in ('C1','C2','C3','C4','C5','C6'):
        if required not in index.get('c1_c6',{}): errors.append(f'missing status {required}')
    report={'passed':not errors,'milestone':milestone.get('id'),'checked_files':checked,
            'evidence_entries':len(index.get('evidence',[])),'errors':errors}
    print(json.dumps(report,indent=2,ensure_ascii=False))
    return 0 if not errors else 1


if __name__=='__main__':
    sys.exit(main())
