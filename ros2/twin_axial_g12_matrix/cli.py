"""Uniform 54-run G12 Python campaign, strict repeat checks and read-only gate."""
import argparse
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
import math
from pathlib import Path
import platform

from ros2.twin_axial_s3_transient_v2.cli import ROOT, read, write, sha, snapshot, verify_run
from ros2.twin_axial_s3_transient_v2.validate import g12_config
from ros2.twin_axial_s3_transient import study
from ros2.twin_axial_feasible_matrix.scenes import SCENES
from ros2.twin_axial_navigation.current_model import FLOWS
from .runner import run_case

CASES = ('P0','P1','P2','P3','P4','P0_repeat')
KEYS = tuple(f'{scene}/{flow}/{case}' for scene in SCENES for flow in FLOWS for case in CASES)


def rows(folder):
    return [json.loads(line) for line in (Path(folder)/'states.jsonl').read_text().splitlines() if line]


def repeatability(a, b, ra, rb):
    """R2 plus both trials' complete individual acceptance, never position alone."""
    def index(records):
        result = {}
        for i, row in enumerate(records):
            t = row['t']
            if not math.isfinite(t) or abs(t-(i+1)*.05)>1e-6 or t in result:
                raise ValueError('Invalid repeat simulation time')
            result[round(t,6)] = row
        return result
    aa, bb = index(a), index(b)
    shared = sorted(aa.keys() & bb.keys())
    delta = max((math.dist(aa[t]['state'][:2],bb[t]['state'][:2]) for t in shared), default=None)
    terminal = abs(a[-1]['t']-b[-1]['t']) if a and b else None
    ar, br = ra.get('metrics',{}).get('arrival_time'), rb.get('metrics',{}).get('arrival_time')
    arrival = abs(ar-br) if ar is not None and br is not None else None
    checks = dict(reference_passed=bool(ra.get('passed')), repeat_run_passed=bool(rb.get('passed')),
                  reference_verified=bool(ra.get('record_verified')), repeat_verified=bool(rb.get('record_verified')),
                  matched_time_coverage=bool(shared) and len(shared)>=min(len(a),len(b))-1,
                  position=delta is not None and math.isfinite(delta) and delta<=.02,
                  arrival_time=arrival is not None and math.isfinite(arrival) and arrival<=.10+1e-9,
                  terminal_time=terminal is not None and terminal<=.10+1e-9)
    return dict(passed=all(checks.values()), checks=checks, matched_steps=len(shared),
                max_position_difference_m=delta, arrival_time_difference_s=arrival,
                terminal_time_difference_s=terminal)


def freeze(output):
    snapshot(output)
    data = read(output/'source.json')
    # Include the new adapter and geometry/planning dependencies used by verification.
    for folder in ('ros2/twin_axial_g12_matrix','environment','planner','guidance','navigation'):
        for source in (ROOT/folder).rglob('*.py'):
            if '__pycache__' in source.parts:
                continue
            name = source.relative_to(ROOT).as_posix()
            target = output/'source'/name
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes(source.read_bytes())
            data['files'][name] = sha(source)
    data['count'] = len(data['files'])
    data['aggregate_sha256'] = hashlib.sha256(json.dumps(data['files'],sort_keys=True).encode()).hexdigest()
    write(output/'source.json',data)
    return data


def check_source(directory):
    data = read(directory/'source.json')
    digest = hashlib.sha256(json.dumps(data['files'],sort_keys=True).encode()).hexdigest()
    if digest != data['aggregate_sha256']:
        raise ValueError('Source aggregate mismatch')
    for name, expected in data['files'].items():
        saved = (directory/'source'/name).resolve()
        if not saved.is_relative_to((directory/'source').resolve()):
            raise ValueError('Source path escape')
        if sha(saved)!=expected or sha(ROOT/name)!=expected:
            raise ValueError('Frozen source changed: '+name)
    return digest


def verify_entry(folder, key, digest):
    scene, flow, case = key.split('/')
    plan, report = read(folder/'plan.json'),read(folder/'report.json')
    initial = case.split('_')[0]
    for obj in (plan,report):
        if (obj['scene'],obj['flow'],obj['case'],obj['run_case'])!=(scene,flow,initial,case):
            raise ValueError('Run identity mismatch: '+key)
    if plan['config']!=g12_config() or report['parameters']!=g12_config():
        raise ValueError('G12 configuration changed')
    if report['source_sha256']!=digest or read(folder/'source_ref.json')['aggregate_sha256']!=digest:
        raise ValueError('Trial source reference mismatch')
    result = verify_run(folder,replay=True)
    if result['passed'] and abs(rows(folder)[-1]['t']-result['metrics']['arrival_time'])>1e-8:
        raise ValueError('Unexpected successful terminal time')
    return result


def summarize(directory, verified):
    groups = {}
    for scene in SCENES:
        for flow in FLOWS:
            prefix = f'{scene}/{flow}'
            rr = {case:verified[f'{prefix}/{case}'] for case in CASES}
            try:
                rep = repeatability(rows(directory/prefix/'P0'),rows(directory/prefix/'P0_repeat'),rr['P0'],rr['P0_repeat'])
            except Exception as exc:
                rep = dict(passed=False,error=repr(exc))
            groups[prefix] = dict(passed=all(r.get('passed',False) for r in rr.values()) and rep['passed'],
                successful_initials=sum(rr[c].get('passed',False) for c in CASES[:5]),
                repeat_run_passed=rr['P0_repeat'].get('passed',False),repeatability=rep,
                min_clearance=min((r['metrics']['min_clearance'] for r in rr.values() if 'metrics' in r),default=None),
                max_rms=max((r['metrics']['rms_error'] for r in rr.values() if 'metrics' in r),default=None),
                max_error=max((r['metrics']['max_error'] for r in rr.values() if 'metrics' in r),default=None))
    passed = len(verified)==54 and all(g['passed'] for g in groups.values()) and all(r.get('record_verified',False) for r in verified.values())
    return dict(passed=passed,all_54_passed=passed,runs_passed=sum(r.get('passed',False) for r in verified.values()),
                entries=len(verified),groups=groups,runs=verified,s3_remains_nominal=True,real_started=False,
                scope='54 fresh Python trials under one shared source snapshot; no real-backend validation')


def gate_report(summary, source_ok):
    prerequisite = bool(source_ok and summary.get('all_54_passed'))
    return dict(python_prerequisite_passed=prerequisite,source_matches_frozen=source_ok,
                real_started=False,real_backend_gate_satisfied=False,
                reason=('Python evidence prerequisite passed; a matching G12 real adapter and its gate have not been validated.'
                        if prerequisite else 'Full 54-run Python acceptance/source prerequisite failed; real backend remains blocked.'),
                existing_real_gate_modified=False)


def run(output):
    source = freeze(output)
    digest = check_source(output)
    config = g12_config()
    frozen_candidate = read(ROOT/'artifacts/twin_axial_s3_transient_v2/candidate_G12_20261107.json')
    if config!=frozen_candidate['configuration']:
        raise ValueError('Configuration differs from frozen G12')
    write(output/'environment.json',dict(python=platform.python_version(),platform=platform.platform(),
          packages={name:version(name) for name in ('numpy','shapely','PyYAML')}))
    write(output/'preregistration.json',dict(created_utc=datetime.now(timezone.utc).isoformat(),configuration=config,
          source_sha256=digest,entries=KEYS,scenes=SCENES,flows=FLOWS,initials=study.INITIALS,
          thresholds=study.THRESHOLDS,repeat_limits=dict(position_m=.02,arrival_s=.1,terminal_s=.1),
          all_runs_fresh=True,reused_prior_trials=0,maximum_trials=54,real_started=False,
          steady_certificate='Recorded diagnostic only; numerical dynamic acceptance unchanged',
          policy='Keep all failures, no retries or per-case tuning, all 54 must pass the Python prerequisite'))
    verified = {}
    for key in KEYS:
        check_source(output)
        folder = output/key
        try:
            run_case(folder,*key.split('/'),digest)
            write(folder/'source_ref.json',dict(aggregate_sha256=digest,shared_snapshot='../../../source'))
            manifest = read(folder/'manifest.json')
            manifest['source_ref.json'] = sha(folder/'source_ref.json')
            write(folder/'manifest.json',manifest)
            verified[key] = verify_entry(folder,key,digest)
        except Exception as exc:
            folder.mkdir(parents=True,exist_ok=True)
            write(folder/'execution_error.json',dict(error=repr(exc),key=key,source_sha256=digest))
            verified[key] = dict(passed=False,record_verified=False,error=repr(exc))
        check_source(output)
        write(output/'progress.json',verified)
    summary = summarize(output,verified)
    write(output/'matrix.json',summary)
    write(output/'gate.json',gate_report(summary,True))
    print(json.dumps(dict(entries=summary['entries'],runs_passed=summary['runs_passed'],passed=summary['passed'])),flush=True)
    return summary


def analyze(source,output):
    digest = check_source(source)
    if read(source/'preregistration.json')['source_sha256']!=digest:
        raise ValueError('Preregistration source mismatch')
    verified = {}
    for key in KEYS:
        try:
            verified[key] = verify_entry(source/key,key,digest)
        except Exception as exc:
            verified[key] = dict(passed=False,record_verified=False,error=repr(exc))
    result = summarize(source,verified)
    write(output/'matrix.json',result)
    write(output/'gate.json',gate_report(result,True))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=('run','analyze'))
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--source',type=Path)
    args=parser.parse_args()
    if args.output.exists():parser.error('Output exists; never overwrite records')
    if args.mode=='analyze' and args.source is None:parser.error('--source required')
    args.output.mkdir(parents=True)
    result=run(args.output) if args.mode=='run' else analyze(args.source,args.output)
    raise SystemExit(0 if result['passed'] else 2)


if __name__=='__main__':
    main()
