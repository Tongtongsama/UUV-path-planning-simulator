"""Audit existing records, then screen at most twelve uniform configurations.

Reuses the historical runner without editing it. No real-backend entry point.
"""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import platform
from importlib.metadata import version

import numpy as np
from shapely.geometry import LineString, Point

from core import Pose
from physics.integrators import RK4Integrator
from physics.uuv_3dof import Python3DOFDynamics
from simulation.vehicle_config import load_synthetic_parameters
from ros2.twin_axial_pid.control import AllocatedPID
from ros2.twin_axial_navigation.acceptance import numerically_equal
from ros2.twin_axial_navigation.mission import INITIALS, environment_from_world, segment_clearance
from ros2.twin_axial_navigation.current_model import FLOWS
from ros2.twin_axial_feasible_matrix.scenes import SCENES, world_xml
from ros2.twin_axial_feasible_matrix.guidance import build_path
from ros2.twin_axial_s3_transient import study

ROOT = Path(__file__).resolve().parents[2]
FIXED = {key: study.BASE_CONFIG[key] for key in study.FIXED}
GROUPS = study.GROUPS  # Twelve preregistered combinations, never flow-specific.


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write(path, data):
    def scalar(value):
        if isinstance(value, np.generic):
            return value.item()
        raise TypeError(type(value).__name__)
    Path(path).write_text(json.dumps(data, indent=2, ensure_ascii=False, default=scalar), encoding='utf-8')


def verify_manifest(folder):
    folder = Path(folder).resolve()
    manifest = read(folder / 'manifest.json')
    assert {'plan.json', 'report.json', 'states.jsonl'} <= manifest.keys()
    for name, digest in manifest.items():
        path = (folder / name).resolve()
        assert path.is_relative_to(folder) and sha(path) == digest, str(path)


@contextmanager
def original_rates():
    """Only this isolated Python process changes the runner's study constants."""
    previous = study.FIXED
    study.FIXED = FIXED
    try:
        yield
    finally:
        study.FIXED = previous


def verify_run(folder, replay=False):
    """Recompute metrics from raw states; optionally replay every RK4/PID step."""
    folder = Path(folder)
    verify_manifest(folder)
    plan, report = read(folder / 'plan.json'), read(folder / 'report.json')
    scene, flow, case = plan['scene'], plan['flow'], plan['case']
    config = plan['config']
    rows = [json.loads(line) for line in (folder / 'states.jsonl').read_text().splitlines()]
    assert rows
    x, z, angle = INITIALS[case]
    points, geometry = build_path(SCENES[scene], (x, z))
    assert np.array_equal(points, plan['path'])
    assert min(geometry.values()) >= 1.5 - 1e-9
    path = LineString(points)
    world = environment_from_world(world_xml(case, scene))
    current = np.array(FLOWS[flow])
    prior = np.array([x, z, math.radians(angle), 0., 0., 0.])
    force = np.zeros(3)
    rk, dyn = RK4Integrator(), Python3DOFDynamics(load_synthetic_parameters())
    pid = AllocatedPID(math.radians(70))
    previous_pose = Pose(x=x, z=z)
    hold = arrival = None
    errors, clearances, saturated, pitch, q, speeds = [], [], [], [], [], []
    first = {}
    interval = []
    previous_target = np.zeros(2)
    max_target_rates = np.zeros(2)
    max_state_residual = 0.
    with study.patched_config(config):
        guide = study.base_guidance.Guide(points, current)
        for index, row in enumerate(rows):
            state = np.array(row['state']); t = row['t']
            assert np.isfinite(state).all() and abs(t - (index + 1) * .05) < 1e-8
            if replay:
                expected = rk.step(lambda s: dyn.derivatives(s, force, current), prior, .05)
                max_state_residual = max(max_state_residual, float(np.max(np.abs(expected-state))))
                assert np.allclose(expected, state, atol=1e-9, rtol=1e-10), (folder, index, 'dynamics')
            assert np.allclose(row['applied_force'], force, atol=1e-12, rtol=0)
            prior = state
            pos = Pose(x=state[0], z=state[1])
            clearance = segment_clearance(world, previous_pose, pos)
            previous_pose = pos
            error = path.distance(Point(*state[:2]))
            assert abs(clearance-row['clearance']) < 1e-9
            assert abs(error-row['nav']['path_error']) < 1e-9
            if math.dist(state[:2], points[-1]) <= .5:
                hold = t if hold is None else hold
                if arrival is None and t-hold >= 2-1e-9:
                    arrival = t
            else:
                hold = None
            theta, u, w, rate = state[2:]
            envelope = abs(theta) <= math.radians(75) and max(abs(u), abs(w)) <= 1.5 and abs(rate) <= .8
            for name, violation in [('safety', clearance < .9-1e-6), ('envelope', not envelope), ('max_error', error > .8)]:
                if violation and name not in first:
                    first[name] = dict(t=t, position=state[:2].tolist(), clearance=clearance, path_error=error)
            if replay:
                target, nav = guide.update(state, t)
                record = dict(valid=True, sequence=index, elapsed_end=t, pitch_nose_up_rad=theta,
                              solver_state_ned_frd=dict(t=t, y=state[0], z=20-state[1], u=u, w=w, q=rate))
                _, control = pid.command(record, target, .05)
                assert numerically_equal(nav, row['nav']), (folder, index, 'guidance')
                assert numerically_equal(control, row['control']), (folder, index, 'PID')
            allocation = row['control']['allocation']
            from ros2.twin_axial_v2.allocation import allocate
            assert numerically_equal(allocation, allocate(row['control']['requested_wrench']))
            wrench = allocation['allocated_wrench']
            force = np.array([wrench['X'], wrench['Z'], wrench['M']])
            assert force[1] == 0 and max(map(abs, allocation['allocated_commands'].values())) <= 15+1e-9
            target = np.array([row['control']['target']['u'], row['control']['target']['pitch']])
            max_target_rates = np.maximum(max_target_rates, np.abs(target-previous_target)/.05)
            assert 0 <= target[0] <= .6+1e-12 and abs(target[1]) <= math.radians(70)+1e-12
            previous_target = target
            errors.append(error); clearances.append(clearance); saturated.append(allocation['alpha'] < .999)
            pitch.append(abs(math.degrees(theta))); q.append(abs(rate)); speeds.append(max(abs(u), abs(w)))
            vx, vz = u*math.cos(theta)+w*math.sin(theta), u*math.sin(theta)-w*math.cos(theta)
            if 7 <= state[0] <= 13 and vz < 0 and math.hypot(vx, vz) >= .05:
                cw = current[0]*math.sin(theta)-current[1]*math.cos(theta)
                interval.append(dict(t=t, theta_deg=math.degrees(theta), course_deg=math.degrees(math.atan2(vz,vx)),
                                     u=u, w=w, residual_w=w-cw, q=rate, vx=vx, vz=vz))
    metrics = dict(arrival_time=arrival, min_clearance=min(clearances), rms_error=float(np.sqrt(np.mean(np.square(errors)))),
                   max_error=max(errors), saturation_fraction=float(np.mean(saturated)))
    for key, value in metrics.items():
        assert numerically_equal(value, report['metrics'][key]), (folder, key, value, report['metrics'][key])
    assert max_target_rates[0] <= config['speed_rate_limit']+1e-10
    assert max_target_rates[1] <= config['pitch_rate_limit']+1e-10
    checks = study.case_checks(metrics, arrival is not None, 'envelope' if 'envelope' in first else None)
    # Recompute envelope independently, rather than trusting the saved error string.
    checks['envelope'] = max(pitch) <= 75 and max(q) <= .8 and max(speeds) <= 1.5
    assert checks == report['checks'] and all(checks.values()) == report['passed'], (folder, 'checks')
    detail = None
    if interval:
        steep = min(interval, key=lambda v:v['course_deg'])
        detail = dict(samples=len(interval), sampled_duration_s=len(interval)*.05,
                      deepest_course_sample=steep,
                      seconds_beyond_72_412=sum(v['course_deg'] < -72.412046226 for v in interval)*.05,
                      residual_w_range=[min(v['residual_w'] for v in interval),max(v['residual_w'] for v in interval)])
    return dict(record_verified=True, dynamics_pid_replayed=replay, passed=all(checks.values()), checks=checks,
                metrics=metrics, max_abs_pitch_deg=max(pitch), max_abs_q=max(q), first_violations=first,
                max_reference_rates=max_target_rates.tolist(), descent_window=detail,
                max_one_step_state_residual=max_state_residual if replay else None,
                manifest_sha256=sha(folder/'manifest.json'), steps=len(rows))


def snapshot(output):
    study.source_snapshot(output)
    data = read(output/'source.json')
    for source in Path(__file__).parent.glob('*.py'):
        name = source.relative_to(ROOT).as_posix()
        target = output/'source'/name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(source.read_bytes())
        data['files'][name] = sha(source)
    data['count'] = len(data['files'])
    data['aggregate_sha256'] = hashlib.sha256(''.join(data['files'][k] for k in sorted(data['files'])).encode()).hexdigest()
    write(output/'source.json', data)
    return data['aggregate_sha256']


def audit(output):
    baseline = ROOT/'artifacts/twin_axial_feasible_matrix/python_20261106_02'
    history = ROOT/'artifacts/twin_axial_s3_transient/screen_20261106_02'
    source = read(history/'source.json')
    for name, digest in source['files'].items():
        assert sha(history/'source'/name) == digest and sha(ROOT/name) == digest, name
    result = dict(baseline={}, historical_screen={}, scope='Raw-state audit; historical screen additionally replays RK4/PID')
    for scene, flows in [('S2', FLOWS), ('S3', ['F1'])]:
        for flow in flows:
            for case in study.AUDIT_CASES:
                key = f'{scene}/{flow}/{case}'
                result['baseline'][key] = verify_run(baseline/key)
                write(output/'audit_progress.json', result)
                print('AUDIT', key, flush=True)
    for group in GROUPS:
        for flow in FLOWS:
            key = f"{group['name']}/{flow}"
            result['historical_screen'][key] = verify_run(history/key, replay=True)
            write(output/'audit_progress.json', result)
            print('REPLAY', key, result['historical_screen'][key]['passed'], flush=True)
    result['historical_fixed'] = study.FIXED
    result['required_fixed_for_new_screen'] = FIXED
    result['note'] = 'Historical 0.20 rad/s pitch slew is not evidence under the original 0.12 limit. Overshoot correlation alone is not causation.'
    write(output/'audit.json', result)


def screen(output, audit_path):
    prior_audit = read(audit_path)
    assert len(prior_audit['baseline']) == 24 and len(prior_audit['historical_screen']) == 36
    assert all(v['record_verified'] for section in ('baseline','historical_screen') for v in prior_audit[section].values())
    digest = snapshot(output)
    write(output/'environment.json', dict(python=platform.python_version(), platform=platform.platform(),
          packages={name:version(name) for name in ('numpy','shapely','PyYAML')}))
    write(output/'preregistration.json', dict(created_utc=datetime.now(timezone.utc).isoformat(), groups=GROUPS,
          fixed=FIXED, base_config=study.BASE_CONFIG, flows=list(FLOWS), case='P0', scene='S3', maximum_runs=36,
          source_sha256=digest, audit_sha256=sha(audit_path), thresholds=study.THRESHOLDS,
          stop='No parameter extension after these 12 groups; no full matrix without a uniform passing candidate.',
          s3_remains_nominal=True, real_started=False))
    reports = {}
    with original_rates():
        for group in GROUPS:
            for flow in FLOWS:
                key = f"{group['name']}/{flow}"
                try:
                    study._run(output/key, group, flow, digest)
                    reports[key] = verify_run(output/key, replay=True)
                except Exception as exc:
                    write(output/'interruption.json', dict(key=key, error=repr(exc), completed=list(reports)))
                    raise
                write(output/'verified_progress.json', reports)
    unified = [g['name'] for g in GROUPS if all(reports[f"{g['name']}/{f}"]['passed'] for f in FLOWS)]
    summary = dict(runs=reports, flow_pass_counts={f:sum(reports[f"{g['name']}/{f}"]['passed'] for g in GROUPS) for f in FLOWS},
                   unified_passes=unified, worth_full_matrix=bool(unified), s3_remains_nominal=True, real_started=False,
                   scope='Python S3/P0 screen only; no five-initial or repeatability claim; no impossibility proof')
    write(output/'summary.json', summary)
    print(json.dumps({k:v for k,v in summary.items() if k!='runs'}), flush=True)


def analyze(source, output):
    """Independent read-only replay; verification outputs go in a new directory."""
    source = Path(source)
    snapshot_data = read(source/'source.json')
    for name, digest in snapshot_data['files'].items():
        assert sha(source/'source'/name) == digest and sha(ROOT/name) == digest, name
    saved = read(source/'summary.json')
    keys = {f"{g['name']}/{f}" for g in GROUPS for f in FLOWS}
    assert set(saved['runs']) == keys
    checked = {}
    with original_rates():
        for group in GROUPS:
            for flow in FLOWS:
                key = f"{group['name']}/{flow}"
                assert read(source/key/'plan.json')['config'] == study.config_for_group(group)
                checked[key] = verify_run(source/key, replay=True)
                assert numerically_equal(checked[key], saved['runs'][key]), key
    write(output/'verification.json',dict(record_verification_passed=True,runs=checked,
          note='Verification success is not navigation success.'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['audit','screen','analyze'])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--audit', type=Path)
    parser.add_argument('--source', type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; records cannot be overwritten')
    if args.mode == 'screen' and args.audit is None:
        parser.error('--audit is required before screening')
    if args.mode == 'analyze' and args.source is None:
        parser.error('--source is required for analysis')
    args.output.mkdir(parents=True)
    if args.mode == 'audit':
        audit(args.output)
    elif args.mode == 'screen':
        screen(args.output,args.audit)
    else:
        analyze(args.source,args.output)


if __name__ == '__main__':
    main()
