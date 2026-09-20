"""Deterministic, censored open-space measurements; no Navigator/terminal policy.

Run pilot, inspect its audit, then full/stopping/refinement in NEW directories.
The exact-valued JSON Schema is validated without an additional dependency.
"""
import argparse
from collections import Counter
from dataclasses import asdict, fields, replace
import gzip
import hashlib
import json
from math import atan2, ceil, cos, hypot, pi, radians, sin
from pathlib import Path as FilePath
import platform
import subprocess
import zipfile

import numpy as np
from core import Path, Pose, Twist, VehicleState
from controller import BodyGuidance3DOF, CascadedPID3DOFController, ControlLimits3DOF
from guidance import LOSConfig, PathGeometry, ProgressState, ProjectionConfig, compute_los
from physics import Python3DOFBackend
from physics.frames import body_to_world_velocity_matrix
from simulation.controller_config import load_baseline_controller_parameters
from simulation.vehicle_config import load_synthetic_parameters

ROOT = FilePath(__file__).resolve().parents[1]
CONFIG = ROOT / 'config/envelope_measurement_v1.json'
SCHEMA = ROOT / 'config/envelope_measurement.schema.json'


def encode(value):
    """Canonical finite JSON, also used for deterministic comparisons."""
    def scalar(obj):
        if isinstance(obj, np.generic): return obj.item()
        raise TypeError(f'unsupported JSON type: {type(obj)}')
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False, default=scalar).encode('utf-8')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_config():
    """Validate the closed exact-value subset used by the checked-in schema."""
    cfg = json.loads(CONFIG.read_text(encoding='utf-8'))
    schema = json.loads(SCHEMA.read_text(encoding='utf-8'))
    validate_config(cfg, schema)
    return cfg


def validate_config(cfg, schema):
    """Reject missing/unknown keys, bool-as-number, nonfinite or changed values."""
    if not isinstance(cfg, dict) or set(cfg) != set(schema['required']):
        raise ValueError('configuration keys do not match frozen schema')
    for key, rule in schema['properties'].items():
        if encode(cfg[key]) != encode(rule['const']):
            raise ValueError(f'{key} differs from frozen v1 schema')


def sources():
    """Full runtime/config dependency surface, not just the runner."""
    paths = [SCHEMA, CONFIG]
    for folder in ('core', 'controller', 'physics', 'guidance', 'simulation', 'environment', 'planner', 'navigation', 'trajectory', 'validation'):
        paths.extend((ROOT / folder).rglob('*.py'))
    paths.extend((ROOT / 'config/controllers').glob('*.yaml'))
    paths.extend((ROOT / 'config/vehicles').glob('*.yaml'))
    paths.extend((ROOT / 'tests').rglob('*.py'))
    cfg=load_config()
    for arm in ('common','los'):
        paths.append(ROOT/cfg['stop']['saved_evidence']/arm/(cfg['stop']['saved_entry']+'.json'))
    return {str(p.relative_to(ROOT)).replace('\\', '/'): sha(p) for p in sorted(set(paths))}


def wrap(angle):
    return (angle + pi) % (2 * pi) - pi


def stable(state, heading, cross, speed, band):
    return (abs(cross) <= band['cross_track'] and abs(wrap(state.pose.pitch-heading)) <= band['pitch']
            and abs(state.twist.q) <= band['q'] and abs(state.twist.u-speed) <= band['speed'])


def segment_distance(points, a, b):
    """Vectorized distance to a FINITE segment, including its endpoints."""
    d = np.asarray(b)-a
    mu = np.clip((points-a) @ d / (d @ d), 0., 1.)
    return np.linalg.norm(points-(a+mu[:, None]*d), axis=1)


def corner_fixture(case, cfg):
    """Cruising fixture, explicitly not a pre-aligned full navigation mission."""
    f = cfg['fixture']; angle = radians(case['angle'])
    path = Path((Pose(x=-f['incoming_length'], z=f['z_origin']), Pose(z=f['z_origin']),
                 Pose(x=f['outgoing_length']*cos(angle), z=f['z_origin']+f['outgoing_length']*sin(angle))))
    return path, VehicleState(path[0], Twist(u=case['speed']))


def cases(cfg, stage):
    if stage in ('pilot', 'full', 'refinement'):
        for ci, current in enumerate(cfg['currents']):
            if stage != 'full' and ci: continue
            for speed in cfg['speeds']:
                for angle in cfg['angles_deg']:
                    if stage == 'refinement' and (speed not in cfg['diagnostics']['refinement_speeds'] or angle not in cfg['diagnostics']['refinement_angles_deg']): continue
                    for sign in cfg['signs']:
                        yield dict(id=f'turn_c{ci}_v{speed}_a{sign*angle}', kind='turn', current=current, speed=speed, angle=sign*angle)
    else:
        perturbations = [('base', 0., 0., 0.)]
        for field in ('pitch_offsets_deg', 'q_offsets', 'w_offsets'):
            for value in cfg['stop'][field]:
                if value:
                    perturbations.append((f'{field}_{value}', radians(value) if field == 'pitch_offsets_deg' else 0.,
                                          value if field == 'q_offsets' else 0., value if field == 'w_offsets' else 0.))
        for ci, current in enumerate(cfg['currents']):
            for speed in cfg['speeds']:
                for label, pitch, q, w in perturbations:
                    yield dict(id=f'stop_c{ci}_v{speed}_{label}', kind='stop', current=current, speed=speed, pitch_offset=pitch, q_offset=q, w_offset=w)
        yield dict(id='stop_saved_detour', kind='saved_stop')


def make_controller(cfg):
    return CascadedPID3DOFController(load_baseline_controller_parameters(ROOT / cfg['controller']))


def boundary_margin(state, cfg):
    a, b, c, d = cfg['fixture']['boundary']; p = state.pose
    return min(p.x-a, b-p.x, p.z-c, d-p.z)-cfg['fixture']['required_clearance']


def world_velocity(state):
    return (body_to_world_velocity_matrix(state.pose.pitch)[:2, :2] @ np.array([state.twist.u, state.twist.w])).tolist()


def execute(case, cfg, dt=None):
    """Fresh per-repeat state. Hold criteria require qualifying interval endpoints."""
    dt = cfg['dt'] if dt is None else dt
    controller = make_controller(cfg)
    physics = Python3DOFBackend(load_synthetic_parameters(ROOT/cfg['vehicle']))
    limits = ControlLimits3DOF(**cfg['limits'])
    current = np.asarray(case.get('current', [0., 0.]), dtype=float)
    warmup = []; states = []; steps = []; previous_pitch = None
    ready = False; held = 0.; prior_ok = False
    status = 'RIGHT_CENSORED'; reason = 'MEASUREMENT_BUDGET'
    entry_settled = False; entry_check_done = False; departure_start = None; departure = None
    recovery_start = None; recovery = None; departure_held = recovery_held = 0.
    f = cfg['fixture']; band = cfg['stable']; is_turn = case['kind'] == 'turn'
    if is_turn:
        path, state = corner_fixture(case, cfg)
        geometry = PathGeometry(path); progress = ProgressState(geometry, state.pose)
        angle = radians(case['angle']); tangent = np.array([cos(angle), sin(angle)])
        pc = ProjectionConfig(**cfg['projection'])
        lc = LOSConfig(cfg['lookahead'], case['speed'], cfg['terminal_entry_distance'])
        budget = cfg['turn_budget_seconds']
    elif case['kind'] == 'saved_stop':
        from validation.terminal_handoff_replays import load_entry, restore_controller
        common, history, entry, state = load_entry(ROOT/cfg['stop']['saved_evidence'], cfg['stop']['saved_entry'])
        controller = restore_controller(common, history, entry)
        current = np.asarray(common['current'], dtype=float)
        entry_settled = False
        budget = cfg['stop']['budget_seconds']
    else:
        state = VehicleState(Pose(z=f['z_origin']), Twist(u=case['speed']))
        # Fixed-time warmup: no result-dependent budget extension; memory is retained.
        for _ in range(round(cfg['stop']['warmup_seconds']/dt)):
            target = BodyGuidance3DOF(case['speed'], 0., 0.)
            output = controller.step_guidance(state, target, dt, limits)
            ok = stable(state, 0., 0., case['speed'], band)
            held = held+dt if ok and prior_ok else 0.; prior_ok = ok
            warmup.append(dict(state=asdict(state), target=asdict(target), control=asdict(output), diagnostics=asdict(controller.last_diagnostics)))
            state = physics.step(state, output, current, dt)
        entry_settled = held >= band['hold_seconds'] and stable(state, 0., 0., case['speed'], band)
        # Translation invariance: reset position/time only, retain velocity and PID memory.
        state = VehicleState(Pose(z=f['z_origin'], pitch=state.pose.pitch+case['pitch_offset']),
                             replace(state.twist, q=state.twist.q+case['q_offset'], w=state.twist.w+case['w_offset']))
        budget = cfg['stop']['budget_seconds']
    initial = state; initial_memory = asdict(controller.memory); start_time = state.timestamp
    held = 0.; prior_ok = False; states.append(asdict(state))
    for index in range(round(budget/dt)+1):
        elapsed = index*dt
        if is_turn:
            x = state.pose.x; z = state.pose.z-f['z_origin']
            out_s = float(np.array([x,z]) @ tangent); out_cross = -sin(angle)*x+cos(angle)*z
            incoming_ok = stable(state, 0., z, case['speed'], band)
            if not entry_check_done:
                if x < f['warmup_end_x']:
                    held = held+dt if incoming_ok and prior_ok else 0.; prior_ok = incoming_ok
                else:
                    entry_settled = held >= band['hold_seconds'] and incoming_ok
                    entry_check_done = True
            if entry_check_done:
                if not incoming_ok:
                    if departure_start is None: departure_start = -x; departure_held = 0.
                    else: departure_held += dt
                    if departure is None and departure_held+1e-9 >= band['hold_seconds']: departure = departure_start
                else: departure_start = None; departure_held = 0.
                out_ok = progress.segment_index == 1 and out_s >= 0 and stable(state, angle, out_cross, case['speed'], band)
                if out_ok:
                    if recovery_start is None: recovery_start = out_s; recovery_held = 0.
                    else: recovery_held += dt
                    if recovery is None and recovery_held+1e-9 >= band['hold_seconds']: recovery = recovery_start
                else: recovery_start = None; recovery_held = 0.
                if progress.segment_index == 1 and out_s >= f['measurement_outgoing']:
                    status = 'MEASUREMENT_COMPLETE'; reason = None; break
            command = compute_los(state.pose, progress, pc, lc, previous_pitch)
            if command.terminal_ready:
                status = 'RIGHT_CENSORED'; reason = 'TERMINAL_GUARD'; break
            progress = command.projection.next_state; previous_pitch = command.pitch_ref
            target = BodyGuidance3DOF(command.surge_speed_ref, command.pitch_ref, command.pitch_rate_ref)
            projection = {item.name: (asdict(getattr(command.projection,item.name)) if item.name=='projection_point' else getattr(command.projection,item.name))
                          for item in fields(command.projection) if item.name!='next_state'}
        else:
            ok = hypot(state.twist.u, state.twist.w) <= cfg['stop']['ground_speed'] and abs(state.twist.q) <= cfg['stop']['q']
            held = held+dt if ok and prior_ok else 0.; prior_ok = ok
            if held+1e-9 >= cfg['stop']['hold_seconds']:
                status = 'MEASUREMENT_COMPLETE'; reason = None; break
            target = BodyGuidance3DOF(-float(np.clip(state.twist.u, -cfg['stop']['reverse_target'], cfg['stop']['reverse_target'])), initial.pose.pitch, 0.)
            projection = None
        if index == round(budget/dt): break
        output = controller.step_guidance(state, target, dt, limits)
        nxt = physics.step(state, output, current, dt)
        # Convex rectangle: endpoint minima also bound the complete linear step.
        margin = min(boundary_margin(state, cfg), boundary_margin(nxt, cfg))
        steps.append(dict(time=state.timestamp, elapsed=elapsed, phase='TURN_MEASUREMENT' if is_turn else 'STOP_MEASUREMENT',
                          target=asdict(target), applied=asdict(output), diagnostics=asdict(controller.last_diagnostics),
                          projection=projection, world_velocity=world_velocity(state), boundary_margin=margin))
        states.append(asdict(nxt)); state = nxt
        if margin < 0:
            status = 'RIGHT_CENSORED'; reason = 'OUT_OF_BOUNDS'; break
    report = dict(case=case, dt=dt, status=status, censor_reason=reason, entry_settled=entry_settled,
                  initial_controller_memory=initial_memory, final_controller_memory=asdict(controller.memory),
                  elapsed=state.timestamp-start_time, current=current.tolist(), d_entry=departure, d_exit=recovery,
                  entry_event_reason=None if departure is not None else 'NOT_OBSERVED',
                  exit_event_reason=None if recovery is not None else 'RIGHT_CENSORED',
                  terminal_entered=False, path=[asdict(p) for p in path] if is_turn else None)
    report.update(measure(states, steps, case, cfg, dt))
    if not is_turn:
        report['stopping_distance'] = report['maximum_forward_displacement'] if status=='MEASUREMENT_COMPLETE' else None
        report['stopping_distance_reason'] = None if status=='MEASUREMENT_COMPLETE' else 'RIGHT_CENSORED_NOT_A_STOPPING_BOUND'
    report['eligible_for_settled_entry_envelope'] = entry_settled and status == 'MEASUREMENT_COMPLETE'
    return dict(summary=report, states=states, steps=steps, warmup=warmup)


def measure(states, steps, case, cfg, dt):
    """Offline diagnostics; neither thresholds nor outcomes feed back to control."""
    p = np.array([[s['pose']['x'], s['pose']['z']] for s in states])
    q = np.array([s['twist']['q'] for s in states])
    velocities = np.array([world_velocity(VehicleState(Pose(**s['pose']), Twist(**s['twist']), s['timestamp'])) for s in states])
    result = dict(max_abs_q=float(np.max(abs(q))), rate_exceedance_seconds=float(np.sum(abs(q[:-1]) > cfg['diagnostics']['rate_line'])*dt),
                  minimum_boundary_margin=min((s['boundary_margin'] for s in steps), default=None),
                  moment_saturation_fraction=sum(abs(s['applied']['tau_m']) >= cfg['limits']['maximum_tau_m']-1e-10 for s in steps)/max(1,len(steps)),
                  control_effort_proxy=sum(s['applied']['tau_x']**2+s['applied']['tau_m']**2 for s in steps)*dt,
                  maximum_pitch_reference_step=max((abs(wrap(b['target']['pitch']-a['target']['pitch'])) for a,b in zip(steps,steps[1:])), default=0.))
    if case['kind'] != 'turn':
        heading = states[0]['pose']['pitch']; d=p-p[0]; forward=d @ np.array([cos(heading),sin(heading)])
        result.update(maximum_forward_displacement=float(max(forward)), maximum_lateral_displacement=float(max(abs(d @ np.array([-sin(heading),cos(heading)])))),
                      maximum_displacement=float(max(np.linalg.norm(d,axis=1))), travelled_length=float(sum(np.linalg.norm(np.diff(p,axis=0),axis=1))),
                      final_ground_speed=float(np.linalg.norm(velocities[-1])))
        return result
    path,_ = corner_fixture(case,cfg); a,b,c = [np.array([v.x,v.z]) for v in path]
    deviations = np.minimum(segment_distance(p,a,b),segment_distance(p,b,c))
    # Dense sweep + Lipschitz error bound, not an unsupported exact segment claim.
    spacing=cfg['diagnostics']['sweep_sample_spacing']; sweep_max=0.; largest_gap=0.
    for left,right in zip(p,p[1:]):
        length=float(np.linalg.norm(right-left)); count=max(1,ceil(length/spacing))
        samples=left+np.linspace(0.,1.,count+1)[:,None]*(right-left)
        sweep_max=max(sweep_max,float(max(np.minimum(segment_distance(samples,a,b),segment_distance(samples,b,c)))))
        largest_gap=max(largest_gap,length/count)
    upper=sweep_max+largest_gap/2
    accel=np.gradient(velocities,dt,axis=0) if len(velocities)>1 else np.zeros_like(velocities)
    speed=np.linalg.norm(velocities,axis=1); curvature=np.zeros(len(speed))
    valid=speed>cfg['diagnostics']['curvature_min_speed']; valid[[0,-1]]=False
    curvature[valid]=abs(velocities[valid,0]*accel[valid,1]-velocities[valid,1]*accel[valid,0])/speed[valid]**3
    curved=valid & (curvature>cfg['diagnostics']['curvature_min_abs']); radii=1/curvature[curved]
    projections=[s['projection'] for s in steps]
    result.update(path_deviation_max=float(max(deviations)), path_deviation_rms=float(np.sqrt(np.mean(deviations**2))),
                  swept_deviation_lower=sweep_max, swept_deviation_upper=upper,
                  corridor_results=[dict(extra_margin=m, verdict='PASS_BOUND' if upper<=m else 'FAIL_WITNESS' if sweep_max>m else 'UNRESOLVED') for m in cfg['diagnostics']['corridor_margins']],
                  radius_min=float(min(radii)) if len(radii) else None,
                  radius_quantiles=np.quantile(radii,[.1,.5,.9]).tolist() if len(radii) else None,
                  radius_reason=None if len(radii) else 'NO_VALID_CURVED_SAMPLES', radius_valid_fraction=float(np.mean(curved)),
                  segment_switch_count=sum(s['switch_count'] for s in projections),
                  progress_clamp_count=sum(s['progress_clamped'] for s in projections),
                  switch_limit_count=sum(s['switch_limit_reached'] for s in projections),
                  progress_monotonicity_violations=sum(b['committed_s']<a['committed_s'] for a,b in zip(projections,projections[1:])))
    return result


def audit(result, cfg):
    """Structural acceptance, not a success-rate gate or physical-safety claim."""
    states=result['states']; steps=result['steps']; summary=result['summary']; dt=summary['dt']
    return dict(history_alignment=len(states)==len(steps)+1,
                fixed_clock=all(abs(b['timestamp']-a['timestamp']-dt)<1e-9 for a,b in zip(states,states[1:])),
                limits=all(abs(s['applied']['tau_x'])<=cfg['limits']['maximum_tau_x'] and abs(s['applied']['tau_m'])<=cfg['limits']['maximum_tau_m'] for s in steps),
                no_terminal=not summary['terminal_entered'] and all(s['phase'] in ('TURN_MEASUREMENT','STOP_MEASUREMENT') for s in steps),
                monotonic=summary.get('progress_monotonicity_violations',0)==0,
                finite_json=bool(encode(result)))


def write_json(path, obj):
    path.write_bytes(encode(obj))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage',choices=('pilot','full','stopping','refinement'),required=True)
    parser.add_argument('--output',type=FilePath,required=True)
    parser.add_argument('--reviewed-pilot',type=FilePath)
    args=parser.parse_args(); cfg=load_config(); before=sources()
    if args.stage!='pilot':
        if args.reviewed_pilot is None: parser.error('inspect pilot audit, then pass --reviewed-pilot')
        pilot=json.loads((args.reviewed_pilot/'summary.json').read_text(encoding='utf-8'))
        manifest=json.loads((args.reviewed_pilot/'manifest.json').read_text(encoding='utf-8'))
        if pilot['stage']!='pilot' or len(pilot['cases'])!=30 or not pilot['audit_passed'] or manifest['source_sha256']!=before:
            raise ValueError('pilot missing, unaudited or source version differs')
        if any(sha(args.reviewed_pilot/name)!=checksum for name,checksum in manifest['artifact_sha256'].items()):
            raise ValueError('pilot evidence checksum mismatch')
    args.output.mkdir(parents=True,exist_ok=False)
    write_json(args.output/'config.json',cfg); (args.output/'schema.json').write_bytes(SCHEMA.read_bytes())
    write_json(args.output/'source_manifest.json',before)
    with zipfile.ZipFile(args.output/'source_snapshot.zip','w',compression=zipfile.ZIP_DEFLATED) as archive:
        for name in before: archive.write(ROOT/name,name)
    summaries=[]; checks=[]
    for case in cases(cfg,args.stage):
        first=None; checksum=None
        for repeat in range(cfg['repeats']):
            result=execute(case,cfg,cfg['dt']/2 if args.stage=='refinement' else None)
            check=audit(result,cfg); checks.append(check)
            payload=encode(result); current_hash=hashlib.sha256(payload).hexdigest()
            (args.output/f"{case['id']}_r{repeat}.json.gz").write_bytes(gzip.compress(payload,compresslevel=1,mtime=0))
            if repeat==0: first=result['summary']; checksum=current_hash
            elif checksum!=current_hash: raise AssertionError('repeat mismatch; evidence preserved')
        first['deterministic']=True; summaries.append(first)
        print(f"{args.stage} {len(summaries)} {case['id']} {first['status']} settled={first['entry_settled']}",flush=True)
    if sources()!=before: raise RuntimeError('source changed during measurement')
    summary=dict(stage=args.stage,cases=summaries,status_counts=dict(Counter(s['status'] for s in summaries)),
                 eligible_count=sum(s['eligible_for_settled_entry_envelope'] for s in summaries),
                 audit_passed=all(all(c.values()) for c in checks),repeat_count=cfg['repeats'])
    write_json(args.output/'summary.json',summary); write_json(args.output/'audit.json',checks)
    git=subprocess.run(['git','rev-parse','HEAD'],cwd=ROOT,capture_output=True,text=True,encoding='utf-8',errors='replace')
    manifest=dict(source_sha256=before,git_commit=git.stdout.strip() if git.returncode==0 else None,
                  git_commit_note='Source hashes are authoritative; working tree may differ from HEAD.',
                  python=platform.python_version(),numpy=np.__version__,
                  artifact_sha256={p.name:sha(p) for p in sorted(args.output.iterdir()) if p.is_file()})
    write_json(args.output/'manifest.json',manifest)
    if not summary['audit_passed']: raise AssertionError('audit failed; do not proceed')


if __name__=='__main__':
    main()
