"""Preregistered finite P0/P1 comparison: geometry first, then paired closed loops."""
import argparse
from collections import Counter
from dataclasses import asdict, replace
import json
from math import atan2, pi
from pathlib import Path
import sys

from controller import CascadedPID3DOFController
from guidance import LOSConfig, ProjectionConfig
from navigation import Navigator, LOSNavigationAdapter, ControlledStartupPolicy, TerminalCapturePolicy
from physics import Python3DOFBackend
from planner import AStarConfig, EnvironmentPlanningSpace, PlanningRequest
from planner.candidates import CandidateConfig, CandidatePlanner, simplify_path, geometry_cost
from planner.validation import validate_path
from trajectory import generate_trajectory
from simulation.vehicle_config import load_synthetic_parameters
from simulation.controller_config import load_baseline_controller_parameters
from validation.guidance_matched_comparison import (ROOT, prepare, FrozenPlanner, common_metrics,
    failure_evidence, compact_control, digest, fingerprint, RUNTIME_FOLDERS)
from validation.frozen_candidate_regression import signed_rectangle_margin
from validation.navigation_acceptance import clean, check_execution


def write(path, data):
    path.write_text(json.dumps(clean(data), indent=2, allow_nan=False), encoding='utf-8')


def turn_events(result, path, lookahead, threshold):
    """Record pre-application lookahead-crossing and command-onset snapshots separately.

    These observations are NOT automatically the isolated experiment's entry.
    State history[i] and controller diagnostics contain the pre-step state/memory.
    """
    corners, arc = [], 0.
    for i, (a, b) in enumerate(zip(path, path.poses[1:])):
        arc += a.distance_to(b)
        if i < len(path)-2:
            corners.append((i, arc, atan2(b.z-a.z, b.x-a.x)))
    output = []
    for corner, arc, incoming in corners:
        eligible = [(i, s) for i, s in enumerate(result.control_history)
                    if s.guidance_command is not None and s.guidance_phase == 'PATH_FOLLOWING']
        first = next(((i, s) for i, s in eligible if s.path_progress_s+lookahead >= arc), None)
        onset = next(((i, s) for i, s in eligible if s.path_progress_s+lookahead >= arc and
                      abs((s.guidance_command.pitch_ref-incoming+pi) % (2*pi)-pi) > threshold), None)
        switch = next(((i, s) for i, s in eligible if s.guidance_command.segment_index > corner), None)
        def snapshot(event):
            if event is None:
                return None
            i, s = event
            return dict(step=i, state_before=asdict(result.state_history[i]),
                controller_diagnostics=asdict(s.controller_diagnostics) if s.controller_diagnostics else None,
                command=compact_control(s),
                previous_state=asdict(result.state_history[max(0, i-1)]))
        output.append(dict(corner_index=corner, corner_arc=arc, incoming_heading=incoming,
            preturn=snapshot(first), first_departing_command=snapshot(onset), switch=snapshot(switch),
            evidence_status='UNKNOWN', reason='No matched isolated-entry state/memory/corridor evidence; switching is not entry'))
    return output


def inputs():
    config = json.loads((ROOT/'config/planner_paired_v1.json').read_text())
    settings = json.loads((ROOT/'config/navigation_v08.json').read_text())
    settings['trajectory'].update(nominal_speed=config['nominal_speed'], shortcut=False)
    phase = json.loads((ROOT/'config/guidance_phase_b.json').read_text())
    parameters = load_baseline_controller_parameters(ROOT/phase['controller'])
    prepared = [p for p in prepare(settings, parameters) if p[0] in config['scenes']]
    if [p[0] for p in prepared] != config['scenes']:
        raise ValueError('scene order mismatch')
    return config, phase, parameters, prepared


def generate(prepared, config):
    pairs, audit = [], []
    cc = CandidateConfig(**config['candidate'])
    for name, req, cfg, tcfg, baseline, _, common in prepared:
        space = EnvironmentPlanningSpace(req.environment)
        preq = PlanningRequest(req.initial_state, req.goal, req.planning_constraints)
        margin = lambda pose, radius: signed_rectangle_margin(pose, req.environment.boundary.bounds, radius)
        p0 = replace(baseline, path=simplify_path(baseline.path))
        candidate = CandidatePlanner(AStarConfig(**common['astar']), margin, cc)
        p1 = candidate.plan(preq, space)
        row = dict(scene=name, candidates=candidate.diagnostics, arms={})
        for arm, plan in [('P0', p0), ('P1', p1)]:
            if plan.success:
                if not validate_path(plan.path, space, preq.constraints, request=preq).valid:
                    raise ValueError('simplified path failed continuous validation')
                generated = generate_trajectory(plan.path, space, preq.constraints, tcfg,
                    start_time=req.initial_state.timestamp, request=preq)
                shape = geometry_cost(plan.path, space, preq.constraints.required_clearance, margin, cc)
                row['arms'][arm] = dict(status=plan.status.name, geometry=shape,
                    path=[asdict(p) for p in plan.path], planning_time=plan.planning_time,
                    expanded_nodes=plan.expanded_nodes)
            else:
                generated = None
                row['arms'][arm] = dict(status=plan.status.name, planning_time=plan.planning_time)
            pairs.append((name, arm, req, cfg, tcfg, plan, generated))
        row['path_changed'] = bool(p1.success and p0.path != p1.path)
        audit.append(row)
    return pairs, audit


def stable_audit(audit):
    """Wall time excluded, all geometry, scores, node counts and selection included."""
    def strip(v):
        if isinstance(v, dict):
            return {k: strip(x) for k, x in v.items() if k != 'planning_time'}
        if isinstance(v, list):
            return [strip(x) for x in v]
        return v
    return fingerprint(strip(audit))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stage', choices=['geometry', 'execute'], required=True)
    args = parser.parse_args()
    config, phase, parameters, prepared = inputs()
    pairs, audit = generate(prepared, config)
    if args.stage == 'geometry':
        args.output.mkdir(parents=True, exist_ok=False)
        sources = {str(p.relative_to(ROOT)).replace('\\', '/'): digest(p)
                   for folder in RUNTIME_FOLDERS | {'validation'} for p in (ROOT/folder).rglob('*')
                   if p.is_file() and p.suffix in ('.py', '.json', '.yaml')}
        write(args.output/'freeze.json', dict(config=config, phase_b=phase,
            source_sha256=sources, geometry_sha256=stable_audit(audit), executable=sys.executable))
        write(args.output/'geometry.json', audit)
        print('Geometry frozen:', sum(r['path_changed'] for r in audit), 'of', len(audit), 'changed', flush=True)
        return
    frozen = json.loads((args.output/'freeze.json').read_text())
    for name, expected in frozen['source_sha256'].items():
        if digest(ROOT/name) != expected:
            raise ValueError('source changed after geometry freeze: '+name)
    if stable_audit(audit) != frozen['geometry_sha256']:
        raise ValueError('geometry repeat mismatch')
    run_dir = args.output/'runs'
    run_dir.mkdir(exist_ok=False)
    rows = []
    for name, arm, req, cfg, tcfg, plan, generated in pairs:
        hashes = []
        for repeat in (1, 2):
            if not plan.success:
                row = dict(scene=name, arm=arm, repeat=repeat, status='PLANNING_REJECTED',
                           planning_status=plan.status.name)
                write(run_dir/f'{name}_{arm}_{repeat}.json', row)
                hashes.append(fingerprint(dict(status=plan.status.name)))
                rows.append(row)
                continue
            nav = Navigator(FrozenPlanner(plan), CascadedPID3DOFController(parameters),
                Python3DOFBackend(load_synthetic_parameters()), cfg, tcfg,
                generator=lambda *a, **k: generated, startup_policy=ControlledStartupPolicy(),
                capture_policy=TerminalCapturePolicy(), guidance_adapter=LOSNavigationAdapter(
                    ProjectionConfig(**phase['projection']), LOSConfig(**{**phase['los'], 'nominal_speed':tcfg.nominal_speed})))
            result = nav.run(req)
            history = dict(states=[asdict(s) for s in result.state_history],
                references=[asdict(s) for s in result.reference_history],
                controls=[compact_control(s) for s in result.control_history])
            metrics = common_metrics(result, plan.path, req.environment,
                req.planning_constraints.required_clearance, cfg.dt, 'los')
            row = dict(scene=name, arm=arm, repeat=repeat, status=result.status.name,
                message=result.message, metrics=metrics, checks=check_execution(result, req, cfg),
                path_length=plan.path.length(), planning_time=plan.planning_time,
                expanded_nodes=plan.expanded_nodes,
                entered_terminal_region=any(s.pose.distance_to(req.goal)<=config['terminal_region_distance'] for s in result.state_history),
                failure=failure_evidence(result, plan.path, req.environment,
                    req.planning_constraints.required_clearance, cfg.dt, 'los') if result.status.name!='SUCCESS' else None)
            events = turn_events(result, plan.path, phase['los']['lookahead'], config['turn_command_threshold_rad'])
            signature = fingerprint(dict(history=history, metrics=metrics, events=events, status=result.status.name))
            hashes.append(signature)
            write(run_dir/f'{name}_{arm}_{repeat}.json', dict(summary=row, history=history, turn_events=events, deterministic_sha256=signature))
            rows.append(row)
            print(name, arm, repeat, result.status.name, flush=True)
        if hashes[0] != hashes[1]:
            raise ValueError('independent repeat mismatch: '+name+' '+arm)
    write(args.output/'summary.json', dict(rows=rows, deterministic=True,
        counts={arm:dict(Counter(r['status'] for r in rows if r['arm']==arm and r['repeat']==1)) for arm in ('P0','P1')}))
    write(args.output/'manifest.json', {str(p.relative_to(args.output)):digest(p) for p in args.output.rglob('*') if p.is_file()})


if __name__ == '__main__':
    main()
