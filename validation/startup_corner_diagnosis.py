"""Read-only production-policy study: separate launch, corner and terminal effects."""
import argparse
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
from math import atan2, cos, sin
from pathlib import Path as FilePath
import subprocess
import sys

import numpy as np
from core import Path, Pose, Twist, VehicleState
from environment import Boundary, WorldModel
from validation.terminal_capture_acceptance import run
from validation.tracking_diagnostics import ROOT, cases, polyline_distance
from validation.navigation_acceptance import clean


def phase_metrics(history, path, begin, end, clearance):
    """Half-open time window of transitions; include both endpoints for geometry."""
    controls = history['controls']
    ids = [i for i, c in enumerate(controls) if begin <= c['time'] < end]
    if not ids:
        return None
    points = np.array([[s['pose']['x'], s['pose']['z']] for s in history['states']])
    refs = history['original_references']
    vertices = np.array([[p.x, p.z] for p in path])
    distances = polyline_distance(points, vertices)
    endpoint_ids = sorted(set(ids + [i + 1 for i in ids]))
    lag, cross = [], []
    for i in ids:
        ref = refs[i]['pose']
        dx, dz = points[i] - [ref['x'], ref['z']]
        lag.append(-dx*cos(ref['pitch'])-dz*sin(ref['pitch']))
        cross.append(-dx*sin(ref['pitch'])+dz*cos(ref['pitch']))
    minimum = min(ids, key=lambda i: float('inf') if controls[i]['segment_clearance'] is None else controls[i]['segment_clearance'])
    gap = controls[minimum]['segment_clearance']
    finite_gap = gap is not None and np.isfinite(gap)
    diagnostics = [controls[i]['controller_diagnostics'] for i in ids]
    ratios = [abs(controls[i]['applied']['tau_m'])/d['effective_limits']['maximum_tau_m'] for i, d in zip(ids, diagnostics)]
    return dict(transitions=len(ids), observed_begin=controls[ids[0]]['time'],
        observed_end=history['states'][ids[-1]+1]['timestamp'],
        max_path_distance=float(max(distances[endpoint_ids])),
        max_abs_timed_cross_track=float(max(abs(np.array(cross)))),
        max_reference_lead=float(max(lag)),
        max_effective_pitch_error_deg=float(np.rad2deg(max(abs(d['pitch_error']) for d in diagnostics))),
        torque_saturation_fraction=float(np.mean(np.array(ratios) >= 1-1e-12)),
        torque_above_90_percent_fraction=float(np.mean(np.array(ratios) >= .9)),
        minimum_clearance=float(gap) if finite_gap else None,
        minimum_net_margin=float(gap-clearance) if finite_gap else None,
        minimum_clearance_transition_time=controls[minimum]['time'] if finite_gap else None,
        minimum_clearance_segment=points[minimum:minimum+2].tolist() if finite_gap else None,
        observed_envelope_min=points[endpoint_ids].min(axis=0).tolist(),
        observed_envelope_max=points[endpoint_ids].max(axis=0).tolist())


def fixtures():
    base = next(c for c in cases() if c[0] == 'detour')
    _, path, world, start, constraints, *_ = base
    a, b, c = path[0], path[1], path[2]
    incoming = atan2(b.z-a.z, b.x-a.x)
    outgoing = atan2(c.z-b.z, c.x-b.x)
    open_world = WorldModel(Boundary.rectangle(x_min=-30, x_max=30, z_min=-30, z_max=10))
    for speed in (.2, .3, .4, .5):
        corner_time = np.hypot(b.x-a.x, b.z-a.z)/speed
        for aligned in (False, True):
            initial = replace(start, pose=replace(start.pose, pitch=incoming if aligned else 0.))
            name = f'detour_{speed}_{"aligned" if aligned else "original"}'
            yield (name, path, world, initial, constraints, speed), float(corner_time), 'detour'
            # Remove the corner and obstacles; preserve original incoming direction.
            line = Path.from_list([a, Pose(x=a.x+8*cos(incoming), z=a.z+8*sin(incoming))])
            yield (name.replace('detour', 'launch'), line, open_world, initial, constraints, speed), None, 'launch'
        # Same absolute incoming/outgoing headings, 8 m run-in, cruising initial surge.
        pivot = Pose(x=0, z=-10)
        first = Pose(x=-8*cos(incoming), z=-10-8*sin(incoming), pitch=incoming)
        last = Pose(x=8*cos(outgoing), z=-10+8*sin(outgoing))
        corner = Path.from_list([first, pivot, last])
        initial = VehicleState(first, Twist(u=speed))
        yield (f'corner_{speed}', corner, open_world, initial, constraints, speed), 8/speed, 'corner'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=FilePath, required=True)
    output = parser.parse_args().output
    output.mkdir(parents=True, exist_ok=False)
    reports, histories = [], {}
    for inputs, turn, family in fixtures():
        report, history = run(inputs, enabled=True)
        again, repeat = run(inputs, enabled=True)
        if report != again or history != repeat:
            raise AssertionError('Non-deterministic execution')
        end = report['metrics']['reference_duration']
        windows = {'moving': (0., end)}
        if turn is not None:
            windows.update(pre_corner=(0., turn), post_corner=(turn, end),
                           corner_entry=(max(0., turn-2), turn),
                           corner_first_two_seconds=(turn, min(turn+2, end)))
        report.update(family=family, reference_corner_time=turn, deterministic=True,
            phases={key: phase_metrics(history, inputs[1], lo, hi, inputs[4].required_clearance)
                    for key, (lo, hi) in windows.items()})
        reports.append(report)
        histories[inputs[0]] = history
        (output/(inputs[0]+'.json')).write_text(json.dumps(clean(dict(summary=report, history=history)), allow_nan=False), encoding='utf-8')
        print(inputs[0], report['status'], flush=True)
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    fig = Figure(figsize=(12, 8), layout='constrained')
    FigureCanvasAgg(fig)
    for ax, speed in zip(fig.subplots(2, 2).flat, (.2, .3, .4, .5)):
        for suffix in ('original', 'aligned'):
            name = f'detour_{speed}_{suffix}'
            states = histories[name]['states']
            ax.plot([s['pose']['x'] for s in states], [s['pose']['z'] for s in states], label=suffix)
        report = next(r for r in reports if r['name'] == f'detour_{speed}_original')
        ax.plot([p['x'] for p in report['path']], [p['z'] for p in report['path']], 'k--', label='reference')
        ax.fill([4.9, 5.1, 5.1, 4.9], [-7, -7, -3, -3], color='gray', alpha=.5)
        ax.set(title=f'{speed} m/s (alignment is diagnostic only)', xlabel='x (m)', ylabel='z (m)', aspect='equal')
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.savefig(output/'detour.png', dpi=140)
    fig = Figure(figsize=(9, 5), layout='constrained')
    FigureCanvasAgg(fig)
    ax = fig.subplots()
    for speed in (.2, .3, .4, .5):
        report = next(r for r in reports if r['name'] == f'corner_{speed}')
        states = histories[report['name']]['states']
        selected = [s for s in states if report['reference_corner_time']-2 <= s['timestamp'] < report['metrics']['reference_duration']]
        ax.plot([s['pose']['x'] for s in selected], [s['pose']['z'] for s in selected], label=f'{speed} m/s')
    ax.plot([p['x'] for p in report['path']], [p['z'] for p in report['path']], 'k--', label='reference')
    ax.set(xlim=(-1.2, 4), ylim=(-10.7, -7.3), aspect='equal', xlabel='x (m)', ylabel='z (m)',
           title='Isolated corner: observed trajectories, no obstacle clearance guarantee')
    ax.legend(); ax.grid(alpha=.2)
    fig.savefig(output/'corner.png', dpi=140)
    tests = subprocess.run([sys.executable, '-m', 'pytest', 'tests', '-q', '-p', 'no:cacheprovider'], cwd=ROOT, capture_output=True, text=True)
    (output/'pytest.log').write_text(tests.stdout+tests.stderr, encoding='utf-8')
    summary = dict(date=datetime.now(timezone.utc).isoformat(), executable=sys.executable,
        installed=subprocess.check_output([sys.executable, '-m', 'pip', 'freeze'], text=True).splitlines(),
        cases=reports, tests_exit_code=tests.returncode,
        diagnostic_checks_passed=all(all(r['checks'].values()) for r in reports),
        source_sha256={p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for folder in ('core','controller','navigation','physics','planner','trajectory','simulation','environment','validation','tests','config')
            for p in (ROOT/folder).rglob('*') if p.is_file() and p.suffix in ('.py','.yaml','.json')})
    (output/'summary.json').write_text(json.dumps(clean(summary), indent=2, allow_nan=False), encoding='utf-8')
    (output/'manifest.json').write_text(json.dumps({p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in output.iterdir() if p.is_file()}, indent=2), encoding='utf-8')
    if tests.returncode or not summary['diagnostic_checks_passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
