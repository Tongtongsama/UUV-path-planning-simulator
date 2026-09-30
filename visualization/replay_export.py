"""Export verified frozen histories to a self-contained, offline replay.

No planner, controller, Navigator or Physics import/call. Full timestamps and
state samples are retained; display interpolation is performed only in the UI.
"""
import argparse
import hashlib
import json
from math import isfinite
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENES = ('double_baffle_detour', 'horizontal', 'detour', 'edge_forced_detour', 'zigzag_maze_detour')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


def verify_manifest(base):
    """Verify each listed source before extracting presentation data."""
    entries = load(base/'manifest.json')
    for name, expected in entries.items():
        path = (base/name.replace('\\', '/')).resolve()
        if not path.is_relative_to(base.resolve()) or digest(path) != expected:
            raise ValueError('Source manifest mismatch: '+name)


def pack_arm(data):
    """Retain original floats; classify collision at the logged step endpoint."""
    history = data['history']; states = history['states']; controls = history['controls']
    if len(states) != len(controls)+1 or not states: raise ValueError('Invalid history lengths')
    samples = [[s['timestamp'], s['pose']['x'], s['pose']['z'], s['pose']['pitch'],
                s['twist']['u'], s['twist']['w'], s['twist']['q']] for s in states]
    if any(not isfinite(v) for s in samples for v in s): raise ValueError('Nonfinite state')
    if any(b[0] <= a[0] for a,b in zip(samples,samples[1:])): raise ValueError('Nonmonotone time')
    rows=[]; collision=None
    for i,c in enumerate(controls):
        if abs(c['time']-samples[i][0]) > 1e-9: raise ValueError('Control clock mismatch')
        if c['collision'] and collision is None:
            collision=dict(step=i, interval=[samples[i][0],samples[i+1][0]], detected_at=samples[i+1][0])
        rows.append([c['time'],c['applied']['tau_x'],c['applied']['tau_m'],c['guidance_phase'],
                     c['capture_phase'],c['startup_phase'],c['collision'],c['in_bounds'],c['segment_clearance']])
    # Even an externally supplied history continuing after a collision cannot
    # animate post-collision motion. Source evidence remains untouched.
    if collision is not None:
        samples=samples[:collision['step']+2]; rows=rows[:collision['step']+1]
    status='COLLISION' if collision else data['summary']['status']
    metrics=data['summary']['metrics']
    return dict(samples=samples, controls=rows, status=status, collision=collision,
                end=samples[-1][0], metrics=metrics)


def build_payload(root=ROOT):
    source=root/'artifacts/terminal_approach_full_v1'
    verify_manifest(source)
    paired=root/'artifacts/planner_paired_v1'
    paired_manifest={k.replace('\\','/'):v for k,v in load(paired/'manifest.json').items()}
    if digest(paired/'geometry.json') != paired_manifest['geometry.json']: raise ValueError('Path evidence mismatch')
    freeze=load(source/'freeze.json')
    source_hashes={k.replace('\\','/'):v for k,v in freeze['source_sha256'].items()}
    geometry=load(paired/'geometry.json')
    if digest(paired/'geometry.json') != freeze['geometry_sha256']: raise ValueError('Geometry binding mismatch')
    files={str(source.relative_to(root))+'/manifest.json':digest(source/'manifest.json'),
           str((paired/'geometry.json').relative_to(root)):digest(paired/'geometry.json')}
    suites=[]
    for name in ('planner_acceptance_v1.json','planner_acceptance_v2.json'):
        relative='config/scenarios/'+name; path=root/relative
        if digest(path) != source_hashes[relative]: raise ValueError('Frozen scene config changed')
        files[relative]=digest(path);suites.append(load(path))
    cases=[]
    for name in SCENES:
        suite,case=(suites[0],dict(obstacles=[])) if name=='horizontal' else next(
            (s,c) for s in suites for c in s['cases'] if c['name']==name)
        arms={}
        for arm in ('P1','APPROACH'):
            relative=f'runs/{name}_{arm}_1.json';file=source/relative
            files[str(file.relative_to(root))]=digest(file)
            arms[arm]=pack_arm(load(file))
        if arms['P1']['samples'][0] != arms['APPROACH']['samples'][0]: raise ValueError('Mismatched initial state')
        path=next(g for g in geometry if g['scene']==name)['arms']['P1']['path']
        cases.append(dict(name=name,boundary=suite['boundary'],obstacles=case['obstacles'],
            goal=suite['goal'],start=suite['start'],constraints=suite['constraints'],
            path=[[p['x'],p['z']] for p in path],arms=arms,
            start_time=min(a['samples'][0][0] for a in arms.values()),
            end_time=max(a['end'] for a in arms.values())))
    return dict(schema=1,source_commit='f6b6d38',scenes=cases,sources=files,
        sample_columns=['time_s','x_m','z_m','pitch_rad','u_mps','w_mps','q_radps'],
        control_columns=['time_s','applied_tau_x_N','applied_tau_m_Nm','phase','capture_phase','startup_phase',
                         'collision','in_bounds','segment_clearance_m'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    payload=build_payload()
    template=(ROOT/'visualization/replay/player.html').read_text(encoding='utf-8')
    script=(ROOT/'visualization/replay/player.js').read_text(encoding='utf-8')
    data=json.dumps(payload,ensure_ascii=False,allow_nan=False,separators=(',',':')).replace('<','\\u003c')
    args.output.mkdir(parents=True,exist_ok=False)
    html=template.replace('/* PLAYER_CODE */',script).replace('/* REPLAY_DATA */',data)
    (args.output/'index.html').write_text(html,encoding='utf-8')
    (args.output/'replay_data.json').write_text(json.dumps(payload,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    record=dict(kind='offline_history_replay',interpolation='display_only_linear_raw_unwrapped_pitch',
        source_files=payload['sources'],exporter_sha256=digest(Path(__file__)),
        player_sha256=digest(ROOT/'visualization/replay/player.js'),
        template_sha256=digest(ROOT/'visualization/replay/player.html'),
        outputs={p.name:digest(p) for p in args.output.iterdir() if p.is_file()})
    (args.output/'manifest.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
    print('Verified offline export:',args.output/'index.html')


if __name__=='__main__':main()
