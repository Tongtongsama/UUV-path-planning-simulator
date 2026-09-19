"""Verify frozen evidence and produce descriptive statistics and scientific plots."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.patches import Rectangle, Circle

ROOT=Path(__file__).resolve().parents[1]
COLORS={'SUCCESS':'#23856d','COLLISION':'#c53b40','OUT_OF_BOUNDS':'#d78125',
        'STARTUP_FAILED':'#6b65a5','CAPTURE_FAILED':'#80594a'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();src=args.evidence
    summary=json.loads((src/'summary.json').read_text(encoding='utf-8'))
    manifest=json.loads((src/'manifest.json').read_text(encoding='utf-8'))
    for name,expected in manifest.items():
        if digest(src/name)!=expected:raise ValueError(f'evidence mismatch: {name}')
    for name,expected in summary['source_sha256'].items():
        if digest(ROOT/name)!=expected:raise ValueError(f'source mismatch: {name}')
    out=args.output;out.mkdir(parents=True,exist_ok=False)
    cases=summary['cases'];counts=Counter(r['status'] for r in cases)
    fields=('simulation_duration','terminal_position_error','terminal_speed','minimum_executed_safety_margin',
            'saturation_fraction','startup_duration','capture_duration','control_effort_proxy')
    rows=[dict(name=r['name'],status=r['status'],**{k:r['metrics'].get(k) for k in fields}) for r in cases]
    statistics=dict(total=len(cases),counts=dict(counts),success_fraction=counts['SUCCESS']/len(cases),
        safety_violations=counts['COLLISION']+counts['OUT_OF_BOUNDS'],rows=rows,
        caveats=['Descriptive fixed-suite counts, not independent random trials.',
                 'Zero-current narrow passage repeats a static case.',
                 'Failure duration is censored; do not average it as completion time.',
                 'Null obstacle clearance means no obstacle, not zero clearance.',
                 'Startup rejection is failure, not collision-free mission success.'])
    (out/'statistics.json').write_text(json.dumps(statistics,indent=2),encoding='utf-8')
    fig=Figure(figsize=(17,18),layout='constrained');FigureCanvasAgg(fig)
    for ax,row in zip(fig.subplots(4,4).flat,cases):
        data=json.loads((src/(row['name']+'.json')).read_text(encoding='utf-8'))
        b=data['boundary'];radius=data['constraints']['vehicle_radius']+data['constraints']['safety_margin']
        for obstacle in data['scene']['obstacles']:
            ax.add_patch(Rectangle((obstacle['x_min'],obstacle['z_min']),obstacle['x_max']-obstacle['x_min'],obstacle['z_max']-obstacle['z_min'],color='#697682'))
        ax.add_patch(Rectangle((b['x_min']+radius,b['z_min']+radius),b['x_max']-b['x_min']-2*radius,b['z_max']-b['z_min']-2*radius,fill=False,ls=':',color='#7e8792'))
        for key,color,style,label in [('raw_path','#a0aab5','--','A*'),('trajectory','#258aa4','-','Reference'),('states','#dd762d','-','Actual')]:
            values=data[key] or []
            poses=values if key=='raw_path' else [s['pose'] for s in values]
            ax.plot([p['x'] for p in poses],[p['z'] for p in poses],style,color=color,lw=1.5,label=label)
        goal=data['goal'];start=data['initial_state']['pose'];last=data['states'][-1]['pose']
        ax.add_patch(Circle((goal['x'],goal['z']),data['navigation']['goal_position_tolerance'],color='#23856d',alpha=.2))
        ax.scatter(start['x'],start['z'],marker='o',color='#263747',s=18)
        ax.scatter(goal['x'],goal['z'],marker='*',color='#23856d',s=60)
        ax.scatter(last['x'],last['z'],marker='x',color=COLORS[row['status']],s=45)
        ax.set(title=f"{row['name']}\n{row['status']} | {row['metrics']['simulation_duration']:.2f} s",
            xlabel='x East (m)',ylabel='z Up (m)',xlim=(b['x_min'],b['x_max']),ylim=(b['z_min'],b['z_max']),aspect='equal')
        ax.grid(alpha=.15)
    fig.legend(*fig.axes[0].get_legend_handles_labels(),loc='outside lower center',ncol=3)
    fig.suptitle('Frozen candidate 2026-09-18 | All 16 original scenarios\nDotted rectangle: eroded boundary. Cross: final state (not always a collision).',fontsize=16)
    fig.savefig(out/'scenario_atlas.png',dpi=140);fig.savefig(out/'scenario_atlas.pdf')
    fig=Figure(figsize=(15,9),layout='constrained');FigureCanvasAgg(fig)
    ax,err,time=fig.subplots(1,3,gridspec_kw={'width_ratios':[1,2,2]})
    labels=list(COLORS);ax.barh(labels,[counts[k] for k in labels],color=[COLORS[k] for k in labels]);ax.set(title='Outcomes (n=16)',xlabel='Count',xlim=(0,10))
    names=[r['name'] for r in cases];colors=[COLORS[r['status']] for r in cases]
    err.barh(names,[r['metrics']['terminal_position_error'] for r in cases],color=colors)
    err.axvline(.25,color='black',ls='--');err.set(title='Terminal position error',xlabel='m (dashed: 0.25 m tolerance)');err.invert_yaxis()
    time.barh(names,[r['metrics']['simulation_duration'] for r in cases],color=colors)
    time.set(title='Observed duration (failures are censored)',xlabel='Physical simulation seconds');time.invert_yaxis()
    fig.suptitle('9/16 success (56.25%) — descriptive suite outcome, not robustness probability')
    fig.savefig(out/'statistics.png',dpi=140);fig.savefig(out/'statistics.pdf')
    freeze=dict(version='navigation-v0.8-frozen-candidate-20260918',created=datetime.now(timezone.utc).isoformat(),
        git_head=subprocess.check_output(['git','-c',f'safe.directory={ROOT.as_posix()}','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        evidence_path=src.as_posix(),evidence_manifest_sha256=digest(src/'manifest.json'),
        source_sha256=summary['source_sha256'],packages=summary['installed'],
        policies='Control v0.5.1 coefficient 6; controlled startup; terminal capture; local slowdown 0.2 m/s +/-1 m',
        note='Logical candidate freeze, not a Git tag or global-default change. Original evidence is not overwritten.')
    (out/'freeze.json').write_text(json.dumps(freeze,indent=2),encoding='utf-8')
    (out/'manifest.json').write_text(json.dumps({p.name:digest(p) for p in out.iterdir() if p.is_file()},indent=2),encoding='utf-8')
    print(json.dumps(dict(version=freeze['version'],git_head=freeze['git_head'],counts=dict(counts),output=str(out))))


if __name__=='__main__':main()
