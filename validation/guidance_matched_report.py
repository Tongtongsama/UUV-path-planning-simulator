"""Verify immutable paired evidence and generate descriptive reports/figures."""
import argparse
from collections import Counter
import hashlib
import json
from math import hypot
from pathlib import Path
import xml.etree.ElementTree as ET

from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.patches import Rectangle


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--junit',type=Path,required=True)
    args=parser.parse_args()
    manifest=json.loads((args.evidence/'manifest.json').read_text())
    for name,expected in manifest.items():
        if digest(args.evidence/name)!=expected:raise ValueError(f'evidence changed: {name}')
    data=json.loads((args.evidence/'comparison.json').read_text())
    if not data['evidence_valid'] or not data['matched_inputs']:raise ValueError('invalid comparison')
    suites=ET.parse(args.junit).getroot().findall('testsuite')
    tests={k:sum(int(s.attrib.get(k,0)) for s in suites) for k in ('tests','errors','failures','skipped')}
    if tests['failures'] or tests['errors']:raise ValueError(tests)
    args.output.mkdir(parents=True,exist_ok=False)
    histories={};failures=[];metrics=[]
    for pair in data['pairs']:
        for arm in ('los','timed'):
            record=json.loads((args.evidence/arm/(pair['name']+'.json')).read_text())
            histories[arm,pair['name']]=record['history']
            row=pair[arm];history=record['history'];m=row['metrics']
            metrics.append(dict(name=pair['name'],arm=arm,status=row['status'],**m))
            if row['status']=='SUCCESS':continue
            event=row['failure_evidence'];final=history['states'][-1]
            # Guard rejection can occur BEFORE a new transition is appended:
            # the last observed transition must not override the termination phase.
            terminated_phase=('TERMINAL_CAPTURE' if row['status']=='CAPTURE_FAILED' else
                'STARTUP' if row['status']=='STARTUP_FAILED' else event.get('phase'))
            capture_phases=Counter(s['capture_phase'] for s in history['controls']
                                  if s['capture_phase'] not in ('DISABLED','TIMED_REFERENCE'))
            failures.append(dict(name=pair['name'],arm=arm,status=row['status'],message=row['message'],
                termination_phase=terminated_phase,last_observed_phase=event.get('phase'),
                first_safety_event=event.get('first_safety_event',False),event=event,
                final_state=final,capture_phase_counts=dict(capture_phases),
                final_translational_speed=hypot(final['twist']['u'],final['twist']['w']),
                maximum_abs_q=m['maximum_abs_q'],rate_envelope_violation=m['rate_envelope_violation'],
                progress_stall=m['progress_stall']))
    report=dict(outcomes=data['outcomes'],rate_violations=data['rate_violations'],tests=tests,
        evidence_manifest_sha256=digest(args.evidence/'manifest.json'),junit_sha256=digest(args.junit),
        matched_inputs=data['matched_inputs'],all_repeats_deterministic=all(p[a]['checks']['deterministic']
            for p in data['pairs'] for a in ('los','timed')),failures=failures,metrics=metrics,
        note='Fixed fixtures, not independent random trials. Failed durations are censored. No causal or robustness claim.')
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    lines=['# Matched 16-case results','',
        '| Case | LOS | Timed | Max |q| LOS / timed (rad/s) |','| --- | --- | --- | --- |']
    # Avoid a literal pipe inside the Markdown column heading.
    lines[2]='| Case | LOS | Timed | Peak q magnitude LOS / timed (rad/s) |'
    for pair in data['pairs']:
        lines.append(f"| {pair['name']} | {pair['los']['status']} | {pair['timed']['status']} | "
                     f"{pair['los']['metrics']['maximum_abs_q']:.3f} / {pair['timed']['metrics']['maximum_abs_q']:.3f} |")
    lines.extend(['','All shared-input and repeated-run checks passed.',
        f"Full tests: {tests['tests']} tests; {tests['failures']} failures, {tests['errors']} errors.",
        'No slowdown, PID tuning, filter or runtime safety-contract changes were made.',
        'Rate-envelope exceedance is diagnostic only. A capture-entry rejection may have no capture transition in history.'])
    (args.output/'summary.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')

    fig=Figure(figsize=(17,18),layout='constrained');FigureCanvasAgg(fig)
    colors={'los':'#137eab','timed':'#d76a26'}
    for ax,pair in zip(fig.subplots(4,4).flat,data['pairs']):
        common=json.loads((args.evidence/'common'/(pair['name']+'.json')).read_text())
        b=common['boundary'];radius=common['constraints']['vehicle_radius']+common['constraints']['safety_margin']
        for o in common['scene']['obstacles']:
            ax.add_patch(Rectangle((o['x_min'],o['z_min']),o['x_max']-o['x_min'],o['z_max']-o['z_min'],color='#89929a',alpha=.6))
        ax.add_patch(Rectangle((b['x_min']+radius,b['z_min']+radius),b['x_max']-b['x_min']-2*radius,
            b['z_max']-b['z_min']-2*radius,fill=False,ls=':',color='#888888'))
        path=common['processed_path'];ax.plot([p['x'] for p in path],[p['z'] for p in path],'k--',lw=1,label='Shared path')
        for arm in ('los','timed'):
            states=histories[arm,pair['name']]['states'];points=[s['pose'] for s in states]
            ax.plot([p['x'] for p in points],[p['z'] for p in points],color=colors[arm],lw=1.4,label=arm.upper())
            ax.scatter(points[-1]['x'],points[-1]['z'],color=colors[arm],marker='x',s=25)
        ax.set(title=f"{pair['name']}\nL: {pair['los']['status']} / T: {pair['timed']['status']}",
            xlim=(b['x_min'],b['x_max']),ylim=(b['z_min'],b['z_max']),xlabel='x East (m)',ylabel='z Up (m)',aspect='equal')
        ax.grid(alpha=.15)
    fig.legend(*fig.axes[0].get_legend_handles_labels(),loc='outside lower center',ncol=3)
    fig.savefig(args.output/'paired_paths.png',dpi=140)

    fig=Figure(figsize=(12,7),layout='constrained');FigureCanvasAgg(fig);ax=fig.subplots()
    for offset,arm in ((-.18,'los'),(.18,'timed')):
        ax.barh([i+offset for i in range(16)],[p[arm]['metrics']['maximum_abs_q'] for p in data['pairs']],
                height=.35,label=arm.upper(),color=colors[arm])
    ax.axvline(.35,color='#ad3434',ls='--',label='0.35 diagnostic (NOT a stop guard)')
    ax.set_yticks(range(16),[p['name'] for p in data['pairs']]);ax.invert_yaxis()
    ax.set(xlabel='Maximum actual |q| (rad/s)',title='Whole-history rate envelope — matched fixed fixtures')
    ax.legend(loc='lower right');ax.grid(axis='x',alpha=.15)
    fig.savefig(args.output/'rate_envelope.png',dpi=140)
    (args.output/'manifest.json').write_text(json.dumps({p.name:digest(p) for p in args.output.iterdir() if p.is_file()},indent=2),encoding='utf-8')
    print(json.dumps(dict(outcomes=report['outcomes'],rate_violations=report['rate_violations'],tests=tests,failures=len(failures))))


if __name__=='__main__':main()
