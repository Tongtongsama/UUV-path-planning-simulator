"""Post-hoc fixed-control diagnostics; rejected paths never count as safe successes."""
import argparse
import json
from pathlib import Path
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.patches import Rectangle
from validation.planner_paired import write
from validation.guidance_matched_comparison import ROOT, digest


def progress_plateaus(history, route, switch_radius, minimum_duration=1.):
    """Post-hoc diagnostic, not a new guard: unchanged committed arc for >=1 s.

    Reproduce the existing segment-length-dependent gate radius, without
    modifying it. This records an association, not a controller causal proof.
    """
    controls=history['controls']; states=history['states']; groups=[]; start=None
    for i,s in enumerate(controls):
        following=s['guidance_command'] is not None
        same=(i>0 and following and controls[i-1]['guidance_command'] is not None
              and abs(s['path_progress_s']-controls[i-1]['path_progress_s'])<1e-12)
        if same and start is None:start=i-1
        if start is not None and (not same or i==len(controls)-1):
            end=i if same else i-1
            if controls[end]['time']-controls[start]['time']>=minimum_duration:
                k=controls[start]['guidance_command']['projection']['segment_index']
                if k+2<len(route):
                    a,b,c=route[k:k+3]
                    dx,dz=b['x']-a['x'],b['z']-a['z'];length=float(np.hypot(dx,dz))
                    next_length=float(np.hypot(c['x']-b['x'],c['z']-b['z']))
                    actual=states[end]['pose']
                    groups.append(dict(time_start=controls[start]['time'],time_end=controls[end]['time'],
                        segment_index=k,committed_s=controls[start]['path_progress_s'],
                        segment_length=length,gate_radius=min(switch_radius,.45*length,.45*next_length),
                        end_cross_track=(-dz*(actual['x']-a['x'])+dx*(actual['z']-a['z']))/length))
            start=None
    return groups


def moment_terms(control, parameters):
    """Recover exact P/I/D/FF contributions from recorded controller diagnostics."""
    d=control['controller_diagnostics']
    if d is None:
        return None
    proportional=parameters['pitch_kp']*d['pitch_error']
    derivative=d['pitch_pd']-proportional
    integral=parameters['pitch_ki']*d['integral_after']['pitch_error_integral']
    ff=d['restoring_feedforward']
    requested=d['requested']['tau_m']
    if abs(proportional+derivative+integral+ff-requested)>1e-9:
        raise ValueError('moment decomposition mismatch')
    return dict(P=proportional,I=integral,D=derivative,FF=ff,requested=requested,
                output=d['output']['tau_m'],applied=control['applied']['tau_m'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    manifest=json.loads((args.input/'manifest.json').read_text())
    if any(digest(args.input/k)!=v for k,v in manifest.items()):
        raise ValueError('evidence manifest mismatch')
    args.output.mkdir(parents=True,exist_ok=False)
    geometry=json.loads((args.input/'geometry.json').read_text())
    summary=json.loads((args.input/'summary.json').read_text())
    source=json.loads((args.input/'freeze.json').read_text())['config']['source_dataset']
    switch_radius=json.loads((ROOT/source/'freeze.json').read_text())['phase_b']['projection']['switch_radius']
    combined=Figure(figsize=(14,17),layout='constrained'); FigureCanvasAgg(combined)
    axes=combined.subplots(4,3).flat
    diagnostics=[]
    for g,ax in zip(geometry,axes):
        name=g['scene']; scene=g['scene_definition']; b=scene['boundary']
        for o in scene['scene']['obstacles']:
            ax.add_patch(Rectangle((o['x_min'],o['z_min']),o['x_max']-o['x_min'],o['z_max']-o['z_min'],color='#a7adb5'))
        fig=Figure(figsize=(12,10),layout='constrained'); FigureCanvasAgg(fig)
        panels=fig.subplots(4,2,sharex='col')
        titles=[]
        for col,(arm,color) in enumerate([('P1','#3579b8'),('P2','#d87525')]):
            folder='collision' if name in ('edge_forced_detour','zigzag_maze_detour') else 'remaining'
            data=json.loads((args.input/folder/f'{name}_{arm}_1.json').read_text())
            row=data['summary']; titles.append(arm+': '+row['status'])
            if data['history'] is None:
                for panel in panels[:,col]:
                    panel.text(.5,.5,'UNSMOOTHABLE\nNo execution',ha='center',transform=panel.transAxes)
                continue
            controls=data['history']['controls']; states=data['history']['states']
            reference=data['history']['references']
            route=(g['trials'][0]['source_path'] if arm=='P1' else g['trials'][-1]['curved_path'])
            ax.plot([p['x'] for p in route],[p['z'] for p in route],'--',color=color,alpha=.6)
            ax.plot([s['pose']['x'] for s in states],[s['pose']['z'] for s in states],color=color,label=arm)
            t=[s['time'] for s in controls]
            records=[]
            for i,s in enumerate(controls):
                terms=moment_terms(s,scene['controller'])
                effective=s['effective_guidance']
                qref=effective['q'] if effective is not None else reference[i]['twist']['q']
                records.append(dict(time=s['time'],phase=s['guidance_phase'],moment=terms,
                    pitch=states[i]['pose']['pitch'],q=states[i]['twist']['q'],q_ref=qref,
                    pitch_ref=s['controller_diagnostics']['pitch_target'] if s['controller_diagnostics'] else None,
                    pitch_error=s['controller_diagnostics']['pitch_error'] if s['controller_diagnostics'] else None,
                    obstacle_net=None if s['segment_clearance'] is None else s['segment_clearance']-g['required_clearance']))
            valid=[r for r in records if r['moment'] is not None]
            net=[(r['obstacle_net'],r['time']) for r in records if r['obstacle_net'] is not None]
            minimum=min(net) if net else (None,None)
            max_request=max((abs(r['moment']['requested']) for r in valid),default=0)
            diag=dict(scene=name,arm=arm,minimum_obstacle_net=minimum[0],minimum_clearance_time=minimum[1],
                maximum_requested_moment=max_request,
                saturation_duration=sum(abs(r['moment']['requested']-r['moment']['applied'])>1e-9 for r in valid)*scene['navigation']['dt'],
                moment_decomposition_verified=True,
                P2_geometrically_accepted=g['accepted'],evidence_status='UNKNOWN',
                posthoc_progress_plateaus=progress_plateaus(data['history'],route,switch_radius))
            diagnostics.append(diag)
            write(args.output/f'{name}_{arm}_terms.json',dict(summary=diag,steps=records))
            panels[0,col].plot(t,[r['pitch'] for r in records],label='actual pitch')
            panels[0,col].plot(t,[r['pitch_ref'] for r in records],label='effective pitch ref',ls='--')
            panels[1,col].plot(t,[r['q'] for r in records],label='actual q')
            panels[1,col].plot(t,[r['q_ref'] for r in records],label='q ref',ls='--')
            for k in ('requested','output','applied'):
                panels[2,col].plot([r['time'] for r in valid],[r['moment'][k] for r in valid],label=k)
            for k in ('P','I','D','FF'):
                panels[3,col].plot([r['time'] for r in valid],[r['moment'][k] for r in valid],label=k)
            for p,label in zip(panels[:,col],('Pitch rad','q rad/s','Moment N m','Moment terms N m')):
                p.set_ylabel(label);p.legend(fontsize=8);p.grid(alpha=.15)
                if minimum[1] is not None:p.axvline(minimum[1],color='red',ls=':',alpha=.5)
            panels[0,col].set_title(arm+' '+row['status'])
            panels[3,col].set_xlabel('Time s (red dotted: first minimum observed clearance)')
        ax.set(title=name+'\n'+' / '.join(titles),xlim=(b['x_min'],b['x_max']),ylim=(b['z_min'],b['z_max']),
               xlabel='East x m',ylabel='Up z m',aspect='equal')
        ax.legend(fontsize=8);ax.grid(alpha=.15)
        fig.suptitle(name+' | fixed controller; diagnostic only')
        fig.savefig(args.output/f'{name}_attitude.png',dpi=120);fig.clear()
    combined.suptitle('P1 / P2 | 0.2 m/s | rejection is NOT collision avoidance success')
    combined.savefig(args.output/'paired_paths.png',dpi=130);combined.clear()
    write(args.output/'diagnostics.json',diagnostics)
    lines=['# P1/P2 circular-fillet comparison','',str(summary['counts']),'',
           '| Scene | P2 geometry | Candidate changed | Last reason |', '| --- | --- | --- | --- |']
    for g in geometry:
        lines.append(f"| {g['scene']} | {'ACCEPTED' if g['accepted'] else 'UNSMOOTHABLE'} | {g['source_candidate_changed']} | {g['trials'][-1]['reason']} |")
    lines += ['', '| Scene | Arm | Status | Path length m | Physical duration s | Minimum obstacle net m |',
              '| --- | --- | --- | ---: | ---: | ---: |']
    for r in summary['rows']:
        if r['repeat']!=1:continue
        d=next((d for d in diagnostics if d['scene']==r['scene'] and d['arm']==r['arm']),None)
        lines.append(f"| {r['scene']} | {r['arm']} | {r['status']} | {r.get('path_length','—')} | {r.get('metrics',{}).get('simulation_duration','—')} | {d['minimum_obstacle_net'] if d else '—'} |")
    lines += ['', 'Underlying arcs are tangent-continuous, with bounded chord error; sampled Path is still polygonal.',
              'Radius 1 m is a frozen geometric candidate, NOT a validated vehicle minimum radius.',
              'Arc length check prevents fixed lookahead spanning an entire arc from its entry, but does not certify off-path LOS chords.',
              'All P1 histories match archived P1; every independent pair matches. Rejections have no executed history.',
              'No successful PID diagnosis is inferred from rejected P2 cases. Terms are reconstructed and sum-checked, not new control outputs.']
    (args.output/'report.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    write(args.output/'manifest.json',dict(source_manifest_sha256=digest(args.input/'manifest.json'),
        report_source_sha256=digest(Path(__file__)),files={p.name:digest(p) for p in args.output.iterdir() if p.is_file()}))


if __name__=='__main__':
    main()
