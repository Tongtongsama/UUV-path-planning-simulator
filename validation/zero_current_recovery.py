"""Extended zero-current recovery: immutable v1 evidence, independent repeats."""
import argparse
from collections import Counter
from dataclasses import asdict, fields
import gzip
import hashlib
import json
from math import cos, radians, sin
from pathlib import Path
import zipfile

import numpy as np
from controller import BodyGuidance3DOF, ControlLimits3DOF
from guidance import LOSConfig, PathGeometry, ProgressState, ProjectionConfig, compute_los
from physics import Python3DOFBackend
from simulation.vehicle_config import load_synthetic_parameters
from validation import envelope_measurement as v1

ROOT=v1.ROOT
CONFIG=ROOT/'config/zero_current_recovery_v2.json'
SCHEMA=ROOT/'config/zero_current_recovery_v2.schema.json'


def load_config():
    cfg=json.loads(CONFIG.read_text(encoding='utf-8'))
    schema=json.loads(SCHEMA.read_text(encoding='utf-8'))
    if v1.encode(cfg)!=v1.encode(schema['const']): raise ValueError('v2 frozen schema mismatch')
    return cfg


def configuration(cfg):
    base=v1.load_config()
    base['fixture']['outgoing_length']=cfg['outgoing_length']
    base['fixture']['boundary']=cfg['boundary']
    return base


def snapshot_sources():
    result=v1.sources()
    for p in (CONFIG,SCHEMA,ROOT/'docs/envelope_dataset_v1.json'):
        result[p.relative_to(ROOT).as_posix()]=v1.sha(p)
    return result


def verify_v1(cfg):
    """Verify frozen evidence and every old source still used by the v2 plant."""
    index=json.loads((ROOT/cfg['v1_index']).read_text(encoding='utf-8'))
    stage=next(s for s in index['stages'] if s['directory']==cfg['v1_stage'])
    directory=ROOT/cfg['v1_stage']
    if v1.sha(directory/'manifest.json')!=stage['manifest_sha256']: raise ValueError('v1 manifest changed')
    manifest=json.loads((directory/'manifest.json').read_text(encoding='utf-8'))
    for name,checksum in manifest['artifact_sha256'].items():
        if v1.sha(directory/name)!=checksum: raise ValueError(f'v1 artifact changed: {name}')
    for name,checksum in manifest['source_sha256'].items():
        if name.startswith(('core/','physics/','controller/','guidance/','simulation/','config/','validation/envelope_measurement.py')):
            if v1.sha(ROOT/name)!=checksum: raise ValueError(f'frozen dependency changed: {name}')
    return stage['manifest_sha256']


def read_v1(case,repeat,cfg):
    return json.loads(gzip.decompress((ROOT/cfg['v1_stage']/f"{case['id']}_r{repeat}.json.gz").read_bytes()))


def compare_prefix(old,new,cfg):
    """Fail closed by original indices/length/timestamps, not a new location event."""
    count=len(old['steps']); nstates=len(old['states'])
    if nstates!=count+1 or len(new['steps'])<count or len(new['states'])<nstates:
        raise ValueError('prefix history length mismatch')
    event=old['summary']
    if event['status']!='MEASUREMENT_COMPLETE': raise ValueError('unexpected v1 end event')
    if abs(old['states'][-1]['timestamp']-event['elapsed'])>1e-9: raise ValueError('v1 endpoint timestamp mismatch')
    if old['steps'][-1]['projection']['segment_index']!=1: raise ValueError('v1 end segment mismatch')
    angle=radians(event['case']['angle']); p=old['states'][-1]['pose']
    if p['x']*cos(angle)+(p['z']+40)*sin(angle)<8: raise ValueError('v1 end plane mismatch')
    maximum=0.
    def compare(a,b,path):
        nonlocal maximum
        if isinstance(a,dict):
            if set(a)!=set(b): raise ValueError(f'prefix keys: {path}')
            for key in a: compare(a[key],b[key],f'{path}.{key}')
        elif isinstance(a,list):
            if len(a)!=len(b): raise ValueError(f'prefix length: {path}')
            for i,(x,y) in enumerate(zip(a,b)): compare(x,y,f'{path}.{i}')
        elif isinstance(a,bool) or not isinstance(a,(int,float)):
            if a!=b: raise ValueError(f'prefix value: {path}')
        else:
            error=abs(a-b); maximum=max(maximum,error)
            if not np.isfinite(b) or error>cfg['prefix']['atol']+cfg['prefix']['rtol']*abs(a):
                raise ValueError(f'prefix numerical mismatch: {path}: {a} != {b}')
    compare(old['states'],new['states'][:nstates],'states')
    for i,(a,b) in enumerate(zip(old['steps'],new['steps'][:count])):
        a=dict(a); b=dict(b)
        # Explicit whitelist: expanded arena margin; outgoing normalized mu/lambda.
        a.pop('boundary_margin'); b.pop('boundary_margin')
        a['projection']=dict(a['projection']); b['projection']=dict(b['projection'])
        if a['projection']['projected_segment_index']==1:
            for key in ('mu','lambda_raw'): a['projection'].pop(key); b['projection'].pop(key)
        compare(a,b,f'steps.{i}')
    compare(old['summary']['initial_controller_memory'],new['initial_controller_memory'],'initial_memory')
    compare(old['summary']['final_controller_memory'],new['steps'][count-1]['diagnostics']['integral_after'],'v1_end_memory')
    return dict(passed=True,state_count=nstates,step_count=count,end_step_index=count,
                end_timestamp=old['states'][-1]['timestamp'],end_event=event['status'],maximum_numeric_difference=maximum)


def simulate(case,cfg):
    """No terminal/startup calls; history format retains all v1 comparable fields."""
    base=configuration(cfg); dt=cfg['dt']; path,state=v1.corner_fixture(case,base)
    controller=v1.make_controller(base); physics=Python3DOFBackend(load_synthetic_parameters(ROOT/base['vehicle']))
    limits=ControlLimits3DOF(**base['limits']); progress=ProgressState(PathGeometry(path),state.pose)
    pc=ProjectionConfig(**base['projection']); lc=LOSConfig(base['lookahead'],case['speed'],base['terminal_entry_distance'])
    previous=None; states=[asdict(state)]; steps=[]; status='WINDOW_COMPLETE'; failure=None
    memory=asdict(controller.memory)
    for i in range(round(cfg['budget_seconds']/dt)):
        if progress.geometry.total_length-progress.committed_s<=cfg['endpoint_guard']:
            status='RIGHT_CENSORED'; failure='ENDPOINT_GUARD'; break
        command=compute_los(state.pose,progress,pc,lc,previous)
        if command.terminal_ready:
            status='RIGHT_CENSORED'; failure='TERMINAL_READY'; break
        progress=command.projection.next_state; previous=command.pitch_ref
        target=BodyGuidance3DOF(command.surge_speed_ref,command.pitch_ref,command.pitch_rate_ref)
        output=controller.step_guidance(state,target,dt,limits)
        try:
            nxt=physics.step(state,output,np.zeros(2),dt)
        except (ValueError,FloatingPointError,OverflowError) as exc:
            status='RIGHT_CENSORED'; failure=f'NUMERICAL_FAILURE: {exc}'; break
        projection={f.name:asdict(getattr(command.projection,f.name)) if f.name=='projection_point' else getattr(command.projection,f.name)
                    for f in fields(command.projection) if f.name!='next_state'}
        margin=min(v1.boundary_margin(state,base),v1.boundary_margin(nxt,base))
        steps.append(dict(time=state.timestamp,elapsed=i*dt,phase='TURN_MEASUREMENT',target=asdict(target),
                          applied=asdict(output),diagnostics=asdict(controller.last_diagnostics),projection=projection,
                          world_velocity=v1.world_velocity(state),boundary_margin=margin))
        states.append(asdict(nxt)); state=nxt
        if margin<0: status='RIGHT_CENSORED'; failure='OUT_OF_BOUNDS'; break
    return dict(case=case,states=states,steps=steps,initial_controller_memory=memory,status=status,censor_reason=failure,
                path=[asdict(p) for p in path],terminal_entered=False)


def runs(mask):
    """Inclusive sample-index runs; duration is (end-start)*dt, not sample count."""
    padded=np.r_[False,np.asarray(mask,dtype=bool),False].astype(int)
    return list(zip(np.flatnonzero(np.diff(padded)==1).tolist(),(np.flatnonzero(np.diff(padded)==-1)-1).tolist()))


def recovery_events(t,mask,arc,hold,tail):
    intervals=runs(mask)
    qualified=[(a,b) for a,b in intervals if t[b]-t[a]+1e-9>=hold]
    first=qualified[0] if qualified else None
    last=qualified[-1] if qualified else None
    final=next(((a,b) for a,b in intervals if b==len(t)-1),None)
    stable_end=bool(final and t[-1]-t[final[0]]+1e-9>=tail)
    return dict(first_recovery_d_exit=float(arc[first[0]]) if first else None,
                first_recovery_time=float(t[first[0]]) if first else None,
                first_recovery_confirmed_time=float(t[np.searchsorted(t,t[first[0]]+hold-1e-9)]) if first else None,
                last_reentry_d_exit=float(arc[intervals[-1][0]]) if intervals else None,
                last_qualified_reentry_d_exit=float(arc[last[0]]) if last else None,
                reexit_count=sum(b<len(t)-1 and (first is not None and a>=first[0]) for a,b in intervals),
                longest_in_band_duration=max((float(t[b]-t[a]) for a,b in intervals),default=0.),
                stable_through_window_end=stable_end,
                final_in_band_run_start_d_exit=float(arc[final[0]]) if final else None,
                final_in_band_duration=float(t[-1]-t[final[0]]) if final else 0.,
                in_band_intervals=[dict(start=float(t[a]),end=float(t[b]),duration=float(t[b]-t[a])) for a,b in intervals],
                recovery_censor_reason=None if first else 'RECOVERY_RIGHT_CENSORED')


def statistics(t,y):
    if len(y)==0: return None
    x=t-t[0]; denominator=float(np.sum((x-x.mean())**2))
    slope=lambda values: float(np.sum((x-x.mean())*(values-values.mean()))/denominator) if denominator else 0.
    return dict(samples=len(y),duration=float(x[-1]),mean=float(y.mean()),rms=float(np.sqrt(np.mean(y*y))),
                max_abs=float(max(abs(y))),peak_to_peak=float(np.ptp(y)),trend=slope(y),absolute_trend=slope(abs(y)))


def peaks(t,values,prominence,separation):
    """Nearest-valley prominence, earliest plateau, deterministic separation."""
    y=np.abs(values); maxima=[]; minima=[]; i=1
    while i<len(y)-1:
        j=i
        while j+1<len(y) and y[j+1]==y[i]: j+=1
        if j<len(y)-1:
            if y[i]>y[i-1] and y[j]>y[j+1]: maxima.append(i)
            if y[i]<y[i-1] and y[j]<y[j+1]: minima.append(i)
        i=j+1
    records=[]
    for i in maxima:
        left=[k for k in minima if k<i]; right=[k for k in minima if k>i]
        prom=float(y[i]-max(y[left[-1]],y[right[0]])) if left and right else None
        reason='MISSING_VALLEY' if prom is None else 'LOW_PROMINENCE' if prom<prominence else None
        records.append(dict(index=i,time=float(t[i]),amplitude=float(y[i]),prominence=prom,reason=reason))
    selected=[]
    for item in sorted((r for r in records if r['reason'] is None),key=lambda r:(-r['amplitude'],r['index'])):
        if any(abs(item['time']-r['time'])<separation-1e-9 for r in selected): item['reason']='SEPARATION'
        else: selected.append(item)
    selected.sort(key=lambda r:r['index'])
    ratios=[dict(ratio=b['amplitude']/a['amplitude'],period=b['time']-a['time']) for a,b in zip(selected,selected[1:]) if a['amplitude']>0]
    return dict(candidates=records,selected_indices=[r['index'] for r in selected],ratios=ratios)


def analyze(history,cfg,prefix):
    """Each repeat is analyzed independently; metrics are authoritative."""
    states=history['states']; steps=history['steps']; case=history['case']; a=cfg['analysis']; band=v1.load_config()['stable']
    t=np.array([s['timestamp'] for s in states]); p=np.array([[s['pose']['x'],s['pose']['z']+40] for s in states])
    theta=radians(case['angle']); arc=p@np.array([cos(theta),sin(theta)]); cross=p@np.array([-sin(theta),cos(theta)])
    pitch=np.array([v1.wrap(s['pose']['pitch']-theta) for s in states]); q=np.array([s['twist']['q'] for s in states])
    surge=np.array([s['twist']['u']-case['speed'] for s in states])
    segment=np.array([0]+[s['projection']['segment_index'] for s in steps])
    out=segment==1; ids=np.flatnonzero(out)
    mask=out & (arc>=0) & (abs(cross)<=band['cross_track']) & (abs(pitch)<=band['pitch']) & (abs(q)<=band['q']) & (abs(surge)<=band['speed'])
    recovery=recovery_events(t,mask,arc,band['hold_seconds'],a['tail_seconds'])
    signals=dict(cross_track=cross,pitch=pitch,q=q,surge=surge)
    result=dict(case=case,status=history['status'],censor_reason=history['censor_reason'],prefix=prefix,
                duration=float(t[-1]),labels=[],signals={},**recovery)
    result['stable_through_observed_end']=recovery['stable_through_window_end']
    result['stable_through_window_end']=recovery['stable_through_window_end'] and history['status']=='WINDOW_COMPLETE'
    geometry=np.array([[p['x'],p['z']+40] for p in history['path']])
    finite_distance=np.minimum(v1.segment_distance(p,geometry[0],geometry[1]),v1.segment_distance(p,geometry[1],geometry[2]))
    result['finite_polyline_deviation_max']=float(max(finite_distance))
    result['finite_polyline_deviation_rms']=float(np.sqrt(np.mean(finite_distance**2)))
    if not len(ids): result['labels']=['SEGMENT_NOT_ENTERED','INCONCLUSIVE']; return result
    start=ids[0]; tailmask=(t>=max(t[start],t[-1]-a['tail_seconds'])-1e-9); tail_duration=float(t[-1]-t[tailmask][0])
    enough_tail=tail_duration+1e-9>=a['min_tail_seconds']; bins=[]
    for low in np.arange(t[start],t[-1],a['bin_seconds']):
        high=low+a['bin_seconds']; keep=(t>=low-1e-9)&(t<high-1e-9)
        if not keep.any(): continue
        bins.append(dict(start=float(low),end=float(min(high,t[-1])),partial=bool(high>t[-1]+1e-9),
                         signals={name:statistics(t[keep],value[keep]) for name,value in signals.items()}))
    for name,values in signals.items():
        info=dict(tail=statistics(t[tailmask],values[tailmask]),labels=[])
        if name in a['prominence']:
            info['peaks']=peaks(t[start:],values[start:],a['prominence'][name],a['peak_separation'])
            ratios=info['peaks']['ratios']; median=float(np.median([r['ratio'] for r in ratios[-a['ratio_tail']:]])) if len(ratios)>=a['minimum_ratios'] else None
            info['recent_peak_ratio_median']=median
            recent=[b['signals'][name]['max_abs'] for b in bins[-a['ratio_tail']:] if not b['partial']]
            direction=recent[-1]-recent[0] if len(recent)>1 else 0.
            wrap_crossing=bool(name=='pitch' and np.any(abs(np.diff(values[start:]))>np.pi))
            if wrap_crossing:
                info['labels'].append('WRAP_CROSSING'); info['unwrapped_tail']=statistics(t[tailmask],np.unwrap(values[start:])[-sum(tailmask):])
                info['unwrapped_peaks']=peaks(t[start:],np.unwrap(values[start:]),a['prominence'][name],a['peak_separation'])
            elif median is None: info['labels'].append('INSUFFICIENT_PEAKS')
            elif median<a['decay_ratio'] and direction<0: info['labels'].append('DECAY_OBSERVED')
            elif median>a['growth_ratio'] and direction>0: info['labels'].append('GROWTH_OBSERVED')
            elif a['decay_ratio']<=median<=a['growth_ratio'] and not recovery['stable_through_window_end']: info['labels'].append('SLOW_OR_PERSISTENT_OSCILLATION')
        result['signals'][name]=info
    bounds=[band['cross_track'],band['pitch'],band['q'],band['speed']]
    tailbins=[b for b in bins if b['start']>=t[-1]-a['tail_seconds']-1e-9 and not b['partial']]
    bounded=bool(tailbins) and all(np.ptp([b['signals'][name]['rms'] for b in tailbins])<=bound*a['band_miss_rms_fraction'] for name,bound in zip(signals,bounds))
    persistently_out=any(np.all(abs(signals[name][tailmask])>bound) for name,bound in zip(signals,bounds))
    if enough_tail and bounded and persistently_out: result['labels'].append('BOUNDED_BAND_MISS_OBSERVED')
    if recovery['first_recovery_confirmed_time'] is not None and recovery['first_recovery_confirmed_time']>prefix['end_timestamp']:
        result['labels'].append('LATE_RECOVERY_OBSERVED')
    for name,info in result['signals'].items():
        result['labels'].extend(f'{name}:{label}' for label in info['labels'])
    ref=np.array([v1.wrap(s['target']['pitch']-theta) for s in steps]); ref_t=t[:-1]
    tailref=(ref_t>=max(t[start],t[-1]-a['reference_tail_seconds'])-1e-9)
    refvalue=ref[tailref]; maximum_step=float(max(abs(np.array([v1.wrap(x) for x in np.diff(refvalue)])),default=0.))
    result['reference_tail']=dict(max_error=float(max(abs(refvalue))) if len(refvalue) else None,maximum_step=maximum_step,
                                  near_tangent=bool(len(refvalue) and max(abs(refvalue))<=a['reference_angle'] and maximum_step<=a['reference_step']))
    vel=np.array([s['world_velocity'] for s in steps]); valid=(np.linalg.norm(vel,axis=1)>a['course_min_speed']) & tailmask[:-1]
    course_turns=[float(np.unwrap(np.arctan2(vel[l:r+1,1],vel[l:r+1,0]))[-1]-np.unwrap(np.arctan2(vel[l:r+1,1],vel[l:r+1,0]))[0]) for l,r in runs(valid)]
    tailprogress=float(arc[-1]-arc[np.flatnonzero(tailmask)[0]])
    if enough_tail and any(abs(v)>=2*np.pi for v in course_turns) and abs(tailprogress)<a['circling_progress']: result['labels'].append('CIRCLING_SUSPECTED')
    saturation=np.array([abs(s['applied']['tau_m'])>=s['diagnostics']['effective_limits']['maximum_tau_m']-1e-10 for s in steps])
    result.update(bins=bins,tail_duration=tail_duration,tail_in_band_fraction=float(mask[tailmask].mean()),
                  tail_course_turns=course_turns,tail_along_progress=tailprogress,
                  moment_saturation_fraction=float(saturation.mean()),moment_saturation_seconds=float(saturation.sum()*cfg['dt']),
                  longest_moment_saturation_seconds=max(((r-l+1)*cfg['dt'] for l,r in runs(saturation)),default=0.),
                  maximum_abs_q=float(max(abs(q))),tail_sufficient=enough_tail,
                  entry_settled=None)
    if not result['labels']: result['labels']=['INCONCLUSIVE']
    return result


def plot_case(history,summary,directory):
    """Static scientific figures; no plotting feedback to the simulation."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    points=np.array([[s['pose']['x'],s['pose']['z']] for s in history['states']]); path=history['path']
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    axes[0].plot([p['x'] for p in path],[p['z'] for p in path],'--',label='finite reference')
    axes[0].plot(points[:,0],points[:,1],label='executed'); axes[0].set_title('Overview (independent axis scales)'); axes[0].legend()
    axes[1].plot(points[:,0],points[:,1]); axes[1].set_xlim(-3,12); axes[1].set_ylim(-52,-28)
    axes[1].set_aspect('equal'); axes[1].set_title('Corner detail (equal scale)')
    for ax in axes: ax.set_xlabel('x / m'); ax.set_ylabel('z / m'); ax.grid(True)
    fig.suptitle(summary['case']['id']+' (open-space measurement)'); fig.tight_layout(); fig.savefig(directory/(summary['case']['id']+'_trajectory.png')); plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(9,8),sharex=True)
    for ax,name,unit in zip(axes,('cross_track','pitch','q'),('m','rad','rad/s')):
        ax.plot([b['start'] for b in summary['bins']],[b['signals'][name]['rms'] for b in summary['bins']],label='10 s RMS')
        ax.plot([b['start'] for b in summary['bins']],[b['signals'][name]['max_abs'] for b in summary['bins']],label='10 s max abs')
        ax.set_ylabel(f'{name} / {unit}'); ax.grid(True); ax.legend()
    axes[-1].set_xlabel('physical time / s'); fig.suptitle(summary['case']['id']); fig.tight_layout(); fig.savefig(directory/(summary['case']['id']+'_bins.png')); plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--stage',choices=('pilot','full'),required=True)
    parser.add_argument('--output',type=Path,required=True); parser.add_argument('--reviewed-pilot',type=Path)
    args=parser.parse_args(); cfg=load_config(); frozen_v1=verify_v1(cfg); sources=snapshot_sources()
    if args.stage=='full':
        if args.reviewed_pilot is None: parser.error('review pilot first')
        m=json.loads((args.reviewed_pilot/'manifest.json').read_text())
        if m['source_sha256']!=sources or any(v1.sha(args.reviewed_pilot/n)!=h for n,h in m['artifact_sha256'].items()): raise ValueError('pilot evidence/version mismatch')
        s=json.loads((args.reviewed_pilot/'summary.json').read_text())
        if len(s['cases'])!=1 or s['cases'][0]['case']['id']!=cfg['pilot'] or not s['verified']: raise ValueError('invalid pilot')
    args.output.mkdir(parents=True,exist_ok=False)
    v1.write_json(args.output/'config.json',cfg); (args.output/'schema.json').write_bytes(SCHEMA.read_bytes())
    with zipfile.ZipFile(args.output/'source_snapshot.zip','w',compression=zipfile.ZIP_DEFLATED) as z:
        for name in sources: z.write(ROOT/name,name)
    summaries=[]
    for case in v1.cases(v1.load_config(),'pilot'):
        if args.stage=='pilot' and case['id']!=cfg['pilot']: continue
        history_hash=None; analysis_hash=None; repeat_summaries=[]
        for repeat in range(cfg['repeats']):
            history=simulate(case,cfg); old=read_v1(case,repeat,cfg)
            (args.output/f"{case['id']}_r{repeat}_history.json.gz").write_bytes(gzip.compress(v1.encode(history),compresslevel=1,mtime=0))
            try: prefix=compare_prefix(old,history,cfg)
            except ValueError as exc:
                v1.write_json(args.output/'PREFIX_FAILURE.json',dict(case=case,repeat=repeat,error=str(exc),interpretation_allowed=False)); raise
            summary=analyze(history,cfg,prefix); summary['entry_settled']=old['summary']['entry_settled']
            h=hashlib.sha256(v1.encode(history)).hexdigest(); ah=hashlib.sha256(v1.encode(summary)).hexdigest()
            if repeat and (h!=history_hash or ah!=analysis_hash): raise AssertionError('independent repeat/history/analysis mismatch')
            history_hash=h; analysis_hash=ah
            identified=dict(repeat_id=repeat,history_sha256=h,analysis_sha256=ah,summary=summary)
            v1.write_json(args.output/f"{case['id']}_r{repeat}_analysis.json",identified); repeat_summaries.append(identified)
        plot_case(history,summary,args.output)
        summaries.append(dict(case=case,repeats=repeat_summaries,deterministic=True))
        print(f"{len(summaries)} {case['id']} {history['status']} recovery={summary['first_recovery_time']} stable_end={summary['stable_through_window_end']} reexit={summary['reexit_count']}",flush=True)
    if sources!=snapshot_sources(): raise RuntimeError('source changed during run')
    v1.write_json(args.output/'summary.json',dict(stage=args.stage,cases=summaries,verified=True,interpretation_allowed=True))
    v1.write_json(args.output/'manifest.json',dict(source_sha256=sources,v1_manifest_sha256=frozen_v1,
        artifact_sha256={p.name:v1.sha(p) for p in sorted(args.output.iterdir()) if p.is_file()},python=__import__('platform').python_version(),numpy=np.__version__))


if __name__=='__main__': main()
