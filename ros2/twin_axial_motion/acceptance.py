"""Read-only C1/C2 acceptance. Fixed limits, simulation time, full time histories."""
import hashlib
import json
import math
from pathlib import Path
from dataclasses import asdict
from .protocol import CASES, MOTION_CASES, FAULT_CASES, FAULT_TIME, duration, guidance, desired_depth, OUTER, CRITERIA
from ros2.twin_axial_pid.control import AllocatedPID, parameters
from ros2.twin_axial_v2.model import model, KEYS
from ros2.twin_axial_v2.allocation import allocate, wrench


def numerically_equal(a,b):
    """Allow only floating-point library roundoff across Windows/Linux."""
    if isinstance(a,dict):
        return isinstance(b,dict) and a.keys()==b.keys() and all(numerically_equal(a[k],b[k]) for k in a)
    if isinstance(a,(int,float)) and not isinstance(a,bool):
        return isinstance(b,(int,float)) and math.isfinite(a) and math.isfinite(b) and math.isclose(a,b,rel_tol=1e-12,abs_tol=1e-12)
    return a==b


def step_metrics(times,values,reference,start,band):
    error=[abs(v-reference) for v in values]
    last_bad=max((i for i,e in enumerate(error) if e>band),default=-1)
    settling=times[last_bad+1]-start if last_bad+1<len(times) else None
    final=[e for t,e in zip(times,error) if t>=times[-1]-5-1e-7]
    direction=1 if reference>=0 else -1
    return {'settling_s':settling,'final_5s_mae':sum(final)/len(final),
            'overshoot':max(0,max(direction*(v-reference) for v in values)),
            'max_absolute_error':max(error)}


def analyze_run(directory):
    try: return _analyze(Path(directory).resolve())
    except Exception as exc: return {'passed':False,'error':str(exc)}


def _analyze(directory):
    def load(name): return json.loads((directory/name).read_text(encoding='utf-8'))
    def lines(name): return [json.loads(l) for l in (directory/name).read_text(encoding='utf-8').splitlines() if l.strip()]
    manifest=load('manifest.json')
    required={'transactions.jsonl','commands.jsonl','gazebo_poses.jsonl','metadata.json','run_status.json','model.yml','controller.json','events.jsonl','feedback_delivery.jsonl'}
    # Windows-created legacy manifests can contain backslashes.
    normalized={k.replace('\\','/'):v for k,v in manifest.items()}
    checks={'recording_hashes':required.issubset(normalized)}
    for name,h in normalized.items():
        p=(directory/name).resolve()
        if not p.is_relative_to(directory) or not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=h:
            raise ValueError('Recording hash mismatch: '+name)
    status=load('run_status.json'); meta=load('metadata.json'); case=meta['case']
    if case not in CASES: raise ValueError('Unknown case')
    checks['unchanged_v2_model']=load('model.yml')==model()
    checks['fixed_controller_configuration']=load('controller.json')['parameters']==asdict(parameters()) and load('controller.json')['tracking_time_s']==1.0 and load('controller.json')['outer']==OUTER and load('controller.json')['criteria']==CRITERIA
    checks['completed']=status['status']==('fault_observed' if case in FAULT_CASES else 'completed') and status['error'] is None
    checks['clean_shutdown']=status.get('cleanup_error') is None and len(status['process_exit_codes'])==3 and all(p['returncode'] in (0,-15) for p in status['process_exit_codes'])
    records=lines('transactions.jsonl'); commands=lines('commands.jsonl'); observations=lines('gazebo_poses.jsonl')
    checks['valid_states']=bool(records) and all(r.get('valid') for r in records)
    if not checks['valid_states']: raise ValueError('Missing or invalid state evidence')
    checks['time_contiguous']=all(abs(r['dt']-.05)<1e-9 and abs(r['solver_state_ned_frd']['t']-r['start_time']-.05)<1e-6 for r in records) and all(abs(a['elapsed_end']-b['elapsed_start'])<1e-6 for a,b in zip(records,records[1:]))
    checks['complete_duration']=records[0]['elapsed_start']<1e-6 and records[-1]['elapsed_end']>=duration(case)+.1
    checks['allocation_and_limits']=all(r['allocation']==allocate(r['allocation']['requested_wrench']) and r['applied_wrench']==wrench(r['applied_commands']) and all(abs(v)<=15+1e-9 for v in r['applied_commands'].values()) and r['applied_wrench']['Z']==0 and r['applied_commands']==(dict.fromkeys(KEYS,0.) if r['watchdog_zero'] else r['allocation']['allocated_commands']) for r in records)
    checks['watchdog_deadline']=all(all(v==0 for v in r['applied_commands'].values()) for r in records if r['start_time']-r['command_simulation_stamp']>.25+1e-6 or r['wall_monotonic']-r['command_received_wall']>1+1e-6)
    checks['no_control_dropouts']=all(not r['watchdog_zero'] for r in records if 2.2<=r['elapsed_start']<(FAULT_TIME if case in FAULT_CASES else duration(case)))
    checks['state_envelope']=all(abs(r['position_enu_m'][1])<=.05 and abs(r['roll_enu_rad'])<=math.radians(2) and abs(r['yaw_enu_rad'])<=math.radians(2) and abs(r['pitch_nose_up_rad'])<=math.radians(75) and all(abs(r['solver_state_ned_frd'][k])<=1.5 for k in ('u','v','w')) and abs(r['solver_state_ned_frd']['q'])<=.8 for r in records)
    checks['initial_state']=math.dist(records[0]['position_enu_m'],[2,0,-30])<1e-4
    covered=set()
    for o in observations:
        for i,r in enumerate(records):
            if abs(o['time']-r['start_time'])>.05+1e-6: continue
            q=r['orientation_enu_flu_wxyz']; v=o['quaternion_xyzw']
            dot=abs(sum(a*b for a,b in zip(q,[v[3],*v[:3]])))
            if math.dist(o['position'],r['position_enu_m'])<=.01 and 2*math.acos(min(1,dot))<=math.radians(1): covered.add(i)
    checks['gazebo_pose_mirror']=len(covered)/len(records)>=.95
    by_sequence={r['sequence']:r for r in records}; sent={c['sequence']:c for c in commands}
    replay=AllocatedPID(); count=0; saturation=0; corrections=0; previous_feedback=None
    for c in commands:
        d=c.get('controller')
        if d is None:
            if c['commands']!={'X':0.,'M':0.}: raise ValueError('Nonzero command without controller evidence')
            continue
        r=by_sequence[d['feedback_sequence']]
        if previous_feedback is not None and d['feedback_sequence']!=previous_feedback+1:
            raise ValueError('Controller feedback sequence not contiguous')
        previous_feedback=d['feedback_sequence']
        desired,outer=guidance(case,r)
        raw,expected=replay.command(r,desired,.05)
        expected['outer']=outer
        if not numerically_equal(raw,c['commands']) or not numerically_equal(expected,d) or abs(c['simulation_stamp']-r['solver_state_ned_frd']['t'])>1e-9:
            raise ValueError('Controller/anti-windup replay mismatch')
        count+=1
        saturation+=d['allocation']['alpha']<.999
        corrections+=any(abs(v)>1e-12 for v in d['backcalculation'].values())
    checks['controller_state_feedback_evidence']=count>=(120 if case in FAULT_CASES else int((duration(case)-.1)/.05))
    for r in records:
        if r['command_sequence']<0: continue
        c=sent[r['command_sequence']]
        if not c['delivered'] or r['allocation']['requested_wrench']!=c['commands']: raise ValueError('Applied command does not match published PID request')
    checks['post_allocation_aw_reproduced']=count>0
    tail=[r for r in records if r['elapsed_start']>=duration(case)+.1-1e-6]
    checks['final_zero_applied']=bool(tail) and all(all(v==0 for v in r['applied_commands'].values()) for r in tail)
    events=lines('events.jsonl'); deliveries=lines('feedback_delivery.jsonl')
    metrics={}
    if case in FAULT_CASES:
        faults=[e for e in events if e['event']=='fault_injected']
        halts=[e for e in events if e['event']=='controller_halted']
        if len(faults)!=1 or len(halts)!=1: raise ValueError('Missing or duplicate fault/stop event')
        fault,halt=faults[0],halts[0]
        checks['scheduled_fault']=fault['kind']==case and FAULT_TIME<=fault['elapsed']<=FAULT_TIME+.051
        checks['integral_reset']=all(v==0 for v in halt['integral_after'].values())
        checks['task_aborted_and_latched']=not any(c.get('controller') for c in commands if c['wall_monotonic']>=halt['wall_monotonic'])
        allowed={'state_loss':('state_timeout','command_watchdog'),'stale_state':('stale_state',),'command_loss':('command_watchdog',)}
        checks['expected_stop_reason']=halt['reason'] in allowed[case]
        pre=[r for r in records if 7<=r['elapsed_start']<FAULT_TIME]
        checks['closed_loop_active_before_fault']=bool(pre) and all(not r['watchdog_zero'] for r in pre) and any(abs(r['applied_wrench']['X'])>1 for r in pre)
        sent_active=[c for c in commands if c['delivered'] and c.get('controller') and c['wall_monotonic']<fault['wall_monotonic']]
        if not sent_active: raise ValueError('No real commands before fault')
        last_stamp=sent_active[-1]['simulation_stamp']
        post=[r for r in records if r['elapsed_start']>=fault['elapsed']-1e-6]
        zero=next((r for r in post if all(v==0 for v in r['applied_commands'].values())),None)
        checks['zero_by_watchdog_plus_one_step']=zero is not None and zero['start_time']<=last_stamp+CRITERIA['fault_max_after_stamp_s']+1e-6
        checks['zero_persists']=zero is not None and records[-1]['elapsed_end']-zero['elapsed_start']>=2 and all(all(v==0 for v in r['applied_commands'].values()) for r in records if r['sequence']>=zero['sequence'])
        checks['controller_stops_promptly']=halt['elapsed']<=fault['elapsed']+.35+1e-6
        after=[d for d in deliveries if d['observer_elapsed']>=fault['elapsed']]
        checks['observer_continues']=len(after)>=40
        if case=='state_loss':
            checks['feedback_drop_proven']=bool(after) and all(d['delivered_sequence'] is None for d in after)
            checks['no_old_nonzero_commands']=not any(c.get('controller') for c in commands if c['wall_monotonic']>=fault['wall_monotonic'])
        elif case=='stale_state':
            checks['frozen_feedback_proven']=bool(after) and all(d['delivered_stamp']==fault['last_feedback_stamp'] for d in after)
            checks['no_old_nonzero_commands']=not any(c.get('controller') for c in commands if c['wall_monotonic']>=fault['wall_monotonic'])
        else:
            dropped=[c for c in commands if c['wall_monotonic']>=fault['wall_monotonic']]
            checks['command_drop_proven']=bool(dropped) and all(not c['delivered'] for c in dropped) and any(c.get('controller') and abs(c['commands']['X'])>1 for c in dropped)
            checks['independent_watchdog_used']=zero is not None and zero['watchdog_zero']
        metrics={'last_delivered_control_stamp':last_stamp,'first_zero_stamp':zero['start_time'] if zero else None,
            'zero_delay_from_last_command_s':zero['start_time']-last_stamp if zero else None,
            'fault_elapsed':fault['elapsed'],'halt_elapsed':halt['elapsed'],'stop_reason':halt['reason']}
    else:
        checks['no_fault_events']=not events
        start=2. if case=='straight' else 8.
        measured=[r for r in records if start<=r['elapsed_end']<=duration(case)+1e-6]
        times=[r['elapsed_end'] for r in measured]
        depth_target=desired_depth(case,start)
        depth=step_metrics(times,[r['solver_state_ned_frd']['z'] for r in measured],depth_target,start,CRITERIA['depth_band_m'])
        # Step sign is set by target change, not by positive absolute depth.
        sign=1 if depth_target>=30 else -1
        overshoot=max(0,max(sign*(r['solver_state_ned_frd']['z']-depth_target) for r in measured))
        depth['overshoot']=overshoot
        speed=step_metrics(times,[r['solver_state_ned_frd']['u'] for r in measured],.4,start,.04)
        tail=[r for r in measured if r['elapsed_end']>=duration(case)-5-1e-7]
        pitch=max(abs(math.degrees(r['pitch_nose_up_rad'])) for r in tail)
        checks['speed_holds']=speed['settling_s'] is not None and speed['settling_s']<=15 and speed['final_5s_mae']<=.02
        checks['depth_final_mae']=depth['final_5s_mae']<=CRITERIA['depth_final_mae_m']
        checks['final_pitch_level']=pitch<=CRITERIA['final_pitch_deg']
        travel=measured[-1]['position_enu_m'][0]-2
        checks['forward_progress']=travel>10 and all(r['solver_state_ned_frd']['u']>=-1e-6 for r in measured)
        if case=='straight':
            checks['straight_depth_hold']=depth['max_absolute_error']<=CRITERIA['straight_depth_error_m']
        else:
            checks['depth_settles']=depth['settling_s'] is not None and depth['settling_s']<=CRITERIA['depth_settling_s']
            checks['depth_overshoot']=overshoot<=CRITERIA['depth_overshoot_m']
            checks['depth_direction']=sign*(measured[-1]['solver_state_ned_frd']['z']-30)>1.8
        metrics={'depth':depth,'speed':speed,'final_pitch_max_deg':pitch,'forward_travel_m':travel}
    return {'passed':all(checks.values()),'checks':checks,'case':case,'metrics':metrics,
            'controller_steps':count,'saturated_steps':saturation,'aw_correction_steps':corrections,
            'valid_steps':len(records),'pose_coverage':len(covered)/len(records),
            'scope':'No-current straight/depth and controller-boundary state/command fault injection; no LOS, paths, currents, or paused-clock wall watchdog experiment'}
