"""Predeclared direction/time acceptance, no fitted thresholds or controllers."""
import hashlib
import json
import math
from .model import DT,KEYS,LIMITS
from .protocol import pulse

def compare_repeat(first,second):
    def read(path):
        return {round(r['elapsed_end'],6):r for line in (path/'transactions.jsonl').read_text().splitlines()
            if (r:=json.loads(line)).get('valid') and r['elapsed_end']<=8+1e-6}
    a,b=read(first),read(second)
    shared=sorted(a.keys() & b.keys())
    position=max((math.dist(a[t]['position_enu_m'],b[t]['position_enu_m']) for t in shared),default=float('inf'))
    velocity=max((abs(a[t]['solver_state_ned_frd'][k]-b[t]['solver_state_ned_frd'][k])
        for t in shared for k in ('u','v','w','p','q','r')),default=float('inf'))
    return {'passed':len(shared)>=160 and position<.02 and velocity<.01,
        'matched_steps':len(shared),'max_position_difference_m':position,
        'max_body_velocity_difference':velocity,'thresholds':{'position_m':.02,'body_velocity':.01}}

def analyze_run(directory):
    def load(name,default):
        p=directory/name
        return json.loads(p.read_text(encoding='utf-8')) if p.exists() else default
    def lines(name):
        p=directory/name
        return [json.loads(line) for line in p.read_text(encoding='utf-8').splitlines() if line.strip()] if p.exists() else []
    checks={}; manifest=load('manifest.json',{})
    checks['recording_hashes']=bool(manifest) and all((directory/k).resolve().is_relative_to(directory.resolve()) and (directory/k).is_file() and
        hashlib.sha256((directory/k).read_bytes()).hexdigest()==v for k,v in manifest.items())
    status=load('run_status.json',{}); meta=load('metadata.json',{}); records=lines('transactions.jsonl')
    checks['run_completed']=status.get('status')=='completed'
    checks['clean_shutdown']=('cleanup_error' in status and status['cleanup_error'] is None and
        len(status.get('process_exit_codes',[]))==3 and
        all(p['returncode'] in (0,-15) for p in status['process_exit_codes']))
    checks['no_invalid_solver_states']=bool(records) and all(r.get('valid') for r in records)
    valid=[r for r in records if r.get('valid')]
    checks['duration_8s']=bool(valid) and valid[-1]['elapsed_end']>=8-1e-6
    checks['time_step_005']=bool(valid) and all(abs(r['solver_state_ned_frd']['t']-r['start_time']-DT)<1e-6 for r in valid)
    checks['contiguous_time']=bool(valid) and all(abs(b['start_time']-a['solver_state_ned_frd']['t'])<1e-6 for a,b in zip(valid,valid[1:]))
    checks['actuator_limits']=bool(valid) and all(abs(r['applied_commands'][k])<=lim for r in valid for k,lim in zip(KEYS,LIMITS))
    observations=lines('gazebo_poses.jsonl')
    # Compare only co-temporal records, allowing one Gazebo tick timestamp offset.
    pairs=[]
    for o in observations:
        candidates=[r for r in valid if abs(o['time']-r['start_time'])<=DT+1e-6]
        if candidates:
            pairs.append(min(math.dist(o['position'],r['position_enu_m']) for r in candidates))
    checks['gazebo_pose_mirror']=len(pairs)>=20 and max(pairs)<.02
    case=meta.get('case')
    active=[r for r in valid if 2.2<=r['elapsed_start']<=3.8]
    final=[r for r in valid if 7.8<=r['elapsed_end']<=8.1]
    if case=='zero':
        checks['zero_input']=bool(valid) and all(all(v==0 for v in r['applied_commands'].values()) for r in valid)
        checks['neutral_rest']=bool(valid) and max(abs(r['solver_state_ned_frd'][k]) for r in valid for k in ('u','v','w','p','q','r'))<1e-5
    elif case=='watchdog':
        before=[r for r in valid if 2.2<=r['elapsed_start']<=2.9]
        after=[r for r in valid if 3.4<=r['elapsed_start']<=7.9]
        checks['nonzero_before_loss']=bool(before) and any(r['applied_commands'][KEYS[0]]>0 for r in before)
        checks['watchdog_zero_after_loss']=bool(after) and all(r['watchdog_zero'] and all(v==0 for v in r['applied_commands'].values()) for r in after)
    elif case in ('surge_positive','surge_negative','heave_positive','heave_negative','pitch_positive','pitch_negative'):
        sign=1 if case.endswith('positive') else -1
        axis='u' if case.startswith('surge') else 'w' if case.startswith('heave') else 'q'
        checks['pulse_reached_solver']=bool(active) and all(not r['watchdog_zero'] and
            r['applied_commands']==pulse(case,3.0) for r in active)
        checks['velocity_direction']=bool(active) and sign*active[-1]['solver_state_ned_frd'][axis]>.01
        displacement=(active[-1]['position_enu_m'][0]-valid[0]['position_enu_m'][0] if axis=='u' else
            -active[-1]['position_enu_m'][2]+valid[0]['position_enu_m'][2] if axis=='w' else
            active[-1]['pitch_nose_up_rad']-valid[0]['pitch_nose_up_rad']) if active else 0
        checks['motion_direction']=bool(active) and sign*displacement>.001
        tail=[r for r in valid if 4.3<=r['elapsed_start']<=7.9]
        checks['input_released']=bool(tail) and all(all(v==0 for v in r['applied_commands'].values()) for r in tail)
        peak=[r for r in valid if 3.9<=r['elapsed_end']<=4.2]
        def energy(r):
            if axis=='u': return .5*25*r['solver_state_ned_frd']['u']**2
            if axis=='w': return .5*28*r['solver_state_ned_frd']['w']**2
            return .5*10*r['solver_state_ned_frd']['q']**2+6*(1-math.cos(r['pitch_nose_up_rad']))
        checks['decay_after_release']=bool(peak and final) and energy(final[-1])<max(map(energy,peak))
    else: checks['known_case']=False
    return {'passed':all(checks.values()),'checks':checks,'case':case,'valid_steps':len(valid),
        'gazebo_observations':len(observations),'error':status.get('error'),
        'scope':'Open-loop sign/time/telemetry smoke test; not PID or real-vehicle validation.'}
