"""Predeclared direction/time acceptance, no fitted thresholds or controllers."""
import hashlib
import json
import math
from .model import DT,KEYS,LIMITS
from .protocol import pulse, duration
from .allocation import allocate, wrench

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
    checks['recording_hashes']={'transactions.jsonl','commands.jsonl','gazebo_poses.jsonl','metadata.json','run_status.json','model.yml'}.issubset(manifest) and all((directory/k).resolve().is_relative_to(directory.resolve()) and (directory/k).is_file() and
        hashlib.sha256((directory/k).read_bytes()).hexdigest()==v for k,v in manifest.items())
    status=load('run_status.json',{}); meta=load('metadata.json',{}); records=lines('transactions.jsonl')
    checks['run_completed']=status.get('status')=='completed'
    checks['clean_shutdown']=('cleanup_error' in status and status['cleanup_error'] is None and
        len(status.get('process_exit_codes',[]))==3 and
        all(p['returncode'] in (0,-15) for p in status['process_exit_codes']))
    checks['no_invalid_solver_states']=bool(records) and all(r.get('valid') for r in records)
    valid=[r for r in records if r.get('valid')]
    checks['complete_duration']=bool(valid) and valid[-1]['elapsed_end']>=duration(meta.get('case'))-1e-6
    checks['time_step_005']=bool(valid) and all(abs(r['solver_state_ned_frd']['t']-r['start_time']-DT)<1e-6 for r in valid)
    checks['contiguous_time']=bool(valid) and all(abs(b['start_time']-a['solver_state_ned_frd']['t'])<1e-6 for a,b in zip(valid,valid[1:]))
    checks['actuator_limits']=bool(valid) and all(abs(r['applied_commands'][k])<=lim for r in valid for k,lim in zip(KEYS,LIMITS))
    observations=lines('gazebo_poses.jsonl')
    # Compare only co-temporal records, allowing one Gazebo tick timestamp offset.
    pairs=[]; covered=set()
    for o in observations:
        for index,r in enumerate(valid):
            if abs(o['time']-r['start_time'])>DT+1e-6: continue
            q=r['orientation_enu_flu_wxyz']; xyzw=o['quaternion_xyzw']
            dot=abs(sum(a*b for a,b in zip(q,[xyzw[3],*xyzw[:3]])))
            angle=2*math.acos(min(1.0,dot))
            distance=math.dist(o['position'],r['position_enu_m'])
            if distance<=.01 and angle<=math.radians(1):
                covered.add(index); pairs.append(distance)
    checks['gazebo_pose_mirror']=bool(valid) and len(covered)/len(valid)>=.95
    case=meta.get('case')
    active=[r for r in valid if 2.2<=r['elapsed_start']<=3.8]
    final=[r for r in valid if 7.8<=r['elapsed_end']<=8.1]
    if case=='zero':
        checks['zero_input']=bool(valid) and all(all(v==0 for v in r['applied_commands'].values()) for r in valid)
        checks['neutral_rest']=bool(valid) and max(abs(r['solver_state_ned_frd'][k]) for r in valid for k in ('u','v','w','p','q','r'))<1e-5
    elif case in ('watchdog','stale_stamp'):
        before=[r for r in valid if 2.2<=r['elapsed_start']<=2.9]
        after=[r for r in valid if 3.4<=r['elapsed_start']<=7.9]
        checks['nonzero_before_loss']=bool(before) and any(r['applied_commands'][KEYS[0]]>0 for r in before)
        checks['watchdog_zero_after_loss']=bool(after) and all(r['watchdog_zero'] and all(v==0 for v in r['applied_commands'].values()) for r in after)
    elif case in ('surge_positive','surge_negative','pitch_positive','pitch_negative'):
        sign=1 if case.endswith('positive') else -1
        axis='u' if case.startswith('surge') else 'w' if case.startswith('heave') else 'q'
        checks['pulse_reached_solver']=bool(active) and all(not r['watchdog_zero'] and
            r['applied_commands']==allocate(pulse(case,3.0))['allocated_commands'] for r in active)
        checks['velocity_direction']=bool(active) and sign*active[-1]['solver_state_ned_frd'][axis]>.01
        displacement=(active[-1]['position_enu_m'][0]-valid[0]['position_enu_m'][0] if axis=='u' else
            -active[-1]['position_enu_m'][2]+valid[0]['position_enu_m'][2] if axis=='w' else
            active[-1]['pitch_nose_up_rad']-valid[0]['pitch_nose_up_rad']) if active else 0
        checks['motion_direction']=bool(active) and sign*displacement>.001
        first=next((r for r in valid if not r['watchdog_zero'] and
                    any(abs(v)>0 for v in r['applied_commands'].values())),None)
        expected=sign*(.08 if axis=='u' else .05)
        acceleration=((first['solver_state_ned_frd'][axis]-first['request_state'][axis])/DT) if first else float('inf')
        checks['initial_acceleration']=abs(acceleration-expected)<=.01*abs(expected)
        tail=[r for r in valid if 4.3<=r['elapsed_start']<=7.9]
        checks['input_released']=bool(tail) and all(all(v==0 for v in r['applied_commands'].values()) for r in tail)
        peak=[r for r in valid if 3.9<=r['elapsed_end']<=4.2]
        def energy(r):
            if axis=='u': return .5*25*r['solver_state_ned_frd']['u']**2
            if axis=='w': return .5*28*r['solver_state_ned_frd']['w']**2
            return .5*10*r['solver_state_ned_frd']['q']**2+6*(1-math.cos(r['pitch_nose_up_rad']))
        checks['decay_after_release']=bool(peak and final) and energy(final[-1])<max(map(energy,peak))
    elif case=='saturation':
        expected=allocate(pulse(case,3))
        checks['saturated_input_reached_solver']=bool(active) and all(
            r['allocation']==expected and r['applied_commands']==expected['allocated_commands'] for r in active)
    else: checks['known_case']=False
    checks['allocation_evidence']=bool(valid) and all(
        r.get('allocation')==allocate(r.get('allocation',{}).get('requested_wrench',{}))
        and r.get('applied_wrench')==wrench(r['applied_commands'])
        and r['applied_commands']==(dict.fromkeys(KEYS,0.0) if r['watchdog_zero'] else r['allocation']['allocated_commands'])
        for r in valid)
    checks['no_direct_heave_input']=bool(valid) and all(r['applied_wrench']['Z']==0 for r in valid)
    checks['watchdog_deadline']=bool(valid) and all(
        all(v==0 for v in r['applied_commands'].values())
        for r in valid if r['start_time']-r['command_simulation_stamp']>.25+1e-6
        or r['wall_monotonic']-r['command_received_wall']>1.0+1e-6)
    checks['initial_coordinates']=bool(valid) and math.dist(valid[0]['position_enu_m'],[2,0,-30])<1e-4 and math.dist(valid[0]['position_planning_xz_m'],[2,-10])<1e-4
    checks['planar_motion']=bool(valid) and all(abs(r['position_enu_m'][1])<=.05 and abs(r['roll_enu_rad'])<=math.radians(2) and abs(r['yaw_enu_rad'])<=math.radians(2) for r in valid)
    checks['pitch_envelope']=bool(valid) and all(abs(r['pitch_nose_up_rad'])<=math.radians(75) for r in valid)
    if case=='zero':
        checks['zero_pose_drift']=bool(valid) and all(math.dist(r['position_enu_m'],[2,0,-30])<=1e-4 and abs(r['pitch_nose_up_rad'])<=1e-4 for r in valid)
        checks['zero_attitude_drift']=bool(valid) and all(abs(r['roll_enu_rad'])<=1e-4 and abs(r['yaw_enu_rad'])<=1e-4 for r in valid)
    if case in ('watchdog','stale_stamp'):
        sent=lines('commands.jsonl')
        checks['loss_injection_proven']=any(2.2<=r['elapsed_start']<=2.9 and not r['watchdog_zero'] for r in valid)
        if case=='stale_stamp':
            nonzero=[c for c in sent if c['commands']['X']>0]
            checks['stale_packets_sent']=len(nonzero)>30 and any(a['simulation_stamp']==b['simulation_stamp'] for a,b in zip(nonzero,nonzero[1:]))

    return {'passed':all(checks.values()),'checks':checks,'case':case,'valid_steps':len(valid),
        'gazebo_observations':len(observations),'error':status.get('error'),
        'pose_coverage':len(covered)/len(valid) if valid else 0,
        'scope':'Twin-axial open-loop allocation/frame/time/direction/watchdog acceptance only. Not full M1-M7, PID, currents or real-vehicle validation.'}
