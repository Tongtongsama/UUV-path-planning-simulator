"""Read-only C1/C2 acceptance. Fixed limits, simulation time, full time histories."""
import hashlib
import json
import math
from pathlib import Path
from dataclasses import asdict
from .mission import CASES, CONFIG, Mission, model_config, initial, planning_pose, path_distance, segment_clearance
from core import Pose
from planner import PlanningConstraints,PlanningRequest,EnvironmentPlanningSpace,validate_path
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
    required={'transactions.jsonl','commands.jsonl','gazebo_poses.jsonl','metadata.json','run_status.json','model.yml','controller.json','plan.json','navigation.jsonl','worlds/minimal.world'}
    # Windows-created legacy manifests can contain backslashes.
    normalized={k.replace('\\','/'):v for k,v in manifest.items()}
    checks={'recording_hashes':required.issubset(normalized)}
    for name,h in normalized.items():
        p=(directory/name).resolve()
        if not p.is_relative_to(directory) or not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=h:
            raise ValueError('Recording hash mismatch: '+name)
    status=load('run_status.json'); meta=load('metadata.json'); case=meta['case']
    if case not in CASES: raise ValueError('Unknown case')
    checks['unchanged_v2_model']=load('model.yml')==model_config(case)
    checks['fixed_controller_configuration']=load('controller.json')['parameters']==asdict(parameters()) and load('controller.json')['tracking_time_s']==1.0
    checks['completed']=status['status']=='completed' and status['error'] is None
    checks['clean_shutdown']=status.get('cleanup_error') is None and len(status['process_exit_codes'])==3 and all(p['returncode'] in (0,-15) for p in status['process_exit_codes'])
    records=lines('transactions.jsonl'); commands=lines('commands.jsonl'); observations=lines('gazebo_poses.jsonl')
    checks['valid_states']=bool(records) and all(r.get('valid') for r in records)
    if not checks['valid_states']: raise ValueError('Missing or invalid state evidence')
    checks['time_contiguous']=all(abs(r['dt']-.05)<1e-9 and abs(r['solver_state_ned_frd']['t']-r['start_time']-.05)<1e-6 for r in records) and all(abs(a['elapsed_end']-b['elapsed_start'])<1e-6 for a,b in zip(records,records[1:]))
    checks['complete_duration']=records[0]['elapsed_start']<1e-6 and status.get('arrival_time') is not None and records[-1]['elapsed_end']>=status['arrival_time']+.1
    checks['allocation_and_limits']=all(r['allocation']==allocate(r['allocation']['requested_wrench']) and r['applied_wrench']==wrench(r['applied_commands']) and all(abs(v)<=15+1e-9 for v in r['applied_commands'].values()) and r['applied_wrench']['Z']==0 and r['applied_commands']==(dict.fromkeys(KEYS,0.) if r['watchdog_zero'] else r['allocation']['allocated_commands']) for r in records)
    checks['watchdog_deadline']=all(all(v==0 for v in r['applied_commands'].values()) for r in records if r['start_time']-r['command_simulation_stamp']>.25+1e-6 or r['wall_monotonic']-r['command_received_wall']>1+1e-6)
    checks['no_control_dropouts']=all(not r['watchdog_zero'] for r in records if 2.2<=r['elapsed_start']<(status.get('arrival_time') or CONFIG['timeout_s']))
    checks['state_envelope']=all(abs(r['position_enu_m'][1])<=.05 and abs(r['roll_enu_rad'])<=math.radians(2) and abs(r['yaw_enu_rad'])<=math.radians(2) and abs(r['pitch_nose_up_rad'])<=math.radians(75) and all(abs(r['solver_state_ned_frd'][k])<=1.5 for k in ('u','v','w')) and abs(r['solver_state_ned_frd']['q'])<=.8 for r in records)
    checks['initial_state']=math.dist(records[0]['position_enu_m'],[initial(case).pose.x,0,initial(case).pose.z-20])<1e-4
    covered=set()
    for o in observations:
        for i,r in enumerate(records):
            if abs(o['time']-r['start_time'])>.05+1e-6: continue
            q=r['orientation_enu_flu_wxyz']; v=o['quaternion_xyzw']
            dot=abs(sum(a*b for a,b in zip(q,[v[3],*v[:3]])))
            if math.dist(o['position'],r['position_enu_m'])<=.01 and 2*math.acos(min(1,dot))<=math.radians(1): covered.add(i)
    checks['gazebo_pose_mirror']=len(covered)/len(records)>=.95
    by_sequence={r['sequence']:r for r in records}; sent={c['sequence']:c for c in commands}
    plan=load('plan.json'); mission=Mission(plan,(directory/'worlds/minimal.world').read_text())
    request=PlanningRequest(initial(case),Pose(x=18,z=-10),PlanningConstraints(.7,.8,goal_tolerance=0.))
    checks['planned_path_valid']=validate_path(mission.path,EnvironmentPlanningSpace(mission.world),request.constraints,request=request).valid
    recorded_nav={r['sequence']:r for r in lines('navigation.jsonl')}
    targets={}; clearances=[]; distances=[]; prev=initial(case).pose
    for r in records:
        p=planning_pose(r)
        clearances.append(segment_clearance(mission.world,prev,p)); prev=p
        distances.append(path_distance(mission.path,p))
        desired,nav=mission.update(r)
        targets[r['sequence']]=(desired,nav)
        if r['sequence'] in recorded_nav and not numerically_equal(recorded_nav[r['sequence']],{'sequence':r['sequence'],**nav}):
            raise ValueError('LOS/navigation evidence mismatch')
    checks['navigation_coverage']=len(recorded_nav)>=len(records)-1
    checks['safety']=min(clearances)>=.9-1e-6
    checks['arrived']=mission.arrived and status.get('arrival_time') is not None and abs(mission.arrival_time-status['arrival_time'])<1e-6
    checks['within_300s']=mission.arrival_time is not None and mission.arrival_time<=300
    rms=math.sqrt(sum(d*d for d in distances)/len(distances))
    checks['tracking_rms']=rms<=.35
    checks['tracking_max']=max(distances)<=.8
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
        desired,nav=targets[r['sequence']]
        raw,expected=replay.command(r,desired,.05)
        expected['navigation']=nav
        if not numerically_equal(raw,c['commands']) or not numerically_equal(expected,d) or abs(c['simulation_stamp']-r['solver_state_ned_frd']['t'])>1e-9:
            raise ValueError('Controller/anti-windup replay mismatch')
        count+=1
        saturation+=d['allocation']['alpha']<.999
        corrections+=any(abs(v)>1e-12 for v in d['backcalculation'].values())
    checks['controller_state_feedback_evidence']=count>=int(((status.get('arrival_time') or records[-1]['elapsed_end'])-.1)/.05)
    for r in records:
        if r['command_sequence']<0: continue
        c=sent[r['command_sequence']]
        if r['allocation']['requested_wrench']!=c['commands']: raise ValueError('Applied command does not match published PID request')
    checks['post_allocation_aw_reproduced']=count>0
    arrival=status.get('arrival_time')
    tail=[r for r in records if arrival is not None and r['elapsed_start']>=arrival+.1-1e-6]
    checks['final_zero_applied']=bool(tail) and all(all(v==0 for v in r['applied_commands'].values()) for r in tail)
    fraction=saturation/count if count else 1.
    checks['saturation_fraction']=fraction<=.2
    return {'passed':all(checks.values()),'checks':checks,'case':case,
            'arrival_time_s':mission.arrival_time,'goal_held_s':(0 if mission.inside_since is None else records[-1]['elapsed_end']-mission.inside_since),
            'minimum_clearance_m':min(clearances),'minimum_safety_margin_m':min(clearances)-.9,
            'path_rms_m':rms,'path_max_m':max(distances),'saturation_fraction':fraction,
            'controller_steps':count,'valid_steps':len(records),'pose_coverage':len(covered)/len(records),
            'error':status['error'],'scope':'S1/F0 integration acceptance only; not all scenarios/currents or full dissertation matrix'}


def analyze_suite(directory):
    directory=Path(directory)
    reports={name:analyze_run(directory/name) for name in (*CASES,'P0_repeat')}
    try:
        def rows(name):
            return {round(r['elapsed_end'],6):r for r in map(json.loads,(directory/name/'transactions.jsonl').read_text().splitlines()) if r.get('valid')}
        a,b=rows('P0'),rows('P0_repeat'); shared=sorted(a.keys()&b.keys())
        delta=max(math.dist(a[t]['position_enu_m'],b[t]['position_enu_m']) for t in shared)
        arrival=abs(reports['P0']['arrival_time_s']-reports['P0_repeat']['arrival_time_s'])
        repeat={'passed':reports['P0']['passed'] and reports['P0_repeat']['passed'] and len(shared)>=min(len(a),len(b))-1 and delta<=.02 and arrival<=.1,
                'max_position_difference_m':delta,'arrival_time_difference_s':arrival,'matched_steps':len(shared)}
    except Exception as exc:repeat={'passed':False,'error':str(exc)}
    count=sum(reports[c]['passed'] for c in CASES)
    safe=all(r.get('checks',{}).get('safety',False) for r in reports.values())
    return {'passed':count>=4 and reports['P0']['passed'] and safe and repeat['passed'],
            'successful_initials':count,'initials':5,'all_runs_passed':all(r['passed'] for r in reports.values()),
            'all_runs_safe':safe,'repeatability':repeat,'runs':reports,
            'scope':'S1 single_baffle_v1, F0 no current; five fixed initials plus P0 repeat'}
