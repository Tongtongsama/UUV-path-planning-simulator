"""Build the 2026-10-28 minimal-dynamics freeze evidence index (read-only)."""
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
MILESTONE=ROOT/'artifacts/milestones/lotusim_minimal_dynamics_20261028'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rel(path):
    return Path(path).resolve().relative_to(ROOT).as_posix()


def add_file(files,path):
    p=ROOT/path
    if p.is_file(): files[path]=sha(p)


def acceptance_hashes(directory,labels):
    runs={}
    for label in labels:
        p=ROOT/directory/label/'acceptance.json'
        if p.is_file(): runs[f'{directory}/{label}/acceptance.json']=sha(p)
    return runs


def suite_entry(identifier,category,directory,description):
    directory=Path(directory).as_posix()
    suite_path=ROOT/directory/'suite.json'
    suite=json.loads(suite_path.read_text(encoding='utf-8'))
    labels=list(suite.get('runs',{}))
    runs=acceptance_hashes(directory,labels)
    accepted=sum(1 for p in runs if json.loads((ROOT/p).read_text(encoding='utf-8')).get('passed') is True)
    return {'id':identifier,'category':category,'path':directory,'description':description,
            'result':{'suite_passed':suite.get('passed'),'runs_total':len(labels),
                      'runs_passed':accepted,'all_runs_passed':suite.get('all_runs_passed')},
            'files':{f'{directory}/suite.json':sha(suite_path)},'runs':runs}


def numeric_entry(identifier,directory,description):
    directory=Path(directory).as_posix(); files={}
    add_file(files,f'{directory}/acceptance.json'); add_file(files,f'{directory}/manifest.json')
    result={}
    p=ROOT/directory/'acceptance.json'
    if p.is_file():
        data=json.loads(p.read_text(encoding='utf-8')); result={'passed':data.get('passed')}
    return {'id':identifier,'category':'numerics','path':directory,'description':description,
            'result':result,'files':files,'runs':{}}


def replay_entry(identifier,directory,description):
    directory=Path(directory).as_posix(); files={}
    for name in ('summary.json','gazebo.log','state/worlds/replay.world'):
        add_file(files,f'{directory}/{name}')
    result={}
    p=ROOT/directory/'summary.json'
    if p.is_file():
        data=json.loads(p.read_text(encoding='utf-8'))
        result={'status':data.get('status'),'commands':data.get('commands'),
                'observations':data.get('observations'),'source_unchanged':data.get('source_unchanged')}
    return {'id':identifier,'category':'display','path':directory,'description':description,
            'result':result,'files':files,'runs':{}}


def main():
    evidence=[
        suite_entry('bridge-minimal-xdyn','bridge','artifacts/minimal_xdyn/validation_20261020_01',
                    'Minimal LOTUSim/xdyn bridge: allocation fixed cases, vertical plane, no current; 8+1 repeat.'),
        suite_entry('model-v2-open-loop','model','artifacts/twin_axial_v2/validation_20261026_01',
                    'uuv_twin_axial_v2 nine-case open-loop suite: allocation, limits, directions, Gazebo mirror, repeatability.'),
        numeric_entry('numerics-v2','artifacts/twin_axial_v2_numerics/validation_20261026_01',
                      'M4/M5/M6 no-current numerical checks after the added-mass Coriolis revision.'),
        numeric_entry('numerics-current','artifacts/twin_axial_current/numerics_20261027_final',
                      'F0/F1/F2 current/relative-velocity coordinates, stationary drift, co-moving case, RK derivative and coordinate round-trip.'),
        suite_entry('pid-steps','pid','artifacts/twin_axial_pid/validation_20261027_final',
                    'C1 speed step, C2 positive/negative pitch steps and supplemental allocation recovery.'),
        suite_entry('motion-fault','fault','artifacts/twin_axial_motion/validation_20261027_final',
                    'Straight/depth motion and state_loss/stale_state/command_loss boundary faults.'),
        suite_entry('navigation-s1','navigation','artifacts/twin_axial_current/navigation_20261027_final',
                    'S1 single-baffle constant-current navigation: F0/P0, F1/F2 P0-P4, repeats.'),
        suite_entry('navigation-s2-scaled','navigation','artifacts/twin_axial_current_double/navigation_20261028_final',
                    'Scaled S2 double-baffle constant-current navigation: F0/P0, F1/F2 P0-P4, repeats.'),
        replay_entry('replay-s1','artifacts/twin_axial_current_replays/verify_flow_retry',
                     'Display-only S1 replay with baffle/path/flow arrow/drifter; not acceptance evidence.'),
        replay_entry('replay-s2','artifacts/twin_axial_current_double_replays/verify_headless_F2_P0',
                     'Display-only scaled-S2 replay with two baffles/path/flow arrow/drifter; not acceptance evidence.'),
    ]
    docs={}
    for name in ('lotusim_minimal_dynamics_milestone_20261028.md','lotusim_integration_and_stage_report_20261027.md',
                 'lotusim_twin_axial_v2.md','lotusim_twin_axial_numerical_validation.md',
                 'lotusim_twin_axial_pid.md','lotusim_twin_axial_motion.md',
                 'lotusim_minimal_xdyn.md','lotusim_twin_axial_navigation.md',
                 'lotusim_twin_axial_current.md','lotusim_twin_axial_current_double.md',
                 'lotusim_recorded_current_suite_replay.md','dissertation_scope_baseline_v1.md',
                 'dissertation_model_acceptance_v1.md'):
        add_file(docs,f'docs/{name}')
    evidence.append({'id':'docs','category':'documentation','path':'docs','description':'Milestone report and stage evidence documents.',
                     'result':{'files':len(docs)},'files':docs,'runs':{}})
    code={}
    for name in ('ros2/twin_axial_v2/model.py','ros2/twin_axial_v2/allocation.py','ros2/twin_axial_v2/gateway.py',
                 'ros2/twin_axial_pid/control.py','ros2/twin_axial_motion/protocol.py','ros2/twin_axial_motion/safety.py',
                 'ros2/twin_axial_navigation/mission.py','ros2/twin_axial_current/model.py','ros2/twin_axial_current/mission.py',
                 'ros2/twin_axial_current_double/mission.py','ros2/twin_axial_current_double/acceptance.py',
                 'ros2/twin_axial_current_double/trial.py','ros2/twin_axial_current_double/cli.py',
                 'ros2/recorded_replay/core.py','ros2/recorded_replay/cli.py',
                 'tests/test_recorded_replay.py','tests/test_twin_axial_current.py','tests/test_twin_axial_current_double.py'):
        add_file(code,name)
    evidence.append({'id':'code','category':'code','path':'repo','description':'Freeze-time implementation and scenario files.',
                     'result':{'files':len(code)},'files':code,'runs':{}})

    protected={}; all_unchanged=True
    for name in ('twin_protected_before.json','twin_axial_v2_protected_before.json','pid_protected_before.json',
                 'motion_protected_before.json','navigation_protected_before.json','current_protected_before.json'):
        path=ROOT/'artifacts'/name
        data=json.loads(path.read_text(encoding='utf-8'))
        missing=[]; changed=[]
        for relative,expected in data.items():
            p=ROOT/relative.replace('\\','/')
            if not p.is_file(): missing.append(relative)
            elif sha(p)!=expected: changed.append(relative)
        all_unchanged=all_unchanged and not missing and not changed
        protected[name]={'sha256':sha(path),'entries':len(data),'missing':len(missing),'changed':len(changed),
                         'unchanged':not missing and not changed}
    index={
        'schema':1,
        'milestone':{'id':'lotusim-minimal-dynamics-20261028','date':'2026-10-28',
                     'status':'frozen_within_scope','scope':'Synthetic twin-axial UUV: model/numerics/open-loop/PID/fault/S1/S2-scaled navigation.',
                     'code_changed_by_freeze':False,'acceptance_changed_by_freeze':False,'frozen_data_changed_by_freeze':False,
                     'generated_at_utc':datetime.now(timezone.utc).isoformat()},
        'c1_c6':{
            'C1':{'status':'satisfied_scope_limited','evidence':['bridge-minimal-xdyn','model-v2-open-loop','numerics-v2','numerics-current'],
                  'gap':'Synthetic longitudinal model; not identified/full 6-DoF.'},
            'C2':{'status':'partial','evidence':['navigation-s1','navigation-s2-scaled'],
                  'gap':'Full-height S2, S3, complete 45-run matrix.'},
            'C3':{'status':'satisfied_scope_limited','evidence':['bridge-minimal-xdyn','pid-steps','motion-fault','navigation-s1','navigation-s2-scaled'],
                  'gap':'Gateway crash/network/hardware/pause fault matrix.'},
            'C4':{'status':'partial','evidence':['navigation-s1','navigation-s2-scaled'],
                  'gap':'S3, full-height S2, complete N1-N6 matrix.'},
            'C5':{'status':'partial','evidence':['navigation-s1','navigation-s2-scaled'],
                  'gap':'45-run matrix, stronger/other currents, sensitivity.'},
            'C6':{'status':'partial','evidence':['docs','code','protected_code'],
                  'gap':'Dissertation, final figures, independent-machine reproduction.'}},
        'evidence':evidence,
        'protected_code':{'all_unchanged':all_unchanged,'lists':protected},
        'tests':{'command':'python -m pytest tests/test_recorded_replay.py tests/test_twin_axial_current.py tests/test_twin_axial_current_double.py -q',
                 'passed':37,'breakdown':{'recorded_replay':15,'twin_axial_current':19,'twin_axial_current_double':3},
                 'note':'Freeze-time read-only test run; not a rerun of the Gazebo experiments.'},
        'remaining_tasks':[
            'Full-height S2 and S3: decide whether to change speed/pitch/actuator limits or freeze a documented alternative; do not pass off the scaled S2.',
            'Complete 45-run matrix (3 scenes x 3 currents x 5 initials) and N1-N6/R1-R2; no retries replacing failed cases.',
            'Complete C5 current/sensitivity matrix: stronger/other flows, damping sensitivity, shear/turbulence out of scope.',
            'Complete C3 fault matrix: gateway crash, network interruption, paused clock, hardware/actuator faults; current evidence is message-boundary only.',
            'Identify/validate a real vehicle model if required; current model is synthetic.',
            'Finish C6: dissertation, figures, defense package, independent-machine reproduction and GUI visibility confirmation.']
    }
    (MILESTONE/'evidence_index.json').write_text(json.dumps(index,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps({'passed':all_unchanged,'evidence':len(evidence),'protected':len(protected),
                      'output':str(MILESTONE/'evidence_index.json')},indent=2))


if __name__=='__main__':
    main()
