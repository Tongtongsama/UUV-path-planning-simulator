"""Seventeen new trials, one provenance-bound reference; no real backend."""
import argparse
import json
import shutil
import traceback
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import patch
import numpy as np
from core import Pose
from planner import AStarPlanner,AStarConfig,PlanningConstraints,PlanningRequest,EnvironmentPlanningSpace
from planner.candidates import simplify_path
from trajectory import generate_trajectory,TrajectoryConfig
from ros2.twin_axial_navigation.mission import initial
from ros2.twin_axial_s4_planning_v1 import study as base
from ros2.twin_axial_g12_matrix import runner
from ros2.twin_axial_g12_matrix.cli import repeatability,rows
from ros2.twin_axial_s3_transient_v2 import cli as verifier

ROOT=base.ROOT
REFERENCE=ROOT/'artifacts/twin_axial_s4_planning_20261116_04'
CASES=('P0','P1','P2','P3','P4','P0_repeat')
KEYS=tuple(f'{flow}/{case}' for flow in runner.FLOWS for case in CASES)
NEW=tuple(k for k in KEYS if k!='F0/P0')
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8'))
write=base.write
sha=base.sha

def plan(case):
    case='P0' if case=='P0_repeat' else case
    space=EnvironmentPlanningSpace(base.world()); constraints=PlanningConstraints(.7,.8,goal_tolerance=0.)
    request=PlanningRequest(initial(case),Pose(x=18,z=-10),constraints)
    result=AStarPlanner(AStarConfig(resolution=.5,connectivity=8)).plan(request,space)
    if not result.success: raise ValueError('A* rejected: '+str(result.message))
    short=simplify_path(generate_trajectory(result.path,space,constraints,TrajectoryConfig(sample_spacing=.05),request=request).shortcut_path)
    dense=generate_trajectory(short,space,constraints,TrajectoryConfig(sample_spacing=.05,shortcut=False),request=request).sampled_path
    return {'path':base.points(dense).tolist(),'geometry':base.geometric(short,base.world()),'expanded_nodes':result.expanded_nodes}

def verify(folder,p):
    with patch.dict(verifier.SCENES,{'S4':base.SPEC['obstacles_xmin_xmax_zmin_zmax']}), \
         patch.object(verifier,'build_path',return_value=(np.array(p['path']),{'clearance':p['geometry']['min_clearance_m']})), \
         patch.object(verifier,'world_xml',return_value=base.world_xml()):
        return verifier.verify_run(folder,replay=True)

def register(out):
    out.mkdir(parents=True,exist_ok=False); base.contract()
    old_source=read(REFERENCE/'source_manifest.json')
    assert all(sha(ROOT/n)==h and sha(REFERENCE/'source'/n)==h for n,h in old_source.items())
    assert all(sha(REFERENCE/n)==h for n,h in read(REFERENCE/'final_manifest.json').items())
    reference_plan=read(REFERENCE/'plans.json')['B_safe_shortcut']
    assert plan('P0')['path']==reference_plan['path']
    assert read(REFERENCE/'B_safe_shortcut/report.json')['parameters']==runner.g12_config()
    ref_verify=verify(REFERENCE/'B_safe_shortcut',reference_plan)
    assert ref_verify['passed']
    files=dict(old_source)
    for p in Path(__file__).parent.glob('*.py'):files[p.relative_to(ROOT).as_posix()]=sha(p)
    for n in files:
        dest=out/'source'/n;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/n,dest)
    write(out/'source_manifest.json',files)
    write(out/'preregistration.json',{
        'registered_utc':datetime.now(timezone.utc).isoformat(),'study':'S4/B 17 additional Python trials',
        'specification':base.SPEC,'g12':runner.g12_config(),'thresholds':runner.study.THRESHOLDS,
        'flows':runner.CURRENT_FLOWS,'initials':runner.INITIALS,'new_runs':NEW,'all_entries':KEYS,
        'reference':'F0/P0 copied unchanged from '+str(REFERENCE/'B_safe_shortcut'),
        'reference_manifest_sha256':sha(REFERENCE/'B_safe_shortcut/manifest.json'),
        'source_manifest_sha256':sha(out/'source_manifest.json'),
        'path_policy':'Same frozen B planning rules from each actual initial position; identical path across flows for a given initial; no parameter tuning',
        'repeat_policy':'Existing G12 matrix repeatability: both runs individually pass and are replay-verified, original R2 position/time limits unchanged',
        'failure_policy':'One attempt each; keep geometric rejection, exception and full failed prefix; do not replace results',
        'decision_policy':'Recommend bounded real pilot only if all 18 individual checks, all replay verifications and all three repeat checks pass; actual launch additionally needs a dedicated S4 adapter and live gate',
        'real_started':False})
    target=out/'F0/P0';target.parent.mkdir(parents=True);shutil.copytree(REFERENCE/'B_safe_shortcut',target)
    write(out/'reference_verification.json',ref_verify)
    old={};links={}
    for p in sorted((ROOT/'artifacts').rglob('*')):
        if out in p.parents:continue
        if p.lstat().st_file_attributes & 0x400:links[p.relative_to(ROOT).as_posix()]=base.link_metadata(p)
        elif p.is_file():old[p.relative_to(ROOT).as_posix()]=sha(p)
    write(out/'old_artifacts_before.json',old);write(out/'old_links_before.json',links)
    print(json.dumps({'registered':str(out),'new_runs':len(NEW),'reference_entries':1,'protected_files':len(old)}),flush=True)

def run(out):
    reg=read(out/'preregistration.json');files=read(out/'source_manifest.json')
    assert reg['g12']==runner.g12_config() and reg['specification']==base.SPEC
    assert all(sha(ROOT/n)==h and sha(out/'source'/n)==h for n,h in files.items())
    assert reg['source_manifest_sha256']==sha(out/'source_manifest.json')
    plans={};rejections={}
    for case in CASES:
        try:plans[case]=plan(case)
        except Exception:rejections[case]=traceback.format_exc()
    assert plans['P0']['path']==read(out/'F0/P0/plan.json')['path']
    write(out/'plans.json',plans);write(out/'geometry_rejections.json',rejections)
    for key in NEW:
        flow,case=key.split('/')
        if case in rejections:
            write(out/(key.replace('/','_')+'_rejection.json'),{'passed':False,'reason':rejections[case]});continue
        p=plans[case]
        try:
            with patch.dict(runner.study.SCENES,{'S4':base.SPEC['obstacles_xmin_xmax_zmin_zmax']}), \
                 patch.object(runner.study,'build_path',return_value=(np.array(p['path']),p['geometry'])), \
                 patch.object(runner.study,'world_xml',return_value=base.world_xml()), \
                 patch.object(runner.study,'preflight',return_value={'passed':None,'reason':'S4 geometry checked; old S1-S3 steady certificate not applicable'}):
                runner.run_case(out/key,'S4',flow,case,sha(out/'source_manifest.json'))
        except Exception:write(out/(key.replace('/','_')+'_exception.json'),{'passed':False,'exception':traceback.format_exc()})
    verified={}
    for key in KEYS:
        try:
            flow,case=key.split('/');p=plans[case];saved=read(out/key/'plan.json');report=read(out/key/'report.json')
            assert saved['config']==runner.g12_config()==report['parameters']
            assert saved['path']==p['path'] and saved['flow']==flow and saved['run_case']==case
            expected=read(REFERENCE/'B_safe_shortcut/report.json')['source_sha256'] if key=='F0/P0' else sha(out/'source_manifest.json')
            assert report['source_sha256']==expected
            verified[key]=verify(out/key,p)
        except Exception:verified[key]={'passed':False,'record_verified':False,'exception':traceback.format_exc()}
        write(out/(key.replace('/','_')+'_verification.json'),verified[key])
    repeats={}
    for flow in runner.FLOWS:
        a,b=f'{flow}/P0',f'{flow}/P0_repeat'
        try:
            repeats[flow]=repeatability(rows(out/a),rows(out/b),verified[a],verified[b])
            repeats[flow]['states_byte_identical']=sha(out/a/'states.jsonl')==sha(out/b/'states.jsonl')
        except Exception:repeats[flow]={'passed':False,'exception':traceback.format_exc()}
    write(out/'verification.json',verified);write(out/'repeatability.json',repeats)
    passed=all(v['passed'] and v.get('record_verified') for v in verified.values()) and all(r['passed'] for r in repeats.values())
    write(out/'decision.json',{'all_18_python_passed':passed,'new_runs_requested':17,'reference_entries':1,
          'worth_real_pilot':passed,'real_gate_ready':False,'real_started':False,
          'reason':'A dedicated S4 real adapter, evidence binding and fresh transport/pose/MAS/shutdown gate are still required; no inherited S1-S3 gate approval'})
    base.contract();old=read(out/'old_artifacts_before.json');links=read(out/'old_links_before.json')
    changed=[n for n,h in {**old,**files}.items() if not (ROOT/n).is_file() or sha(ROOT/n)!=h]
    changed.extend(n for n,m in links.items() if base.link_metadata(ROOT/n)!=m)
    write(out/'preservation.json',{'passed':not changed,'old_artifact_files':len(old),'source_files':len(files),
        'stable_reparse_metadata_count':len(links),'reparse_target_content_verified':False,'differences':changed})
    print(json.dumps({'verified_passed':sum(v['passed'] for v in verified.values()),'repeat_passed':sum(v['passed'] for v in repeats.values()),'preservation':not changed}),flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=['register','run']);parser.add_argument('output')
    args=parser.parse_args();(register if args.action=='register' else run)(Path(args.output).resolve())
