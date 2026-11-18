import argparse
import json
import shutil
import traceback
from datetime import datetime,timezone
from pathlib import Path
from unittest.mock import patch
import numpy as np
from core import Pose,Path as GeometryPath
from planner import AStarPlanner,AStarConfig,PlanningConstraints,PlanningRequest,EnvironmentPlanningSpace,validate_path
from planner.candidates import simplify_path
from trajectory import generate_trajectory,TrajectoryConfig
from ros2.twin_axial_navigation.mission import initial
from ros2.twin_axial_s4_planning_v1 import study as base
from ros2.twin_axial_s4_validation_v1.cli import verify,read,write,sha
from ros2.twin_axial_g12_matrix import runner

ROOT=base.ROOT
SPEC={'candidate':'S4_B_terminal_straight_v1','staging_point':[14.,-10.],'goal':[18.,-10.],
      'terminal_length_m':4.,'grid_resolution_m':.5,'connectivity':8,'planning_clearance_m':1.5,
      'sample_spacing_m':.05,'smoothing':False,
      'rule':'A* to staging point, existing safe shortcut and collinear reduction on prefix only, append goal; preserve staging vertex in final resampling',
      'order':['F0','F1','F2'],'case':'P0','stop':'Stop immediately after any geometry rejection, run failure or verification failure; no retries or replacement'}

def make_plan():
    space=EnvironmentPlanningSpace(base.world());limits=PlanningConstraints(.7,.8,goal_tolerance=0.)
    stage=Pose(x=14,z=-10);goal=Pose(x=18,z=-10)
    request=PlanningRequest(initial('P0'),stage,limits)
    result=AStarPlanner(AStarConfig(resolution=.5,connectivity=8)).plan(request,space)
    if not result.success:raise ValueError('Prefix A* rejected: '+str(result.message))
    short=simplify_path(generate_trajectory(result.path,space,limits,TrajectoryConfig(sample_spacing=.05),request=request).shortcut_path)
    combined=GeometryPath.from_list([*short.poses,goal])
    final_request=PlanningRequest(initial('P0'),goal,limits)
    assert validate_path(combined,space,limits,request=final_request).valid
    dense=generate_trajectory(combined,space,limits,TrajectoryConfig(sample_spacing=.05,shortcut=False),request=final_request).sampled_path
    return {'path':base.points(dense).tolist(),'anchors':base.points(combined).tolist(),
            'geometry':base.geometric(combined,base.world()),'expanded_nodes':result.expanded_nodes}

def register(out):
    out.mkdir(parents=True,exist_ok=False);base.contract()
    parent=ROOT/'artifacts/twin_axial_s4_validation_20261117_01'
    files=read(parent/'source_manifest.json')
    assert all(sha(ROOT/n)==h and sha(parent/'source'/n)==h for n,h in files.items())
    assert all(sha(parent/n)==h for n,h in read(parent/'final_manifest.json').items())
    for p in Path(__file__).parent.glob('*.py'):files[p.relative_to(ROOT).as_posix()]=sha(p)
    for n in files:
        dst=out/'source'/n;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/n,dst)
    write(out/'source_manifest.json',files)
    write(out/'preregistration.json',{'registered_utc':datetime.now(timezone.utc).isoformat(),
        'candidate':SPEC,'unchanged_scene':base.SPEC,'g12':runner.g12_config(),'thresholds':runner.study.THRESHOLDS,
        'source_manifest_sha256':sha(out/'source_manifest.json'),'real_started':False,
        'interpretation':'Staging point is a geometry constraint, not a second arrival target. One candidate only; all flows share the same path.',
        'extension':'Only three original acceptances and offline verifications all pass permits recommending remaining 15; no automatic expansion'})
    old={};links={}
    for p in sorted((ROOT/'artifacts').rglob('*')):
        if out in p.parents:continue
        if p.lstat().st_file_attributes & 0x400:links[p.relative_to(ROOT).as_posix()]=base.link_metadata(p)
        elif p.is_file():old[p.relative_to(ROOT).as_posix()]=sha(p)
    write(out/'old_artifacts_before.json',old);write(out/'old_links_before.json',links)
    print('Registered one fixed candidate; no planning or dynamics executed.',flush=True)

def run(out):
    reg=read(out/'preregistration.json');files=read(out/'source_manifest.json')
    assert reg['candidate']==SPEC and reg['g12']==runner.g12_config()
    assert reg['source_manifest_sha256']==sha(out/'source_manifest.json')
    assert all(sha(ROOT/n)==h and sha(out/'source'/n)==h for n,h in files.items())
    outcomes={};reason=None
    try:
        p=make_plan();write(out/'plan.json',p)
        write(out/'geometry_check.json',{'passed':True,'min_clearance_m':p['geometry']['min_clearance_m'],
             'staging_vertex_retained':[14.,-10.] in p['path'],'terminal_length_m':4.,'terminal_angle_deg':0.})
    except Exception:
        reason='geometry_rejected';write(out/'geometry_check.json',{'passed':False,'exception':traceback.format_exc()})
    if reason is None:
        for flow in SPEC['order']:
            try:
                with patch.dict(runner.study.SCENES,{'S4':base.SPEC['obstacles_xmin_xmax_zmin_zmax']}), \
                     patch.object(runner.study,'build_path',return_value=(np.array(p['path']),p['geometry'])), \
                     patch.object(runner.study,'world_xml',return_value=base.world_xml()), \
                     patch.object(runner.study,'preflight',return_value={'passed':None,'reason':'S4 full geometry checked; S1-S3 certificate not applicable'}):
                    report=runner.run_case(out/flow/'P0','S4',flow,'P0',sha(out/'source_manifest.json'))
                result=verify(out/flow/'P0',p);write(out/(flow+'_verification.json'),result)
                outcomes[flow]={'passed':report['passed'] and result['passed'],'verified':result['record_verified'],'metrics':result['metrics']}
                if not outcomes[flow]['passed']:
                    reason=flow+'_original_acceptance_failed';break
            except Exception:
                reason=flow+'_exception';outcomes[flow]={'passed':False,'exception':traceback.format_exc()}
                write(out/(flow+'_exception.json'),outcomes[flow]);break
    write(out/'decision.json',{'outcomes':outcomes,'stop_reason':reason,
          'not_run':[f for f in SPEC['order'] if f not in outcomes],
          'worth_remaining_15':reason is None and len(outcomes)==3,'real_started':False,'real_gate_ready':False})
    base.contract();old=read(out/'old_artifacts_before.json');links=read(out/'old_links_before.json')
    changed=[n for n,h in {**old,**files}.items() if not (ROOT/n).is_file() or sha(ROOT/n)!=h]
    changed.extend(n for n,m in links.items() if base.link_metadata(ROOT/n)!=m)
    write(out/'preservation.json',{'passed':not changed,'old_artifact_files':len(old),'source_files':len(files),
          'stable_reparse_metadata_count':len(links),'reparse_target_content_verified':False,'differences':changed})
    print(json.dumps({'outcomes':outcomes,'stop_reason':reason,'preservation':not changed}),flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=['register','run']);parser.add_argument('output')
    a=parser.parse_args();(register if a.action=='register' else run)(Path(a.output).resolve())
