"""Reuse frozen A*, shortcut and G12 loop with scoped geometry-only injection."""
import argparse
import hashlib
import json
import math
import os
import shutil
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path as FilePath
from unittest.mock import patch
from xml.etree import ElementTree as ET

import numpy as np
from core import Path, Pose
from environment import Boundary, Obstacle, WorldModel
from planner import AStarPlanner, AStarConfig, PlanningConstraints, PlanningRequest, EnvironmentPlanningSpace, validate_path
from planner.candidates import simplify_path
from trajectory import generate_trajectory, TrajectoryConfig
from ros2.twin_axial_navigation.mission import initial, segment_clearance
from ros2.twin_axial_g12_matrix import runner
from ros2.twin_axial_g12_boundary_v3.gate import contract

ROOT = FilePath(__file__).resolve().parents[2]
SPEC = {
    'scene': 'S4_multi_island_wide_v1', 'boundary_xz': [0, -20, 20, 0],
    'start': [2, -10], 'goal': [18, -10],
    'obstacles_xmin_xmax_zmin_zmax': [[8,10,-11,-9], [4,6,-18,-14],
                                   [13,15,-6,-2], [13,15,-18,-14], [3,5,-6,-2]],
    'grid_resolution_m': .5, 'connectivity': 8, 'planning_clearance_m': 1.5,
    'sample_spacing_m': .05, 'corner_trim_m': 1.5,
    'trim_factors': [1, .5, .25, .125], 'bezier_intervals': 30,
    'variants': ['A_raw_astar', 'B_safe_shortcut', 'C_safe_smoothing'],
    'runs': 'Exactly one S4/F0/P0 per geometrically valid variant; no retries or tuning',
    'stop': 'Retain geometry rejections and dynamic failures; never alter scene or thresholds',
    'comparison': 'B vs A: shortcut effect; C vs B: smoothing effect; identical G12 and sampling',
    'extension_rule': 'Consider only if C passes all original Python checks, is safe, and improves tracking or arrival relative to A/B; otherwise report limitations before any new study',
}

def sha(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
    return h.hexdigest()

def link_metadata(path):
    s=path.lstat()
    data={'size':s.st_size,'mtime_ns':s.st_mtime_ns,'attributes':s.st_file_attributes & 0x410,
          'reparse_tag':s.st_reparse_tag}
    try: data['target']=os.readlink(path)
    except (OSError,ValueError): data['target']='WSL reparse point: Windows cannot resolve; target content not verified'
    return data

def write(path, data):
    with FilePath(path).open('x',encoding='utf-8') as f:
        json.dump(data,f,indent=2,ensure_ascii=False,default=lambda x:x.item())

def world():
    return WorldModel(Boundary.rectangle(x_min=0,x_max=20,z_min=-20,z_max=0),
        [Obstacle.rectangle(f'S4_{i}',x_min=a,x_max=b,z_min=c,z_max=d)
         for i,(a,b,c,d) in enumerate(SPEC['obstacles_xmin_xmax_zmin_zmax'])])

def world_xml():
    root=ET.Element('sdf',version='1.7'); w=ET.SubElement(root,'world',name='S4')
    for i,(a,b,c,d) in enumerate(SPEC['obstacles_xmin_xmax_zmin_zmax']):
        m=ET.SubElement(w,'model',name=f'double_baffle_obstacle_{i}')
        ET.SubElement(m,'pose').text=f'{(a+b)/2} 0 {(c+d)/2-20} 0 0 0'
        link=ET.SubElement(m,'link',name='base_link')
        geom=ET.SubElement(ET.SubElement(link,'collision',name='box'),'geometry')
        ET.SubElement(ET.SubElement(geom,'box'),'size').text=f'{b-a} 4 {d-c}'
    return ET.tostring(root,encoding='unicode')

def points(path): return np.array([[p.x,p.z] for p in path],float)
def path_of(coords): return Path.from_list([Pose(x=float(x),z=float(z)) for x,z in coords])

def smooth(path,space,constraints,request):
    """Quadratic corner rounding, deterministic bounded backoff, safe fallback.

    Executed reference is the checked polyline, not an unchecked analytic curve.
    This generalizes the old baffle-specific quadratic construction only.
    """
    src=points(simplify_path(path)); output=[src[0]]; audit=[]
    for i in range(1,len(src)-1):
        a,b,c=src[i-1:i+2]; left=np.linalg.norm(a-b); right=np.linalg.norm(c-b)
        base=min(SPEC['corner_trim_m'],.4*left,.4*right)
        accepted=False; attempts=[]
        for factor in SPEC['trim_factors']:
            cut=base*factor; entry=b+(a-b)*cut/left; leave=b+(c-b)*cut/right
            curve=[(1-t)**2*entry+2*t*(1-t)*b+t*t*leave
                   for t in np.linspace(0,1,SPEC['bezier_intervals']+1)]
            local=path_of([output[-1],*curve,c])
            valid=validate_path(local,space,constraints).valid
            attempts.append({'trim_m':float(cut),'valid':bool(valid)})
            if valid:
                output.extend(curve); accepted=True; break
        if not accepted: output.append(b)
        audit.append({'corner':i,'smoothed':accepted,'attempts':attempts})
    output.append(src[-1]); result=path_of(output)
    if not validate_path(result,space,constraints,request=request).valid:
        raise ValueError('Final smoothing geometry rejected')
    return result,audit

def geometric(path,w):
    xy=points(path); d=np.diff(xy,axis=0); lengths=np.linalg.norm(d,axis=1)
    angles=np.arctan2(d[:,1],d[:,0]); turns=np.abs(np.arctan2(np.sin(np.diff(angles)),np.cos(np.diff(angles))))
    return {'length_m':float(sum(lengths)), 'turn_count':int(sum(turns>1e-8)),
            'total_turn_deg':float(np.degrees(sum(turns))),
            'max_turn_deg':float(np.degrees(max(turns,default=0))),
            'min_clearance_m':min(segment_clearance(w,a,b) for a,b in zip(path,path.poses[1:])),
            'vertices':len(path)}

def register(output):
    output.mkdir(parents=True,exist_ok=False)
    contract()  # Read-only frozen v6 / boundary release verification.
    source={}
    for folder in ['core','environment','planner','trajectory','navigation','guidance','controller','physics','simulation','ros2']:
        for p in sorted((ROOT/folder).rglob('*.py')):
            name=p.relative_to(ROOT).as_posix(); source[name]=sha(p)
            target=output/'source'/name; target.parent.mkdir(parents=True,exist_ok=True); shutil.copyfile(p,target)
    write(output/'source_manifest.json',source)
    write(output/'preregistration.json',{'registered_utc':datetime.now(timezone.utc).isoformat(),
          'specification':SPEC,'g12':runner.g12_config(),'thresholds':runner.study.THRESHOLDS,
          'source_manifest_sha256':sha(output/'source_manifest.json'), 'real_started':False})
    # Inventory every pre-existing artifact byte; this directory is excluded.
    protected={}; links={}
    for p in sorted((ROOT/'artifacts').rglob('*')):
        if output in p.parents: continue
        if p.lstat().st_file_attributes & 0x400:
            links[p.relative_to(ROOT).as_posix()]=link_metadata(p)
            continue
        if p.is_file():
            protected[p.relative_to(ROOT).as_posix()]=sha(p)
    write(output/'old_artifacts_before.json',protected)
    write(output/'old_links_before.json',links)
    print(json.dumps({'registered':str(output),'protected_artifact_files':len(protected)}),flush=True)

def run(output):
    registration=json.loads((output/'preregistration.json').read_text(encoding='utf-8'))
    assert registration['specification']==SPEC and registration['g12']==runner.g12_config()
    manifest=json.loads((output/'source_manifest.json').read_text(encoding='utf-8'))
    assert all(sha(ROOT/n)==h for n,h in manifest.items())
    w=world(); space=EnvironmentPlanningSpace(w); constraints=PlanningConstraints(.7,.8,goal_tolerance=0.)
    request=PlanningRequest(initial('P0'),Pose(x=18,z=-10),constraints)
    t=time.perf_counter(); result=AStarPlanner(AStarConfig(resolution=.5,connectivity=8)).plan(request,space)
    search_seconds=time.perf_counter()-t
    write(output/'search.json',{'success':result.success,'message':result.message,'expanded_nodes':result.expanded_nodes,'seconds':search_seconds})
    if not result.success: raise RuntimeError('A* rejected; see search.json')
    raw=result.path
    t=time.perf_counter()
    shortcut=simplify_path(generate_trajectory(raw,space,constraints,TrajectoryConfig(sample_spacing=.05),request=request).shortcut_path)
    shortcut_seconds=time.perf_counter()-t
    t=time.perf_counter(); rounded,audit=smooth(shortcut,space,constraints,request); smooth_seconds=time.perf_counter()-t
    paths=[raw,shortcut,rounded]; plans={}
    for name,path,post in zip(SPEC['variants'],paths,[0,shortcut_seconds,shortcut_seconds+smooth_seconds]):
        dense=generate_trajectory(path,space,constraints,TrajectoryConfig(sample_spacing=.05,shortcut=False),request=request).sampled_path
        plans[name]={'path':points(dense).tolist(),'geometry':geometric(path,w),
                     'search_seconds_shared':search_seconds,'postprocessing_seconds':post,
                     'expanded_nodes_shared':result.expanded_nodes,'smoothing_audit':audit if name.startswith('C') else []}
    write(output/'plans.json',plans)  # All plans fixed before any dynamics.
    reports={}
    for name,plan in plans.items():
        try:
            # Only path/SDF/preflight diagnostic are supplied. Frozen run_case,
            # Guide, dynamics, integration, PID, allocation and checks are reused.
            with patch.dict(runner.study.SCENES,{'S4':SPEC['obstacles_xmin_xmax_zmin_zmax']}), \
                 patch.object(runner.study,'build_path',return_value=(np.array(plan['path']),plan['geometry'])), \
                 patch.object(runner.study,'world_xml',return_value=world_xml()), \
                 patch.object(runner.study,'preflight',return_value={'passed':None,'reason':'S1-S3 topology-specific steady certificate not applicable to S4; full geometric check passed'}):
                reports[name]=runner.run_case(output/name,'S4','F0','P0',sha(output/'source_manifest.json'))
        except Exception:
            reports[name]={'passed':False,'exception':traceback.format_exc()}
            write(output/(name+'_exception.json'),reports[name])
    write(output/'results.json',reports)
    contract()
    old=json.loads((output/'old_artifacts_before.json').read_text(encoding='utf-8'))
    differences=[n for n,h in {**old,**manifest}.items() if not (ROOT/n).is_file() or sha(ROOT/n)!=h]
    links=json.loads((output/'old_links_before.json').read_text(encoding='utf-8'))
    differences.extend(n for n,metadata in links.items() if link_metadata(ROOT/n)!=metadata)
    write(output/'preservation.json',{'passed':not differences,'old_artifact_files':len(old),'source_files':len(manifest),'reparse_metadata_unchanged':len(links),'reparse_target_content_verified':False,'differences':differences,'real_started':False})
    print(json.dumps({'completed':len(reports),'passed':sum(r['passed'] for r in reports.values()),'preservation':not differences}),flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('action',choices=['register','run']); parser.add_argument('output')
    args=parser.parse_args(); output=FilePath(args.output).resolve()
    (register if args.action=='register' else run)(output)
