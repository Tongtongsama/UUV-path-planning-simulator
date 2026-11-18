import argparse
import json
import shutil
import traceback
from datetime import datetime,timezone
from pathlib import Path
from unittest.mock import patch
import numpy as np
from ros2.twin_axial_s4_terminal_candidate_v1 import cli as candidate
from ros2.twin_axial_s4_validation_v1.cli import verify,read,write,sha
from ros2.twin_axial_s4_planning_v1 import study as base
from ros2.twin_axial_g12_matrix import runner
from ros2.twin_axial_g12_matrix.cli import repeatability,rows
from ros2.twin_axial_navigation.mission import initial

ROOT=base.ROOT
PARENT=ROOT/'artifacts/twin_axial_s4_terminal_candidate_20261117_01'
CASES=('P0','P1','P2','P3','P4','P0_repeat')
KEYS=tuple(f'{f}/{c}' for f in runner.FLOWS for c in CASES)
NEW=tuple(k for k in KEYS if not k.endswith('/P0'))

def plan(case):
    # Geometry input only. Reuse the candidate implementation byte-for-byte.
    state=initial('P0' if case=='P0_repeat' else case)
    with patch.object(candidate,'initial',return_value=state):
        return candidate.make_plan()

def register(out):
    out.mkdir(parents=True,exist_ok=False);base.contract()
    files=read(PARENT/'source_manifest.json')
    assert all(sha(ROOT/n)==h and sha(PARENT/'source'/n)==h for n,h in files.items())
    assert all(sha(PARENT/n)==h for n,h in read(PARENT/'final_manifest.json').items())
    assert read(PARENT/'decision.json')['worth_remaining_15']
    for p in Path(__file__).parent.glob('*.py'):files[p.relative_to(ROOT).as_posix()]=sha(p)
    for n in files:
        dst=out/'source'/n;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/n,dst)
    write(out/'source_manifest.json',files)
    write(out/'preregistration.json',{'registered_utc':datetime.now(timezone.utc).isoformat(),
        'candidate':candidate.SPEC,'scene':base.SPEC,'g12':runner.g12_config(),'thresholds':runner.study.THRESHOLDS,
        'new_entries':NEW,'all_entries':KEYS,'flows':runner.CURRENT_FLOWS,'initials':runner.INITIALS,
        'phase_policy':'Complete the 15 newly authorized attempts, preserve every failure/rejection; no retries or tuning. Earlier stop-on-first-failure policy applied to the completed three-P0 screening stage.',
        'path_policy':'Same candidate planner with actual initial input; shared path across flows for each initial; staging (14,-10) and goal (18,-10) unchanged',
        'references':{f:str(PARENT/f/'P0') for f in runner.FLOWS},
        'reference_manifests':{f:sha(PARENT/f/'P0/manifest.json') for f in runner.FLOWS},
        'source_manifest_sha256':sha(out/'source_manifest.json'),
        'decision_policy':'Recommend real pilot only after all 18 original acceptances and offline verifications and all three strict repeat checks pass; dedicated real adapter and fresh gate still required',
        'real_started':False})
    p=plan('P0');assert p['path']==read(PARENT/'plan.json')['path']
    for f in runner.FLOWS:
        checked=verify(PARENT/f/'P0',p);assert checked['passed']
        dst=out/f/'P0';dst.parent.mkdir(parents=True);shutil.copytree(PARENT/f/'P0',dst)
        write(out/(f+'_reference_verification.json'),checked)
    old={};links={}
    for p in sorted((ROOT/'artifacts').rglob('*')):
        if out in p.parents:continue
        if p.lstat().st_file_attributes & 0x400:links[p.relative_to(ROOT).as_posix()]=base.link_metadata(p)
        elif p.is_file():old[p.relative_to(ROOT).as_posix()]=sha(p)
    write(out/'old_artifacts_before.json',old);write(out/'old_links_before.json',links)
    print(json.dumps({'registered':True,'new_attempts':15,'audited_references':3,'protected_files':len(old)}),flush=True)

def run(out):
    reg=read(out/'preregistration.json');files=read(out/'source_manifest.json')
    assert reg['candidate']==candidate.SPEC and reg['g12']==runner.g12_config() and reg['thresholds']==runner.study.THRESHOLDS
    assert reg['source_manifest_sha256']==sha(out/'source_manifest.json')
    assert all(sha(ROOT/n)==h and sha(out/'source'/n)==h for n,h in files.items())
    plans={};rejections={}
    for c in CASES:
        try:plans[c]=plan(c)
        except Exception:rejections[c]=traceback.format_exc()
    write(out/'plans.json',plans);write(out/'geometry_rejections.json',rejections)
    for key in NEW:
        f,c=key.split('/')
        if c in rejections:
            write(out/(key.replace('/','_')+'_rejection.json'),{'passed':False,'reason':rejections[c]});continue
        p=plans[c]
        try:
            with patch.dict(runner.study.SCENES,{'S4':base.SPEC['obstacles_xmin_xmax_zmin_zmax']}), \
                 patch.object(runner.study,'build_path',return_value=(np.array(p['path']),p['geometry'])), \
                 patch.object(runner.study,'world_xml',return_value=base.world_xml()), \
                 patch.object(runner.study,'preflight',return_value={'passed':None,'reason':'S4 full geometry checked; S1-S3 certificate not applicable'}):
                runner.run_case(out/key,'S4',f,c,sha(out/'source_manifest.json'))
        except Exception:write(out/(key.replace('/','_')+'_exception.json'),{'passed':False,'exception':traceback.format_exc()})
    verified={}
    for key in KEYS:
        f,c=key.split('/')
        try:
            saved=read(out/key/'plan.json');report=read(out/key/'report.json')
            assert saved['path']==plans[c]['path'] and saved['config']==runner.g12_config()==report['parameters']
            assert saved['flow']==f and saved['run_case']==c and saved['case']==('P0' if c=='P0_repeat' else c)
            expected=sha(PARENT/'source_manifest.json') if c=='P0' else sha(out/'source_manifest.json')
            assert report['source_sha256']==expected
            verified[key]=verify(out/key,plans[c])
        except Exception:verified[key]={'passed':False,'record_verified':False,'exception':traceback.format_exc()}
        write(out/(key.replace('/','_')+'_verification.json'),verified[key])
    repeats={}
    for f in runner.FLOWS:
        a,b=f+'/P0',f+'/P0_repeat'
        try:
            repeats[f]=repeatability(rows(out/a),rows(out/b),verified[a],verified[b])
            repeats[f]['states_byte_identical']=sha(out/a/'states.jsonl')==sha(out/b/'states.jsonl')
        except Exception:repeats[f]={'passed':False,'exception':traceback.format_exc()}
    write(out/'verification.json',verified);write(out/'repeatability.json',repeats)
    passed=all(v['passed'] and v.get('record_verified') for v in verified.values()) and all(r['passed'] for r in repeats.values())
    write(out/'decision.json',{'all_18_original_passed':all(v['passed'] for v in verified.values()),
        'all_18_verified':all(v.get('record_verified') for v in verified.values()),
        'all_three_repeats_passed':all(r['passed'] for r in repeats.values()),
        'worth_real_pilot':passed,'real_gate_ready':False,'real_started':False,
        'reason':'Python robustness decision only; a dedicated terminal-candidate real adapter, source/configuration binding and fresh joint transport/pose/MAS/shutdown gate remain necessary'})
    base.contract();old=read(out/'old_artifacts_before.json');links=read(out/'old_links_before.json')
    changed=[n for n,h in {**old,**files}.items() if not (ROOT/n).is_file() or sha(ROOT/n)!=h]
    changed.extend(n for n,m in links.items() if base.link_metadata(ROOT/n)!=m)
    write(out/'preservation.json',{'passed':not changed,'old_artifact_files':len(old),'source_files':len(files),
        'stable_reparse_metadata_count':len(links),'reparse_target_content_verified':False,'differences':changed})
    print(json.dumps({'original_passed':sum(v['passed'] for v in verified.values()),'repeats_passed':sum(r['passed'] for r in repeats.values()),'preservation':not changed}),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['register','run']);p.add_argument('output')
    a=p.parse_args();(register if a.action=='register' else run)(Path(a.output).resolve())
