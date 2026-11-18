import json
from dataclasses import dataclass
from pathlib import Path
from ros2.twin_axial_g12_link_v6 import gate as parent
from ros2.twin_axial_g12_link_v6.acceptance import numerically_equal
from ros2.twin_axial_s4_terminal_validation_v1.cli import ROOT,PARENT,KEYS,plan,verify,read,write,sha
from ros2.twin_axial_g12_matrix.cli import rows,repeatability
from .mission import Mission,make_plan,world_xml

DATA=ROOT/'artifacts/twin_axial_s4_terminal_validation_20261117_01'
RELEASE=Path(__file__).with_name('release_01.json')
ALLOWED=(('S4','F0','P0'),('S4','F1','P0'),('S4','F2','P0'),('S4','F2','P2'),('S4','F2','P4'))
require=parent.require

def seal():
    previous=parent.contract();files={**previous['files'],**read(DATA/'source_manifest.json')}
    for p in Path(__file__).parent.iterdir():
        if p.suffix in ('.py','.xml','.sh'):files[p.relative_to(ROOT).as_posix()]=sha(p)
    for n,h in files.items():require(sha(ROOT/n)==h,'Source drift '+n)
    write(RELEASE,{'revision':'S4_terminal_real_v1_20261118','parent_release_sha256':sha(parent.RELEASE),
        'python_manifest_sha256':sha(DATA/'final_manifest.json'),'files':files,'allowed':[list(x) for x in ALLOWED]})

def contract():
    parent.contract();r=read(RELEASE)
    require(r['parent_release_sha256']==sha(parent.RELEASE),'v6 release drift')
    require(r['python_manifest_sha256']==sha(DATA/'final_manifest.json'),'Python manifest drift')
    for n,h in r['files'].items():require(sha(ROOT/n)==h,'S4 source drift '+n)
    return r

def python_check(replay=True):
    contract()
    for n,h in read(DATA/'final_manifest.json').items():require(sha(DATA/n)==h,'Python evidence changed '+n)
    for n,h in read(DATA/'source_manifest.json').items():
        require(sha(ROOT/n)==h and sha(DATA/'source'/n)==h,'Python snapshot drift '+n)
    require(all(sha(PARENT/n)==h for n,h in read(PARENT/'final_manifest.json').items()),'Reference provenance drift')
    reports={};equivalent_steps=0
    for key in KEYS:
        flow,case=key.split('/');actual_case='P0' if case=='P0_repeat' else case
        folder=DATA/key;p=plan(case);saved=read(folder/'plan.json');report=read(folder/'report.json')
        expected=sha(PARENT/'source_manifest.json') if case=='P0' else sha(DATA/'source_manifest.json')
        require(report['source_sha256']==expected,'Run source binding')
        require(saved['flow']==flow and saved['case']==actual_case and saved['run_case']==case,'Run identity')
        real=make_plan(actual_case,'S4',flow)
        require(saved['path']==real['path']==p['path'] and saved['config']==real['config']==report['parameters'],'Plan/config drift')
        checked=verify(folder,p) if replay else read(DATA/(key.replace('/','_')+'_verification.json'))
        require(checked['passed'] and checked['record_verified'] and checked['dynamics_pid_replayed'],'Python failure '+key)
        if replay:
            require(numerically_equal(checked,read(DATA/(key.replace('/','_')+'_verification.json'))),'Verification drift '+key)
            mission=Mission(real,world_xml(actual_case))
            for i,row in enumerate(rows(folder)):
                x,z,theta,u,w,q=row['state'];t=row['t']
                target,nav=mission.update({'elapsed_end':t,'pitch_nose_up_rad':theta,
                    'solver_state_ned_frd':{'t':t,'y':x,'z':20-z,'u':u,'w':w,'q':q}})
                require(all(numerically_equal(nav[k],v) for k,v in row['nav'].items()),'Adapter guidance drift '+key)
                require(numerically_equal(target.u,row['control']['target']['u']) and numerically_equal(target.pitch,row['control']['target']['pitch']),'Adapter target drift')
                equivalent_steps+=1
            require(mission.arrived and mission.arrival_time==checked['metrics']['arrival_time'],'Adapter arrival drift')
        reports[key]=checked
    repeats={}
    for f in ('F0','F1','F2'):
        a,b=f+'/P0',f+'/P0_repeat';repeats[f]=repeatability(rows(DATA/a),rows(DATA/b),reports[a],reports[b])
        require(repeats[f]['passed'],'Repeat failed '+f)
    return {'passed':True,'entries':18,'repeats':repeats,'equivalent_steps':equivalent_steps,
            'source_manifest_sha256':sha(DATA/'source_manifest.json'),'evidence_manifest_sha256':sha(DATA/'final_manifest.json')}

@dataclass(frozen=True)
class S4Proof:
    report:dict
    probes:dict
    release_sha256:str
    parent_proof:object

def pilot_evaluate(normal,repeat,abort):
    evidence=python_check(replay=True);proof=parent.pilot_evaluate(normal,repeat,abort)
    report={**proof.report,'s4_release_sha256':sha(RELEASE),'s4_python':evidence,
        'allowed':[list(x) for x in ALLOWED],'scope':'S4 five-case launch prerequisites only; v6 zero-input probes are shared transport evidence, not S4 navigation acceptance',
        'formal_trials_started':False}
    return S4Proof(report,proof.probes,sha(RELEASE),proof)

def guard(proof,runtime=True,gui=True):
    require(isinstance(proof,S4Proof),'Typed S4 proof required');contract()
    require(proof.release_sha256==sha(RELEASE),'Release drift')
    evidence=python_check(replay=False)
    require(evidence['evidence_manifest_sha256']==proof.report['s4_python']['evidence_manifest_sha256'],'Evidence changed')
    parent.guard(proof.parent_proof,runtime=runtime,gui=gui)

def verify_record_binding(directory):
    directory=Path(directory);contract();p=read(directory/'evidence_gate.json');meta=read(directory/'metadata.json')
    require((meta['scene'],meta['flow'],meta['case']) in ALLOWED,'Unregistered real case')
    require(p['passed'] and p['ready_to_run'] and p['s4_release_sha256']==sha(RELEASE)==sha(directory/'adapter_release.json'),'Missing bound S4 launch proof')
    for n,h in read(RELEASE)['files'].items():require(sha(directory/'source'/n)==h,'Trial source drift')
    python_check(replay=False)
    for name in ('normal','repeat','abort'):
        r=parent.abort_evidence(directory/'transport_evidence'/name) if name=='abort' else parent.analyze(directory/'transport_evidence'/name)
        require(r==p['transport'][name] and r['passed'],'Transport binding drift')
    return True
