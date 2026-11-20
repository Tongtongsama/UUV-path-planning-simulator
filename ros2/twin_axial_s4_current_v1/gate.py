"""Joint S4 three-flow gate: F0/F1 control evidence + F2 measured-current evidence."""
import json
from dataclasses import dataclass
from pathlib import Path
from ros2.twin_axial_g12_link_v6 import gate as parent
from ros2.twin_axial_s4_terminal_validation_v1.cli import ROOT,sha,read,write

RELEASE=Path(__file__).with_name('release_s4_current_02.json')
EVIDENCE=ROOT/'artifacts/s4_current_python_20261120_01/F2/P0'
S4DATA=ROOT/'artifacts/twin_axial_s4_terminal_validation_20261117_01'
ALLOWED=tuple(('S4',f,c) for f in ('F0','F1','F2') for c in ('P0','P1','P2','P3','P4'))
require=parent.require


def seal():
    previous=parent.contract(); files={**previous['files']}
    for p in Path(__file__).parent.iterdir():
        if p.suffix in ('.py','.xml','.sh'): files[p.relative_to(ROOT).as_posix()]=sha(p)
    for n,h in files.items(): require(sha(ROOT/n)==h,'Source drift '+n)
    write(RELEASE,{'revision':'S4_current_v1_20261120','parent_release_sha256':sha(parent.RELEASE),
        'python_evidence':str(EVIDENCE.relative_to(ROOT)),'python_evidence_sha256':sha(EVIDENCE/'report.json'),
        's4_python_evidence':{f:str((S4DATA/f/'P0/report.json').relative_to(ROOT)) for f in ('F0','F1')},
        's4_python_evidence_sha256':{f:sha(S4DATA/f/'P0/report.json') for f in ('F0','F1')},
        'files':files,'allowed':[list(x) for x in ALLOWED]})


def contract():
    parent.contract(); r=read(RELEASE)
    require(r['parent_release_sha256']==sha(parent.RELEASE),'v6 release drift')
    require(r['python_evidence_sha256']==sha(EVIDENCE/'report.json'),'Python evidence drift')
    for f,h in r.get('s4_python_evidence_sha256',{}).items():
        require(h==sha(S4DATA/f/'P0/report.json'),'S4 Python evidence drift '+f)
    for n,h in r['files'].items(): require(sha(ROOT/n)==h,'S4-current source drift '+n)
    return r


def python_evidence(flow='F2'):
    if flow=='F2':
        report=json.loads((EVIDENCE/'report.json').read_text())
        require(report.get('passed') and report.get('scene')=='S4' and report.get('case')=='P0' and report.get('flow')=='F2','S4+v10 Python evidence not passed')
        return report
    report=json.loads((S4DATA/flow/'P0/report.json').read_text())
    require(report.get('passed') and report.get('scene')=='S4' and report.get('case')=='P0' and report.get('flow')==flow,'S4 '+flow+' Python evidence not passed')
    return report


@dataclass(frozen=True)
class Proof:
    report: dict
    probes: dict
    release_sha256: str
    parent_proof: object


def pilot_evaluate(normal,repeat,abort):
    contract()
    evs={f:python_evidence(f) for f in ('F0','F1','F2')}
    p=parent.pilot_evaluate(normal,repeat,abort)
    report={**p.report,'s4_current_python':evs,'s4_current_evidence_sha256':sha(EVIDENCE/'report.json'),
            'allowed':[list(x) for x in ALLOWED],
            'scope':'S4 three-flow joint gate: F0/F1 control, F2 measured; P0-P4',
            'formal_trials_started':False}
    report['passed']=bool(all(e['passed'] for e in evs.values()) and report.get('runtime',{}).get('passed') and report.get('evidence_passed'))
    report['ready_to_run']=report['passed']
    return Proof(report,p.probes,sha(RELEASE),p)


def guard(proof,runtime=True,gui=True):
    require(isinstance(proof,Proof),'Typed S4-current proof required')
    contract(); require(proof.release_sha256==sha(RELEASE),'Release drift')
    require(proof.report.get('passed') and proof.report.get('ready_to_run'),'S4-current gate not ready')
    for f in ('F0','F1','F2'): python_evidence(f)
    require(proof.report.get('runtime',{}).get('passed'),'Runtime readiness not passed')


def verify_record_binding(directory):
    contract()
    require(sha(Path(directory)/'adapter_release.json')==sha(RELEASE),'Adapter release mismatch')
    meta=json.loads((Path(directory)/'metadata.json').read_text())
    require((meta['scene'],meta['flow'],meta['case']) in ALLOWED,'Unregistered joint case')
    return True
