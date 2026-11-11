"""Frozen v3 evidence gate. Missing live transport evidence always blocks launch."""
from dataclasses import dataclass
from pathlib import Path
import json,time,hashlib
from ros2.twin_axial_g12_real import gate as base
from ros2.twin_axial_s3_transient_v2.cli import ROOT,read,write,sha
RELEASE=Path(__file__).with_name('release.json')

def require(ok,message):
    if not ok:raise ValueError(message)

def seal(directory):
    require(not RELEASE.exists(),'Release exists')
    parent=base.check_contract(directory)
    files=dict(parent['adapter_files'])
    for folder in ('twin_axial_g12_real_v2','twin_axial_g12_real_v3'):
        for p in (ROOT/'ros2'/folder).glob('*.py'):files[p.relative_to(ROOT).as_posix()]=sha(p)
    data={**parent,'adapter_files':files,'parent_release_sha256':sha(base.RELEASE),'revision':'g12_real_v3',
          'transport_policy':'Raw same-stack state/command/pose/action/lifecycle evidence required; missing blocks launch'}
    write(RELEASE,data)
    return data

def contract(directory):
    parent=base.check_contract(directory);release=read(RELEASE)
    require(release['parent_release_sha256']==sha(base.RELEASE),'Parent release drift')
    for key in ('python_files','python_evidence','configuration','runtime_files'):
        require(release[key]==parent[key],'Frozen contract mismatch: '+key)
    for name,digest in release['adapter_files'].items():
        require(sha(ROOT/name)==digest,'Adapter source drift: '+name)
    for folder in ('twin_axial_g12_real_v2','twin_axial_g12_real_v3'):
        require(all(p.relative_to(ROOT).as_posix() in release['adapter_files'] for p in (ROOT/'ros2'/folder).glob('*.py')),'Unbound source')
    return release

def transport_check(directory,gui=True):
    if directory is None:return dict(passed=False,reason='Missing actual-stack transport evidence; offline tests are not live validation')
    directory=Path(directory).resolve();meta=read(directory/'probe.json');manifest=read(directory/'manifest.json')
    require(set(manifest)=={'probe.json','events.jsonl'},'Incomplete probe manifest')
    for name,digest in manifest.items():require(sha(directory/name)==digest,'Probe hash mismatch')
    require(meta['origin']=='actual_integration_probe' and meta['gui']==gui,'Wrong probe scope')
    require(meta['adapter_release_sha256']==sha(RELEASE),'Probe revision mismatch')
    require(meta['runtime_files']==read(RELEASE)['runtime_files'],'Probe runtime mismatch')
    require(0<=time.time()-meta['completed_unix']<=600,'Probe expired/future')
    rows=[json.loads(s) for s in (directory/'events.jsonl').read_text().splitlines() if s]
    groups={k:[r for r in rows if r['channel']==k] for k in ('state','command','pose','action','termination')}
    for channel in ('state','command','pose'):
        records=groups[channel]
        require(len(records)>=30,'Insufficient '+channel+' samples')
        require([r['sequence'] for r in records]==list(range(len(records))),'Noncontiguous '+channel)
        require(all(r['sent']==r['received'] and r['sender_pid']!=r['receiver_pid'] for r in records),'Unmatched process payload '+channel)
        require(all(0<=r['received_monotonic']-r['sent_monotonic']<=.25 for r in records),'Transport delay '+channel)
    require(all(r['message_type']=='lotusim_msgs/msg/VesselPositionArray' for r in groups['pose']),'Wrong pose type')
    require(len(groups['action'])>=1 and all(r['accepted'] and r['status']==4 and r['results'] and all(r['results']) for r in groups['action']),'Action incomplete')
    require(len(groups['termination'])==1,'Missing termination evidence')
    end=groups['termination'][0]
    require(end['zero_applied'] and end['last_step']==end['step_after_cleanup'] and end['cleanup_completed'],'Integration continued during cleanup')
    return dict(passed=True,manifest_sha256=sha(directory/'manifest.json'),directory=str(directory),samples={k:len(v) for k,v in groups.items()})

@dataclass(frozen=True)
class Proof:
    base_proof: object
    release_sha256: str
    transport_directory: object
    report: dict

def evaluate(directory,runtime=False,gui=True,transport=None):
    contract(directory)
    proof=base.evaluate(directory,runtime=runtime,gui=gui)
    try:live=transport_check(transport,gui)
    except Exception as exc:live=dict(passed=False,reason=str(exc))
    report={**proof.report,'adapter_release_sha256':sha(RELEASE),'transport':live,
            'offline_evidence_passed':proof.report['evidence_passed'],
            'passed':bool(proof.report['passed'] and live['passed']),
            'ready_to_run':bool(proof.report['ready_to_run'] and live['passed']), 'real_started':False}
    return Proof(proof,sha(RELEASE),transport,report)

def guard(proof,runtime=True,gui=True):
    require(isinstance(proof,Proof),'In-process v3 proof required')
    require(proof.report['ready_to_run'] and proof.report['passed'],'Live launch gate not passed')
    require(sha(RELEASE)==proof.release_sha256,'Release changed')
    contract(proof.base_proof.directory)
    base.guard(proof.base_proof,runtime=runtime,gui=gui)
    live=transport_check(proof.transport_directory,gui)
    require(live['passed'] and live==proof.report['transport'],'Transport evidence changed')

def verify_record_binding(directory):
    directory=Path(directory);proof=read(directory/'evidence_gate.json');release=read(directory/'adapter_release.json')
    require(proof['passed'] and proof['ready_to_run'] and proof['transport']['passed'],'Missing passing launch evidence')
    require(sha(directory/'adapter_release.json')==sha(RELEASE)==proof['adapter_release_sha256'],'Wrong release')
    require(release['python_source_sha256']==base.APPROVED_PYTHON_SOURCE,'Wrong Python source')
    for name,digest in {**release['python_files'],**release['adapter_files']}.items():
        p=directory/'source'/name
        require(p.is_file() and sha(p)==digest,'Missing/changed source snapshot: '+name)
    probe=directory/'transport_evidence'
    manifest=read(probe/'manifest.json')
    require(sha(probe/'manifest.json')==proof['transport']['manifest_sha256'],'Probe binding mismatch')
    require(set(manifest)=={'probe.json','events.jsonl'},'Missing probe records')
    for name,digest in manifest.items():require(sha(probe/name)==digest,'Changed probe record')
    return True
