"""Case-bound boundary gate on top of the unmodified v6 engineering gate."""
import json,hashlib,shutil
from pathlib import Path
from dataclasses import dataclass
from ros2.twin_axial_g12_boundary_v2.config import CASES
from ros2.twin_axial_g12_boundary_v2.python_trial import verify
from ros2.twin_axial_g12_boundary_v2 import gate as python_revision
import math
from ros2.twin_axial_g12_link_v6 import gate as parent
from ros2.twin_axial_s3_transient_v2.cli import ROOT,read,sha
RELEASE=Path(__file__).with_name('release_01.json')
require=parent.require

def replay_equal(a,b):
 if isinstance(a,dict):return isinstance(b,dict) and a.keys()==b.keys() and all(replay_equal(a[k],b[k]) for k in a)
 if isinstance(a,(list,tuple)):return isinstance(b,(list,tuple)) and len(a)==len(b) and all(replay_equal(x,y) for x,y in zip(a,b))
 if isinstance(a,(bool,str)) or a is None:return a==b
 if isinstance(a,(int,float)):
  return isinstance(b,(int,float)) and math.isfinite(a) and math.isfinite(b) and math.isclose(a,b,abs_tol=1e-9,rel_tol=1e-10)
 # NumPy booleans are converted without allowing numeric truthiness changes.
 if hasattr(a,'item'):return replay_equal(a.item(),b)
 return a==b

def contract():
 parent.contract();python_revision.contract();data=read(RELEASE)
 require(data['python_release_sha256']==sha(python_revision.RELEASE),'Python release drift')
 require(data['parent_release_sha256']==sha(parent.RELEASE),'Parent release drift')
 for n,h in data['files'].items():require(sha(ROOT/n)==h,'Boundary source drift: '+n)
 return data

def python_check(root,case,replay=True):
 root=Path(root);release=contract();registration=read(root.parent/'preregistration.json')
 require(registration['cases']==CASES and registration['release_sha256']==sha(RELEASE),'Registration mismatch')
 require(read(root/'configurations.json')==CASES,'Python configuration drift')
 for n,h in read(python_revision.RELEASE)['files'].items():require(sha(root/'source'/n)==h,'Python snapshot drift: '+n)
 reports={}
 for name in CASES:
  report=read(root/name/'report.json')
  require(report['source_sha256']==sha(python_revision.RELEASE),'Python release not bound')
  raw=verify(root/name,name) if replay else read(root/(name+'_verification.json'))
  require(raw['record_verified'] and raw['dynamics_pid_replayed'],'Missing full Python replay')
  require(replay_equal(raw,read(root/(name+'_verification.json'))),'Python replay changed')
  raw=read(root/(name+'_verification.json'))  # Canonical stored diagnostics after bounded cross-platform verification.
  reports[name]=raw
 require(reports[case]['passed'],'Corresponding Python case failed; real launch refused')
 return {'case':case,'release_sha256':sha(RELEASE),'config':CASES[case],
         'manifests':{n:sha(root/n/'manifest.json') for n in CASES},'reports':reports}

@dataclass(frozen=True)
class BoundaryProof:
 report:dict
 probes:dict
 release_sha256:str
 parent_proof:object
 case:str
 python_root:Path

def pilot_evaluate(normal,repeat,abort,case,python_root):
 evidence=python_check(python_root,case)
 proof=parent.pilot_evaluate(normal,repeat,abort)
 report={**proof.report,'boundary_release_sha256':sha(RELEASE),'boundary_python':evidence,
         'scope':'Only registered S2/P0 boundary case '+case+'; not nominal acceptance',
         'boundary_case':case}
 return BoundaryProof(report,proof.probes,sha(RELEASE),proof,case,Path(python_root).resolve())

def guard(proof,runtime=True,gui=True):
 require(isinstance(proof,BoundaryProof),'Boundary proof required')
 contract();require(proof.release_sha256==sha(RELEASE),'Boundary release changed')
 require(python_check(proof.python_root,proof.case)==proof.report['boundary_python'],'Python proof changed')
 parent.guard(proof.parent_proof,runtime=runtime,gui=gui)

def verify_record_binding(directory):
 directory=Path(directory);proof=read(directory/'evidence_gate.json');release=contract()
 require(proof['passed'] and proof['ready_to_run'],'Missing live gate')
 require(sha(directory/'adapter_release.json')==sha(RELEASE)==proof['boundary_release_sha256'],'Boundary release mismatch')
 for n,h in release['files'].items():require(sha(directory/'source'/n)==h,'Boundary trial snapshot drift')
 for label in ('normal','repeat','abort'):
  actual=parent.abort_evidence(directory/'transport_evidence'/label) if label=='abort' else parent.analyze(directory/'transport_evidence'/label)
  require(actual==proof['transport'][label] and actual['passed'],'Transport binding mismatch')
 case=read(directory/'metadata.json')['flow']
 require(case==proof['boundary_case'] and proof['boundary_python']['config']==CASES[case],'Case binding mismatch')
 # Python evidence is frozen in a sibling directory, with hashes recorded in the gate.
 require(python_check(directory.parents[1]/'python',case)==proof['boundary_python'],'Python binding mismatch')
 return True
