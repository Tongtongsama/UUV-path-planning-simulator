"""Read-only proof of frozen Python evidence and real adapter equivalence."""
from dataclasses import dataclass, asdict
import hashlib
import importlib
import json
import os
import re
from pathlib import Path
import shutil
import socket
import sys

import numpy as np
from ros2.twin_axial_g12_matrix import cli as matrix
from ros2.twin_axial_s3_transient_v2.cli import ROOT,read,write,sha
from ros2.twin_axial_navigation.acceptance import numerically_equal
from .mission import Mission,make_plan,world_xml,GUIDE_CONFIG,study,planning_pose,FLOWS

APPROVED_PYTHON_SOURCE='a44047dddd1efef230492bde6f093428fc860d61748d5464e82df713115a11d9'
RELEASE=Path(__file__).with_name('release_20261110_02.json')


def require(condition,message):
    if not condition:raise ValueError(message)


def canonical(value):
    return json.loads(json.dumps(value))


def evidence_files(directory):
    names=['source.json','preregistration.json','matrix.json','environment.json']
    for key in matrix.KEYS:
        names.extend(f'{key}/{name}' for name in ('plan.json','report.json','states.jsonl','manifest.json','source_ref.json'))
    return {name:sha(directory/name) for name in names}


def seal_release(directory,runtime_contract=None):
    """Development operation; refuses to overwrite an existing adapter release."""
    require(not RELEASE.exists(),'Adapter release already exists')
    require(matrix.check_source(directory)==APPROVED_PYTHON_SOURCE,'Unexpected Python source revision')
    # Include every extra real transport dependency and its pinned Nix definition.
    paths=list(Path(__file__).parent.glob('*.py'))+[Path(__file__).with_name('run-nix.sh')]
    paths += list((ROOT/'ros2/twin_axial_motion').glob('*.py'))
    paths += [ROOT/'ros2/twin_axial_navigation/nix/flake.nix',ROOT/'ros2/twin_axial_navigation/nix/flake.lock']
    files={p.relative_to(ROOT).as_posix():sha(p) for p in paths}
    data=dict(schema=1,python_source_sha256=APPROVED_PYTHON_SOURCE,configuration=GUIDE_CONFIG,
              python_files=read(directory/'source.json')['files'],
              runtime_files=read(runtime_contract)['runtime_files'] if runtime_contract else {},
              python_evidence=evidence_files(directory),adapter_files=files,
              policy='All 54 raw records, all repeats, shared source and feedback-target equivalence must pass')
    write(RELEASE,data)
    return data


def check_contract(directory):
    require(__debug__,'Optimized Python disables inherited assertions; gate refuses -O')
    directory=Path(directory).resolve()
    release=read(RELEASE)
    require(release['python_source_sha256']==APPROVED_PYTHON_SOURCE,'Unapproved Python revision')
    require(release['configuration']==GUIDE_CONFIG==matrix.g12_config(),'G12 configuration drift')
    require(matrix.check_source(directory)==APPROVED_PYTHON_SOURCE,'Python source mismatch')
    require(release['python_files']==read(directory/'source.json')['files'],'Python source inventory mismatch')
    require(set(release['python_evidence'])==set(evidence_files(directory)),'Incomplete evidence index')
    for name,expected in release['python_evidence'].items():
        p=(directory/name).resolve()
        require(p.is_relative_to(directory) and sha(p)==expected,'Evidence changed: '+name)
    for name,expected in release['adapter_files'].items():
        p=(ROOT/name).resolve()
        require(p.is_relative_to(ROOT.resolve()) and sha(p)==expected,'Adapter source changed: '+name)
    pre=read(directory/'preregistration.json')
    require(pre['entries']==list(matrix.KEYS),'Matrix membership/order changed')
    require(pre['configuration']==GUIDE_CONFIG,'Preregistered configuration changed')
    require(pre['source_sha256']==APPROVED_PYTHON_SOURCE,'Preregistered source changed')
    require(pre['scenes']==canonical(matrix.SCENES) and pre['flows']==canonical(matrix.FLOWS),'Scene/current changed')
    require(pre['initials']==canonical(matrix.study.INITIALS),'Initial states changed')
    require(pre['thresholds']==matrix.study.THRESHOLDS,'Thresholds changed')
    require(pre['repeat_limits']==dict(position_m=.02,arrival_s=.1,terminal_s=.1),'Repeat thresholds changed')
    require(pre['all_runs_fresh'] and pre['reused_prior_trials']==0 and pre['maximum_trials']==54,'Unexpected campaign provenance')
    expected_dirs={directory/key for key in matrix.KEYS}
    actual_dirs={p.parent for p in directory.glob('S*/*/*/report.json')}
    require(actual_dirs==expected_dirs,'Missing or extra trial')
    return release


def feedback_equivalence(folder):
    plan=read(folder/'plan.json')
    actual_plan=make_plan(plan['case'],plan['scene'],plan['flow'])
    require(actual_plan['config']==plan['config'] and actual_plan['path']==plan['path'],'Real adapter path/config mismatch')
    mission=Mission(actual_plan,world_xml(plan['case'],plan['scene'],True))
    reference=study.base_guidance.Guide(plan['path'],FLOWS[plan['flow']])
    count=0;max_coordinate_error=0.;max_diagnostic_error=0.
    for row in matrix.rows(folder):
        x,z,theta,u,w,q=row['state'];t=row['t']
        record=dict(valid=True,elapsed_end=t,pitch_nose_up_rad=theta,
                    solver_state_ned_frd=dict(t=t,y=x,z=20-z,u=u,w=w,q=q))
        target,nav=mission.update(record)
        pose=planning_pose(record)
        mapped=[pose.x,pose.z,pose.pitch,u,w,q]
        coordinate_error=max(abs(a-b) for a,b in zip(mapped,row['state']))
        require(coordinate_error<1e-9,'Feedback coordinate roundtrip mismatch')
        max_coordinate_error=max(max_coordinate_error,coordinate_error)
        with study.patched_config(GUIDE_CONFIG):
            reference_target,reference_nav=reference.update(mapped,t)
        require(numerically_equal(asdict(target),asdict(reference_target)),'Same-input guide target mismatch')
        require(numerically_equal({k:nav[k] for k in reference_nav},reference_nav),'Same-input guide diagnostics mismatch')
        require(numerically_equal(asdict(target),row['control']['target']),'Real adapter target mismatch')
        # Coordinate roundtrip can amplify last-bit differences in diagnostic angles.
        # Keep the original strict target comparison and same-input guide comparison;
        # bound and report diagnostic roundoff separately, never physical acceptance.
        for key,expected in row['nav'].items():
            actual=nav[key]
            if isinstance(expected,(int,float)) and not isinstance(expected,bool):
                error=abs(actual-expected)
                require(np.isfinite(error) and error<1e-9,'Archived guidance diagnostic mismatch: '+key)
                max_diagnostic_error=max(max_diagnostic_error,error)
            else:require(actual==expected,'Archived guidance discrete mismatch: '+key)
        require(abs(nav['swept_clearance_m']-row['clearance'])<1e-9,'Real adapter safety mismatch')
        count+=1
    report=read(folder/'report.json')
    require(mission.arrived and abs(mission.arrival_time-report['metrics']['arrival_time'])<1e-8,'Real adapter arrival mismatch')
    return dict(passed=True,steps=count,max_coordinate_roundtrip_error=max_coordinate_error,
                max_archived_diagnostic_roundoff=max_diagnostic_error)


def runtime_check(gui=True):
    """Inspect imports, executables, display and ports; no nodes or processes start."""
    checks={'linux':sys.platform.startswith('linux')};detail={}
    runtime_files={}
    for name in ('xdyn-for-cs','lotusim','python3'):
        path=shutil.which(name);checks['executable_'+name]=bool(path);detail[name]=path
        if path:runtime_files[str(Path(path).resolve())]=sha(path)
    # The pinned LOTUSim wrapper calls an absolute lotusim-env/gz closure;
    # gz need not be exposed in the outer devShell PATH. Never execute wrappers here.
    try:
        launcher=Path(detail['lotusim']).read_text()
        env_path=re.search(r'/nix/store/[^\s"\x27]+/bin/lotusim-env',launcher).group()
        env_text=Path(env_path).read_text()
        gz_path=re.search(r'/nix/store/[^\s"\x27]+/bin/gz(?=[\s"\x27])',env_text).group()
        checks['gazebo_closure']=all(Path(p).is_file() and os.access(p,os.X_OK) for p in (env_path,gz_path))
        detail['lotusim_env']=env_path;detail['gz_closure']=gz_path
        runtime_files.update({str(Path(p).resolve()):sha(p) for p in (env_path,gz_path)})
    except Exception as exc:checks['gazebo_closure']=False;detail['gazebo_closure']=repr(exc)
    for name in ('rclpy','websockets','lotusim_msgs.msg','lotusim_msgs.action','std_msgs.msg'):
        try:importlib.import_module(name);checks['import_'+name]=True
        except Exception as exc:checks['import_'+name]=False;detail[name]=repr(exc)
    # Load generated ROS type support without initializing rclpy or creating nodes.
    try:
        from lotusim_msgs.msg import VesselCmdArray,VesselPositionArray,VesselCmd,MASCmd
        from lotusim_msgs.action import MASCmdArray
        from std_msgs.msg import String
        from rclpy.type_support import check_for_type_support
        for cls in (VesselCmdArray,VesselPositionArray,VesselCmd,MASCmd,MASCmdArray,String):
            check_for_type_support(cls)
        checks['ros_type_support']=True
    except Exception as exc:checks['ros_type_support']=False;detail['ros_type_support']=repr(exc)
    lock=os.environ.get('UUV_XDYN_LOCK')
    checks['pinned_lock']=bool(lock and Path(lock).is_file() and sha(lock)==sha(ROOT/'ros2/twin_axial_navigation/nix/flake.lock'))
    for port in (12356,12357):
        try:
            with socket.socket() as sock:sock.bind(('127.0.0.1',port))
            checks[f'port_{port}_free']=True
        except OSError as exc:checks[f'port_{port}_free']=False;detail[f'port_{port}']=str(exc)
    checks['gui_environment']=not gui or bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
    checks['gui_driver_bridge']=not gui or Path('/run/opengl-driver').is_dir() or any(shutil.which(n) for n in (os.environ.get('LOTUSIM_GL_WRAPPER','nixGL'),'nixGLIntel','nixGLNvidia'))
    if RELEASE.exists():
        expected=read(RELEASE).get('runtime_files',{})
        checks['frozen_runtime_files']=bool(expected) and runtime_files==expected
    detail['DISPLAY']=os.environ.get('DISPLAY');detail['WAYLAND_DISPLAY']=os.environ.get('WAYLAND_DISPLAY')
    return dict(passed=all(checks.values()),checks=checks,details=detail,runtime_files=runtime_files,scope='Read-only readiness at check time; no solver, ROS node or Gazebo launch')


@dataclass(frozen=True)
class Proof:
    directory: Path
    release_sha256: str
    gui: bool
    runtime_passed: bool
    report: dict


def evaluate(directory,runtime=False,gui=True):
    directory=Path(directory).resolve()
    release=check_contract(directory)
    verified={};bridge={}
    for key in matrix.KEYS:
        verified[key]=matrix.verify_entry(directory/key,key,APPROVED_PYTHON_SOURCE)
        require(verified[key]['passed'],'Python trial failed: '+key)
        bridge[key]=feedback_equivalence(directory/key)
        if key.endswith('P0_repeat'):
            print('Verified Python evidence and real-guide equivalence: '+key.rsplit('/',1)[0],file=sys.stderr,flush=True)
    summary=matrix.summarize(directory,verified)
    require(summary['all_54_passed'],'Full matrix/repeat acceptance failed')
    saved=read(directory/'matrix.json')
    require(numerically_equal(summary,saved),'Saved matrix differs from raw reanalysis')
    check_contract(directory)
    runtime_result=runtime_check(gui) if runtime else None
    report=dict(passed=True,evidence_passed=True,python_trials_verified=54,
                repeat_groups_verified=9,adapter_equivalence=bridge,
                python_source_sha256=APPROVED_PYTHON_SOURCE,adapter_release_sha256=sha(RELEASE),
                runtime=runtime_result,ready_to_run=bool(runtime_result and runtime_result['passed']),
                real_started=False,gui=gui,s3_remains_nominal=True,
                scope='Launch prerequisites only; no real-backend experimental validation')
    if runtime and not runtime_result['passed']:report['passed']=False
    return Proof(directory,sha(RELEASE),gui,bool(runtime_result and runtime_result['passed']),report)


def guard(proof,runtime=True,gui=True):
    require(isinstance(proof,Proof),'A full in-process evidence proof is required; cached JSON is not authorization')
    require(proof.report['evidence_passed'] and proof.gui==gui,'Incompatible proof')
    require(sha(RELEASE)==proof.release_sha256,'Adapter release changed after gate')
    check_contract(proof.directory)
    if runtime:
        require(proof.runtime_passed,'Runtime check was not passed')
        require(runtime_check(gui)['passed'],'Runtime readiness changed')


def verify_record_binding(directory):
    """Bind future real recordings to the approved Python evidence and adapter."""
    directory=Path(directory)
    proof=read(directory/'evidence_gate.json');release=read(directory/'adapter_release.json')
    require(proof['evidence_passed'] and proof['ready_to_run'],'Real run lacks passing launch gate')
    require(proof['python_source_sha256']==APPROVED_PYTHON_SOURCE==release['python_source_sha256'],'Wrong Python evidence revision')
    require(sha(directory/'adapter_release.json')==proof['adapter_release_sha256'],'Recorded adapter release hash mismatch')
    require(sha(directory/'adapter_release.json')==sha(RELEASE),'Recorded release is not this adapter revision')
    require(release['configuration']==GUIDE_CONFIG,'Recorded G12 configuration mismatch')
    for name,digest in {**release['python_files'],**release['adapter_files']}.items():
        saved=directory/'source'/name
        require(saved.is_file() and sha(saved)==digest,'Missing/mismatched real adapter source: '+name)
    return True
