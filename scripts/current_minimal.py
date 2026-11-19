"""Unified minimal entry for the frozen v4/v5 S3/F2 current-aware candidates."""
import argparse, importlib, json, sys
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))

CANDIDATES={
    'v4':{'module':'ros2.twin_axial_g12_current_v4','release':'ros2/twin_axial_g12_current_v4/release_current_04.json','kind':'synthetic_schedule'},
    'v5':{'module':'ros2.twin_axial_g12_current_v5_ramp','release':'ros2/twin_axial_g12_current_v5_ramp/release_current_08.json','kind':'measured_uniform_delayed_onset'},
    'v6':{'module':'ros2.twin_axial_g12_current_v6','release':'ros2/twin_axial_g12_current_v6/release_current_09.json','kind':'measured_timeseries_delayed_onset'},
    'v7':{'module':'ros2.twin_axial_g12_current_v7','release':'ros2/twin_axial_g12_current_v7/release_current_10.json','kind':'measured_three_sample_delayed_onset'},
    'v8':{'module':'ros2.twin_axial_g12_current_v8','release':'ros2/twin_axial_g12_current_v8/release_current_11.json','kind':'measured_four_sample_delayed_onset'},
    'v10':{'module':'ros2.twin_axial_g12_current_v10','release':'ros2/twin_axial_g12_current_v10/release_current_13.json','kind':'measured_six_sample_slow_onset'},
}

def main():
    p=argparse.ArgumentParser(description='Run one frozen S3/F2 current-aware case.')
    p.add_argument('--candidate',choices=tuple(CANDIDATES),required=True)
    p.add_argument('--case',choices=('P0','P2'),required=True)
    p.add_argument('--gate-dir',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--expected-release-sha256',default=None)
    a=p.parse_args()
    if a.output.exists(): raise SystemExit(f'Output exists: {a.output}')
    spec=CANDIDATES[a.candidate]
    gate=importlib.import_module(spec['module']+'.gate')
    trial=importlib.import_module(spec['module']+'.trial')
    cli=importlib.import_module('ros2.twin_axial_s3_transient_v2.cli')
    release_sha=cli.sha(gate.RELEASE)
    if a.expected_release_sha256 and a.expected_release_sha256!=release_sha: raise SystemExit('Release hash mismatch')
    gate_dir=a.gate_dir.resolve(); report=json.loads((gate_dir/'gate.json').read_text())
    probes={k:gate_dir/k for k in ('normal','repeat','abort')}
    proof=gate.PilotProof(report,probes,release_sha)
    args=SimpleNamespace(output=a.output.resolve(),scene='S3',flow='F2',case=a.case,gui=True,
        normal_probe=probes['normal'],repeat_probe=probes['repeat'],abort_probe=probes['abort'],
        gate_proof=proof,real_started=False)
    print(f'{a.candidate} {a.case} kind={spec["kind"]} release={release_sha}',flush=True)
    trial.run_trial(args)
    schedule=json.loads((a.output.resolve()/'current_schedule.json').read_text(encoding='utf-8'))
    truth=schedule.get('true_current',{})
    if a.candidate in ('v5','v6','v7','v8','v10'):
        assert truth.get('source_type')=='measured', f'{a.candidate} truth is not measured'
        assert truth.get('ramp'), f'{a.candidate} measured current must declare an onset ramp'
    else:
        assert truth.get('source_type')=='synthetic_schedule', 'v4 truth is not synthetic'
    (a.output.resolve()/'entry_validation.json').write_text(json.dumps({
        'candidate':a.candidate,'case':a.case,'release_sha256':release_sha,
        'current_kind':truth.get('source_type'),'current_source_id':truth.get('source_id'),
        'current_window':truth.get('time_utc') or truth.get('window_utc'),'onset':truth.get('ramp')},indent=2),encoding='utf-8')

if __name__=='__main__': main()
