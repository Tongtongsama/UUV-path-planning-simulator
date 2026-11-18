"""Minimal project entry for the frozen v4 S3/F2 current-aware candidate."""
import argparse,json,sys
from pathlib import Path
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from ros2.twin_axial_g12_current_v4.gate import PilotProof,RELEASE
from ros2.twin_axial_g12_current_v4.trial import run_trial
from ros2.twin_axial_g12_current_v4.schedule import manifest
from ros2.twin_axial_s3_transient_v2.cli import sha


def main():
    p=argparse.ArgumentParser(description='Run one frozen v4 S3/F2 current-aware case.')
    p.add_argument('--case',choices=('P0','P2'),required=True)
    p.add_argument('--gate-dir',type=Path,required=True,help='directory containing gate.json and probe dirs')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--expected-release-sha256',default=None)
    a=p.parse_args()
    if a.output.exists(): raise SystemExit(f'Output exists: {a.output}')
    release_sha=sha(RELEASE)
    if a.expected_release_sha256 and a.expected_release_sha256!=release_sha:
        raise SystemExit('Release hash does not match expected')
    gate_dir=a.gate_dir.resolve(); report=json.loads((gate_dir/'gate.json').read_text())
    probes={k:gate_dir/k for k in ('normal','repeat','abort')}
    proof=PilotProof(report,probes,release_sha)
    out=a.output.resolve()
    args=SimpleNamespace(output=out,scene='S3',flow='F2',case=a.case,gui=True,
        normal_probe=probes['normal'],repeat_probe=probes['repeat'],abort_probe=probes['abort'],
        gate_proof=proof,real_started=False)
    print('v4 minimal',a.case,'release',release_sha,'gate',gate_dir,flush=True)
    run_trial(args)

if __name__=='__main__': main()
