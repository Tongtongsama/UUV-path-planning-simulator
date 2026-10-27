"""Build a portable frozen-history demonstration; never rerun algorithms."""
import hashlib
import json
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from visualization.replay_export import build_payload

def main():
    dest=Path(sys.argv[1]).resolve()
    if dest.exists(): raise SystemExit('Choose a new package directory')
    payload=build_payload(ROOT)  # Verify the original frozen evidence before extraction.
    payload['scenes']=[s for s in payload['scenes'] if s['name']=='double_baffle_detour']
    dest.mkdir(parents=True)
    (dest/'demo').mkdir(); (dest/'demo/__init__.py').write_text('')
    original=ROOT/'ros2/lotusim_replay/lotusim_replay'
    for name in ('core.py','gazebo_scene.py','gazebo_player.py','recording.py'):
        text=(original/name).read_text(encoding='utf-8')
        if name=='core.py':
            text=text[:text.index('def load_verified_payload(')]+'''def load_verified_payload(repository_root):
    import json
    from .verify import verify
    root=Path(repository_root).resolve()
    verify(root)
    return json.loads((root/'payload.json').read_text(encoding='utf-8'))
'''
        if name=='gazebo_player.py':
            text=text.replace("parser.add_argument('--repo-root'", "parser.add_argument('--auto-close', action='store_true')\n    parser.add_argument('--start-delay', type=float, default=10)\n    parser.add_argument('--repo-root'")
            text=text.replace('epoch=time.monotonic()', 'time.sleep(max(0,args.start_delay))\n        epoch=time.monotonic()')
            text=text.replace("print('Replay complete; final poses held. Ctrl+C to remove.',flush=True)", "if args.auto_close: return\n        print('Replay complete; final poses held. Ctrl+C to remove.',flush=True)")
        (dest/'demo'/name).write_text(text,encoding='utf-8',newline='\n')
    for name in ('launch.py','verify.py'):
        shutil.copy2(Path(__file__).parent/name,dest/'demo'/name)
    for name in ('run.sh','README.md','lotusim.world'):
        shutil.copy2(Path(__file__).parent/name,dest/name)
    (dest/'payload.json').write_text(json.dumps(payload,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    shutil.copy2(ROOT/'artifacts/replay_display_v1/index.html',dest/'OPEN_DEMO.html')
    shutil.copy2(ROOT/'License',dest/'LICENSE')
    for name in ('flake.nix','flake.lock'):
        shutil.copy2(ROOT/'ros2/minimal_xdyn/nix'/name,dest/name)
    flake=(dest/'flake.nix').read_text(encoding='utf-8')
    flake=flake.replace('Pinned minimal LOTUSim-xdyn open-loop bench','Pinned LOTUSim frozen-history demonstration')
    flake=flake.replace('pkgs.python3Packages.websockets ', '')
    flake=flake.replace('lotusim.inputs.lxdyn.packages.${system}.xdyn ', '')
    (dest/'flake.nix').write_text(flake,encoding='utf-8',newline='\n')
    provenance={'kind':'frozen-history display, not live dynamics or control',
        'frozen_algorithm_commit':payload['source_commit'],'verified_original_sources':payload['sources'],
        'derived_files':'payload contains only double_baffle_detour; HTML is existing five-scene display',
        'package_changes':'loader verifies packaged payload; optional auto-close and start delay; isolated launcher'}
    (dest/'PROVENANCE.json').write_text(json.dumps(provenance,indent=2,ensure_ascii=False),encoding='utf-8')
    manifest={p.relative_to(dest).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(dest.rglob('*')) if p.is_file()}
    (dest/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print(dest)

if __name__=='__main__': main()
