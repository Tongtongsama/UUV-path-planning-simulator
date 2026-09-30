"""Freeze replay output, browser evidence and media after export/acceptance."""
import argparse
import json
from pathlib import Path
from .replay_export import digest, load, ROOT


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();folder=args.output
    if (folder/'delivery_manifest.json').exists():raise ValueError('Already frozen; no overwrite')
    export=load(folder/'manifest.json')
    for name,sha in export['source_files'].items():
        if digest(ROOT/name)!=sha:raise ValueError('Original source changed: '+name)
    for name,sha in export['outputs'].items():
        if digest(folder/name)!=sha:raise ValueError('Export changed: '+name)
    qa=load(folder/'browser_acceptance.json')
    if not qa['passed'] or not qa.get('video') or qa['video']['decoded']['ready']<2:
        raise ValueError('Browser or video acceptance missing')
    record=dict(status='VERIFIED_OFFLINE_REPLAY',simulation_rerun=False,
        runtime_tests='Original 847 passed evidence unchanged; replay-only tests and browser acceptance separate.',
        files={str(p.relative_to(folder)).replace('\\','/'):digest(p) for p in folder.rglob('*') if p.is_file()},
        source_code={str(p.relative_to(ROOT)).replace('\\','/'):digest(p) for p in
            [ROOT/'visualization/replay_export.py',ROOT/'visualization/replay_finalize.py',
             ROOT/'visualization/replay/player.html',ROOT/'visualization/replay/player.js',ROOT/'scripts/test_replay.cjs']})
    (folder/'delivery_manifest.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
    print('Frozen:',folder,'manifest SHA-256',digest(folder/'delivery_manifest.json'))


if __name__=='__main__':main()
