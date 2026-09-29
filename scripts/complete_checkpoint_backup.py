"""Attach recoverable Git sources and verify frozen evidence in an existing backup.

Run after backup_artifacts.ps1. Never overwrite an existing source package.
Generated receipts are backup metadata, not substitutes for artifacts.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''): h.update(block)
    return h.hexdigest()


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT)


def safe_path(root, relative):
    result = (root/relative.replace('\\', '/')).resolve()
    if not result.is_relative_to(root.resolve()): raise ValueError('escaping manifest path')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--destination', type=Path, required=True)
    args = parser.parse_args(); destination = args.destination.resolve()
    if destination.is_relative_to(ROOT): raise ValueError('backup must be outside repository')
    receipt = json.loads((destination/'backup_manifest.json').read_text(encoding='utf-8-sig'))
    if receipt['status'] != 'VERIFIED': raise ValueError('artifact backup not verified')
    # Recheck every copied artifact, not just the small dataset indices.
    for entry in receipt['files']:
        if sha(safe_path(destination/'artifacts', entry['path'])) != entry['sha256']:
            raise ValueError('backup content mismatch: '+entry['path'])
    package = destination/'checkpoint_sources'
    package.mkdir(exist_ok=False)
    head = git('rev-parse', 'HEAD').decode().strip()
    git('bundle', 'create', str(package/'repository.bundle'), '--all')
    git('bundle', 'verify', str(package/'repository.bundle'))
    for label, ref in [('checkpoint', head), ('p2b', 'c9d809e')]:
        git('archive', '--format=zip', '--output='+str(package/(label+'.zip')), ref)
        with zipfile.ZipFile(package/(label+'.zip')) as archive:
            if archive.testzip() is not None: raise ValueError('source ZIP CRC failed')
    # Copy authoritative indices and referenced configuration with original bytes.
    verified = []
    indices = ['docs/planner_paired_v1_dataset.json', 'docs/planner_curved_v1_dataset.json',
               'docs/terminal_approach_v1_dataset.json', 'docs/p2b_representation_dataset.json']
    for name in indices:
        local = ROOT/name
        content = local.read_bytes() if local.exists() else git('show', 'c9d809e:'+name)
        target = safe_path(package, name); target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(content)
        index = json.loads(content)
        for path, expected in index.get('files', index.get('manifests', {})).items():
            if path.startswith('artifacts/'):
                stored = safe_path(destination, path)
            else:
                stored = safe_path(package, path); stored.parent.mkdir(parents=True, exist_ok=True)
                data = (ROOT/path).read_bytes() if (ROOT/path).exists() else git('show', 'c9d809e:'+path)
                if stored.exists() and stored.read_bytes() != data: raise ValueError('conflicting configuration')
                if not stored.exists(): stored.write_bytes(data)
            if sha(stored) != expected: raise ValueError('frozen index mismatch: '+path)
            verified.append(path)
    # Reverse-verify pilot/full diagnostics, plots, and independent P2b evidence.
    manifests = ['terminal_approach_v1', 'terminal_approach_full_v1',
                 'terminal_approach_pilot_report_v1', 'terminal_approach_full_report_v1',
                 'p2b_representation_v1', 'p2b_report_final']
    evidence_files = 0
    for folder in manifests:
        base = destination/'artifacts'/folder
        for name, expected in json.loads((base/'manifest.json').read_text()).items():
            if sha(safe_path(base, name)) != expected: raise ValueError('evidence manifest mismatch')
            evidence_files += 1
    environment = subprocess.check_output([sys.executable, '-m', 'pip', 'freeze']).decode()
    (package/'environment_observed.txt').write_text(environment, encoding='utf-8')
    record = dict(status='VERIFIED', head=head, separate_p2b_ref='c9d809e',
        artifact_count=receipt['file_count'], artifact_bytes=receipt['total_bytes'],
        reverse_verified_indices=verified, reverse_verified_evidence_files=evidence_files,
        git_bundle='repository.bundle', python=sys.version,
        environment_note='Observed at packaging, not asserted as a historic lockfile.',
        storage_note='Local D-drive backup only; no offsite or separate-physical-device claim.',
        files={str(p.relative_to(package)).replace('\\','/'):sha(p) for p in package.rglob('*') if p.is_file()})
    result = package/'verification.json'
    result.write_text(json.dumps(record, indent=2), encoding='utf-8')
    print(json.dumps(dict(status='VERIFIED', head=head, evidence_files=evidence_files,
                         verification_sha256=sha(result), source_files=len(record['files'])), indent=2))


if __name__ == '__main__': main()
