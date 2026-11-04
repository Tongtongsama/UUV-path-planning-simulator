"""Read-only artifact snapshots. Run under Linux/WSL to preserve Nix symlinks.

create: archive_experiment_records.py create
verify: archive_experiment_records.py verify BACKUP
restore: archive_experiment_records.py restore BACKUP NEW_DIRECTORY
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
STORE = ROOT / 'experiment_backups'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def create():
    if os.name != 'posix':
        raise SystemExit('Run under WSL/Linux: Windows cannot read WSL Nix symlinks.')
    STORE.mkdir(exist_ok=True)
    entries, excluded = [], []
    for directory, dirs, files in os.walk(ROOT / 'artifacts', followlinks=False):
        for name in sorted(dirs + files):
            p = Path(directory) / name
            rel = p.relative_to(ROOT).as_posix()
            if name in ('node_modules', '__pycache__', '.pytest_cache'):
                excluded.append(rel)
                if name in dirs:
                    dirs.remove(name)
                continue
            mode = p.lstat().st_mode
            if stat.S_ISLNK(mode):
                entries.append({'path': rel, 'link': os.readlink(p)})
            elif stat.S_ISREG(mode):
                data = p.read_bytes()
                if name in ('.env', 'id_rsa', 'credentials.json') or b'-----BEGIN PRIVATE KEY-----' in data:
                    raise ValueError('Review possible credential: ' + rel)
                entries.append({'path': rel, 'bytes': len(data), 'sha256': sha(data)})
    entries.sort(key=lambda x: x['path'])
    identity = sha(json.dumps(entries, sort_keys=True).encode())
    dest = STORE / identity[:16]
    if dest.exists():
        verify(dest)
        print('Unchanged snapshot:', dest)
        return
    with tempfile.TemporaryDirectory(dir=STORE) as tmp:
        tmp = Path(tmp)
        groups, group, size = [], [], 0
        for e in entries:
            if 'link' in e:
                continue
            if group and size + e['bytes'] > 32 * 1024**2:
                groups.append(group); group, size = [], 0
            group.append(e); size += e['bytes']
        if group:
            groups.append(group)
        archives = []
        for i, group in enumerate(groups):
            name = f'records-{i:04d}.zip'
            with zipfile.ZipFile(tmp / name, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as z:
                for e in group:
                    data = (ROOT / e['path']).read_bytes()
                    if sha(data) != e['sha256']:
                        raise ValueError('Source changed: ' + e['path'])
                    z.writestr(e['path'], data)
                    e['archive'] = name
            if (tmp / name).stat().st_size > 49 * 1024**2:
                raise ValueError('Archive too large for repository policy: ' + name)
            archives.append({'path': name, 'sha256': sha((tmp / name).read_bytes())})
        manifest = {'identity': identity, 'files': entries, 'archives': archives,
                    'excluded': excluded, 'scope': 'Current artifacts only; external symlink targets are NOT included. Historical recovery directory is separate.'}
        (tmp / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
        verify(tmp)
        dest.mkdir()
        for p in tmp.iterdir():
            p.rename(dest / p.name)
    print('Created:', dest)


def verify(backup, target=None):
    backup = Path(backup)
    m = json.loads((backup / 'manifest.json').read_text(encoding='utf-8'))
    expected = {e['path']: e for e in m['files'] if 'sha256' in e}
    seen = set()
    for a in m['archives']:
        p = backup / a['path']
        assert sha(p.read_bytes()) == a['sha256'], p
        with zipfile.ZipFile(p) as z:
            for name in z.namelist():
                e = expected[name]
                assert name not in seen and e['archive'] == a['path']
                data = z.read(name)
                assert sha(data) == e['sha256'] and len(data) == e['bytes'], name
                if target:
                    out = (target / name).resolve()
                    assert out.is_relative_to(target.resolve())
                    out.parent.mkdir(parents=True, exist_ok=True)
                    with out.open('xb') as f:
                        f.write(data)
                seen.add(name)
    assert seen == set(expected)
    # Links are metadata only: never create arbitrary external symlinks on restore.
    if target:
        (target / 'external_links.json').write_text(json.dumps([e for e in m['files'] if 'link' in e], indent=2))
    print(json.dumps({'verified_files': len(seen), 'links_metadata_only': len(m['files'])-len(seen)}))


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('action', choices=['create', 'verify', 'restore'])
    ap.add_argument('backup', nargs='?')
    ap.add_argument('destination', nargs='?')
    args = ap.parse_args()
    if args.action == 'create':
        create()
    elif args.action == 'verify':
        verify(args.backup)
    else:
        target = Path(args.destination)
        target.mkdir(parents=True, exist_ok=False)
        verify(args.backup, target)
