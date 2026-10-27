"""Verify package inputs; output recordings are deliberately not package inputs."""
import hashlib
import json
from pathlib import Path

def verify(root):
    root=Path(root).resolve()
    entries=json.loads((root/'manifest.json').read_text())
    if not entries: raise ValueError('Empty manifest')
    for name,digest in entries.items():
        path=(root/name).resolve()
        if not path.is_relative_to(root) or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise ValueError('Package checksum mismatch: '+name)
    return len(entries)

if __name__=='__main__':
    print('Package verified:',verify(Path(__file__).resolve().parents[1]),'files')
