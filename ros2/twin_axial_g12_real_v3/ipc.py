"""Append-only JSONL reader. Commit only complete, validated byte records."""
import json


class JsonlReader:
    def __init__(self, path):
        self.path = path
        self.offset = 0
        self.identity = None

    def read(self):
        if not self.path.exists():
            if self.identity is not None:
                raise ValueError('IPC file disappeared')
            return []
        with self.path.open('rb') as stream:
            import os
            info = os.fstat(stream.fileno())
            identity = (info.st_dev, info.st_ino)
            if self.identity is not None and identity != self.identity:
                raise ValueError('IPC file replaced')
            if info.st_size < self.offset:
                raise ValueError('IPC file truncated')
            stream.seek(self.offset)
            data = stream.read()
        end = data.rfind(b'\n') + 1
        block = data[:end]
        records = []
        for line in block.splitlines():
            if not line.strip():
                raise ValueError('Empty IPC record')
            item = json.loads(line.decode('utf-8'))
            if not isinstance(item, dict):
                raise ValueError('IPC record must be an object')
            records.append(item)
        self.identity = identity
        self.offset += end
        return records

