"""Bounded append-only reads. ENODATA never commits bytes or refreshes commands."""
import errno
import json
import os
import time
from pathlib import Path


class JsonlReader:
    def __init__(self, path, on_retry=None):
        self.path = Path(path)
        self.offset = 0
        self.identity = None
        self.on_retry = on_retry

    def read(self):
        for attempt in range(3):
            try:
                return self._read()
            except OSError as exc:
                if exc.errno != getattr(errno, 'ENODATA', 61):
                    raise
                if self.on_retry:
                    self.on_retry(dict(errno=exc.errno, offset=self.offset,
                                       attempt=attempt, wall=time.monotonic()))
                if attempt == 2:
                    raise
                time.sleep(.002)

    def _read(self):
        try:
            stream = self.path.open('rb', buffering=0)
        except FileNotFoundError:
            if self.identity is not None:
                raise ValueError('IPC file disappeared')
            return []
        with stream:
            info = os.fstat(stream.fileno())
            identity = (info.st_dev, info.st_ino)
            if self.identity is not None and identity != self.identity:
                raise ValueError('IPC file replaced')
            if info.st_size < self.offset:
                raise ValueError('IPC file truncated')
            stream.seek(self.offset)
            data = stream.read(info.st_size - self.offset)
        end = data.rfind(b'\n') + 1
        records = []
        for line in data[:end].splitlines():
            if not line.strip():
                raise ValueError('Empty IPC record')
            item = json.loads(line.decode('utf-8'))
            if not isinstance(item, dict):
                raise ValueError('IPC record must be an object')
            records.append(item)
        self.identity = identity
        self.offset += end
        return records
