"""Download verified Release assets: script INDEX_JSON NEW_DIRECTORY."""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import sys
import urllib.parse
import urllib.request


def download(index, destination):
    record = json.loads(Path(index).read_text(encoding='utf-8'))
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=False)

    def fetch(asset):
        name = asset['name']
        assert Path(name).name == name and '/' not in name and '\\' not in name
        url = asset['url']
        assert url.startswith('https://github.com/Tongtongsama/UUV-path-planning-simulator/releases/download/')
        with urllib.request.urlopen(url, timeout=600) as response:
            data = response.read()
        assert len(data) == asset['bytes'] and hashlib.sha256(data).hexdigest() == asset['sha256'], name
        with (destination / name).open('xb') as f:
            f.write(data)
        return name

    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        for name in pool.map(fetch, record['assets']):
            print('Verified:', name, flush=True)


if __name__ == '__main__':
    download(sys.argv[1], sys.argv[2])
