"""Publish verified archive assets to GitHub Releases using Git credential helper.

Credentials remain in memory and are never printed or written to disk.
Usage: python scripts/publish_experiment_backup.py experiment_backups/SNAPSHOT
"""
import concurrent.futures
import json
from pathlib import Path
import subprocess
import sys
import urllib.error
import urllib.parse
import urllib.request
from archive_experiment_records import verify, sha

REPO = 'Tongtongsama/UUV-path-planning-simulator'


def publish(directory):
    directory = Path(directory).resolve()
    verify(directory)
    manifest = json.loads((directory / 'manifest.json').read_text())
    tag = 'experiment-backup-' + manifest['identity'][:16]
    credential = subprocess.run(['git', 'credential', 'fill'], input='protocol=https\nhost=github.com\n\n',
                                text=True, capture_output=True, check=True).stdout
    fields = dict(line.split('=', 1) for line in credential.splitlines() if '=' in line)
    token = fields['password']

    def request(url, data=None, raw=False):
        headers = {'Authorization': 'Bearer ' + token, 'Accept': 'application/vnd.github+json',
                   'X-GitHub-Api-Version': '2022-11-28', 'User-Agent': 'UUV-experiment-backup'}
        if data is not None:
            headers['Content-Type'] = 'application/octet-stream' if raw else 'application/json'
            if not raw:
                data = json.dumps(data).encode()
        with urllib.request.urlopen(urllib.request.Request(url, data=data, headers=headers), timeout=600) as response:
            return json.load(response)

    base = 'https://api.github.com/repos/' + REPO
    try:
        release = request(base + '/releases/tags/' + tag)
    except urllib.error.HTTPError as error:
        if error.code != 404:
            raise
        release = request(base + '/releases', {
            'tag_name': tag, 'target_commitish': subprocess.check_output(['git','rev-parse','HEAD'], text=True).strip(),
            'name': 'Experiment records ' + manifest['identity'][:16], 'make_latest': 'false',
            'body': 'Current artifacts snapshot, including successful and failed experiments. '
                    'Verify manifest SHA-256 before restoring to a new directory. '
                    'External Nix symlink targets and separate historical recovery directory are not included.'})
    existing = {a['name']: a for a in request(base + f"/releases/{release['id']}/assets?per_page=100")}
    # More than 100 archive assets may be present.
    page = 2
    while len(existing) >= 100:
        more = request(base + f"/releases/{release['id']}/assets?per_page=100&page={page}")
        if not more:
            break
        existing.update({a['name']: a for a in more}); page += 1

    def upload(path):
        data = path.read_bytes()
        digest = sha(data)
        if path.name in existing:
            a = existing[path.name]
        else:
            a = request(release['upload_url'].split('{')[0] + '?name=' + urllib.parse.quote(path.name), data, raw=True)
        if a.get('digest') != 'sha256:' + digest:
            # Older GitHub responses may omit digest; independently download then hash.
            with urllib.request.urlopen(urllib.request.Request(a['url'], headers={
                'Authorization': 'Bearer ' + token, 'Accept': 'application/octet-stream',
                'User-Agent': 'UUV-experiment-backup'}), timeout=600) as r:
                assert sha(r.read()) == digest, path.name
        assert a['size'] == len(data), path.name
        print('Verified upload:', path.name, flush=True)
        return {'name': path.name, 'sha256': digest, 'url': a['browser_download_url'], 'bytes': len(data)}

    paths = [directory / a['path'] for a in manifest['archives']] + [directory / 'manifest.json']
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        assets = list(pool.map(upload, paths))
    (directory / 'github_release.json').write_text(json.dumps({'release': release['html_url'],
        'assets': assets, 'verified': True}, indent=2), encoding='utf-8')
    print(release['html_url'])


if __name__ == '__main__':
    publish(sys.argv[1])
