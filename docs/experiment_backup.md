# Experiment record backup

`scripts/archive_experiment_records.py` creates a separate, content-identified
snapshot under `experiment_backups/`. Original `artifacts/` files are read only.
It includes successful and failed runs, telemetry, acceptance reports, recorded
source snapshots, figures and manifests present in the current artifacts tree.
It does not assert that missing old records have been recovered.

Run creation using Python 3 in WSL/Linux (Nix symlinks cannot be read correctly
through Windows Python):

```bash
cd /mnt/c/Users/Songyuan/uuv_simulator
python3 scripts/archive_experiment_records.py create
```

Use a Python-enabled Nix shell if `python3` is not on PATH. Snapshots with the
same file inventory and hashes are reused. Archives are limited to 49 MiB each.
All archive members are decompressed and checked against individual SHA-256
hashes before the snapshot is published locally. Archive payloads are uploaded
as GitHub Release assets, verified against their SHA-256 digests. Only manifests,
download URLs and tools enter Git history. Publish with:

```text
python scripts/publish_experiment_backup.py experiment_backups/SNAPSHOT_ID
```

This uses the existing Git credential helper without saving credentials. Download
all assets from the Release into one directory before running verify or restore.
`github_release.json` records individual download URLs and hashes.

For the first snapshot, after cloning this branch on another computer:

```powershell
python scripts/download_experiment_backup.py experiment_backups/cc4e4a74890c1810/github_release.json downloaded_records
python scripts/archive_experiment_records.py verify downloaded_records
python scripts/archive_experiment_records.py restore downloaded_records restored_records
```

The last command produces `restored_records/artifacts/`. Both destination folders
must be new. The download helper supports public Release assets; private repository
access requires an authenticated download method.

Verify or restore using Python 3 (Windows is also supported for these actions):

```text
python scripts/archive_experiment_records.py verify experiment_backups/SNAPSHOT_ID
python scripts/archive_experiment_records.py restore experiment_backups/SNAPSHOT_ID NEW_EMPTY_DESTINATION
```

Restore refuses an existing destination and writes `artifacts/` below the new
destination, without overwriting current research. External symlinks are
recorded in `external_links.json`, not recreated or followed. Nix dependencies
must be installed separately. `node_modules`, `__pycache__` and `.pytest_cache`
are excluded and listed in the manifest. The separate historical restoration
directory outside this repository is not included.

The daily 20:00 Asia/Singapore automation creates/verifies this backup, uploads
Release assets, then commits manifests and URLs with code and documentation.
Raw `artifacts/` stays ignored.
No model, algorithm, acceptance rule or frozen record is modified. As archives
grow, consider dedicated backup storage; do not remove existing backups merely
to make an upload succeed. An interrupted upload can be rerun: existing assets
are checked rather than overwritten.
