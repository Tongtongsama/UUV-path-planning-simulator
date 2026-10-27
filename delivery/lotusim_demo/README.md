# UUV / LOTUSim demonstration for Dr Lu

This is a small, standalone **frozen-history Gazebo replay**, not a live planning,
control or xdyn experiment. No original repository, user account, Python venv,
LRAUV mesh or private file is needed. Original algorithms and frozen evidence are unchanged.

## Option A — immediate offline viewing (any desktop OS)

Open `OPEN_DEMO.html` in Chrome or Edge. No installation or internet is required.
Select **double_baffle_detour**. This existing browser display also contains four
other historical scenes and supports pause, timeline scrubbing and speed changes.
It is the fallback if Gazebo graphics are slow or unavailable.

## Option B — the same scene inside LOTUSim / Gazebo

Prerequisite: **x86_64 Linux with Nix**, or Windows with **WSL2 + NixOS** already
installed. A plain PowerShell window cannot run this Linux launch command.
GUI additionally needs working Linux graphics (WSLg under Windows).
The package does not install a Linux distribution, GPU driver or remote desktop.
ARM / Apple Silicon and native Windows execution are not supported by this flake.
Official starting points: https://nixos.org/download/ and
https://learn.microsoft.com/windows/wsl/install . Installing WSL alone does not install NixOS.

1. Unzip the full package. Prefer a Linux home-directory path such as `~/uuv-demo`
   to reduce Windows-mounted-filesystem overhead. Keep all files together.
2. Open a terminal in that directory and run:

```bash
bash run.sh
```

First launch needs internet and may download several GB of dependencies. Nix
prepares the pinned LOTUSim/ROS environment automatically. The zip contains source
and data, not the entire simulator installation. No ROS build or website UI is required.
Do not launch the LOTUSim web Demo or a second server for this package.

In Gazebo, select `replay_<id>_scene` in Entity Tree and use **Move To**; orient the
camera along the y direction to see the x-z obstacle plane. There is a 10-second
delay after entities are created. For more setup time use `--start-delay 30`.
Orange is APPROACH, white marks the nose, green is the reference path.
The historical motion lasts about 98.3 seconds at 1x. Final poses stay visible.
Press **Ctrl+C in the terminal** to clean up only this demo's processes and entities.
Keep Gazebo unpaused: this is wall-clock replay, not a simulation-time controller.

```bash
# Both historical arms: blue P1, orange APPROACH, common time axis
bash run.sh --compare --rate 2

# Finish and close automatically (no persistent GUI window)
bash run.sh --auto-close

# Functional check without graphics, approximately 10 seconds of playback
bash run.sh --headless --auto-close --rate 10 --start-delay 0

# Check packaged data and ROS environment without starting Gazebo
bash run.sh --validate-only
```

For GUI stutter, use the HTML option for presentation and the headless command to
verify LOTUSim integration. No remote-desktop session is required by these scripts.

## Output and acceptance

Every launch creates a new `runs/<timestamp_id>/` directory, so repeat launches
never overwrite prior evidence. `gazebo.log` contains simulator diagnostics.
`recording/summary.json` must have `status: completed`, `gazebo_observed: true`,
and nonzero commands/observations. This verifies actual LOTUSim pose feedback,
not just successful transmission. `recording/` contains environment, commands,
observed poses, historical controls and SHA-256 manifests.
P1 TIMEOUT and APPROACH SUCCESS are **historical** outcomes, not freshly measured
controller performance. The package does not simulate hydrodynamics or collisions.

`manifest.json` checks packaged inputs; `PROVENANCE.json` retains original evidence
hashes. The builder verified source evidence before extracting the single Gazebo
scene. Original raw experiment files are not included in this small display pack.
Optional standalone checksum check with Python 3.10+: `python3 -m demo.verify`.

ROS domain 73 and a unique Gazebo partition isolate this demo. Do not run multiple
copies concurrently on the same ROS domain. Working data stay inside this package;
Nix dependencies use the usual user cache/store. Third-party dependencies retain
their upstream licenses; this package's UUV source license is in `LICENSE`.
