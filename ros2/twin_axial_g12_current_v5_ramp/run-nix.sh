#!/usr/bin/env bash
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
export UUV_XDYN_LOCK=/home/nixos/.cache/uuv-twin-axial-g12-real-nix/flake.lock
export PYTHONDONTWRITEBYTECODE=1
cd "$repo"
exec nix develop --offline --no-write-lock-file /home/nixos/.cache/uuv-twin-axial-g12-real-nix --command python3 -m "ros2.twin_axial_g12_current_v5_ramp.$@"
