#!/usr/bin/env bash
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cache="${XDG_CACHE_HOME:-$HOME/.cache}/uuv-twin-axial-g12-real-nix"
mkdir -p -- "$cache"
cp -- "$repo/ros2/twin_axial_navigation/nix/flake.nix" "$cache/flake.nix"
cp -- "$repo/ros2/twin_axial_navigation/nix/flake.lock" "$cache/flake.lock"
cd "$cache"
exec nix develop --offline --no-write-lock-file . --command bash -c '
  repo=$1; shift
  cd -- "$repo"
  export UUV_XDYN_LOCK="${XDG_CACHE_HOME:-$HOME/.cache}/uuv-twin-axial-g12-real-nix/flake.lock"
  export PYTHONDONTWRITEBYTECODE=1
  exec python3 -m ros2.twin_axial_g12_real.cli "$@"
' feasible "$repo" "$@"
