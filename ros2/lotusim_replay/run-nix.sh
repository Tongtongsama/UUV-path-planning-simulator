#!/usr/bin/env bash
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
# Keep the flake outside the Git checkout and its non-ASCII Windows path.
# This also lets Nix see flake.nix before the adapter is committed.
# Preserve flake.lock between invocations for reproducible dependencies.
flake_dir="${XDG_CACHE_HOME:-$HOME/.cache}/uuv-lotusim-replay-nix"
mkdir -p -- "$flake_dir"
cp -- "$repo/ros2/lotusim_replay/nix/flake.nix" "$flake_dir/flake.nix"
cd "$flake_dir"
exec nix develop . --command bash -c '
  repo=$1
  shift
  cd -- "$repo"
  exec python3 -m ros2.lotusim_replay.lotusim_replay.gazebo_player --repo-root "$repo" "$@"
' uuv-lotusim-replay "$repo" "$@"
