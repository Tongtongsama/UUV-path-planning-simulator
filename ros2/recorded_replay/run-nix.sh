#!/usr/bin/env bash
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cache="${XDG_CACHE_HOME:-$HOME/.cache}/uuv-recorded-replay-nix"
mkdir -p -- "$cache"
cp -- "$repo/ros2/minimal_xdyn/nix/flake.nix" "$cache/flake.nix"
cp -- "$repo/ros2/minimal_xdyn/nix/flake.lock" "$cache/flake.lock"
cd "$cache"
exec nix develop . --command bash -c '
  repo=$1; shift
  cd -- "$repo"
  exec python3 -m ros2.recorded_replay.cli "$@"
' uuv-recorded-replay "$repo" "$@"
