#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if ! command -v nix >/dev/null; then
  echo 'Nix is required. Use x86_64 Linux with Nix, or NixOS inside WSL2. See README.md.' >&2
  exit 1
fi
cache="${XDG_CACHE_HOME:-$HOME/.cache}/uuv-dr-lu-demo-v1"
mkdir -p "$cache"
cp "$root/flake.nix" "$root/flake.lock" "$cache/"
cd "$cache"
exec nix --extra-experimental-features 'nix-command flakes' develop . --command bash -c '
  cd -- "$1"; shift
  exec python3 -m demo.launch "$@"
' uuv-demo "$root" "$@"
