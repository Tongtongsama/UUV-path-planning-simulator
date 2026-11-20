#!/usr/bin/env bash
set -euo pipefail
cd /mnt/c/Users/Songyuan/uuv_simulator
export UUV_XDYN_LOCK=/home/nixos/.cache/uuv-twin-axial-g12-real-nix/flake.lock
exec nix develop --offline --no-write-lock-file /home/nixos/.cache/uuv-twin-axial-g12-real-nix --command python3 scripts/s4_current_joint_trial.py
