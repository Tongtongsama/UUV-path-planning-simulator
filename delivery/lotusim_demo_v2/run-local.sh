#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if ! python3 -c 'import rclpy; from lotusim_msgs.action import MASCmdArray' 2>/dev/null; then
  echo 'Source your LOTUSim/ROS environment first (rclpy and lotusim_msgs are required), or use bash run.sh.' >&2
  exit 1
fi
exec python3 -m demo.launch "$@"
