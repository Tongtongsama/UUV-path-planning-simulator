#!/usr/bin/env bash
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
out="$1"
rm -rf -- "$out"
mkdir -p -- "$out"
export ROS_DOMAIN_ID=74
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
cd -- "$repo"

python3 -m ros2.twin_axial_g12_real_v2.transport_probe state-sub "$out/state_sub.json" &
state_sub=$!
python3 -m ros2.twin_axial_g12_real_v2.transport_probe state-pub "$out/state_pub.json"
wait "$state_sub"

python3 -m ros2.twin_axial_g12_real_v2.transport_probe pose-sub "$out/pose_sub.json" &
pose_sub=$!
python3 -m ros2.twin_axial_g12_real_v2.transport_probe pose-pub "$out/pose_pub.json"
wait "$pose_sub"

python3 -m ros2.twin_axial_g12_real_v2.transport_probe action-server "$out/action_server.json" &
action_server=$!
sleep 2
python3 -m ros2.twin_axial_g12_real_v2.transport_probe action-client "$out/action_client.json"
wait "$action_server"
