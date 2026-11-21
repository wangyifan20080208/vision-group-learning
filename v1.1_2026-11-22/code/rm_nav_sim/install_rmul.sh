#!/usr/bin/env bash
set -e
WS="$HOME/rm_ws/nav_sim_ws"
SRC=/mnt/d/dshworkspace/rm_nav_sim/maps_rmul

mkdir -p "$WS/maps/rmul"
cp -f "$SRC/map.pgm" "$WS/maps/rmul/map.pgm"
cp -f "$SRC/map.yaml" "$WS/maps/rmul/map.yaml"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

python3 "$WS/src/rm_nav_sim/tools/patch_params.py" \
  --out "$WS/config/nav2_params_rmul.yaml" \
  --map "$WS/maps/rmul/map.yaml" \
  --start-x 0.0 --start-y -2.5 --start-yaw 0.0

echo "完成"
