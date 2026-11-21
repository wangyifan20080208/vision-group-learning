#!/usr/bin/env bash
# 把 RMUL 2027 3v3 场地地图接进 nav_sim_ws
set -e
WS="$HOME/rm_ws/nav_sim_ws"
SRC=/mnt/d/dshworkspace/rm_nav_sim/maps_2027

mkdir -p "$WS/maps/rmul2027"
cp -f "$SRC/map.pgm"  "$WS/maps/rmul2027/map.pgm"
cp -f "$SRC/map.yaml" "$WS/maps/rmul2027/map.yaml"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

python3 "$WS/src/rm_nav_sim/tools/patch_params.py" \
  --out "$WS/config/nav2_params_2027.yaml" \
  --map "$WS/maps/rmul2027/map.yaml" \
  --start-x -0.22 --start-y -2.22 --start-yaw 0.0

echo "完成：2027 地图已装入 $WS/maps/rmul2027"
