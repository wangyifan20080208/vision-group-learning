#!/usr/bin/env bash
# 底盘侧置+前倾雷达场景：启动 → 评估 → 清理
#   bash verify_tilted.sh <spin_dps> <pitch_deg> <offset_x> <lidar_h> <deskew> <秒>
set +u
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

SPIN=$(printf "%.1f" "${1:-0}")
PITCH=$(printf "%.1f" "${2:-30}")
OFFX=$(printf "%.3f" "${3:-0.15}")
LH=$(printf "%.3f" "${4:-0.275}")
DSK=${5:-true}
DUR=${6:-8}
MAP="$HOME/rm_ws/nav_sim_ws/maps/rmul2027/map.yaml"

cleanup() {
  pkill -f "ros2 launch rm_nav_sim" 2>/dev/null
  pkill -9 -f "grid_sim|deskew_node|nav2_|lifecycle_manager|map_server|planner_server|controller_server|bt_navigator|behavior_server|waypoint_follower|smoother_server|velocity_smoother" 2>/dev/null
  sleep 1.5
}
trap cleanup EXIT
cleanup > /dev/null 2>&1

nohup ros2 launch rm_nav_sim nav_tilted.launch.py \
  map:="$MAP" \
  params:="$HOME/rm_ws/nav_sim_ws/config/nav2_params_2027.yaml" \
  start_x:=-0.22 start_y:=-2.22 start_yaw:=0.0 \
  spin_dps:="$SPIN" pitch_deg:="$PITCH" offset_x:="$OFFX" \
  lidar_h:="$LH" enable_deskew:="$DSK" \
  enable_ego:=true ego_filter:=true ego_range:=0.45 \
  rviz:=false > /tmp/tilted.log 2>&1 &

for i in $(seq 1 40); do
  if ros2 topic list 2>/dev/null | grep -qx /scan; then break; fi
  sleep 1
done
sleep 3

echo "############ 底盘自转=${SPIN}°/s  倾角=${PITCH}°  偏心=${OFFX}m  高=${LH}m  deskew=${DSK} ############"
python3 /mnt/d/dshworkspace/rm_nav_sim/tools/scan_eval.py "$MAP" "$DUR" 2>&1
echo
