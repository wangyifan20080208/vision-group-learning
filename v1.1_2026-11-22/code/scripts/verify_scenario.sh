#!/usr/bin/env bash
# 单个验证场景：启动 → 评估 → 清理
#   bash verify_scenario.sh <rate> <deskew> <ego> <ego_filter> <ego_range> <秒>
set +u
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

RATE=$(printf "%.3f" "${1:-0}")
DSK=${2:-true}
EGO=${3:-true}
EFIL=${4:-true}
ERNG=$(printf "%.3f" "${5:-0.45}")
DUR=${6:-8}
MAP="$HOME/rm_ws/nav_sim_ws/maps/rmul2027/map.yaml"

cleanup() {
  pkill -f "ros2 launch rm_nav_sim" 2>/dev/null
  pkill -9 -f "grid_sim|deskew_node|nav2_|lifecycle_manager|map_server|planner_server|controller_server|bt_navigator|behavior_server|waypoint_follower|smoother_server|velocity_smoother" 2>/dev/null
  sleep 1.5
}
trap cleanup EXIT

cleanup > /dev/null 2>&1

nohup ros2 launch rm_nav_sim nav_gimbal.launch.py \
  map:="$MAP" \
  params:="$HOME/rm_ws/nav_sim_ws/config/nav2_params_2027.yaml" \
  start_x:=-0.22 start_y:=-2.22 start_yaw:=0.0 \
  gimbal_rate:="$RATE" enable_deskew:="$DSK" \
  enable_ego:="$EGO" ego_filter:="$EFIL" ego_range:="$ERNG" \
  rviz:=false > /tmp/gimbal.log 2>&1 &

for i in $(seq 1 40); do
  if ros2 topic list 2>/dev/null | grep -qx /scan; then break; fi
  sleep 1
done
sleep 3

echo "############ 场景: 云台=${RATE}°/s  deskew=${DSK}  ego=${EGO}  ego_filter=${EFIL}  ego_range=${ERNG}m ############"
python3 /mnt/d/dshworkspace/rm_nav_sim/tools/scan_eval.py "$MAP" "$DUR" 2>&1
echo
