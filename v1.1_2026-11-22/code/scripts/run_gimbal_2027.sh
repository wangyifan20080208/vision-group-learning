#!/usr/bin/env bash
# 云台载倒置雷达场景的一键运行
#   bash run_gimbal_2027.sh [云台角速度 deg/s] [deskew:true|false] [ego:true|false]
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
export DISPLAY=:0

RATE=${1:-0}
DSK=${2:-true}
EGO=${3:-true}
EGO_FILTER=${4:-true}
EGO_RANGE=${5:-0.45}

# 必须带小数点：ROS 2 会把 "0" 解析成整数，与 double 参数类型冲突导致节点崩溃
RATE=$(printf "%.3f" "$RATE")
EGO_RANGE=$(printf "%.3f" "$EGO_RANGE")

# 防重复启动
if pgrep -f "ros2 launch rm_nav_sim" > /dev/null; then
  echo ">>> 清理旧实例"
  pkill -9 -f "grid_sim|deskew_node|nav2_|lifecycle_manager|amcl|planner_server|controller_server|bt_navigator|behavior_server|map_server|waypoint_follower|smoother_server|velocity_smoother|rviz2|ros2 launch rm_nav_sim" 2>/dev/null
  sleep 2
fi

exec ros2 launch rm_nav_sim nav_gimbal.launch.py \
  map:="$HOME/rm_ws/nav_sim_ws/maps/rmul2027/map.yaml" \
  params:="$HOME/rm_ws/nav_sim_ws/config/nav2_params_2027.yaml" \
  start_x:=-0.22 start_y:=-2.22 start_yaw:=0.0 \
  gimbal_rate:="$RATE" enable_deskew:="$DSK" enable_ego:="$EGO" \
  ego_filter:="$EGO_FILTER" ego_range:="$EGO_RANGE" \
  rviz:=false
