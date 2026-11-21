#!/usr/bin/env bash
# 启动 RMUL 2027 场地的导航仿真（真值定位模式）
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

# 防重复启动：两个实例会同时发布 /scan 和 /odom
if pgrep -f "ros2 launch rm_nav_sim" > /dev/null; then
  echo ">>> 清理已在运行的仿真"
  pkill -9 -f "grid_sim|nav2_|lifecycle_manager|amcl|planner_server|controller_server|bt_navigator|behavior_server|map_server|waypoint_follower|smoother_server|velocity_smoother|ros2 launch rm_nav_sim" 2>/dev/null
  sleep 2
fi
exec ros2 launch rm_nav_sim nav_truth.launch.py \
  map:="$HOME/rm_ws/nav_sim_ws/maps/rmul2027/map.yaml" \
  params:="$HOME/rm_ws/nav_sim_ws/config/nav2_params_2027.yaml" \
  start_x:=-0.22 start_y:=-2.22 start_yaw:=0.0 rviz:=false
