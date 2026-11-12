#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

echo "=== build 目录里的 launch 文件是否含 rviz ==="
grep -c "rviz2" "$HOME/rm_ws/nav_sim_ws/build/rm_nav_sim/launch/nav.launch.py"

echo "=== 前台跑 25 秒，只看 rviz 相关行 ==="
timeout 25 ros2 launch rm_nav_sim nav.launch.py \
  map:="$HOME/rm_ws/nav_sim_ws/maps/rmul/map.yaml" \
  params:="$HOME/rm_ws/nav_sim_ws/config/nav2_params_rmul.yaml" \
  start_x:=0.0 start_y:=-2.5 start_yaw:=0.0 rviz:=true 2>&1 \
  | grep -iE "rviz|died|not found|error" | head -20
