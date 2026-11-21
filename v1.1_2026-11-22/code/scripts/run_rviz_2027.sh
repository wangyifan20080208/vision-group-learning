#!/usr/bin/env bash
# 启动 Nav2（2027 场地）+ 前台 RViz2
#   bash run_rviz_2027.sh          真值定位
#   LAUNCH=nav bash run_rviz_2027.sh
#   SOFT=1 bash run_rviz_2027.sh
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
export DISPLAY=:0

# 防重复启动：多开一个实例会让 /scan 和 /odom 出现两个发布者，
# 在 RViz 里表现为"位置乱跳、底盘乱转、扫描对不上地图"
OLD=$(pgrep -f "ros2 launch rm_nav_sim" | wc -l)
if [ "$OLD" -gt 0 ]; then
  echo ">>> 发现 ${OLD} 个已在运行的仿真，先清理"
  pkill -9 -f "grid_sim|nav2_|lifecycle_manager|amcl|planner_server|controller_server|bt_navigator|behavior_server|map_server|waypoint_follower|smoother_server|velocity_smoother|rviz2|ros2 launch rm_nav_sim" 2>/dev/null
  sleep 2
fi

LAUNCH_FILE=${LAUNCH:-nav_truth}
if [ "${SOFT:-0}" = "1" ]; then
  echo ">>> 软件渲染"; export LIBGL_ALWAYS_SOFTWARE=1
else
  echo ">>> GPU 渲染"; unset LIBGL_ALWAYS_SOFTWARE
fi
echo ">>> 场地: RMUL 2027 3v3 | 定位: ${LAUNCH_FILE}"

nohup ros2 launch rm_nav_sim "${LAUNCH_FILE}.launch.py" \
  map:="$HOME/rm_ws/nav_sim_ws/maps/rmul2027/map.yaml" \
  params:="$HOME/rm_ws/nav_sim_ws/config/nav2_params_2027.yaml" \
  start_x:=-0.22 start_y:=-2.22 start_yaw:=0.0 rviz:=false \
  > /tmp/nav2027.log 2>&1 &
echo "Nav2 启动中（日志 /tmp/nav2027.log）..."

for i in $(seq 1 60); do
  if ros2 action list 2>/dev/null | grep -q navigate_to_pose; then
    echo "Nav2 就绪（${i}s），打开 RViz..."
    break
  fi
  sleep 1
done
sleep 2

exec rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
