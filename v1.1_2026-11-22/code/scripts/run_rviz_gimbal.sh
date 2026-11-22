#!/usr/bin/env bash
# 云台载倒置雷达场景 + RViz
#   bash run_rviz_gimbal.sh           云台 180°/s
#   bash run_rviz_gimbal.sh 0         云台静止（看对齐效果最直观）
#   SOFT=1 bash run_rviz_gimbal.sh    软件渲染（地图显示异常时用）
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
export DISPLAY=:0

RATE=$(printf "%.1f" "${1:-180}")
LAUNCH_FILE=${LAUNCH:-nav_gimbal}

if [ "${SOFT:-0}" = "1" ]; then
  echo ">>> 软件渲染"; export LIBGL_ALWAYS_SOFTWARE=1
else
  echo ">>> GPU 渲染（Mesa d3d12 / Xe）"; unset LIBGL_ALWAYS_SOFTWARE
fi

# 防重复：多实例会让 /scan 与 /odom 出现多个发布者，画面乱跳
if pgrep -f "ros2 launch rm_nav_sim" > /dev/null; then
  echo ">>> 清理已在运行的仿真"
  pkill -9 -f "grid_sim|deskew_node|nav2_|lifecycle_manager|amcl|planner_server|controller_server|bt_navigator|behavior_server|map_server|waypoint_follower|smoother_server|velocity_smoother|rviz2|ros2 launch rm_nav_sim" 2>/dev/null
  sleep 2
fi

echo ">>> 场地 RMUL2027 3v3 | 雷达：倒置/同轴/随云台 | 云台 ${RATE}°/s | 去畸变 ON"

nohup ros2 launch rm_nav_sim "${LAUNCH_FILE}.launch.py" \
  map:="$HOME/rm_ws/nav_sim_ws/maps/rmul2027/map.yaml" \
  params:="$HOME/rm_ws/nav_sim_ws/config/nav2_params_2027.yaml" \
  start_x:=-0.22 start_y:=-2.22 start_yaw:=0.0 \
  gimbal_rate:="$RATE" enable_deskew:=true \
  enable_ego:=true ego_filter:=true ego_range:=0.450 \
  rviz:=false > /tmp/gimbal.log 2>&1 &

echo "Nav2 启动中（日志 /tmp/gimbal.log）..."
for i in $(seq 1 60); do
  if ros2 action list 2>/dev/null | grep -q navigate_to_pose; then
    echo "就绪（${i}s），打开 RViz..."
    break
  fi
  sleep 1
done
sleep 2

exec rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
