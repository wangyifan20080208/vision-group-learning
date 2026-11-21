#!/usr/bin/env bash
# 启动仿真 + Nav2，然后前台打开 RViz2
#   bash run_rviz.sh               真值定位（扫描与地图严丝合缝，推荐）
#   LAUNCH=nav bash run_rviz.sh    带 AMCL（会漂，用来练定位）
#   SOFT=1 bash run_rviz.sh        强制软件渲染（地图 shader 出问题时用）
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
export DISPLAY=:0

LAUNCH_FILE=${LAUNCH:-nav_truth}

if [ "${SOFT:-0}" = "1" ]; then
  echo ">>> 软件渲染"
  export LIBGL_ALWAYS_SOFTWARE=1
else
  echo ">>> GPU 渲染（Mesa d3d12 / Xe）"
  unset LIBGL_ALWAYS_SOFTWARE
fi
echo ">>> 定位模式: ${LAUNCH_FILE}"

nohup ros2 launch rm_nav_sim "${LAUNCH_FILE}.launch.py" \
  map:="$HOME/rm_ws/nav_sim_ws/maps/rmul/map.yaml" \
  params:="$HOME/rm_ws/nav_sim_ws/config/nav2_params_rmul.yaml" \
  start_x:=0.0 start_y:=-2.5 start_yaw:=0.0 rviz:=false \
  > /tmp/nav.log 2>&1 &
echo "Nav2 启动中（日志 /tmp/nav.log）..."

for i in $(seq 1 60); do
  if ros2 action list 2>/dev/null | grep -q navigate_to_pose; then
    echo "Nav2 就绪（${i}s），打开 RViz..."
    break
  fi
  sleep 1
done
sleep 2

exec rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
