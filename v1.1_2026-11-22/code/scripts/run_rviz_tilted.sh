#!/usr/bin/env bash
# 底盘侧置前倾雷达 + 小陀螺 + RViz
#   bash run_rviz_tilted.sh              默认 20° / 240°/s
#   bash run_rviz_tilted.sh 30 180       自定义 倾角 / 转速
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
export DISPLAY=:0

PITCH=$(printf "%.1f" "${1:-20}")
SPIN=$(printf "%.1f" "${2:-180}")
OFFX=$(printf "%.3f" "${3:-0.15}")
LH=$(printf "%.3f" "${4:-0.275}")
MAP="$HOME/rm_ws/nav_sim_ws/maps/rmul2027/map.yaml"
PARAMS="$HOME/rm_ws/nav_sim_ws/config/nav2_params_2027.yaml"

# 防重复启动：多开一个实例会让 /scan 出现两个发布者、map->odom 重复发布，
# RViz 里表现为位置乱跳、TF 冲突、Nav2 卡在 unconfigured
if pgrep -f "ros2 launch rm_nav_sim" > /dev/null || \
   pgrep -f "static_transform_publisher" > /dev/null; then
  echo ">>> 发现残留进程，彻底清理"
  bash /mnt/d/dshworkspace/scripts/kill_all.sh
fi

echo ">>> 场景：倾角 ${PITCH}° | 自转 ${SPIN}°/s | 偏心 ${OFFX}m | 高 ${LH}m"

nohup ros2 launch rm_nav_sim nav_tilted.launch.py \
  map:="$MAP" params:="$PARAMS" \
  start_x:=-0.22 start_y:=-2.22 start_yaw:=0.0 \
  pitch_deg:="$PITCH" spin_dps:="$SPIN" \
  offset_x:="$OFFX" lidar_h:="$LH" \
  enable_deskew:=true enable_ego:=true ego_filter:=true ego_range:=0.45 \
  rviz:=false > /tmp/tilted_nav.log 2>&1 &
echo "Nav2 启动中（日志 /tmp/tilted_nav.log）..."

for i in $(seq 1 60); do
  if ros2 action list 2>/dev/null | grep -q navigate_to_pose; then
    echo "Nav2 就绪（${i}s），打开 RViz..."
    break
  fi
  sleep 1
done
sleep 2

exec rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
