#!/usr/bin/env bash
# 冒烟测试：确认云台/倒置雷达链路的话题都在
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
MAP="$HOME/rm_ws/nav_sim_ws/maps/rmul2027/map.yaml"

echo "=== 进程 ==="
pgrep -a -f "grid_sim|deskew_node" | grep -v pgrep | head -4

echo "=== 关键话题 ==="
for t in /scan_raw /scan /livox/imu /odom /map; do
  printf "%-14s " "$t"
  if ros2 topic list 2>/dev/null | grep -qx "$t"; then
    ros2 topic info "$t" 2>/dev/null | grep -i "publisher" | tr -d '\n'
    echo
  else
    echo "缺失"
  fi
done

echo "=== 频率 ==="
echo -n "/scan_raw  "; timeout 5 ros2 topic hz /scan_raw 2>&1 | head -1
echo -n "/scan      "; timeout 5 ros2 topic hz /scan 2>&1 | head -1
echo -n "/livox/imu "; timeout 5 ros2 topic hz /livox/imu 2>&1 | head -1

echo "=== TF 链 ==="
timeout 4 ros2 run tf2_ros tf2_echo base_footprint gimbal_yaw 2>&1 | grep -A1 Translation | head -2
timeout 4 ros2 run tf2_ros tf2_echo gimbal_yaw laser 2>&1 | grep -A1 Translation | head -2

echo "=== /scan 帧与束数 ==="
timeout 5 ros2 topic echo /scan --once --field header 2>&1 | head -4
