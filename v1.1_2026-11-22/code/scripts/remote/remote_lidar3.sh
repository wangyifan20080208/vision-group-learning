#!/usr/bin/env bash
# 用正确的消息类型验证点云
set +u
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"

echo "=== 驱动是否在跑 ==="
pgrep -c -f livox_ros_driver2_node

echo
echo "=== /livox/lidar 按 CustomMsg 取一帧（只看点数与头部）==="
timeout 15 ros2 topic echo /livox/lidar livox_ros_driver2/msg/CustomMsg \
  --field point_num --once 2>&1 | head -3

echo
echo "=== 完整一帧的头部字段 ==="
timeout 15 ros2 topic echo /livox/lidar livox_ros_driver2/msg/CustomMsg --once 2>&1 \
  | head -12

echo
echo "=== 频率（指定类型）==="
timeout 12 ros2 topic hz /livox/lidar livox_ros_driver2/msg/CustomMsg 2>&1 | tail -3

echo
echo "=== 带宽 ==="
timeout 12 ros2 topic bw /livox/lidar livox_ros_driver2/msg/CustomMsg 2>&1 | tail -3

echo
echo "=== 单帧点数统计（连续 5 帧）==="
timeout 20 ros2 topic echo /livox/lidar livox_ros_driver2/msg/CustomMsg \
  --field point_num 2>/dev/null | grep -v "^---" | head -6

echo
echo "=== 第一个点的内容（确认有 offset_time）==="
timeout 20 ros2 topic echo /livox/lidar livox_ros_driver2/msg/CustomMsg \
  --field points 2>/dev/null | head -20
