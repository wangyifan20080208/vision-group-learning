#!/usr/bin/env bash
# 干净启动雷达并验证
set +u
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"

echo "=== MID360s 配置文件 ==="
cat "$HOME/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360s_config.json" 2>/dev/null

echo
echo "=== 用的哪个 launch ==="
head -30 "$HOME/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/msg_MID360s_launch.py" 2>/dev/null | grep -E "json|config|publish|frame" | head -8

echo
echo "=== 清理 ==="
pkill -f livox_ros_driver2_node 2>/dev/null
pkill -f "rviz2" 2>/dev/null
sleep 2

echo "=== 启动（仅消息，无 rviz）==="
nohup ros2 launch livox_ros_driver2 msg_MID360s_launch.py > /tmp/livox.log 2>&1 &
sleep 10
echo "进程: $(pgrep -c -f livox_ros_driver2_node)"

echo
echo "=== 话题 ==="
ros2 topic list 2>/dev/null | grep -i livox

echo
echo "=== 频率 ==="
for t in /livox/lidar /livox/imu; do
  printf "%-16s " "$t"
  timeout 8 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" || echo "无数据"
done

echo
echo "=== 点云字段（要找 offset_time）==="
timeout 10 ros2 topic echo /livox/lidar --once --field fields 2>/dev/null | head -34

echo
echo "=== 点云一帧的头部信息 ==="
timeout 10 ros2 topic echo /livox/lidar --once 2>/dev/null | head -14

echo
echo "=== IMU 数据 ==="
timeout 10 ros2 topic echo /livox/imu --once 2>/dev/null | head -18

echo
echo "=== 驱动日志 ==="
tail -12 /tmp/livox.log
