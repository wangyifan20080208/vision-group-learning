#!/usr/bin/env bash
# 测试雷达：启动 livox_ros_driver2，检查话题
set +u
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"

echo "=== livox launch 文件 ==="
ls "$HOME/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/launch/" 2>/dev/null

echo
echo "=== 清理旧进程 ==="
pkill -f livox_ros_driver2_node 2>/dev/null
sleep 1

echo "=== 启动 MID360 驱动 ==="
LAUNCH="$HOME/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/launch/msg_MID360_launch.py"
if [ ! -f "$LAUNCH" ]; then
  LAUNCH=$(ls "$HOME/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/launch/"*MID360* 2>/dev/null | head -1)
fi
echo "使用: $LAUNCH"

nohup ros2 launch "$LAUNCH" > /tmp/livox.log 2>&1 &
sleep 12

echo
echo "=== 话题 ==="
ros2 topic list 2>/dev/null | grep -i livox

echo
echo "=== 频率 ==="
for t in /livox/lidar /livox/imu; do
  echo -n "$t: "
  timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" || echo "无数据"
done

echo
echo "=== 点云字段（确认有逐点时间戳）==="
timeout 8 ros2 topic echo /livox/lidar --once --field fields 2>/dev/null | head -30

echo
echo "=== IMU 示例 ==="
timeout 8 ros2 topic echo /livox/imu --once 2>/dev/null | head -16

echo
echo "=== 驱动日志尾部 ==="
tail -8 /tmp/livox.log
