#!/usr/bin/env bash
# 诊断点云无数据
set +u
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"

echo "=== 驱动版本 ==="
grep -riE "version" "$HOME/ws_livox/src/livox_ros_driver2/package.xml" 2>/dev/null | head -3
ls "$HOME/ws_livox/src/livox_ros_driver2/" | head -12

echo
echo "=== 完整驱动日志 ==="
cat /tmp/livox.log

echo
echo "=== 话题发布者 ==="
ros2 topic info /livox/lidar -v 2>/dev/null | head -12
echo "---"
ros2 topic info /livox/imu -v 2>/dev/null | head -8

echo
echo "=== 点云话题带宽（有数据就会有非零）==="
timeout 12 ros2 topic bw /livox/lidar 2>/dev/null | tail -4 || echo "  无带宽数据"

echo
echo "=== 雷达端口收包统计 ==="
if command -v ss >/dev/null; then
  ss -lunp 2>/dev/null | grep -E "56[0-9]{3}" | head
fi

echo
echo "=== 与雷达的 UDP 会话 ==="
ss -un 2>/dev/null | grep "192.168.1.153" | head -6 || echo "  无"

echo
echo "=== 网卡收包计数（看有没有流量）==="
cat /sys/class/net/enp1s0/statistics/rx_packets 2>/dev/null
sleep 3
cat /sys/class/net/enp1s0/statistics/rx_packets 2>/dev/null
echo "（两个数字差值大说明有流量）"

echo
echo "=== 驱动支持的雷达型号 ==="
grep -rn "Mid360s\|MID360s\|kLidarType" "$HOME/ws_livox/src/livox_ros_driver2/src/include/" 2>/dev/null | head -8
grep -rn "MID360s" "$HOME/ws_livox/src/livox_ros_driver2/src/" 2>/dev/null | head -6
