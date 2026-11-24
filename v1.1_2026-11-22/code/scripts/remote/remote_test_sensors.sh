#!/usr/bin/env bash
# 测试真机传感器链路：Livox → 去畸变 → /scan，以及相机
set +u
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$HOME/rm_ws/sentry_ws/install/setup.bash"

echo "=== 清理旧进程 ==="
pkill -f livox_ros_driver2_node 2>/dev/null
pkill -f livox_scan_node 2>/dev/null
pkill -f hik_camera_node 2>/dev/null
pkill -f "/opt/MVS/bin/MVS" 2>/dev/null
sleep 2

echo "=== 启动传感器（先只开雷达，验证链路）==="
nohup ros2 launch sentry_robot sensors.launch.py \
  lidar:=true camera:=false inverted:=true deskew:=true \
  > /tmp/sensors.log 2>&1 &
sleep 14

echo "=== 进程 ==="
for p in livox_ros_driver2_node livox_scan_node; do
  printf "  %-24s %s\n" "$p" "$(pgrep -c -f $p)"
done

echo
echo "=== TF 树 ==="
timeout 6 ros2 run tf2_ros tf2_echo base_footprint laser 2>&1 | head -10

echo
echo "=== 话题 ==="
ros2 topic list 2>/dev/null | grep -E "livox|scan|cloud"

echo
echo "=== 频率 ==="
for t in /livox/imu /livox/lidar /scan /cloud_deskewed; do
  printf "  %-20s " "$t"
  timeout 8 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" || echo "无"
done

echo
echo "=== /scan 质量 ==="
timeout 10 python3 - <<'PY'
import math, sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

rclpy.init()
n = Node("scan_check")
box = {}
n.create_subscription(LaserScan, "/scan", lambda m: box.setdefault("m", m), 10)
import time
t0 = time.time()
while "m" not in box and time.time() - t0 < 9:
    rclpy.spin_once(n, timeout_sec=0.2)
if "m" not in box:
    print("  没收到 /scan"); sys.exit()
m = box["m"]
rr = [r for r in m.ranges if r > 0]
print(f"  frame_id = {m.header.frame_id}")
print(f"  束数 {len(m.ranges)}  有效 {len(rr)} ({len(rr)/len(m.ranges)*100:.1f}%)")
if rr:
    rr.sort()
    print(f"  最近 {rr[0]:.2f} m  中位 {rr[len(rr)//2]:.2f} m  最远 {rr[-1]:.2f} m")
n.destroy_node(); rclpy.try_shutdown()
PY

echo
echo "=== 节点日志 ==="
grep -E "livox_scan|livox_lidar_publisher" /tmp/sensors.log | tail -12
