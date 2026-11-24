#!/usr/bin/env bash
# 测试我们的 sensors launch（去掉 TimerAction 后）
set +u
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$HOME/rm_ws/sentry_ws/install/setup.bash"

DRV="$HOME/ws_livox/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node"
echo "=== 清理 ==="
pkill -9 -f "$DRV" 2>/dev/null
pkill -9 -f "sentry_robot" 2>/dev/null
sleep 3

echo "=== 启动 sensors（雷达 only）==="
cd "$HOME" || exit 1
nohup ros2 launch sentry_robot sensors.launch.py \
  lidar:=true camera:=false inverted:=true deskew:=true \
  > /tmp/sensors.log 2>&1 &
sleep 15

echo "进程:"
printf "  driver     %s\n" "$(pgrep -f "$DRV" | wc -l)"
printf "  livox_scan %s\n" "$(pgrep -f livox_scan_node | wc -l)"

echo
echo "话题频率:"
for t in /livox/imu /livox/lidar /scan /cloud_deskewed; do
  printf "  %-18s " "$t"
  timeout 8 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" || echo "无"
done

echo
echo "TF 链:"
timeout 5 ros2 run tf2_ros tf2_echo base_footprint laser 2>&1 | grep -A3 Translation | head -5

echo
echo "=== /scan 内容 ==="
timeout 12 python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
rclpy.init(); n = Node("chk"); box = {}
n.create_subscription(LaserScan, "/scan", lambda m: box.setdefault("m", m), 10)
t0 = time.time()
while "m" not in box and time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.2)
if "m" in box:
    m = box["m"]; rr = sorted(r for r in m.ranges if r > 0)
    print(f"  frame={m.header.frame_id} 束数={len(m.ranges)} 有效={len(rr)}"
          f" ({len(rr)/len(m.ranges)*100:.0f}%)")
    if rr:
        print(f"  最近 {rr[0]:.2f} m  中位 {rr[len(rr)//2]:.2f} m  最远 {rr[-1]:.2f} m")
else:
    print("  没收到 /scan")
n.destroy_node(); rclpy.try_shutdown()
PY

echo
echo "=== 关键日志 ==="
grep -E "livox_scan|died|ERROR" /tmp/sensors.log | tail -8
