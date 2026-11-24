#!/usr/bin/env bash
# 最终验收：雷达 + 相机 同时工作
set +u
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$HOME/rm_ws/sentry_ws/install/setup.bash"
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH
DRV="$HOME/ws_livox/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node"

echo "=== 清理 ==="
pkill -9 -f "$DRV" 2>/dev/null
pkill -9 -f sentry_robot 2>/dev/null
pkill -9 -f "/opt/MVS/bin/MVS" 2>/dev/null
sleep 4

echo "=== 启动 sensors（雷达 + 相机）==="
cd "$HOME" || exit 1
nohup ros2 launch sentry_robot sensors.launch.py \
  lidar:=true camera:=true inverted:=true deskew:=true \
  exposure_us:=2000.0 gain_db:=17.0 frame_rate:=100.0 \
  > /tmp/final.log 2>&1 &
sleep 18

echo "进程:"
printf "  雷达驱动    %s\n" "$(pgrep -f "$DRV" | wc -l)"
printf "  livox_scan  %s\n" "$(pgrep -f livox_scan_node | wc -l)"
printf "  相机节点    %s\n" "$(pgrep -f hik_camera_node | wc -l)"

echo
echo "话题频率："
for t in /livox/imu /livox/lidar /scan /cloud_deskewed /camera/image_raw; do
  printf "  %-20s " "$t"
  timeout 8 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" || echo "无数据"
done

echo
echo "TF 链（雷达挂云台·倒置同轴）："
timeout 5 ros2 run tf2_ros tf2_echo base_footprint laser 2>&1 \
  | grep -E "Translation|RPY" | head -4 | sed 's/^/  /'

echo
echo "/scan 质量："
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
    print(f"  frame={m.header.frame_id}  束数={len(m.ranges)}  "
          f"有效={len(rr)} ({len(rr)/len(m.ranges)*100:.0f}%)")
    if rr: print(f"  最近 {rr[0]:.2f} m   中位 {rr[len(rr)//2]:.2f} m   最远 {rr[-1]:.2f} m")
else:
    print("  没收到 /scan")
n.destroy_node(); rclpy.try_shutdown()
PY

echo
echo "图像："
timeout 10 ros2 topic echo /camera/image_raw --field width --once 2>/dev/null | head -1 | sed 's/^/  宽 /'
timeout 10 ros2 topic echo /camera/image_raw --field height --once 2>/dev/null | head -1 | sed 's/^/  高 /'
timeout 10 ros2 topic echo /camera/image_raw --field encoding --once 2>/dev/null | head -1 | sed 's/^/  编码 /'

echo
echo "错误检查："
grep -iE "died|ERROR|Traceback" /tmp/final.log | grep -v glsl | head -5 | sed 's/^/  /' || echo "  无"
