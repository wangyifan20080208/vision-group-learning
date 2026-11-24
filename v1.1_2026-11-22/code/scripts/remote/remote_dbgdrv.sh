#!/usr/bin/env bash
# 排查雷达驱动 SIGBUS。注意：pkill 的模式必须精确到二进制路径，
# 否则 -f 会匹配到「执行本脚本的 bash 进程」把自己杀掉。
set +u
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"

DRV="$HOME/ws_livox/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node"

echo "=== 清理（精确匹配二进制路径）==="
pkill -9 -f "$DRV" 2>/dev/null
sleep 3
echo "残留驱动: $(pgrep -f "$DRV" | wc -l)"

echo
echo "=== 内核日志 ==="
dmesg 2>/dev/null | tail -12 || echo "  (需要权限)"

echo
echo "=== 内存 ==="
free -h | head -2

echo
echo "=== 直接跑官方 launch（对照）==="
cd "$HOME" || exit 1
nohup ros2 launch livox_ros_driver2 msg_MID360s_launch.py \
  > /tmp/livox_direct.log 2>&1 &
sleep 14

echo "驱动进程数: $(pgrep -f "$DRV" | wc -l)"
ros2 topic list 2>/dev/null | grep livox | sed 's/^/  /'
for t in /livox/imu /livox/lidar; do
  printf "  %-16s " "$t"
  timeout 8 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" || echo "无数据"
done

echo
echo "=== 对照日志尾部 ==="
tail -8 /tmp/livox_direct.log

echo
echo "=== 官方 launch 里声明的参数个数 ==="
grep -cE "parameters=\[" \
  "$HOME/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/msg_MID360s_launch.py"
grep -nE "publish_freq|frame_id|xfer_format|multi_topic|output_type" \
  "$HOME/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/msg_MID360s_launch.py" \
  | head -10
