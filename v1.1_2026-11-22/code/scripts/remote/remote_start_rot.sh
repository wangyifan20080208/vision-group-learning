#!/usr/bin/env bash
# 启动雷达 + 录制，供「转动雷达」测试
set +u
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$HOME/rm_ws/sentry_ws/install/setup.bash"
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH

DRV="$HOME/ws_livox/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node"
DUR=${1:-240}
TEST="$HOME/rm_ws/sentry_ws/src/sentry_robot/sentry_robot/scan_rotate_test.py"

echo "=== 清理 ==="
pkill -9 -f "$DRV" 2>/dev/null
pkill -9 -f livox_scan_node 2>/dev/null
pkill -9 -f scan_rotate_test 2>/dev/null
pkill -9 -f hik_camera_node 2>/dev/null
pkill -9 -f "/opt/MVS/bin/MVS" 2>/dev/null
sleep 3

echo "=== 启动雷达驱动 + livox_scan（倒置关闭，按桌面实际朝向）==="
cd "$HOME" || exit 1
setsid nohup ros2 launch sentry_robot sensors.launch.py \
  lidar:=true camera:=false inverted:=false deskew:=true \
  > /tmp/rot_sensors.log 2>&1 < /dev/null &
sleep 14

echo "驱动 $(pgrep -f "$DRV" | wc -l)  livox_scan $(pgrep -f livox_scan_node | wc -l)"
printf "  /scan 频率: "
timeout 8 ros2 topic hz /scan 2>/dev/null | grep -m1 "average rate" || echo "无数据"

echo
echo "=== 启动录制（${DUR}s，后台）==="
setsid nohup python3 "$TEST" record "$DUR" \
  > /tmp/rot_record.log 2>&1 < /dev/null &
sleep 4
echo "录制进程: $(pgrep -f scan_rotate_test | wc -l)"
head -2 /tmp/rot_record.log
echo
echo ">>> 现在开始转动雷达 <<<"
