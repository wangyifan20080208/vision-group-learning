#!/usr/bin/env bash
# 测试相机节点
set +u
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/sentry_ws/install/setup.bash"
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH

echo "=== 确认 MVS 图形界面已关（否则独占相机）==="
pgrep -af "/opt/MVS/bin/MVS" || echo "  已关闭 ✓"

echo
echo "=== 启动相机节点 ==="
pkill -f hik_camera_node 2>/dev/null
sleep 1
nohup ros2 run sentry_robot hik_camera_node --ros-args \
  -p exposure_us:=2000.0 -p gain_db:=17.0 -p frame_rate:=100.0 \
  > /tmp/cam.log 2>&1 &
sleep 10

echo "进程: $(pgrep -f hik_camera_node | wc -l)"
echo
echo "=== 话题 ==="
ros2 topic list 2>/dev/null | grep -i camera
printf "  频率: "
timeout 10 ros2 topic hz /camera/image_raw 2>/dev/null | grep -m1 "average rate" || echo "无数据"

echo
echo "=== 图像信息 ==="
timeout 12 ros2 topic echo /camera/image_raw --field height --once 2>/dev/null | head -2
timeout 12 ros2 topic echo /camera/image_raw --field width --once 2>/dev/null | head -2
timeout 12 ros2 topic echo /camera/image_raw --field encoding --once 2>/dev/null | head -2

echo
echo "=== 日志 ==="
tail -8 /tmp/cam.log

pkill -f hik_camera_node 2>/dev/null
echo
echo "（已停止相机节点）"
