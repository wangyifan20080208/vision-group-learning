#!/usr/bin/env bash
# 释放相机与雷达的占用进程，并检查 ws_livox 安装结构
set +u

echo "=========== 释放前：占用进程 ==========="
ps -eo pid,ppid,comm,args --sort=pid 2>/dev/null \
  | grep -iE "MVS|MvLogServer|MvViewer|LivoxViewer|livox_ros|rviz|ros2 " \
  | grep -v grep

echo
echo "=========== 谁在监听雷达端口 ==========="
if command -v ss >/dev/null; then
  ss -lunp 2>/dev/null | grep -E "56[0-9]{3}" | head -10 || echo "  没有进程监听 56xxx"
fi

echo
echo "=========== 释放 ==========="
# 海康 MVS 图形界面（独占相机）
pkill -f "/opt/MVS/bin/MVS"       2>/dev/null && echo "  已关闭 MVS 图形界面"
pkill -f "MvLogServer"            2>/dev/null && echo "  已关闭 MvLogServer"
pkill -f "MvViewer"               2>/dev/null && echo "  已关闭 MvViewer"
# Livox Viewer（独占雷达）
pkill -f "LivoxViewer"            2>/dev/null && echo "  已关闭 LivoxViewer"
pkill -f "livox_ros_driver2_node" 2>/dev/null && echo "  已关闭 livox 驱动"
sleep 2

echo
echo "=========== 释放后确认 ==========="
left=$(ps -eo comm,args 2>/dev/null | grep -iE "MVS|MvLogServer|LivoxViewer|livox_ros" | grep -v grep | wc -l)
echo "  残留占用进程: $left"

echo
echo "=========== ws_livox 安装结构 ==========="
find "$HOME/ws_livox/install" -maxdepth 4 -type d 2>/dev/null | head -15
echo
echo "--- launch 文件 ---"
find "$HOME/ws_livox/install" -name "*.py" -path "*launch*" 2>/dev/null | head -10
find "$HOME/ws_livox/src/livox_ros_driver2" -name "*.py" -path "*launch*" 2>/dev/null | head -10
echo
echo "--- 包是否在 ament 索引 ---"
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash" 2>/dev/null
ros2 pkg list 2>/dev/null | grep -i livox || echo "  未注册"
