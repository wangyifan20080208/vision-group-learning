#!/usr/bin/env bash
# 检查雷达驱动配置与相机可用性
set +u

echo "=== ws_livox 工作区 ==="
find "$HOME/ws_livox" -maxdepth 2 -type d -not -path "*/build/*" -not -path "*/log/*" 2>/dev/null | head -12
echo "是否已编译: $([ -d "$HOME/ws_livox/install" ] && echo 是 || echo 否)"

echo
echo "=== livox_ros_driver2 可执行 ==="
ls "$HOME/ws_livox/install/livox_ros_driver2/lib/livox_ros_driver2/" 2>/dev/null | head

echo
echo "=== livox 配置 JSON ==="
find "$HOME/ws_livox/src/livox_ros_driver2" -name "*.json" 2>/dev/null | head -10

echo
echo "=== MID360_config.json 内容 ==="
CFG=$(find "$HOME/ws_livox/src/livox_ros_driver2" -name "MID360_config.json" 2>/dev/null | head -1)
if [ -n "$CFG" ]; then
  echo "文件: $CFG"
  cat "$CFG"
else
  echo "未找到 MID360_config.json"
fi

echo
echo "=== 主机网卡信息（livox 需要 host_net_info 匹配）==="
ip -br addr show enp1s0

echo
echo "=== 雷达连通性 ==="
ping -c2 -W2 192.168.1.153 2>&1 | tail -3

echo
echo "=== 相机 MVS ==="
ls /opt/MVS/lib/64/ 2>/dev/null | head -5
echo "MVS 示例程序:"
ls /opt/MVS/samples/ 2>/dev/null | head -5

echo
echo "=== python MVS 绑定 ==="
find / -maxdepth 6 -name "MvImport" -type d 2>/dev/null | head -3
find /opt/MVS -name "MvCameraControl*.py" 2>/dev/null | head -4

echo
echo "=== USB 相机权限 ==="
ls -l /dev/bus/usb/002/ 2>/dev/null | head -8
id

echo
echo "=== 已装 python 包（相关）==="
python3 -c "import numpy; print('numpy', numpy.__version__)" 2>/dev/null
python3 -c "import cv2; print('cv2', cv2.__version__)" 2>/dev/null || echo "cv2 未装"
python3 -c "import pygame; print('pygame', pygame.version.ver)" 2>/dev/null || echo "pygame 未装"
