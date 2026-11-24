#!/usr/bin/env bash
# 目标机器现状调查
set +u
source /opt/ros/humble/setup.bash 2>/dev/null

echo "=== 系统 ==="
lsb_release -ds; uname -r; echo "核数 $(nproc)  内存 $(free -g | awk 'NR==2{print $2}')GB"

echo
echo "=== ROS ==="
echo "包数 $(ros2 pkg list 2>/dev/null | wc -l)"
echo "Nav2 $(ros2 pkg list 2>/dev/null | grep -cE '^nav2_')"
echo "slam_toolbox $(ros2 pkg list 2>/dev/null | grep -c slam_toolbox)"

echo
echo "=== 工作区 ==="
find "$HOME/rm_ws" -maxdepth 3 -type d -not -path "*/build/*" -not -path "*/log/*" 2>/dev/null | head -20

echo
echo "=== 代码仓库 ==="
if [ -d "$HOME/vision-group-learning/.git" ]; then
  cd "$HOME/vision-group-learning"
  echo "分支 $(git branch --show-current)  提交 $(git rev-parse --short HEAD)"
  echo "版本目录:"
  ls -d v*/ 2>/dev/null | sed 's/^/  /'
else
  echo "无 ~/vision-group-learning"
fi

echo
echo "=== 雷达网络 (enp1s0 = 192.168.1.50) ==="
ip -br addr show enp1s0
echo "扫描 192.168.1.0/24 …"
for i in $(seq 1 254); do
  ( ping -c1 -W1 192.168.1.$i >/dev/null 2>&1 && echo "  UP 192.168.1.$i" ) &
done
wait
echo "扫描完成"

echo
echo "=== USB 设备 ==="
lsusb 2>/dev/null | grep -viE 'hub|root|Intel|Realtek|Logitech' | head -10

echo
echo "=== 串口 ==="
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "无串口设备"

echo
echo "=== livox 相关 ==="
find "$HOME" -maxdepth 4 -iname "*livox*" -o -maxdepth 4 -iname "*MID360*" 2>/dev/null | head -10

echo
echo "=== 相机 SDK (MVS) ==="
ls /opt/MVS 2>/dev/null && echo "MVS 已装" || echo "MVS 未装"
python3 -c "import MvCameraControl" 2>/dev/null && echo "python MVS 可用" || echo "python MVS 不可用"
