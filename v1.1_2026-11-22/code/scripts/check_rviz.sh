#!/usr/bin/env bash
echo "=== rviz2 包路径 ==="
PFX=$(ros2 pkg prefix rviz2 2>/dev/null)
echo "prefix=$PFX"
if [ -n "$PFX" ]; then
  ls "$PFX/lib/rviz2" 2>&1 | head -5
fi
echo
echo "=== GPU 设备 ==="
ls -l /dev/dri 2>&1 | head -5
ls -l /dev/dxg 2>&1 | head -5
echo
echo "=== Mesa / GL ==="
which glxinfo 2>/dev/null || echo "no glxinfo"
ls /usr/lib/x86_64-linux-gnu/dri/ 2>/dev/null | head -10
echo
echo "=== 尝试启动 rviz2（15 秒后杀掉）==="
timeout 15 ros2 run rviz2 rviz2 2>&1 | tail -15
