#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
WS="$HOME/rm_ws/sentry_ws"
cd "$WS" || exit 1

echo "=== 上传的源文件检查 ==="
ls -la src/sentry_robot/sentry_robot/
echo "  __init__.py 大小: $(stat -c%s src/sentry_robot/sentry_robot/__init__.py 2>/dev/null) 字节"
echo "  resource 大小:    $(stat -c%s src/sentry_robot/resource/sentry_robot 2>/dev/null) 字节"

echo
echo "=== 方案 A：不带 --symlink-install 重新编译 ==="
rm -rf build/sentry_robot install/sentry_robot
colcon build --packages-select sentry_robot 2>&1 | tail -5
echo "  lib/sentry_robot: $(ls install/sentry_robot/lib/sentry_robot/ 2>/dev/null | tr '\n' ' ' || echo 无)"
echo "  bin:              $(ls install/sentry_robot/bin/ 2>/dev/null | tr '\n' ' ' || echo 无)"

if [ ! -d install/sentry_robot/lib/sentry_robot ]; then
  echo
  echo "=== 方案 B：手动把脚本放到 lib/<pkg>/ ==="
  mkdir -p install/sentry_robot/lib/sentry_robot
  for f in install/sentry_robot/bin/*; do
    [ -f "$f" ] || continue
    n=$(basename "$f")
    ln -sf "$f" "install/sentry_robot/lib/sentry_robot/$n"
    echo "  链接 $n"
  done
fi

echo
echo "=== 结果 ==="
source install/setup.bash 2>/dev/null
ros2 pkg executables sentry_robot 2>/dev/null
