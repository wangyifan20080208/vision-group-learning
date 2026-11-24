#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
WS="$HOME/rm_ws/sentry_ws"
cd "$WS" || exit 1

echo "=== install 结构 ==="
find install/sentry_robot -maxdepth 2 -type d 2>/dev/null | head -12

echo
echo "=== lib/sentry_robot ==="
ls -la install/sentry_robot/lib/sentry_robot/ 2>/dev/null || echo "  不存在"

echo
echo "=== build 日志尾部 ==="
tail -25 "log/latest_build/sentry_robot/stdout_stderr.log" 2>/dev/null || \
  find log -name "stdout_stderr.log" -path "*sentry_robot*" | head -1 | xargs tail -25 2>/dev/null

echo
echo "=== 源码 setup.py entry_points ==="
grep -A6 entry_points src/sentry_robot/setup.py

echo
echo "=== 强制重建 ==="
rm -rf build/sentry_robot install/sentry_robot
colcon build --symlink-install --packages-select sentry_robot 2>&1 | tail -8

echo
echo "=== 重建后 ==="
ls install/sentry_robot/lib/sentry_robot/ 2>/dev/null || echo "  仍然不存在"
source install/setup.bash 2>/dev/null
ros2 pkg executables sentry_robot 2>/dev/null
