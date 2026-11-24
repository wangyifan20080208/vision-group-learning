#!/usr/bin/env bash
set +u
B="$HOME/rm_ws/sentry_ws/install/sentry_robot"
echo "=== lib/sentry_robot 实际内容 ==="
ls -la "$B/lib/sentry_robot/" 2>/dev/null || echo "  目录不存在"

echo
echo "=== bin 实际内容 ==="
ls -la "$B/bin/" 2>/dev/null || echo "  不存在"

echo
echo "=== 软链是否可解析 ==="
for f in "$B"/lib/sentry_robot/*; do
  [ -e "$f" ] || { echo "  $(basename $f): 断链"; continue; }
  printf "  %-20s -> %s\n" "$(basename $f)" "$(readlink "$f" 2>/dev/null || echo 普通文件)"
  [ -x "$f" ] && echo "       可执行 ✓" || echo "       不可执行 ✗"
done

echo
echo "=== 目录权限 ==="
stat -c "%A %U:%G %n" "$B/lib" "$B/lib/sentry_robot" "$B/bin" 2>/dev/null

echo
echo "=== ament 索引里的可执行 ==="
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/sentry_ws/install/setup.bash"
ros2 pkg executables sentry_robot
echo "退出码 $?"

echo
echo "=== ros2 launch 实际查找路径 ==="
python3 - <<'PY'
import os
from ament_index_python.packages import get_package_prefix
p = get_package_prefix("sentry_robot")
libexec = os.path.join(p, "lib", "sentry_robot")
print("  prefix :", p)
print("  libexec:", libexec, "存在" if os.path.isdir(libexec) else "不存在")
if os.path.isdir(libexec):
    for f in sorted(os.listdir(libexec)):
        fp = os.path.join(libexec, f)
        print(f"    {f}  可执行={os.access(fp, os.X_OK)}  实体={os.path.isfile(fp)}")
PY
